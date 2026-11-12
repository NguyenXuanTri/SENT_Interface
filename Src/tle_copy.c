/*
    PB12 => IF3-1 : CSQ1
    PB14 => IF1-1 : DATA1
    PA8  => IF3-2 : CSQ2
    PB15 => IF1-2 : DATA2
    PB13 => IF2-1, IF2-2 : SCK

    Code này đọc, nhận, xử lý raw -> thang 1-100, xử lý min max
*/

#include "tle5014.h"
#include <string.h>
#include <stdlib.h>

#define TLE5014_COMMAND              0xC001U   /* RW=1, ADDR=0x02, LEN=1 (giữ nguyên của bạn) */
#define TLE5014_SSC_HALF_PERIOD      64U
#define TLE5014_SAMPLE_INTERVAL_MS   10U       /* tay ga cần ~100 Hz, 100 ms là quá chậm */

/* Đặt 1 để dùng các biến live_xxx quan sát chân trong debugger, 0 khi chạy thật */
#define TLE5014_DEBUG_PINS           1

/* Mask các bit lỗi trong safety word. Điền theo bảng safety word trong datasheet
 * TLE5014SP (bản addendum 16D không có). 0 = chưa kiểm tra bit nào. */
#define TLE5014_SAFETY_ERR_MASK      0x0000U

#if TLE5014_DEBUG_PINS
extern volatile uint8_t live_PB12;
extern volatile uint8_t live_PA8;
extern volatile uint8_t live_PB13;
extern volatile uint8_t live_PB14;
extern volatile uint8_t live_PB15;

static void check(void)
{
    live_PB12 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    live_PA8  = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    live_PB13 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    live_PB14 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    live_PB15 = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
}
#else
#define check() ((void)0)
#endif

typedef struct
{
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
    GPIO_TypeDef *data_port;
    uint16_t      data_pin;
} TLE5014_Pins_t;

static const TLE5014_Pins_t ch_pins[] =
{
    {GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_14},   /* CH1 */
    {GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_15}    /* CH2 */
};

static uint8_t hw_ready;

/* ================================================================== */
/*  Tầng giao tiếp SSC bit-bang                                       */
/* ================================================================== */

static void HalfDelay(void)
{
    volatile uint32_t n;
    for (n = 0U; n < TLE5014_SSC_HALF_PERIOD; n++) { __NOP(); }
}

static void ShortDelay(uint32_t loops)
{
    volatile uint32_t n;
    for (n = 0U; n < loops; n++) { __NOP(); }
}

static void SetDataMode(const TLE5014_Pins_t *pn, uint32_t mode)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = pn->data_pin;
    g.Mode  = mode;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(pn->data_port, &g);
}

static void ClockPulse(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    HalfDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HalfDelay();
}

/* Gửi 16 bit MSB-first. DATA được giữ ổn định suốt cả xung SCK nên
 * sensor latch ở cạnh nào cũng đúng. */
static void WriteWord(const TLE5014_Pins_t *pn, uint16_t w)
{
    uint8_t i;
    for (i = 0U; i < 16U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pn->data_port, pn->data_pin,
                          (w & 0x8000U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        ClockPulse();
        w <<= 1U;
        check();
    }
}

/* Đọc 16 bit MSB-first. Sensor đổi DATA ở cạnh lên, master đọc cuối nửa chu kỳ
 * mức cao (ngay trước cạnh xuống) khi DATA đã ổn định.
 * DATA phải đã được chuyển sang INPUT trước khi gọi. */
static uint16_t ReadWord(const TLE5014_Pins_t *pn)
{
    uint8_t  i;
    uint16_t w = 0U;

    for (i = 0U; i < 16U; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HalfDelay();

        w <<= 1U;
        if (HAL_GPIO_ReadPin(pn->data_port, pn->data_pin) == GPIO_PIN_SET)
        {
            w |= 1U;
        }
        check();

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HalfDelay();
    }
    return w;
}

/* ================================================================== */
/*  Init / ReadAngle                                                  */
/* ================================================================== */

TLE5014_Status_t TLE5014_Init(TLE5014_Handle_t *sensor, UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef g = {0};

    if (sensor == NULL) return TLE5014_ERROR;

    memset(sensor, 0, sizeof(*sensor));
    sensor->uart               = uart;
    sensor->sample_interval_ms = TLE5014_SAMPLE_INTERVAL_MS;
    sensor->channel1_status    = TLE5014_NOT_INITIALIZED;
    sensor->channel2_status    = TLE5014_NOT_INITIALIZED;
    sensor->throttle.state     = TLE5014_THR_NOT_CALIBRATED;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CS HIGH, SCK LOW */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

    /* CS1 (PB12), SCK (PB13) */
    g.Pin   = GPIO_PIN_12 | GPIO_PIN_13;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    /* CS2 (PA8) */
    g.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &g);

    /* DATA mặc định là input (có pull-up) */
    SetDataMode(&ch_pins[0], GPIO_MODE_INPUT);
    SetDataMode(&ch_pins[1], GPIO_MODE_INPUT);

    hw_ready                 = 1U;
    sensor->last_sample_tick = HAL_GetTick();
    sensor->initialized      = 1U;

    return TLE5014_OK;
}

TLE5014_Status_t TLE5014_ReadAngle(TLE5014_Channel_t channel, TLE5014_Angle_t *angle)
{
    const TLE5014_Pins_t *pn;
    uint16_t raw, safety;

    if ((channel != TLE5014_CHANNEL_1) && (channel != TLE5014_CHANNEL_2))
        return TLE5014_ERROR;
    if (angle == NULL)
        return TLE5014_ERROR;
    if (hw_ready == 0U)
        return TLE5014_NOT_INITIALIZED;

    pn = &ch_pins[channel];

    /* 1. CS LOW + gửi command */
    SetDataMode(pn, GPIO_MODE_OUTPUT_PP);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(pn->cs_port, pn->cs_pin, GPIO_PIN_RESET);
    ShortDelay(15U);    

    WriteWord(pn, TLE5014_COMMAND);

    /* 2. Turn-around: nhả DATA trước khi sensor lái đường dây */
    SetDataMode(pn, GPIO_MODE_INPUT);
    ShortDelay(20U); 

    /* 3. Đọc angle word + safety word */
    raw    = ReadWord(pn);
    safety = ReadWord(pn);

    /* 4. CS HIGH */
    HAL_GPIO_WritePin(pn->cs_port, pn->cs_pin, GPIO_PIN_SET);
    check();
    HalfDelay();

    angle->raw_angle     = raw;
    angle->safety_word   = safety;
    angle->angle_counts  = raw & TLE5014_ANGLE_MASK;    // raw & 0x7FFF để lấy 15 bit góc
    angle->angle_degrees = (float)angle->angle_counts * (360.0f / 32768.0f);

    /* 5. Kiểm tra hợp lệ */
    if (((raw == 0xFFFFU) && (safety == 0xFFFFU)) ||    /* DATA bị kéo lên: sensor không trả lời */
        ((raw == 0x0000U) && (safety == 0x0000U)))      /* DATA bị kéo xuống / ngắn mạch         */
    {
        return TLE5014_BUS_ERROR;
    }
    if ((safety & TLE5014_SAFETY_ERR_MASK) != 0U)
    {
        return TLE5014_SAFETY_ERROR;
    }

    return TLE5014_OK;
}

/* ================================================================== */
/*  TAY GA                                                            */
/* ================================================================== */

/* Vị trí tay ga của 1 kênh, thang 0..1000. Trả -1 nếu calibration không hợp lệ.
 * - Tự nhận hướng quay (tăng/giảm) từ min->max, nên CH2 (quay ngược CH1) không cần xử lý riêng.
 * - Xử lý quay vòng 0/360 độ bằng phép & 0x7FFF.
 * - Ngoài hành trình: gần min hơn -> 0, gần max hơn -> 1000. */
static int32_t CalcPermille(uint16_t cnt, uint16_t cmin, uint16_t cmax)
{
    // fwd = khoảng cách từ min->max theo hướng thuận (tăng dần)
    // up = 1 nếu hành trình thuận < 180 độ, 0 nếu > 180 độ
    uint16_t fwd  = (uint16_t)((cmax - cmin) & TLE5014_ANGLE_MASK); 
    uint8_t  up   = (fwd <= 0x4000U);       
    
    // span = khoảng cách từ min->max theo hướng thuận
    // d = khoảng cách từ min->cnt theo hướng thuận
    uint16_t span = up ? fwd : (uint16_t)((cmin - cmax) & TLE5014_ANGLE_MASK);
    uint16_t d    = up ? (uint16_t)((cnt - cmin) & TLE5014_ANGLE_MASK)
                       : (uint16_t)((cmin - cnt) & TLE5014_ANGLE_MASK);

    // kiểm tra hành trình hợp lệ
    if (span < TLE5014_THR_MIN_SPAN)
        return -1;

    // nếu d > span thì tay ga đang nằm ngoài min..max
    if (d > span)
    {
        // over = khoảng cách từ max->cnt theo hướng thuận
        // gap = khoảng cách từ max->min theo hướng thuận
        uint16_t over = (uint16_t)(d - span);
        uint16_t gap  = (uint16_t)(TLE5014_COUNTS_PER_REV - span);

        // nếu tay ga nằm ngoài hành trình, trả 0 hoặc 1000 tùy gần min hay max hơn.
        return (over < (gap / 2U)) ? 1000 : 0;
    }
    return ((int32_t)d * 1000) / (int32_t)span; // trả 0..1000
}

/* 0..1000 -> 1..100 có dead-zone hai đầu */
static uint8_t PermilleToPercent(int32_t pm)
{
    const int32_t lo    = TLE5014_THR_DEADZONE_LOW;
    const int32_t hi    = 1000 - TLE5014_THR_DEADZONE_HIGH;
    const int32_t range = hi - lo;
    int32_t v;

    if (pm <= lo) return 1U;
    if (pm >= hi) return 100U;

    v = 1 + (((pm - lo) * 99) + (range / 2)) / range;
    if (v < 1)   v = 1;
    if (v > 100) v = 100;
    return (uint8_t)v;
}

static void Throttle_Update(TLE5014_Handle_t *s)
{
    TLE5014_Throttle_t *t = &s->throttle;   // con trỏ đến cấu trúc tay ga
    int32_t pm1 = 0, pm2 = 0, pm;           // permille 0..1000
    uint8_t bad = 0U;                       // 1 nếu có lỗi đọc kênh hoặc mismatch > TLE5014_THR_MISMATCH_MAX

    // Nếu chưa calibrate min/max thì báo lỗi
    if (t->cal_flags != 0x03U)
    {
        t->state = TLE5014_THR_NOT_CALIBRATED;  // chưa calibrate min/max
        t->value = 0U;
        return;
    }

    // Nếu kênh 1 hoặc kênh 2 chưa đọc được thì báo lỗi
    if ((s->channel1_status != TLE5014_OK) || (s->channel2_status != TLE5014_OK))
    {
        bad = 1U;
    }
    else
    {
        // Tính vị trí tay ga theo thang 0..1000 (permille) từ 2 kênh
        pm1 = CalcPermille(s->channel1.angle_counts, t->min_cnt[0], t->max_cnt[0]);
        pm2 = CalcPermille(s->channel2.angle_counts, t->min_cnt[1], t->max_cnt[1]);

        // lưu giá trị permille của 2 kênh để debug
        t->pm_ch[0] = (int16_t)pm1;     
        t->pm_ch[1] = (int16_t)pm2;

        // nếu 1 trong 2 kênh trả -1 (calibration không hợp lệ) 
        // hoặc chênh lệch > TLE5014_THR_MISMATCH_MAX thì báo lỗi
        if ((pm1 < 0) || (pm2 < 0)) bad = 1U;
        else if (abs((int)(pm1 - pm2)) > TLE5014_THR_MISMATCH_MAX) bad = 1U;
    }

    // nếu có lỗi đọc kênh hoặc mismatch > TLE5014_THR_MISMATCH_MAX thì tăng fault_cnt, 
    // nếu fault_cnt >= TLE5014_THR_FAULT_LIMIT thì báo lỗi, ngược lại giữ giá trị cũ.
    if (bad != 0U)
    {
        // nếu fault_cnt < 255 thì tăng fault_cnt, 
        // nếu >= TLE5014_THR_FAULT_LIMIT thì báo lỗi
        if (t->fault_cnt < 255U) t->fault_cnt++;

        // nếu fault_cnt >= TLE5014_THR_FAULT_LIMIT thì báo lỗi, ngược lại giữ giá trị cũ.
        // reset bộ lọc và trả 0 để an toàn (không ga)
        if (t->fault_cnt >= TLE5014_THR_FAULT_LIMIT)
        {
            t->state      = TLE5014_THR_FAULT;
            t->value      = 0U;          /* an toàn: báo 0 = không ga */
            t->filt_valid = 0U;
        }
        return;                          /* chưa đủ số lần lỗi: giữ giá trị cũ */
    }

    t->fault_cnt = 0U;

    pm = (pm1 + pm2) / 2;                /* trung bình 2 kênh */

    // Nếu bộ lọc chưa có giá trị thì gán luôn, nếu đã có thì lọc bằng IIR
    if (t->filt_valid == 0U)
    {
        t->filt_x16   = pm * 16;        // lưu giá trị permille * 16 để lọc IIR
        t->filt_valid = 1U;             // đánh dấu bộ lọc đã có giá trị
    }
    else
    {
        // lọc IIR: filt = filt + (pm - filt) / TLE5014_THR_FILTER_DIV
        t->filt_x16 += ((pm * 16) - t->filt_x16) / TLE5014_THR_FILTER_DIV;
    }

    t->permille = (uint16_t)(t->filt_x16 / 16);     // trả giá trị tay ga 0..1000
    t->value    = PermilleToPercent(t->permille);   // trả giá trị tay ga 1..100
    t->state    = TLE5014_THR_OK;                   // tay ga OK
}


//----------------- TÍNH MỐC CALIBRATION (ĐO MIN/MAX) ---------------------
/* Đọc trung bình 8 mẫu (tránh lỗi trung bình khi góc nằm sát 0/360) */
static TLE5014_Status_t ReadAveraged(TLE5014_Channel_t ch, uint16_t *cnt)
{
    TLE5014_Angle_t a;
    TLE5014_Status_t st;
    uint16_t ref = 0U;
    int32_t  acc = 0;
    uint8_t  i;

    // Đo 
    for (i = 0U; i < 8U; i++)
    {
        st = TLE5014_ReadAngle(ch, &a);
        if (st != TLE5014_OK) return st;

        if (i == 0U) ref = a.angle_counts;      // lấy mẫu đầu tiên làm tham chiếu
        // tính tổng chênh lệch từ ref, dùng phép toán modulo 0x7FFF để xử lý quay vòng
        acc += (int32_t)(((a.angle_counts - ref + 0x4000) & 0x7FFF) - 0x4000);
        HAL_Delay(2);
    }
    *cnt = (uint16_t)((int32_t)ref + (acc / 8)) & TLE5014_ANGLE_MASK;

    return TLE5014_OK;   // trả giá trị cnt trung bình 8 mẫu, thang 0..32767 và trả về OK
}

static TLE5014_Status_t Calibrate(TLE5014_Handle_t *s, uint8_t is_max)
{
    uint16_t c1, c2;        // giá trị tay ga đọc được từ 2 kênh
    TLE5014_Status_t st;    // trạng thái đọc 2 kênh

    // Kiểm tra handle và trạng thái khởi tạo
    if ((s == NULL) || (s->initialized == 0U)) return TLE5014_NOT_INITIALIZED;

    // st = OK nếu đọc 2 kênh thành công, c1/c2 = giá trị tay ga 0..32767
    st = ReadAveraged(TLE5014_CHANNEL_1, &c1);
    if (st != TLE5014_OK) return st;
    st = ReadAveraged(TLE5014_CHANNEL_2, &c2);
    if (st != TLE5014_OK) return st;

    // Nếu max thì lưu vào max_cnt, nếu min thì lưu vào min_cnt. Cập nhật cờ cal_flags.
    if (is_max)
    {
        s->throttle.max_cnt[0] = c1;
        s->throttle.max_cnt[1] = c2;
        s->throttle.cal_flags |= 0x02U;     // cờ báo đã calibrate max
    }
    else
    {
        s->throttle.min_cnt[0] = c1;
        s->throttle.min_cnt[1] = c2;
        s->throttle.cal_flags |= 0x01U;
    }

    // Reset bộ lọc và bộ đếm lỗi
    s->throttle.filt_valid = 0U;
    s->throttle.fault_cnt  = 0U;
    return TLE5014_OK;
}

TLE5014_Status_t TLE5014_Throttle_CalibrateMin(TLE5014_Handle_t *sensor)
{
    return Calibrate(sensor, 0U);
}

TLE5014_Status_t TLE5014_Throttle_CalibrateMax(TLE5014_Handle_t *sensor)
{
    return Calibrate(sensor, 1U);
}


// nap giá trị min/max từ flash (không cần đọc sensor) để khởi tạo tay ga
void TLE5014_Throttle_SetCalibration(TLE5014_Handle_t *sensor,
                                     uint16_t min1, uint16_t max1,
                                     uint16_t min2, uint16_t max2)
{
    if (sensor == NULL) return;
    sensor->throttle.min_cnt[0] = min1 & TLE5014_ANGLE_MASK;
    sensor->throttle.max_cnt[0] = max1 & TLE5014_ANGLE_MASK;
    sensor->throttle.min_cnt[1] = min2 & TLE5014_ANGLE_MASK;
    sensor->throttle.max_cnt[1] = max2 & TLE5014_ANGLE_MASK;
    sensor->throttle.cal_flags  = 0x03U;    // đánh dấu đã calibrate min/max
    sensor->throttle.filt_valid = 0U;       // reset bộ lọc
}

// Lấy giá trị tay ga 0..100, trả 0 nếu chưa calibrate hoặc lỗi
uint8_t TLE5014_Throttle_Get(const TLE5014_Handle_t *sensor)
{
    return (sensor != NULL) ? sensor->throttle.value : 0U;
}

/* ================================================================== */
/*  Task                                                              */
/* ================================================================== */

void TLE5014_Task(TLE5014_Handle_t *sensor)
{
    uint32_t        now;
    TLE5014_Angle_t ch1 = {0};
    TLE5014_Angle_t ch2 = {0};

    if ((sensor == NULL) || (sensor->initialized == 0U))
        return;

    now = HAL_GetTick();
    if ((uint32_t)(now - sensor->last_sample_tick) < sensor->sample_interval_ms)
        return;

    sensor->last_sample_tick = now;

    sensor->channel1_status = TLE5014_ReadAngle(TLE5014_CHANNEL_1, &ch1);
    sensor->channel1        = ch1;

    sensor->channel2_status = TLE5014_ReadAngle(TLE5014_CHANNEL_2, &ch2);
    sensor->channel2        = ch2;

    Throttle_Update(sensor);
}
