#include "tle5014.h"
#include <string.h>

/* ============================================================================
 *  Cấu hình bắt cạnh: EXTI (PB12, cạnh xuống) + timer 16-bit chạy tự do
 *  - Dùng timer trên APB1 (TIM2/TIM3/TIM4).
 *  - 4 MHz => 0,25 us/tick, tràn sau 16,4 ms (đủ cho mọi khoảng cạnh-cạnh của SENT)
 * ========================================================================== */
#define TLE5014_TIM                 TIM4
#define TLE5014_TIM_CLK_ENABLE()    __HAL_RCC_TIM4_CLK_ENABLE()
#define TLE5014_TIM_TICK_HZ         4000000UL
#define TLE5014_EXTI_IRQn           EXTI15_10_IRQn   /* PB12 -> EXTI line 12 */
#define TLE5014_EXTI_PRIORITY       0U               /* ưu tiên cao nhất      */

/* ===== Thông số SENT của TLE5014S16 (single secure sensor, UT = 3 us) ===== */
#define SENT_UT_US                  3UL
#define SENT_SYNC_UT                56UL
#define SENT_TOLERANCE_PERCENT      10UL    /* cửa sổ nhận sync: +-10% (clock cảm biến +-5%) */
#define SENT_MIN_NIBBLE_UT          12UL
#define SENT_MAX_NIBBLE_UT          27UL
#define SENT_NIBBLE_COUNT           8U      /* status, d1..d3, rc(2), inverted d1, crc */
#define SENT_CRC_POLYNOMIAL         0x0DU   /* x^4 + x^3 + x^2 + 1 */
#define SENT_CRC_SEED               0x05U

#define FRAME_RING_SIZE             8U      /* lũy thừa của 2: Task() được phép trễ ~8 frame (~6,8 ms) */

enum { RX_WAIT_SYNC = 0U, RX_COLLECT = 1U };

typedef struct
{
    uint8_t  nib[SENT_NIBBLE_COUNT];
    uint16_t ut_q4;     /* tick/UT x 16 */
    uint32_t tick;      /* HAL_GetTick() lúc frame hoàn tất */
    uint32_t seq;       /* số thứ tự frame (ISR đánh) */
} RawFrame_t;

TLE5014_SENT_Debug_t tle5014_sent_debug;

/* ---------------------------------------------------------------------------
 *  Biến dùng trong ngắt (ISR ghi, main đọc)
 * ------------------------------------------------------------------------- */
static uint32_t sync_min_ticks;
static uint32_t sync_max_ticks;

static volatile uint8_t  rx_flush;          /* main yêu cầu ISR bỏ trạng thái cũ */
static uint8_t           rx_state;
static uint8_t           rx_idx;
static uint8_t           rx_have_last;
static uint16_t          rx_last_cnt;
static uint32_t          rx_ut_q4;
static uint8_t           rx_nibbles[SENT_NIBBLE_COUNT];

static RawFrame_t        frame_ring[FRAME_RING_SIZE];
static volatile uint32_t frame_head;        /* ISR tăng sau khi ghi xong 1 frame */
static uint32_t          frame_tail;        /* chỉ main dùng */

static volatile uint32_t isr_edges;
static volatile uint32_t isr_sync_found;
static volatile uint32_t isr_nibble_errors;
static volatile uint32_t isr_resyncs;
static volatile uint32_t isr_last_interval;

/* ---------------------------------------------------------------------------
 *  Biến chỉ main dùng
 * ------------------------------------------------------------------------- */
static uint8_t           hw_ready;
static uint32_t          last_frame_tick;
static uint8_t           signal_lost;

static uint8_t           rc_valid;
static uint8_t           rc_last;
static uint32_t          rc_last_seq;

static TLE5014_Angle_t   g_angle;           /* góc hợp lệ gần nhất */
static uint8_t           g_valid;
static TLE5014_Status_t  g_status;
static uint32_t          g_last_valid_tick;

static uint32_t          dbg_overruns;
static uint32_t          dbg_signal_losses;
static uint32_t          dbg_frame_errors;
static uint32_t          dbg_crc_errors;
static uint32_t          dbg_rolling_errors;
static uint32_t          dbg_sensor_errors;
static uint32_t          dbg_frames_ok;

/* ============================================================================
 *  PHẦN NGẮT
 * ========================================================================== */
static uint8_t IsSyncLike(uint32_t interval)
{
    return (uint8_t)((interval >= sync_min_ticks) && (interval <= sync_max_ticks));
}

static void StartFrame(uint32_t sync_interval)
{
    /* tick/UT x16, làm tròn: UT đo được từ chính xung sync nên bù được sai số clock */
    rx_ut_q4 = (sync_interval * 16UL + (SENT_SYNC_UT / 2UL)) / SENT_SYNC_UT;
    rx_idx   = 0U;
    rx_state = RX_COLLECT;
    isr_sync_found++;
}

static void PublishFrame(void)
{
    uint32_t head = frame_head;
    RawFrame_t *slot = &frame_ring[head & (FRAME_RING_SIZE - 1U)];
    uint8_t i;

    for (i = 0U; i < SENT_NIBBLE_COUNT; i++)
    {
        slot->nib[i] = rx_nibbles[i];
    }
    slot->ut_q4 = (uint16_t)rx_ut_q4;
    slot->tick  = HAL_GetTick();
    slot->seq   = head;

    __DMB();            /* ghi xong dữ liệu rồi mới báo cho main */
    frame_head = head + 1U;
}

/* Gọi từ EXTI15_10_IRQHandler khi có cạnh xuống trên PB12 */
void TLE5014_EdgeIRQ(void)
{
    uint16_t now = (uint16_t)TLE5014_TIM->CNT;   /* lấy mốc thời gian NGAY ĐẦU ngắt */
    uint32_t interval;
    uint32_t units;

    isr_edges++;

    // Nếu main yêu cầu bỏ trạng thái cũ (mất tín hiệu) thì ISR cũng bỏ mốc thời gian cũ
    if (rx_flush != 0U)
    {
        rx_flush     = 0U;
        rx_have_last = 0U;
        rx_state     = RX_WAIT_SYNC;
    }

    // Nếu chưa có mốc thời gian trước đó thì chỉ ghi mốc hiện tại, không tính khoảng cách
    if (rx_have_last == 0U)
    {
        rx_have_last = 1U;
        rx_last_cnt  = now;
        return;
    }

    // Tính khoảng cách giữa 2 cạnh xuống liên tiếp (tính bằng tick của timer 16-bit)
    interval     = (uint16_t)(now - rx_last_cnt);   /* số học 16 bit: không lo timer tràn */
    rx_last_cnt  = now;
    isr_last_interval = interval;

    // Nếu đang chờ sync thì kiểm tra khoảng cách có giống sync không, 
    // nếu có thì bắt đầu frame
    if (rx_state == RX_WAIT_SYNC)
    {
        if (IsSyncLike(interval) != 0U)
        {
            StartFrame(interval);
        }
        return;
    }

    /* RX_COLLECT: khoảng giữa 2 cạnh xuống = (12 + giá trị) UT */
    units = (interval * 16UL + (rx_ut_q4 / 2UL)) / rx_ut_q4;

    // Nếu giá trị hợp lệ thì lưu nibble, nếu đủ 8 nibble thì publish frame
    if ((units >= SENT_MIN_NIBBLE_UT) && (units <= SENT_MAX_NIBBLE_UT))
    {
        rx_nibbles[rx_idx] = (uint8_t)(units - SENT_MIN_NIBBLE_UT);
        rx_idx++;
        if (rx_idx >= SENT_NIBBLE_COUNT)
        {
            PublishFrame();
            rx_state = RX_WAIT_SYNC;
        }
    }
    else if (IsSyncLike(interval) != 0U)
    {
        /* Xung trước đó thực ra là pause pulse (cũng dài ~56 UT) và đây mới là
         * sync thật: bắt đầu lại frame từ xung này thay vì bỏ cả frame. */
        isr_resyncs++;
        StartFrame(interval);
    }
    else
    {
        isr_nibble_errors++;
        rx_state = RX_WAIT_SYNC;
    }
}

/* ============================================================================
 *  PHẦN XỬ LÝ TRONG MAIN
 * ========================================================================== */
static uint8_t CalculateCrc4(const uint8_t *nibbles, uint8_t count)
{
    uint8_t crc = SENT_CRC_SEED;
    uint8_t i;

    for (i = 0U; i < count; i++)
    {
        uint8_t bit;

        for (bit = 0U; bit < 4U; bit++)
        {
            uint8_t input_bit = (uint8_t)((nibbles[i] >> (3U - bit)) & 1U);
            uint8_t feedback  = (uint8_t)(((crc >> 3U) ^ input_bit) & 1U);

            crc = (uint8_t)((crc << 1U) & 0x0FU);
            if (feedback != 0U)
            {
                crc ^= SENT_CRC_POLYNOMIAL;
            }
        }
    }

    return crc;
}

/* Lấy 1 frame ra khỏi vòng đệm. Trả về 1 nếu có. */
static uint8_t PopFrame(RawFrame_t *out)
{
    uint32_t head = frame_head;
    uint32_t avail = head - frame_tail;

    if (avail == 0U)
    {
        return 0U;
    }

    __DMB();

    if (avail > FRAME_RING_SIZE)
    {   /* main chậm hơn ISR hơn cả vòng đệm: bỏ frame cũ nhất */
        dbg_overruns += avail - FRAME_RING_SIZE;
        frame_tail = head - FRAME_RING_SIZE;
    }

    *out = frame_ring[frame_tail & (FRAME_RING_SIZE - 1U)];

    __DMB();

    if ((frame_head - frame_tail) > FRAME_RING_SIZE)
    {   /* ISR vừa ghi đè đúng ô đang chép: bỏ bản chép, lần sau thử lại */
        dbg_overruns++;
        frame_tail = frame_head - FRAME_RING_SIZE;
        return 0U;
    }

    frame_tail++;
    return 1U;
}

/* Giải mã 8 nibble: kiểm tra inverted nibble, CRC, rồi tính góc. */
static TLE5014_Status_t DecodeFrame(const uint8_t n[SENT_NIBBLE_COUNT],
                                    TLE5014_Angle_t *a, uint8_t *sensor_flag)
{
    uint16_t raw;

    memset(a, 0, sizeof(*a));
    *sensor_flag = 0U;

    /* n[6] phải là đảo bit của data nibble 1 (n[1]) */
    if (n[6] != (uint8_t)(n[1] ^ 0x0FU))
    {
        return TLE5014_FRAME_ERROR;
    }

    /* CRC tính trên data nibble 1..3, rolling counter 2 nibble, inverted nibble (không gồm status) */
    if (CalculateCrc4(&n[1], 6U) != n[7])
    {
        return TLE5014_CRC_ERROR;
    }

    raw = (uint16_t)(((uint16_t)n[1] << 8U) | ((uint16_t)n[2] << 4U) | n[3]);
    a->raw_angle       = raw;
    a->safety_word     = n[0];
    a->rolling_counter = (uint8_t)((n[4] << 4U) | n[5]);

    if (((n[0] & 0x01U) != 0U) ||
        (raw == TLE5014_SENT_ERROR_CODE) ||
        (raw < TLE5014_SENT_DATA_MIN) ||
        (raw > TLE5014_SENT_DATA_MAX))
    {
        *sensor_flag = 1U;
    }
    else
    {
        uint32_t pos = (uint32_t)raw - 1U;                      /* 0..4087 */
        float deg = (float)pos * (360.0f / 4087.0f);
        if (deg >= 360.0f)
        {
            deg = 0.0f;                                         /* 360° quay về 0° */
        }
        a->angle_degrees = deg;
        a->angle_counts  = (uint16_t)(1U + ((pos * 99UL) / 4087UL));
    }

    return TLE5014_OK;
}

/* Rolling counter phải tăng đúng 1 giữa 2 frame liên tiếp. Nếu giữa chúng có frame bị
 * mất (seq cách nhau > 1) thì chỉ kiểm tra là counter không đứng yên.
 * Frame bị ISR loại (lỗi nibble) không có seq, nên cũng bị phát hiện ở đây. */
static uint8_t CheckRolling(uint8_t rc, uint32_t seq)
{
    uint8_t ok = 1U;

    if (rc_valid != 0U)
    {
        uint8_t delta = (uint8_t)(rc - rc_last);

        if ((seq - rc_last_seq) == 1U)
        {
            ok = (uint8_t)(delta == 1U);
        }
        else
        {
            ok = (uint8_t)(delta != 0U);
        }
    }

    rc_last     = rc;
    rc_last_seq = seq;
    rc_valid    = 1U;
    return ok;
}

static void ApplyResult(TLE5014_Handle_t *s, TLE5014_Status_t st,
                        const TLE5014_Angle_t *a, uint32_t tick)
{
    switch (st)
    {
        case TLE5014_OK:             dbg_frames_ok++;      break;
        case TLE5014_FRAME_ERROR:    dbg_frame_errors++;   break;
        case TLE5014_CRC_ERROR:      dbg_crc_errors++;     break;
        case TLE5014_ROLLING_ERROR:  dbg_rolling_errors++; break;
        case TLE5014_SENSOR_ERROR:   dbg_sensor_errors++;  break;
        default: break;
    }

    // Nếu frame hợp lệ thì cập nhật góc hợp lệ gần nhất, 
    // còn không hợp lệ thì chỉ cập nhật trạng thái
    if (st == TLE5014_OK)
    {
        g_angle           = *a;       /* chỉ cập nhật góc khi frame hoàn toàn hợp lệ */
        g_valid           = 1U;
        g_last_valid_tick = tick;
        s->consecutive_faults = 0U;
    }
    else
    {
        if (s->consecutive_faults < 255U)
        {
            s->consecutive_faults++;
        }

        /* Cảm biến tự báo lỗi / mất tín hiệu: mất hiệu lực ngay.
         * Lỗi truyền (khung/CRC/rolling): cho phép tối đa FAULT_LIMIT - 1 frame liên tiếp. */
        if ((st == TLE5014_SENSOR_ERROR) || (st == TLE5014_NO_SIGNAL) ||
            (st == TLE5014_STUCK_LOW) ||
            (s->consecutive_faults >= TLE5014_FAULT_LIMIT))
        {
            g_valid = 0U;
        }
    }

    g_status = st;

    s->channel1_status          = st;
    s->channel1                 = g_angle;
    s->channel1_valid           = g_valid;
    s->channel1_last_valid_tick = g_last_valid_tick;
}

static void UpdateDebug(void)
{
    tle5014_sent_debug.edges               = isr_edges;
    tle5014_sent_debug.sync_found          = isr_sync_found;
    tle5014_sent_debug.frames_captured     = frame_head;
    tle5014_sent_debug.nibble_errors       = isr_nibble_errors;
    tle5014_sent_debug.resyncs             = isr_resyncs;
    tle5014_sent_debug.overruns            = dbg_overruns;
    tle5014_sent_debug.signal_losses       = dbg_signal_losses;
    tle5014_sent_debug.frame_errors        = dbg_frame_errors;
    tle5014_sent_debug.crc_errors          = dbg_crc_errors;
    tle5014_sent_debug.rolling_errors      = dbg_rolling_errors;
    tle5014_sent_debug.sensor_errors       = dbg_sensor_errors;
    tle5014_sent_debug.frames_ok           = dbg_frames_ok;
    tle5014_sent_debug.last_interval_ticks = isr_last_interval;
    tle5014_sent_debug.pin_level =
        (HAL_GPIO_ReadPin(IFB_GPIO_Port, IFB_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/* ============================================================================
 *  API
 * ========================================================================== */
TLE5014_Status_t TLE5014_Init(TLE5014_Handle_t *sensor, UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t tim_clk;
    uint32_t psc;
    uint32_t tick_hz;
    uint32_t nominal_sync_ticks;

    if (sensor == NULL)
    {
        return TLE5014_ERROR;
    }

    hw_ready = 0U;
    HAL_NVIC_DisableIRQ(TLE5014_EXTI_IRQn);

    memset(sensor, 0, sizeof(*sensor));
    sensor->uart            = uart;
    sensor->channel1_status = TLE5014_NOT_INITIALIZED;
    sensor->channel2_status = TLE5014_NOT_INITIALIZED;
    sensor->throttle.state  = TLE5014_THR_NOT_CALIBRATED;

    /* ---- 1) Timer chạy tự do 4 MHz ---- */
    tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
        tim_clk *= 2U;      /* APB1 chia > 1 thì clock timer = 2 x PCLK1 */
    }

    psc = tim_clk / TLE5014_TIM_TICK_HZ;
    if ((psc == 0U) || (psc > 65536U))
    {
        return TLE5014_ERROR;
    }
    tick_hz = tim_clk / psc;

    TLE5014_TIM_CLK_ENABLE();
    TLE5014_TIM->CR1 = 0U;
    TLE5014_TIM->PSC = (uint16_t)(psc - 1U);
    TLE5014_TIM->ARR = 0xFFFFU;
    TLE5014_TIM->EGR = TIM_EGR_UG;
    TLE5014_TIM->CNT = 0U;
    TLE5014_TIM->CR1 = TIM_CR1_CEN;

    /* ---- 2) Cửa sổ nhận xung sync (56 UT +-10%) tính theo tick ---- */
    nominal_sync_ticks = (uint32_t)(((uint64_t)tick_hz * SENT_SYNC_UT * SENT_UT_US) / 1000000ULL);
    sync_min_ticks = (nominal_sync_ticks * (100UL - SENT_TOLERANCE_PERCENT)) / 100UL;
    sync_max_ticks = (nominal_sync_ticks * (100UL + SENT_TOLERANCE_PERCENT)) / 100UL;

    /* ---- 3) Xóa trạng thái bắt cạnh (ngắt đang tắt) ---- */
    rx_flush     = 0U;
    rx_state     = RX_WAIT_SYNC;
    rx_idx       = 0U;
    rx_have_last = 0U;
    frame_head   = 0U;
    frame_tail   = 0U;
    isr_edges = 0U; isr_sync_found = 0U; isr_nibble_errors = 0U;
    isr_resyncs = 0U; isr_last_interval = 0U;

    rc_valid = 0U;
    signal_lost = 0U;
    g_valid = 0U;
    g_status = TLE5014_NO_SIGNAL;       /* chưa có dữ liệu nào */
    g_last_valid_tick = 0U;
    memset(&g_angle, 0, sizeof(g_angle));
    dbg_overruns = 0U; dbg_signal_losses = 0U; dbg_frame_errors = 0U;
    dbg_crc_errors = 0U; dbg_rolling_errors = 0U; dbg_sensor_errors = 0U;
    dbg_frames_ok = 0U;
    memset(&tle5014_sent_debug, 0, sizeof(tle5014_sent_debug));

    /* ---- 4) PB12: EXTI cạnh xuống. Không dùng pull nội vì đã có pull-up ngoài 5 V ---- */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = IFB_Pin;
    gpio.Mode  = GPIO_MODE_IT_FALLING;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IFB_GPIO_Port, &gpio);

    __HAL_GPIO_EXTI_CLEAR_IT(IFB_Pin);
    HAL_NVIC_SetPriority(TLE5014_EXTI_IRQn, TLE5014_EXTI_PRIORITY, 0U);
    HAL_NVIC_EnableIRQ(TLE5014_EXTI_IRQn);

    last_frame_tick = HAL_GetTick();
    sensor->initialized = 1U;
    hw_ready = 1U;
    return TLE5014_OK;
}

void TLE5014_Task(TLE5014_Handle_t *sensor)
{
    RawFrame_t f;
    TLE5014_Angle_t a;  // góc hợp lệ gần nhất (không bị ghi đè khi lỗi)
    TLE5014_Status_t st;
    uint8_t sensor_flag;
    uint8_t processed = 0U;

    if ((sensor == NULL) || (sensor->initialized == 0U))
    {
        return;
    }

    // Nếu có frame nào trong vòng đệm thì xử lý hết, còn không thì kiểm tra mất tín hiệu
    while ((processed < FRAME_RING_SIZE) && (PopFrame(&f) != 0U))
    {
        uint8_t i;

        processed++;
        last_frame_tick = f.tick;
        signal_lost = 0U;

        // Cập nhật debug: frame gần nhất, tick/UT, 8 nibble
        for (i = 0U; i < SENT_NIBBLE_COUNT; i++)
        {
            tle5014_sent_debug.frame[i] = f.nib[i];
        }
        tle5014_sent_debug.ut_q4 = f.ut_q4;

        // Giải mã frame, kiểm tra rolling counter, cập nhật góc hợp lệ gần nhất
        st = DecodeFrame(f.nib, &a, &sensor_flag);

        // Nếu frame hợp lệ thì kiểm tra rolling counter, nếu không hợp lệ thì bỏ qua
        if (st == TLE5014_OK)
        {
            uint8_t rc_ok = CheckRolling(a.rolling_counter, f.seq);

            if (sensor_flag != 0U)
            {
                st = TLE5014_SENSOR_ERROR;
            }
            else if (rc_ok == 0U)
            {
                st = TLE5014_ROLLING_ERROR;
            }
            else
            {
                /* frame hợp lệ */
            }
        }

        // Cập nhật trạng thái, góc hợp lệ gần nhất, debug
        ApplyResult(sensor, st, &a, f.tick);
    }

    // Nếu không có frame nào thì kiểm tra mất tín hiệu: 
    // nếu quá TLE5014_SIGNAL_TIMEOUT_MS kể từ frame cuối cùng
    if ((processed == 0U) &&
        ((uint32_t)(HAL_GetTick() - last_frame_tick) >= TLE5014_SIGNAL_TIMEOUT_MS))
    {
        // Nếu mất tín hiệu lần đầu thì tăng bộ đếm debug signal_losses
        if (signal_lost == 0U)
        {
            signal_lost = 1U;
            dbg_signal_losses++;
        }

        rx_flush = 1U;      /* khi có tín hiệu lại, ISR bỏ mốc thời gian cũ (timer đã quay vòng) */
        rc_valid = 0U;      /* không so rolling counter với frame trước khi mất tín hiệu */

        // Cập nhật trạng thái mất tín hiệu, không cập nhật góc hợp lệ gần nhất
        st = (HAL_GPIO_ReadPin(IFB_GPIO_Port, IFB_Pin) == GPIO_PIN_RESET) ?
             TLE5014_STUCK_LOW : TLE5014_NO_SIGNAL;

        // Cập nhật trạng thái, góc hợp lệ gần nhất, debug
        ApplyResult(sensor, st, &g_angle, last_frame_tick);
    }

    UpdateDebug();
}

TLE5014_Status_t TLE5014_ReadAngle(TLE5014_Channel_t channel, TLE5014_Angle_t *angle)
{
    if ((channel != TLE5014_CHANNEL_1) || (angle == NULL))
    {
        return TLE5014_ERROR;
    }

    if (hw_ready == 0U)
    {
        return TLE5014_NOT_INITIALIZED;
    }

    *angle = g_angle;       /* góc hợp lệ gần nhất; chỉ dùng khi hàm trả về TLE5014_OK */

    if (g_valid == 0U)
    {
        return (g_status != TLE5014_OK) ? g_status : TLE5014_NO_SIGNAL;
    }

    if ((uint32_t)(HAL_GetTick() - g_last_valid_tick) > TLE5014_VALID_AGE_MS)
    {
        return TLE5014_NO_SIGNAL;
    }

    return TLE5014_OK;
}