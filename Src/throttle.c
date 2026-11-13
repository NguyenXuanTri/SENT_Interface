#include "throttle.h"
#include <math.h>
#include <string.h>

#define THR_PI   3.14159265358979f

/* Đưa hiệu hai góc về khoảng (-180, 180]: cho phép điểm zero nằm bất cứ đâu,
 * kể cả khi hành trình đi qua mốc 0°/360°. */
static float Wrap180(float x)
{
    x = fmodf(x, 360.0f);   // hiệu chỉnh về khoảng (-360, 360)
    if (x > 180.0f)
    {
        x -= 360.0f;
    }
    else if (x <= -180.0f)
    {
        x += 360.0f;
    }
    return x;
}

/* Đưa góc về khoảng [0, 360) */
static float Wrap360(float x)
{
    x = fmodf(x, 360.0f);
    if (x < 0.0f)
    {
        x += 360.0f;
    }
    if (x >= 360.0f)
    {
        x = 0.0f;
    }
    return x;
}

static float Clamp(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

static uint8_t IsFinite(float x)
{
    return (uint8_t)(isfinite(x) != 0);
}

static uint8_t ConfigValid(const Throttle_Config_t *c)
{
    if ((IsFinite(c->min_span_deg) == 0U) || (IsFinite(c->max_span_deg) == 0U) ||
        (IsFinite(c->range_margin_deg) == 0U) || (IsFinite(c->dead_low_percent) == 0U) ||
        (IsFinite(c->dead_high_percent) == 0U))
    {
        return 0U;
    }
    if ((c->min_span_deg <= 0.0f) || (c->max_span_deg >= 180.0f) ||
        (c->min_span_deg >= c->max_span_deg))
    {
        return 0U;
    }
    if ((c->range_margin_deg < 0.0f) || (c->dead_low_percent < 0.0f) ||
        (c->dead_high_percent < 0.0f) ||
        ((c->dead_low_percent + c->dead_high_percent) >= 50.0f))   /* vùng chết tổng không quá 50% */
    {
        return 0U;
    }
    return 1U;
}

static void ClearOutput(Throttle_t *thr)
{
    thr->valid             = 0U;
    thr->travel_deg        = 0.0f;
    thr->raw_percent       = 0.0f;
    thr->position_percent  = 0.0f;
    thr->position_permille = 0U;
}

void Throttle_ConfigDefault(Throttle_Config_t *cfg)
{
    cfg->min_span_deg     = 10.0f;
    cfg->max_span_deg     = 120.0f;
    cfg->range_margin_deg = 5.0f;
    cfg->dead_low_percent = 2.0f;
    cfg->dead_high_percent = 2.0f;
}

uint8_t Throttle_Init(Throttle_t *thr, const Throttle_Config_t *cfg)
{
    uint8_t err = 0U;

    memset(thr, 0, sizeof(*thr));
    Throttle_ConfigDefault(&thr->cfg);

    if (cfg != NULL)
    {
        if (ConfigValid(cfg) != 0U)
        {
            thr->cfg = *cfg;
        }
        else
        {
            err = 1U;       /* cấu hình sai: giữ mặc định */
        }
    }

    thr->direction = 1;
    thr->state     = THR_NOT_CALIBRATED;
    ClearOutput(thr);
    return err;
}

/* Đổi điểm hiệu chỉnh thì kết quả hiệu chỉnh cũ mất hiệu lực cho đến khi Commit */
static void InvalidateCalibration(Throttle_t *thr)
{
    thr->calibrated = 0U;
    thr->state      = THR_NOT_CALIBRATED;
    ClearOutput(thr);
}

// Lưu điểm đóng 
Throttle_CalResult_t Throttle_SetClosed(Throttle_t *thr, float angle_deg)
{
    // Nếu góc lỗi (NaN/vô hạn) thì trả về THR_CAL_BAD_INPUT, không lưu điểm đóng
    if (IsFinite(angle_deg) == 0U)
    {
        return THR_CAL_BAD_INPUT;
    }
    
    thr->closed_deg = Wrap360(angle_deg);   // lưu điểm đóng trong khoảng [0, 360)
    thr->closed_set = 1U;                   // đánh dấu đã lưu điểm đóng
    InvalidateCalibration(thr);
    return THR_CAL_OK;
}

// Lưu điểm mở
Throttle_CalResult_t Throttle_SetOpen(Throttle_t *thr, float angle_deg)
{
    if (IsFinite(angle_deg) == 0U)
    {
        return THR_CAL_BAD_INPUT;
    }
    thr->open_deg = Wrap360(angle_deg);
    thr->open_set = 1U;
    InvalidateCalibration(thr);
    return THR_CAL_OK;
}

// Kiểm tra và kích hoạt
Throttle_CalResult_t Throttle_CommitCalibration(Throttle_t *thr)
{
    float diff;
    float span;

    thr->calibrated = 0U;
    thr->state      = THR_NOT_CALIBRATED;
    ClearOutput(thr);

    if ((thr->closed_set == 0U) || (thr->open_set == 0U))
    {
        return THR_CAL_NOT_READY;
    }

    diff = Wrap180(thr->open_deg - thr->closed_deg);
    span = fabsf(diff);

    if (span < thr->cfg.min_span_deg)
    {
        return THR_CAL_SPAN_TOO_SMALL;
    }
    if (span > thr->cfg.max_span_deg)
    {
        return THR_CAL_SPAN_TOO_LARGE;
    }

    thr->direction  = (diff > 0.0f) ? 1 : -1;   /* chiều quay tự nhận từ 2 điểm */
    thr->span_deg   = span;
    thr->calibrated = 1U;
    thr->state      = THR_READY;
    return THR_CAL_OK;
}

// Nạp lại điểm đã lưu
Throttle_CalResult_t Throttle_LoadCalibration(Throttle_t *thr, float closed_deg, float open_deg)
{
    Throttle_CalResult_t r;

    r = Throttle_SetClosed(thr, closed_deg);
    if (r != THR_CAL_OK)
    {
        return r;
    }
    r = Throttle_SetOpen(thr, open_deg);
    if (r != THR_CAL_OK)
    {
        return r;
    }
    return Throttle_CommitCalibration(thr);
}

void Throttle_Update(Throttle_t *thr, float angle_deg, uint8_t angle_valid)
{
    float travel;
    float raw;
    float lo;
    float hi;
    float pos;

    // Nếu chưa hiệu chỉnh thì không cho phép dùng, bỏ qua đầu vào
    if (thr->calibrated == 0U)
    {
        thr->state = THR_NOT_CALIBRATED;
        ClearOutput(thr);
        return;
    }

    // Nếu góc lỗi (NaN/vô hạn) hoặc không hợp lệ thì bỏ qua đầu vào
    if ((angle_valid == 0U) || (IsFinite(angle_deg) == 0U))
    {
        thr->state = THR_INPUT_INVALID;
        ClearOutput(thr);
        return;
    }

    /* Góc đã quay từ điểm đóng theo chiều mở (độ). Âm: thấp hơn điểm đóng. */
    travel = (float)thr->direction * Wrap180(angle_deg - thr->closed_deg);
    thr->travel_deg = travel;

    // Nếu ngoài hành trình cơ khí (lệch nam châm, gãy cơ khí...) thì không cho phép dùng
    if ((travel < -thr->cfg.range_margin_deg) ||
        (travel > (thr->span_deg + thr->cfg.range_margin_deg)))
    {
        /* Ngoài hành trình cơ khí: cảm biến đọc được nhưng vị trí không hợp lý
         * (lệch nam châm, gãy cơ khí...). Không cho phép dùng. */
        thr->state             = THR_OUT_OF_RANGE;
        thr->valid             = 0U;
        thr->raw_percent       = 0.0f;
        thr->position_percent  = 0.0f;
        thr->position_permille = 0U;
        return;
    }

    // Tính phần trăm hành trình (0..100) chưa tính vùng chết
    raw = Clamp((travel / thr->span_deg) * 100.0f, 0.0f, 100.0f);

    // Tính phần trăm hành trình (0..100) đã tính vùng chết
    lo  = thr->cfg.dead_low_percent;
    hi  = thr->cfg.dead_high_percent;
    pos = Clamp(((raw - lo) / (100.0f - lo - hi)) * 100.0f, 0.0f, 100.0f);

    thr->raw_percent       = raw;
    thr->position_percent  = pos;
    thr->position_permille = (uint16_t)((pos * 10.0f) + 0.5f);  //
    thr->state             = THR_READY;
    thr->valid             = 1U;
}

void Throttle_AvgReset(Throttle_Avg_t *avg)
{
    avg->sum_sin = 0.0f;    // tổng sin của các góc đã thêm
    avg->sum_cos = 0.0f;    // tổng cos của các góc đã thêm
    avg->count   = 0U;      // số lượng góc đã thêm (tối đa 65535)
}

// Thêm 1 góc vào trung bình vòng. Nếu góc lỗi (NaN/vô hạn) thì bỏ qua.
void Throttle_AvgAdd(Throttle_Avg_t *avg, float angle_deg)
{
    float rad;

    // Nếu góc lỗi thì bỏ qua
    if ((IsFinite(angle_deg) == 0U) || (avg->count == 0xFFFFU))
    {
        return;
    }

    rad = angle_deg * (THR_PI / 180.0f);    // đổi sang radian
    avg->sum_sin += sinf(rad);              // tổng sin của các góc đã thêm
    avg->sum_cos += cosf(rad);              // tổng cos của các góc đã thêm
    avg->count++;                           // số lượng góc đã thêm (tối đa 65535)
}

// Lấy trung bình vòng của các góc đã thêm. 
// Nếu chưa có góc nào thì trả về 0, không cập nhật *angle_deg.
uint8_t Throttle_AvgGet(const Throttle_Avg_t *avg, float *angle_deg)
{
    float deg;

    // Nếu chưa có góc nào được thêm thì trả về 0, không cập nhật *angle_deg
    if (avg->count == 0U)
    {
        return 0U;
    }

    // Tính trung bình vòng: atan2(tổng sin, tổng cos) -> góc trung bình (độ)
    deg = atan2f(avg->sum_sin, avg->sum_cos) * (180.0f / THR_PI);

    // Đưa về khoảng [0, 360) (có thể âm do atan2 trả về [-180, 180])
    if (deg < 0.0f)
    {
        deg += 360.0f;
    }
    if (deg >= 360.0f)
    {
        deg = 0.0f;
    }
    *angle_deg = deg;
    return 1U;
}

/* ---------------------------------------------------------------------------
 *  Lấy mẫu hiệu chỉnh không chặn
 * ------------------------------------------------------------------------- */
void Throttle_CalBegin(Throttle_Cal_t *cal, Throttle_Point_t point, uint16_t samples,
                       uint32_t interval_ms, uint32_t timeout_ms, uint32_t now_ms)
{
    memset(cal, 0, sizeof(*cal));
    Throttle_AvgReset(&cal->avg);
    cal->point          = point;
    cal->samples_needed = (samples == 0U) ? 1U : samples;
    cal->interval_ms    = interval_ms;
    cal->timeout_ms     = timeout_ms;
    cal->start_ms       = now_ms;
    cal->last_ms        = now_ms - interval_ms;     /* mẫu đầu lấy ngay */
    cal->state          = THR_CALSTEP_SAMPLING;
    cal->result         = THR_CAL_NOT_READY;
 
    /* Tham số bất khả thi: timeout không đủ để lấy hết số mẫu (mẫu đầu lấy ngay, mẫu cuối
     * ở (samples-1)*interval). Báo lỗi ngay thay vì để TIMEOUT sau vài trăm ms. */
    if (((uint64_t)(cal->samples_needed - 1U) * (uint64_t)interval_ms) >= (uint64_t)timeout_ms)
    {
        cal->state  = THR_CALSTEP_FAILED;
        cal->result = THR_CAL_BAD_INPUT;
    }
}
 
static Throttle_CalState_t CalFinish(Throttle_Cal_t *cal, Throttle_CalResult_t r)
{
    cal->result = r;
    cal->state  = (r == THR_CAL_OK) ? THR_CALSTEP_DONE : THR_CALSTEP_FAILED;
    return cal->state;
}
 
Throttle_CalState_t Throttle_CalStep(Throttle_Cal_t *cal, Throttle_t *thr, uint32_t now_ms,
                                     float angle_deg, uint8_t angle_valid)
{
    float mean;
    float resultant;

    if (cal->state != THR_CALSTEP_SAMPLING)
    {
        return cal->state;
    }
 
    if ((uint32_t)(now_ms - cal->start_ms) >= cal->timeout_ms)
    {
        return CalFinish(cal, THR_CAL_TIMEOUT);
    }
 
    if ((uint32_t)(now_ms - cal->last_ms) < cal->interval_ms)
    {
        return THR_CALSTEP_SAMPLING;
    }
    cal->last_ms = now_ms;
 
    if ((angle_valid != 0U) && (IsFinite(angle_deg) != 0U))
    {
        Throttle_AvgAdd(&cal->avg, angle_deg);      /* chỉ nhận mẫu hợp lệ */
    }
 
    if (cal->avg.count < cal->samples_needed)
    {
        return THR_CALSTEP_SAMPLING;
    }
 
    /* Đủ mẫu: kiểm tra tay ga có đứng yên không. Độ dài vector trung bình R gần 1 khi
     * các mẫu cùng hướng; R >= cos(độ lệch cho phép) tương đương độ lệch chuẩn <= ngưỡng. */
    resultant = sqrtf((cal->avg.sum_sin * cal->avg.sum_sin) +
                      (cal->avg.sum_cos * cal->avg.sum_cos)) / (float)cal->avg.count;
    if (resultant < cosf(THR_CAL_MAX_SPREAD_DEG * (THR_PI / 180.0f)))
    {
        return CalFinish(cal, THR_CAL_UNSTABLE);
    }
 
    if (Throttle_AvgGet(&cal->avg, &mean) == 0U)
    {
        return CalFinish(cal, THR_CAL_NOT_READY);
    }
 
    return CalFinish(cal, (cal->point == THR_POINT_CLOSED) ? Throttle_SetClosed(thr, mean)
                                                            : Throttle_SetOpen(thr, mean));
}
 