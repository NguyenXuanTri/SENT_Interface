
#include "throttle_filter.h"

#include <math.h>
#include <string.h>

#define TF_PI 3.14159265358979f

static float Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float Median3(float a, float b, float c)
{
    float lo = (a < b) ? a : b;
    float hi = (a < b) ? b : a;
    float m  = (c < hi) ? c : hi;

    return (lo > m) ? lo : m;
}

static void ClearOutput(ThrFilter_t *f)
{
    f->valid = 0U;
    f->output = 0.0f;
    f->output_permille = 0U;
}

static void ClearState(ThrFilter_t *f)
{
    f->seeded = 0U;
    f->y = 0.0f;

    f->w[0] = 0.0f;
    f->w[1] = 0.0f;
    f->w[2] = 0.0f;

    f->last_ms = 0U;

    ClearOutput(f);
}

static void Publish(ThrFilter_t *f)
{
    f->output = Clamp(f->y, 0.0f, 100.0f);
    f->output_permille =
        (uint16_t)((f->output * 10.0f) + 0.5f);

    f->valid = 1U;
}

/*
 * Chỉ dùng để khởi tạo bộ lọc khi chưa có trạng thái.
 * Đây không phải cơ chế phục hồi an toàn.
 *
 * Sau khi filter bị timeout, tầng Safety phải giữ safe state
 * và chỉ cho phép hoạt động lại khi đã re-arm.
 */
static void Seed(ThrFilter_t *f, float x, uint32_t now_ms)
{
    f->y = x;

    f->w[0] = x;
    f->w[1] = x;
    f->w[2] = x;

    f->last_ms = now_ms;
    f->seeded = 1U;

    Publish(f);
}

static uint8_t ConfigValid(const ThrFilter_Config_t *c)
{
    float sample_rate_hz;

    if (c == NULL)
        return 0U;

    if (!isfinite(c->cutoff_hz) ||
        c->cutoff_hz <= 0.0f)
    {
        return 0U;
    }

    if (c->period_ms < 1U)
        return 0U;

    /*
     * Yêu cầu max_gap >= 2 * period.
     * Dùng uint64_t để tránh tràn phép nhân.
     */
    if ((uint64_t)c->max_gap_ms <
        (2ULL * (uint64_t)c->period_ms))
    {
        return 0U;
    }

    /*
     * Phép trừ tick chỉ được coi là không nhập nhằng
     * khi khoảng thời gian thực nhỏ hơn 2^31 ms.
     */
    if (c->max_gap_ms > 0x7FFFFFFFUL)
        return 0U;

    sample_rate_hz = 1000.0f / (float)c->period_ms;

    if (c->cutoff_hz > (sample_rate_hz / 4.0f))
        return 0U;

    if ((c->median_enable != 0U) &&
        (c->median_enable != 1U))
    {
        return 0U;
    }

    return 1U;
}

void ThrFilter_ConfigDefault(ThrFilter_Config_t *cfg)
{
    if (cfg == NULL)
        return;

    cfg->cutoff_hz = 20.0f;
    cfg->period_ms = 2U;
    cfg->max_gap_ms = 50U;
    cfg->median_enable = 1U;
}

void ThrFilter_Reset(ThrFilter_t *f)
{
    if (f == NULL)
        return;

    ClearState(f);
}

uint8_t ThrFilter_Init(ThrFilter_t *f,
                       const ThrFilter_Config_t *cfg)
{
    uint8_t err = 0U;

    if (f == NULL)
        return 1U;

    memset(f, 0, sizeof(*f));

    ThrFilter_ConfigDefault(&f->cfg);

    if (cfg != NULL)
    {
        if (ConfigValid(cfg))
        {
            f->cfg = *cfg;
        }
        else
        {
            err = 1U;
        }
    }

    f->tau_s =
        1.0f / (2.0f * TF_PI * f->cfg.cutoff_hz);

    ClearState(f);

    return err;
}

uint8_t ThrFilter_Update(ThrFilter_t *f,
                         float x,
                         uint8_t input_valid,
                         uint32_t now_ms)
{
    uint32_t dt_ms;

    float dt_s;
    float alpha;
    float m;

    if (f == NULL)
        return 0U;

    /*
     * Không hợp lệ:
     * - Mất tín hiệu
     * - NaN
     * - Infinity
     *
     * Đưa đầu ra về 0 ngay lập tức.
     */
    if ((input_valid == 0U) || !isfinite(x))
    {
        ClearState(f);
        return 0U;
    }

    /*
     * Giữ hành vi tương thích API cũ:
     * giới hạn dữ liệu đầu vào trong 0..100%.
     */
    x = Clamp(x, 0.0f, 100.0f);

    /*
     * Mẫu đầu tiên:
     * Khởi tạo filter bằng mẫu đầu vào.
     *
     * Việc cho phép cấp công suất khi khởi động
     * phải do Safety State Machine quyết định.
     */
    if (f->seeded == 0U)
    {
        Seed(f, x, now_ms);
        return 1U;
    }

    /*
     * Phép trừ unsigned xử lý được tick tràn:
     * ví dụ FFFFFFF0 -> 00000005.
     */
    dt_ms = (uint32_t)(now_ms - f->last_ms);

    /*
     * Chưa tới chu kỳ lấy mẫu:
     * giữ nguyên đầu ra.
     */
    if (dt_ms < f->cfg.period_ms)
    {
        return f->valid;
    }

    /*
     * QUAN TRỌNG:
     * Không Seed(x) khi thời gian cập nhật quá lâu.
     *
     * Dữ liệu có thể đã cũ hoặc firmware đã bị treo.
     * Reset và báo invalid để tầng Safety xử lý.
     */
    if (dt_ms > f->cfg.max_gap_ms)
    {
        ClearState(f);
        return 0U;
    }

    f->last_ms = now_ms;

    /*
     * Median 3 mẫu.
     */
    f->w[0] = f->w[1];
    f->w[1] = f->w[2];
    f->w[2] = x;

    if (f->cfg.median_enable != 0U)
    {
        m = Median3(f->w[0], f->w[1], f->w[2]);
    }
    else
    {
        m = x;
    }

    /*
     * IIR bậc 1:
     *
     * tau   = 1 / (2*pi*fc)
     * alpha = dt / (tau + dt)
     * y     = y + alpha*(x-y)
     */
    dt_s = (float)dt_ms * 0.001f;

    alpha = dt_s / (f->tau_s + dt_s);

    if (!isfinite(alpha) ||
        alpha < 0.0f ||
        alpha > 1.0f)
    {
        ClearState(f);
        return 0U;
    }

    f->y += alpha * (m - f->y);

    if (!isfinite(f->y))
    {
        ClearState(f);
        return 0U;
    }

    Publish(f);

    return 1U;
}
