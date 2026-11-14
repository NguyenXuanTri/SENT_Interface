#include "throttle_filter.h"
#include <math.h>
#include <string.h>

#define TF_PI   3.14159265358979f

static float Clamp(float x, float lo, float hi)
{
    if (x < lo) { return lo; }
    if (x > hi) { return hi; }
    return x;
}

static float Median3(float a, float b, float c)
{
    /* max(min(a,b), min(max(a,b), c)) */
    float lo = (a < b) ? a : b;
    float hi = (a < b) ? b : a;
    float m  = (c < hi) ? c : hi;
    return (lo > m) ? lo : m;
}

static uint8_t ConfigValid(const ThrFilter_Config_t *c)
{
    float sample_rate_hz;

    if ((isfinite(c->cutoff_hz) == 0) || (c->cutoff_hz <= 0.0f))
    {
        return 0U;
    }
    if ((c->period_ms < 1U) || (c->max_gap_ms < (2U * c->period_ms)))
    {
        return 0U;
    }

    sample_rate_hz = 1000.0f / (float)c->period_ms;
    if (c->cutoff_hz > (sample_rate_hz / 4.0f))     /* cắt quá gần tần số Nyquist thì vô nghĩa */
    {
        return 0U;
    }
    return 1U;
}

static void ClearOutput(ThrFilter_t *f)
{
    f->valid           = 0U;
    f->output          = 0.0f;
    f->output_permille = 0U;
}

static void Publish(ThrFilter_t *f)
{
    f->output          = Clamp(f->y, 0.0f, 100.0f);
    f->output_permille = (uint16_t)((f->output * 10.0f) + 0.5f);
    f->valid           = 1U;
}

/* Nạp lại toàn bộ trạng thái bằng 1 mẫu: đầu ra bằng đúng mẫu đó, không "trượt" từ 0 lên */
static void Seed(ThrFilter_t *f, float x, uint32_t now_ms)
{
    f->y       = x;
    f->w[0]    = x;
    f->w[1]    = x;
    f->w[2]    = x;
    f->last_ms = now_ms;
    f->seeded  = 1U;
    Publish(f);
}

void ThrFilter_ConfigDefault(ThrFilter_Config_t *cfg)
{
    cfg->cutoff_hz     = 20.0f;
    cfg->period_ms     = 2U;
    cfg->max_gap_ms    = 50U;
    cfg->median_enable = 1U;
}

void ThrFilter_Reset(ThrFilter_t *f)
{
    f->seeded  = 0U;
    f->y       = 0.0f;
    f->w[0]    = 0.0f;
    f->w[1]    = 0.0f;
    f->w[2]    = 0.0f;
    f->last_ms = 0U;
    ClearOutput(f);
}

uint8_t ThrFilter_Init(ThrFilter_t *f, const ThrFilter_Config_t *cfg)
{
    uint8_t err = 0U;

    memset(f, 0, sizeof(*f));
    ThrFilter_ConfigDefault(&f->cfg);

    if (cfg != NULL)
    {
        if (ConfigValid(cfg) != 0U)
        {
            f->cfg = *cfg;
        }
        else
        {
            err = 1U;       /* cấu hình sai: giữ mặc định */
        }
    }

    f->tau_s = 1.0f / (2.0f * TF_PI * f->cfg.cutoff_hz);
    ThrFilter_Reset(f);
    return err;
}

uint8_t ThrFilter_Update(ThrFilter_t *f, float x, uint8_t input_valid, uint32_t now_ms)
{
    uint32_t dt_ms;
    float dt_s;
    float alpha;
    float m;

    if ((input_valid == 0U) || (isfinite(x) == 0))
    {
        ThrFilter_Reset(f);     /* không hợp lệ: đầu ra 0, quên trạng thái cũ */
        return 0U;
    }

    x = Clamp(x, 0.0f, 100.0f);

    if (f->seeded == 0U)
    {
        Seed(f, x, now_ms);
        return 1U;
    }

    dt_ms = (uint32_t)(now_ms - f->last_ms);    /* đúng cả khi bộ đếm ms tràn số */

    if (dt_ms < f->cfg.period_ms)
    {
        return f->valid;        /* chưa đến kỳ lấy mẫu: giữ nguyên đầu ra */
    }

    if (dt_ms > f->cfg.max_gap_ms)
    {
        Seed(f, x, now_ms);     /* lâu không cập nhật: trạng thái cũ không còn đúng */
        return 1U;
    }

    f->last_ms = now_ms;

    f->w[0] = f->w[1];
    f->w[1] = f->w[2];
    f->w[2] = x;
    m = (f->cfg.median_enable != 0U) ? Median3(f->w[0], f->w[1], f->w[2]) : x;

    dt_s  = (float)dt_ms * 0.001f;
    alpha = dt_s / (f->tau_s + dt_s);
    f->y += alpha * (m - f->y);

    Publish(f);
    return 1U;
}