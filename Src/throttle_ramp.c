#include "throttle_ramp.h"
#include <math.h>
#include <string.h>

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

static void ClearOutput(ThrRamp_t *r)
{
    r->valid           = 0U;
    r->output          = 0.0f;
    r->output_permille = 0U;
}

static void Publish(ThrRamp_t *r)
{
    r->output          = Clamp(r->y, 0.0f, 100.0f);
    r->output_permille = (uint16_t)((r->output * 10.0f) + 0.5f);
    r->valid           = 1U;
}

static void Seed(ThrRamp_t *r, float x, uint32_t now_ms)
{
    r->y       = x;
    r->last_ms = now_ms;
    r->seeded  = 1U;
    Publish(r);
}

static uint8_t ConfigValid(const ThrRamp_Config_t *c)
{
    if ((IsFinite(c->rate_up_percent_per_s) == 0U) ||
        (IsFinite(c->rate_down_percent_per_s) == 0U))
    {
        return 0U;
    }
    if ((c->rate_up_percent_per_s <= 0.0f) ||
        (c->rate_down_percent_per_s <= 0.0f))
    {
        return 0U;
    }
    /* trần thực tế: tránh cấu hình vô lý (ví dụ 1e9 %/s) */
    if ((c->rate_up_percent_per_s > 10000.0f) ||
        (c->rate_down_percent_per_s > 10000.0f))
    {
        return 0U;
    }
    if (c->max_gap_ms < 1U)
    {
        return 0U;
    }
    return 1U;
}

void ThrRamp_ConfigDefault(ThrRamp_Config_t *cfg)
{
    cfg->rate_up_percent_per_s   = 200.0f;
    cfg->rate_down_percent_per_s = 400.0f;
    cfg->max_gap_ms              = 50U;
}

void ThrRamp_Reset(ThrRamp_t *r)
{
    r->seeded  = 0U;
    r->y       = 0.0f;
    r->last_ms = 0U;
    ClearOutput(r);
}

uint8_t ThrRamp_Init(ThrRamp_t *r, const ThrRamp_Config_t *cfg)
{
    uint8_t err = 0U;

    memset(r, 0, sizeof(*r));
    ThrRamp_ConfigDefault(&r->cfg);

    if (cfg != NULL)
    {
        if (ConfigValid(cfg) != 0U)
        {
            r->cfg = *cfg;
        }
        else
        {
            err = 1U;
        }
    }

    ThrRamp_Reset(r);
    return err;
}

uint8_t ThrRamp_Update(ThrRamp_t *r, float target, uint8_t input_valid, uint32_t now_ms)
{
    uint32_t dt_ms;
    float dt_s;
    float max_up;
    float max_down;

    if ((input_valid == 0U) || (IsFinite(target) == 0U))
    {
        ThrRamp_Reset(r);   /* mất tín hiệu: về 0, quên trạng thái */
        return 0U;
    }

    target = Clamp(target, 0.0f, 100.0f);

    if (r->seeded == 0U)
    {
        Seed(r, target, now_ms);
        return 1U;
    }

    dt_ms = (uint32_t)(now_ms - r->last_ms);   /* đúng cả khi tick tràn */

    if (dt_ms == 0U)
    {
        return r->valid;   /* cùng ms: giữ nguyên */
    }

    if (dt_ms > r->cfg.max_gap_ms)
    {
        Seed(r, target, now_ms);
        return 1U;
    }

    r->last_ms = now_ms;
    dt_s       = (float)dt_ms * 0.001f;
    max_up     = r->cfg.rate_up_percent_per_s * dt_s;
    max_down   = r->cfg.rate_down_percent_per_s * dt_s;

    if (target > r->y + max_up)
    {
        r->y = r->y + max_up;
    }
    else if (target < r->y - max_down)
    {
        r->y = r->y - max_down;
    }
    else
    {
        r->y = target;   /* trong dải cho phép: bám đích */
    }

    Publish(r);
    return 1U;
}