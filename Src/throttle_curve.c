#include "throttle_curve.h"
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

static void ClearOutput(ThrCurve_t *c)
{
    c->valid           = 0U;
    c->output          = 0.0f;
    c->output_permille = 0U;
}

static void Publish(ThrCurve_t *c, float y)
{
    y = Clamp(y, 0.0f, 100.0f);
    c->output          = y;
    c->output_permille = (uint16_t)((y * 10.0f) + 0.5f);
    c->valid           = 1U;
}

static float ApplyExp(float x, float gamma)
{
    /* x đã clamp 0..100; đảm bảo biên */
    if (x <= 0.0f) { return 0.0f; }
    if (x >= 100.0f) { return 100.0f; }
    return 100.0f * powf(x * 0.01f, gamma);
}

static float ApplyLut(float x, const ThrCurve_Config_t *cfg)
{
    const uint8_t n = cfg->lut_count;
    const float *xp = cfg->lut_x;
    const float *yp = cfg->lut_y;
    uint8_t i;

    if (n < 2U) { return x; }
    if (x <= xp[0]) { return yp[0]; }
    if (x >= xp[n - 1U]) { return yp[n - 1U]; }

    i = 0U;
    while ((i + 1U) < n && x > xp[i + 1U])
    {
        i++;
    }
    {
        float x0 = xp[i];
        float x1 = xp[i + 1U];
        float y0 = yp[i];
        float y1 = yp[i + 1U];
        float t  = (x1 > x0) ? ((x - x0) / (x1 - x0)) : 0.0f;
        return y0 + t * (y1 - y0);
    }
}

static uint8_t ConfigValid(const ThrCurve_Config_t *c)
{
    uint8_t i;

    if (c->mode > THR_CURVE_LUT)
    {
        return 0U;
    }
    if (c->mode == THR_CURVE_EXP)
    {
        if ((IsFinite(c->gamma) == 0U) || (c->gamma < 0.2f) || (c->gamma > 5.0f))
        {
            return 0U;
        }
    }
    if (c->mode == THR_CURVE_LUT)
    {
        if ((c->lut_count < 2U) || (c->lut_count > THR_CURVE_LUT_MAX))
        {
            return 0U;
        }
        for (i = 0U; i < c->lut_count; i++)
        {
            if ((IsFinite(c->lut_x[i]) == 0U) || (IsFinite(c->lut_y[i]) == 0U))
            {
                return 0U;
            }
            if ((c->lut_y[i] < 0.0f) || (c->lut_y[i] > 100.0f))
            {
                return 0U;
            }
            if (i > 0U && c->lut_x[i] <= c->lut_x[i - 1U])
            {
                return 0U; /* xp phải tăng nghiêm ngặt */
            }
        }
        /* an toàn: đầu 0, cuối 100 (cho phép sai số nhỏ) */
        if ((c->lut_x[0] > 0.01f) || (c->lut_y[0] > 0.01f))
        {
            return 0U;
        }
        if ((c->lut_x[c->lut_count - 1U] < 99.99f) ||
            (c->lut_y[c->lut_count - 1U] < 99.99f))
        {
            return 0U;
        }
    }
    return 1U;
}

void ThrCurve_ConfigDefault(ThrCurve_Config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->mode  = THR_CURVE_LINEAR;
    cfg->gamma = 1.5f;
    cfg->lut_count = 0U;
}

void ThrCurve_Reset(ThrCurve_t *c)
{
    ClearOutput(c);
}

uint8_t ThrCurve_Init(ThrCurve_t *c, const ThrCurve_Config_t *cfg)
{
    uint8_t err = 0U;

    memset(c, 0, sizeof(*c));
    ThrCurve_ConfigDefault(&c->cfg);

    if (cfg != NULL)
    {
        if (ConfigValid(cfg) != 0U)
        {
            c->cfg = *cfg;
        }
        else
        {
            err = 1U;
        }
    }
    ThrCurve_Reset(c);
    return err;
}

uint8_t ThrCurve_Update(ThrCurve_t *c, float x, uint8_t input_valid)
{
    float y;

    if ((input_valid == 0U) || (IsFinite(x) == 0U))
    {
        ClearOutput(c);
        return 0U;
    }

    x = Clamp(x, 0.0f, 100.0f);

    switch (c->cfg.mode)
    {
    case THR_CURVE_EXP:
        y = ApplyExp(x, c->cfg.gamma);
        break;
    case THR_CURVE_LUT:
        y = ApplyLut(x, &c->cfg);
        break;
    case THR_CURVE_LINEAR:
    default:
        y = x;
        break;
    }

    Publish(c, y);
    return 1U;
}