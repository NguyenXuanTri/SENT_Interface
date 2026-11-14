
#include "throttle_ramp.h"

#include <math.h>
#include <string.h>

static float Clamp(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void ClearOutput(ThrRamp_t *r)
{
    r->valid = 0U;
    r->output = 0.0f;
    r->output_permille = 0U;
}

static void ClearState(ThrRamp_t *r)
{
    r->seeded = 0U;
    r->y = 0.0f;
    r->last_ms = 0U;

    ClearOutput(r);
}

static void Publish(ThrRamp_t *r)
{
    r->output = Clamp(r->y, 0.0f, 100.0f);

    r->output_permille =
        (uint16_t)((r->output * 10.0f) + 0.5f);

    r->valid = 1U;
}

static uint8_t ConfigValid(const ThrRamp_Config_t *c)
{
    if (c == NULL)
        return 0U;

    if (!isfinite(c->rate_up_percent_per_s) ||
        !isfinite(c->rate_down_percent_per_s))
    {
        return 0U;
    }

    if (c->rate_up_percent_per_s <= 0.0f ||
        c->rate_down_percent_per_s <= 0.0f)
    {
        return 0U;
    }

    /*
     * Giới hạn cấu hình để tránh tốc độ vô lý.
     */
    if (c->rate_up_percent_per_s > 10000.0f ||
        c->rate_down_percent_per_s > 10000.0f)
    {
        return 0U;
    }

    if (c->max_gap_ms < 1U ||
        c->max_gap_ms > 0x7FFFFFFFUL)
    {
        return 0U;
    }

    return 1U;
}

void ThrRamp_ConfigDefault(ThrRamp_Config_t *cfg)
{
    if (cfg == NULL)
        return;

    cfg->rate_up_percent_per_s = 200.0f;
    cfg->rate_down_percent_per_s = 400.0f;
    cfg->max_gap_ms = 50U;
}

void ThrRamp_Reset(ThrRamp_t *r)
{
    if (r == NULL)
        return;

    ClearState(r);
}

uint8_t ThrRamp_Init(ThrRamp_t *r,
                     const ThrRamp_Config_t *cfg)
{
    uint8_t err = 0U;

    if (r == NULL)
        return 1U;

    memset(r, 0, sizeof(*r));

    ThrRamp_ConfigDefault(&r->cfg);

    if (cfg != NULL)
    {
        if (ConfigValid(cfg))
        {
            r->cfg = *cfg;
        }
        else
        {
            err = 1U;
        }
    }

    ClearState(r);

    return err;
}

uint8_t ThrRamp_Update(ThrRamp_t *r,
                       float target,
                       uint8_t input_valid,
                       uint32_t now_ms)
{
    uint32_t dt_ms;

    float dt_s;
    float max_up;
    float max_down;

    if (r == NULL)
        return 0U;

    /*
     * Mất tín hiệu hoặc dữ liệu không phải số:
     * yêu cầu đầu ra về 0 ngay.
     */
    if ((input_valid == 0U) || !isfinite(target))
    {
        ClearState(r);
        return 0U;
    }

    /*
     * Giữ tương thích với API hiện tại.
     */
    target = Clamp(target, 0.0f, 100.0f);

    /*
     * Lần đầu khởi tạo:
     *
     * Không Seed(target).
     * Bắt đầu từ 0% để tránh lệnh ga nhảy lên
     * ngay khi module được khởi động.
     *
     * Safety State Machine vẫn phải kiểm tra
     * điều kiện khởi động và cho phép hoạt động.
     */
    if (r->seeded == 0U)
    {
        r->y = 0.0f;
        r->last_ms = now_ms;
        r->seeded = 1U;

        Publish(r);

        return 1U;
    }

    /*
     * Hỗ trợ tick uint32 tràn số.
     */
    dt_ms = (uint32_t)(now_ms - r->last_ms);

    /*
     * Cùng một thời điểm:
     * không cập nhật hai lần.
     */
    if (dt_ms == 0U)
    {
        return r->valid;
    }

    /*
     * Firmware hoặc vòng lặp cập nhật bị trễ:
     *
     * Không tính ramp với dt quá lớn.
     * Không Seed(target).
     * Báo invalid và xóa trạng thái.
     *
     * Tầng Safety phải chuyển sang SAFE state.
     */
    if (dt_ms > r->cfg.max_gap_ms)
    {
        ClearState(r);
        return 0U;
    }

    r->last_ms = now_ms;

    dt_s = (float)dt_ms * 0.001f;

    max_up =
        r->cfg.rate_up_percent_per_s * dt_s;

    max_down =
        r->cfg.rate_down_percent_per_s * dt_s;

    /*
     * Giới hạn tốc độ tăng.
     */
    if (target > r->y + max_up)
    {
        r->y += max_up;
    }
    /*
     * Giới hạn tốc độ giảm.
     */
    else if (target < r->y - max_down)
    {
        r->y -= max_down;
    }
    /*
     * Target nằm trong giới hạn thay đổi:
     * có thể bám target trực tiếp.
     */
    else
    {
        r->y = target;
    }

    /*
     * Kiểm tra trạng thái trước khi xuất.
     */
    if (!isfinite(r->y))
    {
        ClearState(r);
        return 0U;
    }

    r->y = Clamp(r->y, 0.0f, 100.0f);

    Publish(r);

    return 1U;
}
