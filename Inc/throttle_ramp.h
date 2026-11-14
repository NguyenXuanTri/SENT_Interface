#ifndef THROTTLE_RAMP_H
#define THROTTLE_RAMP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  Giới hạn tốc độ thay đổi ga (slew-rate / ramp), 0..100 %.
 *  Không chặn, không phụ thuộc HAL.
 *
 *  Mỗi bước chỉ cho output tiến gần target tối đa:
 *      Δup   = rate_up_percent_per_s   * dt_s
 *      Δdown = rate_down_percent_per_s * dt_s
 *
 *  Thường rate_down > rate_up (nhả ga nhanh hơn vặn ga).
 *
 *  Vị trí:
 *      Throttle_Update -> ThrFilter_Update -> ThrCurve_Update -> ThrRamp_Update
 *
 *  input_valid = 0 → output = 0, quên trạng thái (an toàn: đóng).
 *  Mẫu đầu tiên: seed = target (không trườn từ 0 nếu đã có % sẵn).
 *  max_gap_ms: quá lâu không cập nhật thì seed lại bằng target hiện tại.
 * ========================================================================== */

typedef struct
{
    float    rate_up_percent_per_s;     /* tốc độ tăng tối đa   (mặc định 200) */
    float    rate_down_percent_per_s;   /* tốc độ giảm tối đa   (mặc định 400) */
    uint32_t max_gap_ms;                /* gap lớn → seed lại   (mặc định 50)  */
} ThrRamp_Config_t;

typedef struct
{
    ThrRamp_Config_t cfg;

    uint8_t  seeded;
    float    y;                 /* trạng thái hiện tại */
    uint32_t last_ms;

    uint8_t  valid;
    float    output;            /* 0..100 % */
    uint16_t output_permille;   /* 0..1000  */
} ThrRamp_t;

void    ThrRamp_ConfigDefault(ThrRamp_Config_t *cfg);

/* cfg = NULL: dùng mặc định. Trả 0 nếu OK, khác 0 nếu cfg sai (vẫn dùng mặc định). */
uint8_t ThrRamp_Init(ThrRamp_t *r, const ThrRamp_Config_t *cfg);

void    ThrRamp_Reset(ThrRamp_t *r);

/* target: 0..100 % sau curve; input_valid từ curve.valid; now_ms = HAL_GetTick().
 * Trả về r->valid. */
uint8_t ThrRamp_Update(ThrRamp_t *r, float target, uint8_t input_valid, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_RAMP_H */