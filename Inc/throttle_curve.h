#ifndef THROTTLE_CURVE_H
#define THROTTLE_CURVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  Đường cong ga (0..100 %), không chặn, không phụ thuộc HAL.
 *
 *  Đầu vào: % sau deadzone + filter (thr->position_percent hoặc flt->output).
 *  Đầu ra: % “cảm giác” trước ramp / điều khiển.
 *
 *  Chế độ:
 *    THR_CURVE_LINEAR — out = in
 *    THR_CURVE_EXP    — out = 100 * (in/100)^gamma
 *                       gamma > 1: đầu nhẹ, cuối mạnh
 *                       gamma < 1: đầu nhạy
 *    THR_CURVE_LUT    — bảng N điểm (xp tăng dần), nội suy tuyến tính
 *
 *  Ràng buộc an toàn: out(0) = 0, out(100) = 100; input không hợp lệ → 0.
 *  Vị trí: Throttle_Update -> ThrFilter_Update -> ThrCurve_Update -> (ramp)
 * ========================================================================== */

typedef enum
{
    THR_CURVE_LINEAR = 0,
    THR_CURVE_EXP,
    THR_CURVE_LUT
} ThrCurve_Mode_t;

#define THR_CURVE_LUT_MAX  32

typedef struct
{
    ThrCurve_Mode_t mode;
    float           gamma;          /* chỉ dùng khi EXP; mặc định 1.5         */
    uint8_t         lut_count;      /* 2..THR_CURVE_LUT_MAX khi mode = LUT    */
    float           lut_x[THR_CURVE_LUT_MAX]; /* % vào, tăng dần, 0..100      */
    float           lut_y[THR_CURVE_LUT_MAX]; /* % ra tương ứng                */
} ThrCurve_Config_t;

typedef struct
{
    ThrCurve_Config_t cfg;

    uint8_t  valid;
    float    output;            /* 0..100 % */
    uint16_t output_permille;   /* 0..1000  */
} ThrCurve_t;

void    ThrCurve_ConfigDefault(ThrCurve_Config_t *cfg);

/* cfg = NULL: linear. Trả 0 nếu OK, khác 0 nếu cfg sai (vẫn dùng mặc định). */
uint8_t ThrCurve_Init(ThrCurve_t *c, const ThrCurve_Config_t *cfg);

void    ThrCurve_Reset(ThrCurve_t *c);

/* x: 0..100 % sau filter; input_valid: 1 chỉ khi filter/throttle valid.
 * Trả về c->valid. */
uint8_t ThrCurve_Update(ThrCurve_t *c, float x, uint8_t input_valid);

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_CURVE_H */