#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  Xử lý vị trí tay ga từ góc cảm biến (độ, 0..360).
 *  Module này KHÔNG phụ thuộc HAL nên chạy và kiểm thử được trên PC.
 *
 *  Quy trình hiệu chỉnh:
 *    1) Giữ tay ga ở vị trí ĐÓNG  -> Throttle_SetClosed(angle)
 *    2) Vặn tay ga tới vị trí MỞ HẾT -> Throttle_SetOpen(angle)
 *    3) Throttle_CommitCalibration()  (tự nhận chiều quay từ 2 điểm trên)
 *  Hoặc nạp lại từ flash: Throttle_LoadCalibration(closed, open).
 *
 *  Góc đưa vào nên là giá trị trung bình nhiều mẫu (dùng Throttle_Avg_t để
 *  lấy trung bình đúng khi góc nằm gần mốc 0°/360°).
 * ========================================================================== */

typedef enum
{
    THR_NOT_CALIBRATED = 0,     /* chưa hiệu chỉnh                                        */
    THR_READY,                  /* đang hoạt động bình thường                             */
    THR_INPUT_INVALID,          /* góc đầu vào không hợp lệ (driver báo lỗi / NaN)        */
    THR_OUT_OF_RANGE            /* góc nằm ngoài hành trình cơ khí cho phép               */
} Throttle_State_t;

typedef enum
{
    THR_CAL_OK = 0,
    THR_CAL_BAD_INPUT,          /* góc là NaN/vô hạn                                      */
    THR_CAL_NOT_READY,          /* thiếu điểm đóng hoặc điểm mở                           */
    THR_CAL_SPAN_TOO_SMALL,     /* hành trình quá nhỏ                                     */
    THR_CAL_SPAN_TOO_LARGE      /* hành trình quá lớn (từ 180° trở lên thì không xác định) */
} Throttle_CalResult_t;

typedef struct
{
    float min_span_deg;         /* hành trình tối thiểu chấp nhận khi hiệu chỉnh  (mặc định 10)  */
    float max_span_deg;         /* hành trình tối đa chấp nhận khi hiệu chỉnh     (mặc định 120) */
    float range_margin_deg;     /* dung sai ngoài 2 đầu hành trình                (mặc định 5)   */
    float dead_low_percent;     /* vùng chết đầu hành trình (%)                   (mặc định 2)   */
    float dead_high_percent;    /* vùng chết cuối hành trình (%)                  (mặc định 2)   */
} Throttle_Config_t;

typedef struct
{
    Throttle_Config_t cfg;

    /* --- hiệu chỉnh --- */
    uint8_t  closed_set;
    uint8_t  open_set;
    uint8_t  calibrated;
    float    closed_deg;        /* góc ở vị trí đóng                              */
    float    open_deg;          /* góc ở vị trí mở hết                            */
    int8_t   direction;         /* +1: góc tăng khi mở, -1: góc giảm khi mở       */
    float    span_deg;          /* hành trình (độ), luôn dương                    */

    /* --- đầu ra, cập nhật bởi Throttle_Update() --- */
    Throttle_State_t state;
    uint8_t  valid;             /* 1: position_* được phép dùng                   */
    float    travel_deg;        /* góc đã quay từ điểm đóng theo chiều mở         */
    float    raw_percent;       /* 0..100, chưa tính vùng chết                    */
    float    position_percent;  /* 0..100, đã tính vùng chết                      */
    uint16_t position_permille; /* 0..1000                                        */
} Throttle_t;

/* Trung bình vòng của góc (đúng khi quanh mốc 0°/360°) dùng cho lúc hiệu chỉnh */
typedef struct
{
    float    sum_sin;
    float    sum_cos;
    uint16_t count;
} Throttle_Avg_t;

void Throttle_ConfigDefault(Throttle_Config_t *cfg);

/* cfg = NULL: dùng cấu hình mặc định. Trả về 0 nếu cấu hình hợp lệ, khác 0 nếu sai
 * (khi sai, module vẫn dùng cấu hình mặc định). */
uint8_t Throttle_Init(Throttle_t *thr, const Throttle_Config_t *cfg);

Throttle_CalResult_t Throttle_SetClosed(Throttle_t *thr, float angle_deg);
Throttle_CalResult_t Throttle_SetOpen(Throttle_t *thr, float angle_deg);
Throttle_CalResult_t Throttle_CommitCalibration(Throttle_t *thr);
Throttle_CalResult_t Throttle_LoadCalibration(Throttle_t *thr, float closed_deg, float open_deg);

/* Gọi mỗi khi có góc mới (hoặc mỗi vòng lặp). angle_valid = 1 chỉ khi
 * TLE5014_ReadAngle() trả về TLE5014_OK. Khi không hợp lệ thì valid = 0 và
 * vị trí đầu ra = 0 (an toàn: đóng), nhưng phải kiểm tra thr->valid. */
void Throttle_Update(Throttle_t *thr, float angle_deg, uint8_t angle_valid);

void    Throttle_AvgReset(Throttle_Avg_t *avg);
void    Throttle_AvgAdd(Throttle_Avg_t *avg, float angle_deg);
/* Trả về 1 nếu có dữ liệu và ghi trung bình (0..360) vào *angle_deg */
uint8_t Throttle_AvgGet(const Throttle_Avg_t *avg, float *angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_H */