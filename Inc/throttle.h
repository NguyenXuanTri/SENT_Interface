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
 *  Việc lấy mẫu và lấy trung bình (đúng khi góc gần mốc 0°/360°) do
 *  Throttle_CalBegin() + Throttle_CalStep() làm sẵn; không cần tự gọi
 *  Throttle_AvgAdd().
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
    THR_CAL_SPAN_TOO_LARGE,     /* hành trình quá lớn (từ 180° trở lên thì không xác định) */
    THR_CAL_UNSTABLE,           /* góc dao động quá nhiều khi lấy mẫu (tay ga đang di chuyển) */
    THR_CAL_TIMEOUT             /* hết thời gian mà chưa đủ mẫu hợp lệ                     */
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

/* ---- Lấy mẫu hiệu chỉnh không chặn ---------------------------------------- */
#define THR_CAL_MAX_SPREAD_DEG   2.0f   /* độ lệch chuẩn góc tối đa khi lấy mẫu (độ) */

typedef enum
{
    THR_POINT_CLOSED = 0,
    THR_POINT_OPEN
} Throttle_Point_t;

typedef enum
{
    THR_CALSTEP_IDLE = 0,       /* chưa bắt đầu                                    */
    THR_CALSTEP_SAMPLING,       /* đang lấy mẫu                                    */
    THR_CALSTEP_DONE,           /* xong, điểm đã được ghi vào Throttle_t           */
    THR_CALSTEP_FAILED          /* thất bại, xem lý do ở cal->result               */
} Throttle_CalState_t;

typedef struct
{
    Throttle_Avg_t       avg;
    Throttle_CalState_t  state;
    Throttle_Point_t     point;
    Throttle_CalResult_t result;    /* hợp lệ khi state = DONE hoặc FAILED */
    uint16_t             samples_needed;
    uint32_t             interval_ms;
    uint32_t             timeout_ms;
    uint32_t             start_ms;
    uint32_t             last_ms;
} Throttle_Cal_t;

void Throttle_ConfigDefault(Throttle_Config_t *cfg);

/* cfg = NULL: dùng cấu hình mặc định. Trả về 0 nếu cấu hình hợp lệ, khác 0 nếu sai
 * (khi sai, module vẫn dùng cấu hình mặc định). */
uint8_t Throttle_Init(Throttle_t *thr, const Throttle_Config_t *cfg);

Throttle_CalResult_t Throttle_SetClosed(Throttle_t *thr, float angle_deg);
Throttle_CalResult_t Throttle_SetOpen(Throttle_t *thr, float angle_deg);
Throttle_CalResult_t Throttle_CommitCalibration(Throttle_t *thr);
Throttle_CalResult_t Throttle_LoadCalibration(Throttle_t *thr, float closed_deg, float open_deg);

/* Bắt đầu lấy mẫu cho một điểm (đóng hoặc mở hết). Giữ tay ga đứng yên.
 * samples: số mẫu hợp lệ cần lấy; interval_ms: khoảng cách giữa 2 mẫu;
 * timeout_ms: quá thời gian này mà chưa đủ mẫu thì thất bại.
 * Yêu cầu timeout_ms > (samples - 1) * interval_ms; nếu không thì không bao giờ đủ mẫu
 * kịp, và CalBegin đặt ngay state = FAILED, result = THR_CAL_BAD_INPUT. */
void Throttle_CalBegin(Throttle_Cal_t *cal, Throttle_Point_t point, uint16_t samples,
                       uint32_t interval_ms, uint32_t timeout_ms, uint32_t now_ms);

/* Gọi liên tục trong vòng lặp chính (không chặn). Hàm tự gọi Throttle_AvgAdd()
 * mỗi interval_ms khi angle_valid = 1; đủ mẫu thì tự tính trung bình và ghi vào
 * thr bằng Throttle_SetClosed()/Throttle_SetOpen(). Sau khi cả 2 điểm DONE thì
 * gọi Throttle_CommitCalibration(). */
Throttle_CalState_t Throttle_CalStep(Throttle_Cal_t *cal, Throttle_t *thr, uint32_t now_ms,
                                     float angle_deg, uint8_t angle_valid);

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