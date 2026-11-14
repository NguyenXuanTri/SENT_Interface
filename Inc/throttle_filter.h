#ifndef THROTTLE_FILTER_H
#define THROTTLE_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  Bộ lọc vị trí tay ga (0..100 %), không chặn, không phụ thuộc HAL.
 *
 *  Gồm 2 tầng:
 *    1) Median 3 mẫu: loại gai đơn lẻ (1 mẫu bị nhiễu) mà gần như không làm trễ.
 *       Hai mẫu xấu liên tiếp thì median KHÔNG loại được.
 *    2) IIR bậc 1 (low-pass) có hệ số tính theo thời gian thực:
 *           alpha = dt / (tau + dt),   tau = 1 / (2*pi*cutoff_hz)
 *       nên đáp ứng không đổi dù vòng lặp chính chạy nhanh hay chậm.
 *
 *  Bộ lọc tự lấy mẫu theo chu kỳ cố định period_ms (mặc định 2 ms) từ giá trị
 *  mới nhất bạn đưa vào, nên có thể gọi ThrFilter_Update() MỖI VÒNG LẶP.
 *
 *  Vị trí trong chuỗi xử lý (sau các kiểm tra an toàn, trước đường cong/ramp):
 *      TLE5014_ReadAngle -> Throttle_Update -> ThrFilter_Update -> (curve, ramp)
 *
 *  Không hợp lệ -> đầu ra 0 (an toàn: đóng) và bộ lọc bị đặt lại, nên khi tín
 *  hiệu hợp lệ trở lại thì không trượt từ giá trị cũ. Luôn kiểm tra flt->valid.
 * ========================================================================== */

typedef struct
{
    float    cutoff_hz;       /* tần số cắt IIR                    (mặc định 20 Hz)  */
    uint32_t period_ms;       /* chu kỳ lấy mẫu của bộ lọc          (mặc định 2 ms)   */
    uint32_t max_gap_ms;      /* quá khoảng này không cập nhật thì nạp lại mẫu hiện tại
                                 (dữ liệu cũ)                       (mặc định 50 ms)  */
    uint8_t  median_enable;   /* 1: bật median 3 mẫu                (mặc định 1)      */
} ThrFilter_Config_t;

typedef struct
{
    ThrFilter_Config_t cfg;
    float    tau_s;           /* hằng số thời gian (tính sẵn)                         */

    uint8_t  seeded;          /* đã có mẫu đầu tiên                                   */
    float    y;               /* trạng thái IIR                                       */
    float    w[3];            /* cửa sổ median                                        */
    uint32_t last_ms;         /* thời điểm lấy mẫu gần nhất                           */

    /* --- đầu ra --- */
    uint8_t  valid;           /* 1: output được phép dùng                             */
    float    output;          /* 0..100 %                                             */
    uint16_t output_permille; /* 0..1000                                              */
} ThrFilter_t;

void ThrFilter_ConfigDefault(ThrFilter_Config_t *cfg);

/* cfg = NULL: dùng mặc định. Trả về 0 nếu cấu hình hợp lệ, khác 0 nếu sai
 * (khi sai, bộ lọc dùng cấu hình mặc định). Cấu hình hợp lệ khi:
 *   cutoff_hz > 0 và <= tần số lấy mẫu / 4, period_ms >= 1, max_gap_ms >= 2 * period_ms. */
uint8_t ThrFilter_Init(ThrFilter_t *f, const ThrFilter_Config_t *cfg);

/* Xóa trạng thái; đầu ra = 0, valid = 0. */
void ThrFilter_Reset(ThrFilter_t *f);

/* x: vị trí 0..100 % (thường là thr->position_percent);
 * input_valid: 1 chỉ khi thr->valid = 1;  now_ms: HAL_GetTick().
 * Trả về f->valid. */
uint8_t ThrFilter_Update(ThrFilter_t *f, float x, uint8_t input_valid, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* THROTTLE_FILTER_H */