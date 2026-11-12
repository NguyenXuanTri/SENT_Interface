#ifndef TLE5014_H
#define TLE5014_H

#include "main.h"      /* HAL + IFB_Pin / IFB_GPIO_Port (nhãn PB12 trong CubeMX) */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Dải dữ liệu SENT của bản TLE5014S16 (SENT Data Range 1..4088, error code 4091) ===== */
#define TLE5014_SENT_DATA_MIN        1U
#define TLE5014_SENT_DATA_MAX        4088U
#define TLE5014_SENT_ERROR_CODE      4091U

/* ===== Ngưỡng an toàn (chỉnh theo FTTI của hệ thống) ===== */
#define TLE5014_SIGNAL_TIMEOUT_MS    5U   /* không nhận được frame nào trong 5 ms -> mất tín hiệu     */
#define TLE5014_FAULT_LIMIT          3U   /* số frame lỗi liên tiếp (khung/CRC/rolling) -> góc mất hiệu lực */
#define TLE5014_VALID_AGE_MS         10U  /* góc hợp lệ cũ hơn 10 ms thì không được dùng nữa           */

typedef enum
{
    TLE5014_OK = 0,
    TLE5014_ERROR,              /* tham số sai                                              */
    TLE5014_NOT_INITIALIZED,
    TLE5014_NO_SIGNAL,          /* không có cạnh/frame nào (đường dây ở mức cao: đứt dây, mất nguồn,
                                   quá áp -> ngõ ra tristate)                                */
    TLE5014_STUCK_LOW,          /* không có frame và đường dây ở mức thấp (chạm GND)         */
    TLE5014_FRAME_ERROR,        /* nibble inverted không khớp                                */
    TLE5014_CRC_ERROR,
    TLE5014_ROLLING_ERROR,      /* rolling counter đứng yên / không tăng đúng                */
    TLE5014_SENSOR_ERROR        /* cảm biến tự báo lỗi (status bit 0, mã 4091, ngoài dải)    */
} TLE5014_Status_t;

typedef enum
{
    TLE5014_CHANNEL_1 = 0,
    TLE5014_CHANNEL_2
} TLE5014_Channel_t;

typedef struct
{
    uint16_t raw_angle;         /* 1..4088                                    */
    uint8_t  safety_word;       /* status nibble                              */
    uint8_t  rolling_counter;   /* 8 bit                                      */
    float    angle_degrees;     /* 0..360                                     */
    uint16_t angle_counts;      /* 1..100                                     */
} TLE5014_Angle_t;

/* Giữ nguyên phần throttle cũ của bạn: ở đây chỉ có trạng thái để code biên dịch được */
typedef enum
{
    TLE5014_THR_NOT_CALIBRATED = 0
} TLE5014_ThrottleState_t;

typedef struct
{
    TLE5014_ThrottleState_t state;
} TLE5014_Throttle_t;

typedef struct
{
    UART_HandleTypeDef *uart;
    uint8_t             initialized;

    TLE5014_Status_t    channel1_status;        /* trạng thái của frame xử lý gần nhất         */
    TLE5014_Status_t    channel2_status;
    TLE5014_Angle_t     channel1;               /* góc HỢP LỆ gần nhất (không bị ghi đè khi lỗi) */
    uint8_t             channel1_valid;         /* 1: được phép dùng góc                        */
    uint32_t            channel1_last_valid_tick;
    uint8_t             consecutive_faults;

    TLE5014_Throttle_t  throttle;
} TLE5014_Handle_t;

/* Bộ đếm để debug / chẩn đoán (xem bằng debugger hoặc in UART) */
typedef struct
{
    uint32_t edges;             /* tổng số cạnh xuống bắt được                         */
    uint32_t sync_found;        /* số lần nhận ra xung sync                            */
    uint32_t frames_captured;   /* số frame đủ 8 nibble (ISR)                          */
    uint32_t nibble_errors;     /* nibble ngoài 12..27 UT -> bỏ frame                  */
    uint32_t resyncs;           /* gặp xung giống sync giữa frame -> sync lại          */
    uint32_t overruns;          /* frame bị mất vì Task() gọi không kịp                */
    uint32_t signal_losses;     /* số lần mất tín hiệu                                 */
    uint32_t frame_errors;
    uint32_t crc_errors;
    uint32_t rolling_errors;
    uint32_t sensor_errors;
    uint32_t frames_ok;
    uint32_t last_interval_ticks;
    uint16_t ut_q4;             /* tick/UT x16 đo từ sync của frame gần nhất           */
    uint8_t  pin_level;
    uint8_t  frame[8];          /* 8 nibble của frame gần nhất                         */
} TLE5014_SENT_Debug_t;

extern TLE5014_SENT_Debug_t tle5014_sent_debug;

TLE5014_Status_t TLE5014_Init(TLE5014_Handle_t *sensor, UART_HandleTypeDef *uart);

/* Gọi liên tục trong vòng lặp chính (không chặn). Xử lý mọi frame đã nhận. */
void TLE5014_Task(TLE5014_Handle_t *sensor);

/* Không chặn. Trả về OK chỉ khi góc còn hợp lệ; angle luôn chứa góc hợp lệ gần nhất. */
TLE5014_Status_t TLE5014_ReadAngle(TLE5014_Channel_t channel, TLE5014_Angle_t *angle);

/* Gọi từ ngắt EXTI line 12 (cạnh xuống của PB12). */
void TLE5014_EdgeIRQ(void);

#ifdef __cplusplus
}
#endif

#endif /* TLE5014_H */