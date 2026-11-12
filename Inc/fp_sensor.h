/*
 * r502f.h  -  Điều khiển cảm biến vân tay R502-F bằng STM32 HAL (bản kiểm tra giai đoạn 1)
 *
 * Cách hoạt động: gửi lệnh và đọc phản hồi bằng HAL_UART_Transmit / HAL_UART_Receive
 * có timeout. KHÔNG dùng ngắt, KHÔNG cần callback, KHÔNG cần bật NVIC.
 */
#ifndef fp_sensor_H_
#define fp_sensor_H_

#include <stdint.h>
#include "main.h"

/* ====================================================================== */
/*  CẤU HÌNH - chỉ cần sửa trong khối này                                  */
/* ====================================================================== */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* UART nối với cảm biến (theo bạn xác nhận: TX/RX cảm biến nối USART1) */
#ifndef FP_UART_SENSOR
#define FP_UART_SENSOR			(&huart1)
#endif

/* UART chỉ để xuất log ra máy tính. KHÔNG được trùng với UART của cảm biến. */
#ifndef FP_UART_LOG
#define FP_UART_LOG				(&huart2)
#endif
#ifndef FP_LOG_ENABLE
#define FP_LOG_ENABLE			1		/* 0 = tắt toàn bộ log */
#endif

/* Baud mong đợi của UART cảm biến (chỉ để cảnh báo, code KHÔNG tự đổi baud) */
#ifndef FP_EXPECTED_BAUD
#define FP_EXPECTED_BAUD		57600U
#endif

/* Chế độ LED khi chờ ngón tay:  1 = xanh dương sáng liên tục
 *                               2 = tím nhấp nháy                         */
#ifndef FP_LED_WAIT_MODE
#define FP_LED_WAIT_MODE		1
#endif

/* Khi có ngón tay:  1 = LED xanh lá,  0 = tắt LED */
#ifndef FP_LED_FINGER_GREEN
#define FP_LED_FINGER_GREEN		1
#endif

/* Tham số riêng của chế độ 2. CHƯA kiểm chứng ý nghĩa với module của bạn:
 *  - FP_LED_FLASH_SPEED: tốc độ nhấp nháy, hãy thử vài giá trị và quan sát
 *  - FP_LED_FLASH_TIMES: 0 là số lần lặp; nếu LED chỉ nháy 1 lần rồi dừng thì đổi giá trị này */
#ifndef FP_LED_FLASH_SPEED
#define FP_LED_FLASH_SPEED		0x40U
#endif
#ifndef FP_LED_FLASH_TIMES
#define FP_LED_FLASH_TIMES		0x00U
#endif

/* Chân PCTRL cấp nguồn chính cho R502-F (phải ở mức CAO thì cảm biến mới có nguồn).
 * Suy ra từ thông tin bạn đưa: PCTRL là chân của USART3, WAKEUP đang ở PB10 -> PCTRL là PB11.
 * Nếu sai thì đổi ở đây. Đặt FP_USE_PCTRL = 0 nếu nguồn cảm biến đã cấp cố định.   */
#ifndef FP_USE_PCTRL
#define FP_USE_PCTRL			1
#endif
#define FP_PCTRL_PORT			GPIOB
#define FP_PCTRL_PIN			GPIO_PIN_11
#define FP_PCTRL_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
#define FP_POWER_ON_DELAY_MS	300U		/* chờ cảm biến khởi động sau khi cấp nguồn */

/* Chu kỳ (ms) */
#define FP_HANDSHAKE_RETRY_MS	1000U		/* thử handshake lại khi cảm biến chưa sẵn sàng */
#define FP_POLL_MS				100U		/* hỏi GetImage */
#define FP_NO_FINGER_LOG_MS		1000U		/* báo NO_FINGER định kỳ khi chưa có tay */
#define FP_ERR_LOG_MS			2000U		/* giãn cách tối thiểu giữa hai thông báo lỗi giống nhau */
#define FP_ERR_BEFORE_RESET		3U			/* số lần lỗi liên tiếp thì quay lại bước handshake */

/* ====================================================================== */
/*  Mã lệnh / mã trả về (lấy từ driver_r502_commands.h và driver_r502.h)   */
/* ====================================================================== */
#define R502F_CODE_OK				0x00U
#define R502F_CODE_NO_FINGER		0x02U
#define R502F_CODE_NOT_FOUND 		0x09U

#define R502F_AURA_BREATHING		0x01U
#define R502F_AURA_FLASHING			0x02U
#define R502F_AURA_ALWAYS_ON		0x03U
#define R502F_AURA_ALWAYS_OFF		0x04U

#define R502F_COLOR_RED				0x01U
#define R502F_COLOR_BLUE			0x02U
#define R502F_COLOR_PURPLE			0x03U
#define R502F_COLOR_GREEN			0x04U
#define R502F_COLOR_WHITE			0x00U

/* ====================================================================== */
/*  Kiểu dữ liệu                                                           */
/* ====================================================================== */
typedef enum {
	R502F_OK = 0,			/* nhận được phản hồi hợp lệ */
	R502F_ERR_TX,			/* không gửi được lệnh ra UART */
	R502F_ERR_NO_RESPONSE,	/* không nhận được BYTE NÀO */
	R502F_ERR_TIMEOUT,		/* nhận được một phần gói rồi dừng */
	R502F_ERR_FRAME,		/* có byte nhưng sai header / địa chỉ / loại gói / độ dài */
	R502F_ERR_CHECKSUM		/* đủ gói nhưng sai checksum */
} r502f_status_t;

/* Biến để xem trong Live Expressions / breakpoint */
typedef struct {
	uint8_t  sensor_ready;		/* 1 = đã handshake thành công */
	uint8_t  finger_state;		/* 0 = không tay, 1 = có tay, 2 = chưa biết */
	uint8_t  last_status;		/* r502f_status_t của lệnh gần nhất */
	uint8_t  last_code;			/* mã xác nhận cảm biến trả về (0xFF = chưa có) */
	uint8_t  log_same_uart;		/* 1 = UART log trùng UART cảm biến nên đã tự tắt log */
	uint32_t handshake_count;
	uint32_t get_image_count;
	uint32_t cnt[6];			/* đếm theo r502f_status_t: [0]=OK [1]=TX [2]=NO_RESPONSE [3]=TIMEOUT [4]=FRAME [5]=CHECKSUM */
} r502f_debug_t;

extern r502f_debug_t g_r502f;

/* ====================================================================== */
/*  API                                                                    */
/* ====================================================================== */
void r502f_init(void);		/* cấp nguồn cho cảm biến, chuẩn bị UART và log */

r502f_status_t r502f_handshake(uint8_t *code);
r502f_status_t r502f_get_image(uint8_t *code);		/* code: 0x00 = có tay + đã chụp ảnh, 0x02 = không có tay */
r502f_status_t r502f_aura(uint8_t *code, uint8_t ctrl, uint8_t speed, uint8_t color, uint8_t times);

r502f_status_t r502f_generate_char(
    uint8_t buffer, uint8_t *code);

r502f_status_t r502f_generate_template(
    uint8_t *code);

r502f_status_t r502f_store_template(
    uint8_t buffer, uint16_t id, uint8_t *code);

r502f_status_t r502f_search(
    uint8_t buffer,
    uint16_t start_page,
    uint16_t page_count,
    uint8_t *code,
    uint16_t *id,
    uint16_t *score);

r502f_status_t r502f_delete_template(
    uint16_t id, uint16_t count, uint8_t *code);

r502f_status_t r502f_empty(uint8_t *code);

r502f_status_t r502f_read_index_table(
    uint8_t page, uint8_t table[32], uint8_t *code);

extern volatile uint8_t r502f_dbg_stage;
extern volatile uint8_t r502f_dbg_value;
extern volatile uint16_t r502f_dbg_len;
extern volatile uint8_t r502f_dbg_data_len;
extern volatile uint8_t r502f_dbg_header[7];

void r502f_led_wait(void);		/* LED chờ ngón tay theo FP_LED_WAIT_MODE */
void r502f_led_finger(void);	/* LED khi có ngón tay */

const char *r502f_status_name(r502f_status_t st);

/* Chương trình thử: gọi demo_init() một lần, demo_run() liên tục trong while(1) */
void r502f_run(void);


#endif /* R502F_H_ */
