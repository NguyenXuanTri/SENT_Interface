/*
 * r502f.c  -  Điều khiển cảm biến vân tay R502-F (bản kiểm tra giai đoạn 1)
 *
 * Gói lệnh   : EF 01 | FF FF FF FF | 01 | len(2B) | mã lệnh + tham số | checksum(2B)
 * Gói trả lời: EF 01 | FF FF FF FF | 07 | len(2B) | mã xác nhận + dữ liệu | checksum(2B)
 *   len      = số byte (dữ liệu + 2 byte checksum)
 *   checksum = tổng các byte từ "loại gói" đến hết dữ liệu
 *
 * Cấu trúc file (mỗi hàm làm đúng một việc, đọc từ dưới lên là từ chi tiết đến tổng quát):
 *   1. Log
 *   2. Gửi lệnh
 *   3. Nhận phản hồi
 *   4. Lệnh của cảm biến (handshake, get_image, aura) và LED
 *   5. Khởi tạo
 *   6. Chương trình thử (demo)
 */
#include "fp_sensor.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ---------------- Hằng số giao thức (từ driver_r502_commands.h) ---------------- */
#define R502F_HEADER_1			0xEFU
#define R502F_HEADER_2			0x01U
#define R502F_ADDR_BYTE			0xFFU		/* địa chỉ mặc định 0xFFFFFFFF */
#define R502F_PID_CMD			0x01U
#define R502F_PID_ACK			0x07U

#define R502F_CMD_GEN_CHAR       0x02U
#define R502F_CMD_SEARCH         0x04U
#define R502F_CMD_REG_MODEL      0x05U
#define R502F_CMD_STORE          0x06U
#define R502F_CMD_DELETE_CHAR    0x0CU
#define R502F_CMD_EMPTY          0x0DU
#define R502F_CMD_READ_INDEX     0x1FU
#define R502F_CMD_GET_IMAGE		0x01U
#define R502F_CMD_AURA			0x35U
#define R502F_CMD_HANDSHAKE		0x40U

#define R502F_CMD_MAX			8U			/* số byte lệnh tối đa (aura dùng 5) */
#define R502F_ACK_MAX			40U			/* số byte dữ liệu tối đa trong phản hồi */

/* ---------------- Thời gian chờ ---------------- */
#define R502F_TX_TIMEOUT_MS		100U
#define R502F_ACK_TIMEOUT_MS	2000U		/* tổng thời gian chờ thấy header của phản hồi */
#define R502F_BYTE_TIMEOUT_MS	50U			/* chờ mỗi byte sau khi đã thấy header */

/* ---------------- Biến toàn cục ---------------- */
r502f_debug_t g_r502f;

volatile uint8_t r502f_dbg_stage = 0U;
volatile uint8_t r502f_dbg_value = 0U;
volatile uint16_t r502f_dbg_len = 0U;
volatile uint8_t r502f_dbg_data_len = 0U;
volatile uint8_t r502f_dbg_header[7] = {0};

static UART_HandleTypeDef *log_uart = NULL;

/* ====================================================================== */
/*  1. LOG (chỉ gửi ra UART log, không bao giờ đụng vào UART cảm biến)      */
/* ====================================================================== */
/* In một dòng log kiểu printf, tự thêm \r\n */
static void fp_logf(const char *fmt, ...)
{
#if FP_LOG_ENABLE
	char buf[96];
	va_list ap;

	if (log_uart == NULL) return;

	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf) - 2U, fmt, ap);
	va_end(ap);
	if (n < 0) return;
	if (n > (int)sizeof(buf) - 3) n = (int)sizeof(buf) - 3;
	buf[n++] = '\r';
	buf[n++] = '\n';
	(void)HAL_UART_Transmit(log_uart, (uint8_t *)buf, (uint16_t)n, 100U);
#else
	(void)fmt;
#endif
}
/* Tên ngắn của một trạng thái, dùng khi in log */
const char *r502f_status_name(r502f_status_t st)
{
	switch (st) {
	case R502F_OK:				return "OK";
	case R502F_ERR_TX:			return "TX";
	case R502F_ERR_NO_RESPONSE:	return "NO_RESPONSE";
	case R502F_ERR_TIMEOUT:		return "TIMEOUT";
	case R502F_ERR_FRAME:		return "FRAME";
	case R502F_ERR_CHECKSUM:	return "CHECKSUM";
	default:					return "?";
	}
}

/* In lỗi giao tiếp theo đúng thông báo yêu cầu */
static void fp_log_comm_error(r502f_status_t st)
{
	switch (st) {
	case R502F_ERR_NO_RESPONSE:
		fp_logf("SENSOR_NO_RESPONSE");
		break;
	case R502F_ERR_TIMEOUT:
		fp_logf("UART_TIMEOUT");
		break;
	default:
		fp_logf("SENSOR_COMM_ERROR detail=%s", r502f_status_name(st));
		break;
	}
}

/* ====================================================================== */
/*  2. GỬI LỆNH                                                             */
/* ====================================================================== */

/* Tổng n byte bắt đầu từ p (dùng cho checksum) */
static uint16_t r502f_sum(const uint8_t *p, uint16_t n)
{
	uint16_t sum = 0U;

	for (uint16_t i = 0U; i < n; i++) sum = (uint16_t)(sum + p[i]);
	return sum;
}

/* Dựng gói lệnh vào pkt[], trả về số byte của gói */
static uint8_t r502f_build_packet(uint8_t *pkt, const uint8_t *cmd, uint8_t cmd_len)
{
	uint8_t n = 0U;
	uint16_t plen = (uint16_t)(cmd_len + 2U);

	pkt[n++] = R502F_HEADER_1;
	pkt[n++] = R502F_HEADER_2;
	pkt[n++] = R502F_ADDR_BYTE;
	pkt[n++] = R502F_ADDR_BYTE;
	pkt[n++] = R502F_ADDR_BYTE;
	pkt[n++] = R502F_ADDR_BYTE;
	pkt[n++] = R502F_PID_CMD;
	pkt[n++] = (uint8_t)(plen >> 8);
	pkt[n++] = (uint8_t)(plen & 0xFFU);
	for (uint8_t i = 0U; i < cmd_len; i++) pkt[n++] = cmd[i];

	uint16_t sum = r502f_sum(&pkt[6], (uint16_t)(n - 6U));	/* từ "loại gói" trở đi */
	pkt[n++] = (uint8_t)(sum >> 8);
	pkt[n++] = (uint8_t)(sum & 0xFFU);
	return n;
}

/* Gửi một lệnh. HAL_UART_Transmit chặn đến khi gửi xong nên dùng mảng cục bộ là an toàn. */
static r502f_status_t r502f_send_cmd(const uint8_t *cmd, uint8_t cmd_len)
{
	uint8_t pkt[9U + R502F_CMD_MAX + 2U];

	if (cmd == NULL || cmd_len == 0U || cmd_len > R502F_CMD_MAX) return R502F_ERR_TX;

	uint8_t n = r502f_build_packet(pkt, cmd, cmd_len);
	return (HAL_UART_Transmit(FP_UART_SENSOR, pkt, n, R502F_TX_TIMEOUT_MS) == HAL_OK)
			? R502F_OK : R502F_ERR_TX;
}

/* ====================================================================== */
/*  3. NHẬN PHẢN HỒI                                                        */
/* ====================================================================== */
/* Đọc 1 byte từ cảm biến, chờ tối đa timeout_ms. Trả 1 nếu có byte, 0 nếu hết giờ. */
static uint8_t r502f_rx_byte(uint8_t *b, uint32_t timeout_ms)
{
	return (HAL_UART_Receive(FP_UART_SENSOR, b, 1U, timeout_ms) == HAL_OK) ? 1U : 0U;
}

/* Xả các byte cũ còn trong bộ nhận để không lẫn với phản hồi mới */
static void r502f_flush_rx(void)
{
	uint8_t junk;
	uint8_t guard = 64U;

	while (guard-- > 0U && HAL_UART_Receive(FP_UART_SENSOR, &junk, 1U, 1U) == HAL_OK) {
	}
}

/* Đọc đúng n byte, mỗi byte chờ tối đa R502F_BYTE_TIMEOUT_MS. OK hoặc ERR_TIMEOUT. */
static r502f_status_t r502f_rx_bytes(uint8_t *buf, uint16_t n)
{
	for (uint16_t i = 0U; i < n; i++) {
		if (!r502f_rx_byte(&buf[i], R502F_BYTE_TIMEOUT_MS)) return R502F_ERR_TIMEOUT;
	}
	return R502F_OK;
}

/* Bước 1: tìm header EF 01, bỏ qua byte rác (ví dụ 0x55 khi cảm biến vừa khởi động).
 *   OK              : đã thấy header
 *   ERR_NO_RESPONSE : không nhận được byte nào
 *   ERR_FRAME       : có byte nhưng không có header */
static r502f_status_t r502f_wait_header(void)
{
    uint8_t b;
    uint8_t prev = 0U;
    uint8_t got_any = 0U;
    uint32_t t0 = HAL_GetTick();

    r502f_dbg_stage = 0U;

    while ((uint32_t)(HAL_GetTick() - t0) <
           R502F_ACK_TIMEOUT_MS)
    {
        if (!r502f_rx_byte(&b, 5U))
            continue;

        got_any = 1U;
        r502f_dbg_stage = 1U;
        r502f_dbg_value = b;

        if (prev == R502F_HEADER_1 &&
            b == R502F_HEADER_2)
        {
            r502f_dbg_stage = 2U;
            return R502F_OK;
        }

        prev = b;
    }

    if (got_any)
    {
        r502f_dbg_stage = 3U;
        return R502F_ERR_FRAME;
    }

    r502f_dbg_stage = 4U;
    return R502F_ERR_NO_RESPONSE;
}

/* Bước 2: đọc địa chỉ (4 byte) + loại gói (1) + độ dài (2), kiểm tra, trả độ dài qua *len */
static r502f_status_t r502f_read_header(uint16_t *len)
{
    uint8_t hdr[7];

    r502f_status_t st = r502f_rx_bytes(hdr, 7U);

    if (st != R502F_OK)
    {
        r502f_dbg_stage = 10U;
        return st;
    }

    for (uint8_t i = 0U; i < 7U; i++)
    {
        r502f_dbg_header[i] = hdr[i];
    }

    /* Kiểm tra địa chỉ */
    for (uint8_t i = 0U; i < 4U; i++)
    {
        if (hdr[i] != R502F_ADDR_BYTE)
        {
            r502f_dbg_stage = 20U;
            r502f_dbg_value = hdr[i];
            return R502F_ERR_FRAME;
        }
    }

    /* Kiểm tra PID của gói phản hồi */
    if (hdr[4] != R502F_PID_ACK)
    {
        r502f_dbg_stage = 21U;
        r502f_dbg_value = hdr[4];
        return R502F_ERR_FRAME;
    }

    /* Đọc trường độ dài */
    *len = ((uint16_t)hdr[5] << 8) | hdr[6];
    r502f_dbg_len = *len;

    if (*len < 3U || (*len - 2U) > R502F_ACK_MAX)
    {
        r502f_dbg_stage = 22U;
        return R502F_ERR_FRAME;
    }

    r502f_dbg_stage = 23U;
    return R502F_OK;
}

/* Bước 4: kiểm tra checksum. body[] gồm (len-2) byte dữ liệu rồi 2 byte checksum. */
static r502f_status_t r502f_check_ack_sum(uint16_t len, const uint8_t *body)
{
	uint16_t dlen = (uint16_t)(len - 2U);
	uint16_t sum = (uint16_t)(R502F_PID_ACK + (len >> 8) + (len & 0xFFU) + r502f_sum(body, dlen));
	uint16_t rx_sum = (uint16_t)(((uint16_t)body[dlen] << 8) | body[dlen + 1U]);

	return (sum == rx_sum) ? R502F_OK : R502F_ERR_CHECKSUM;
}

/* Đọc và kiểm tra một gói phản hồi. data[] nhận phần dữ liệu (byte đầu là mã xác nhận). */
static r502f_status_t r502f_read_ack(uint8_t *data, uint8_t *data_len)
{
	uint8_t body[R502F_ACK_MAX + 2U];
	uint16_t len = 0U;
	r502f_status_t st;

	st = r502f_wait_header();
	if (st != R502F_OK) return st;

	st = r502f_read_header(&len);
	if (st != R502F_OK) return st;

	st = r502f_rx_bytes(body, len);					/* dữ liệu + 2 byte checksum */
	if (st != R502F_OK) return st;

	st = r502f_check_ack_sum(len, body);
	if (st != R502F_OK) return st;

	memcpy(data, body, (size_t)(len - 2U));
	*data_len = (uint8_t)(len - 2U);
	return R502F_OK;
}

/* ====================================================================== */
/*  4. LỆNH CỦA CẢM BIẾN                                                    */
/* ====================================================================== */

/* Ghi kết quả lệnh vào g_r502f để xem bằng Live Expressions */
static void r502f_record_result(r502f_status_t st, const uint8_t *ack)
{
	g_r502f.last_status = (uint8_t)st;
	g_r502f.cnt[st]++;
	if (st == R502F_OK) g_r502f.last_code = ack[0];
}

/* Gửi 1 lệnh rồi chờ phản hồi. Chạy tuần tự nên không bao giờ có hai lệnh chồng nhau. */
static r502f_status_t r502f_command_data(const uint8_t *cmd, uint8_t cmd_len, uint8_t *data, uint8_t *data_len)
{
    uint8_t ack[R502F_ACK_MAX];
    uint8_t ack_len = 0U;

    r502f_status_t st;

    if (data_len != NULL)
        *data_len = 0U;

    r502f_flush_rx();

    st = r502f_send_cmd(cmd, cmd_len);

    if (st == R502F_OK)
        st = r502f_read_ack(ack, &ack_len);

    r502f_record_result(st, ack);

    if (st == R502F_OK)
    {
        if (data != NULL)
            memcpy(data, ack, ack_len);

        if (data_len != NULL)
            *data_len = ack_len;
    }

    return st;
}


static r502f_status_t r502f_command(const uint8_t *cmd, uint8_t cmd_len, uint8_t *code)
{
    uint8_t data[R502F_ACK_MAX];
    uint8_t data_len = 0U;

    r502f_status_t st;

    st = r502f_command_data(
        cmd, cmd_len, data, &data_len);

    if (st != R502F_OK)
        return st;

    if (data_len == 0U)
        return R502F_ERR_FRAME;

    if (code != NULL)
        *code = data[0];

    return R502F_OK;
}

r502f_status_t r502f_handshake(uint8_t *code)
{
	static const uint8_t cmd[1] = { R502F_CMD_HANDSHAKE };

	g_r502f.handshake_count++;
	return r502f_command(cmd, 1U, code);
}

r502f_status_t r502f_get_image(uint8_t *code)
{
	static const uint8_t cmd[1] = { R502F_CMD_GET_IMAGE };

	g_r502f.get_image_count++;
	return r502f_command(cmd, 1U, code);
}

r502f_status_t r502f_aura(uint8_t *code, uint8_t ctrl, uint8_t speed, uint8_t color, uint8_t times)
{
	const uint8_t cmd[5] = { R502F_CMD_AURA, ctrl, speed, color, times };

	return r502f_command(cmd, 5U, code);
}
/* Tạo đặc trưng từ ảnh vân tay vào buffer 1 hoặc 2 */
r502f_status_t r502f_generate_char(uint8_t buffer, uint8_t *code)
{
    if (buffer != 1U && buffer != 2U)
        return R502F_ERR_FRAME;

    const uint8_t cmd[2] = {
        R502F_CMD_GEN_CHAR, buffer
    };

    return r502f_command(cmd, 2U, code);
}

/* Ghép đặc trưng của hai lần quét thành một mẫu vân tay */
r502f_status_t r502f_generate_template(uint8_t *code)
{
    const uint8_t cmd[1] = { R502F_CMD_REG_MODEL };

    return r502f_command(cmd, 1U, code);
}

/* Lưu mẫu vào ID */
r502f_status_t r502f_store_template(uint8_t buffer, uint16_t id, uint8_t *code)
{
    const uint8_t cmd[4] = {
        R502F_CMD_STORE,
        buffer,
        (uint8_t)(id >> 8),
        (uint8_t)(id & 0xFFU)
    };

    return r502f_command(cmd, 4U, code);
}

/* Tìm mẫu trong cơ sở dữ liệu */
r502f_status_t r502f_search(uint8_t buffer, uint16_t start_page, uint16_t page_count, uint8_t *code, uint16_t *id, uint16_t *score)
{
    const uint8_t cmd[6] = {
        R502F_CMD_SEARCH,
        buffer,
        (uint8_t)(start_page >> 8),
        (uint8_t)(start_page & 0xFFU),
        (uint8_t)(page_count >> 8),
        (uint8_t)(page_count & 0xFFU)
    };

    uint8_t data[R502F_ACK_MAX];
    uint8_t data_len = 0U;

    r502f_status_t st = r502f_command_data(
        cmd, 6U, data, &data_len);

    if (st != R502F_OK)
        return st;

    if (data_len < 1U)
        return R502F_ERR_FRAME;

    if (code != NULL)
        *code = data[0];

    /* 0x09 là không tìm thấy mẫu */
    if (data[0] == R502F_CODE_NOT_FOUND)
        return R502F_OK;

    if (data[0] != R502F_CODE_OK)
        return R502F_OK;

    if (data_len < 5U)
        return R502F_ERR_FRAME;

    if (id != NULL)
    {
        *id = ((uint16_t)data[1] << 8) | data[2];
    }

    if (score != NULL)
    {
        *score = ((uint16_t)data[3] << 8) | data[4];
    }

    return R502F_OK;
}


/* Xóa một hoặc nhiều ID liên tiếp */
r502f_status_t r502f_delete_template(uint16_t id, uint16_t count, uint8_t *code)
{
    const uint8_t cmd[5] = {
        R502F_CMD_DELETE_CHAR,
        (uint8_t)(id >> 8),
        (uint8_t)(id & 0xFFU),
        (uint8_t)(count >> 8),
        (uint8_t)(count & 0xFFU)
    };

    if (count == 0U)
        return R502F_ERR_FRAME;

    return r502f_command(cmd, 5U, code);
}


/* Xóa toàn bộ mẫu trong cảm biến */
r502f_status_t r502f_empty(uint8_t *code)
{
    const uint8_t cmd[1] = { R502F_CMD_EMPTY };

    return r502f_command(cmd, 1U, code);
}


/* Đọc bảng bit của 256 ID */
r502f_status_t r502f_read_index_table(uint8_t page,uint8_t table[32],uint8_t *code)
{
    const uint8_t cmd[2] = {
        R502F_CMD_READ_INDEX, page
    };

    uint8_t data[R502F_ACK_MAX];
    uint8_t data_len = 0U;

    if (table == NULL)
        return R502F_ERR_FRAME;

    r502f_status_t st = r502f_command_data(
        cmd, 2U, data, &data_len);

    r502f_dbg_data_len = data_len;

    if (st != R502F_OK)
        return st;

    if (data_len < 1U)
    {
        r502f_dbg_stage = 30U;
        return R502F_ERR_FRAME;
    }

    if (code != NULL)
        *code = data[0];

    if (data[0] != R502F_CODE_OK)
        return R502F_OK;

    if (data_len < 33U)
    {
        r502f_dbg_stage = 31U;
        return R502F_ERR_FRAME;
    }

    memcpy(table, &data[1], 32U);

    return R502F_OK;
}
/* Gửi lệnh LED và báo kết quả bằng log (không dừng chương trình nếu lỗi) */
static void r502f_led_set(const char *name, uint8_t ctrl, uint8_t speed, uint8_t color, uint8_t times)
{
	uint8_t code = 0xFFU;
	r502f_status_t st = r502f_aura(&code, ctrl, speed, color, times);

	if (st != R502F_OK) {
		fp_logf("LED_ERROR %s status=%s", name, r502f_status_name(st));
	} else if (code != R502F_CODE_OK) {
		fp_logf("LED_ERROR %s code=0x%02X", name, (unsigned)code);
	}
}

/* LED khi chờ ngón tay, theo FP_LED_WAIT_MODE */
void r502f_led_wait(void)
{
#if FP_LED_WAIT_MODE == 1
	/* Chế độ 1: xanh dương sáng liên tục */
	r502f_led_set("WAIT_BLUE_ON", R502F_AURA_ALWAYS_ON, 0x00U, R502F_COLOR_BLUE, 0x00U);
#elif FP_LED_WAIT_MODE == 2
	/* Chế độ 2: tím nhấp nháy */
	r502f_led_set("WAIT_PURPLE_FLASH", R502F_AURA_FLASHING, FP_LED_FLASH_SPEED, R502F_COLOR_PURPLE, FP_LED_FLASH_TIMES);
#else
#error "FP_LED_WAIT_MODE phai bang 1 hoac 2"
#endif
}

/* LED khi có ngón tay */
void r502f_led_finger(void)
{
#if FP_LED_FINGER_GREEN
	r502f_led_set("", R502F_AURA_ALWAYS_ON, 0x00U, R502F_COLOR_GREEN, 0x00U);
#else
	r502f_led_set("", R502F_AURA_ALWAYS_OFF, 0x00U, R502F_COLOR_GREEN, 0x00U);
#endif
}

/* ====================================================================== */
/*  5. KHỞI TẠO                                                             */
/* ====================================================================== */

/* Xóa biến debug về trạng thái ban đầu */
static void r502f_reset_debug(void)
{
	memset(&g_r502f, 0, sizeof(g_r502f));
	g_r502f.finger_state = 2U;
	g_r502f.last_code = 0xFFU;
}

/* Chọn UART log. Chỉ bật log khi UART log khác UART cảm biến. */
static void r502f_setup_log(void)
{
	log_uart = NULL;
#if FP_LOG_ENABLE
	if (FP_UART_LOG == FP_UART_SENSOR) g_r502f.log_same_uart = 1U;
	else log_uart = FP_UART_LOG;
#endif
}

/* Hủy việc nhận bằng ngắt còn sót từ code cũ, vì ta dùng kiểu polling */
static void r502f_prepare_uart(void)
{
	if (FP_UART_SENSOR->RxState != HAL_UART_STATE_READY) (void)HAL_UART_AbortReceive(FP_UART_SENSOR);
}

/* Cấp nguồn chính cho R502-F bằng chân PCTRL rồi chờ cảm biến khởi động */
static void r502f_power_on(void)
{
#if FP_USE_PCTRL
	GPIO_InitTypeDef gpio = {0};

	FP_PCTRL_CLK_ENABLE();
	gpio.Pin = FP_PCTRL_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(P_CTRL_GPIO_Port, &gpio);
	HAL_GPIO_WritePin(P_CTRL_GPIO_Port, P_CTRL_Pin, GPIO_PIN_SET);
#endif
	HAL_Delay(FP_POWER_ON_DELAY_MS);
}

//void r502f_init(void)
//{
//	r502f_reset_debug();
//	r502f_setup_log();
//	r502f_prepare_uart();
//	r502f_power_on();
//}

/* ====================================================================== */
/*  6. CHƯƠNG TRÌNH THỬ                                                      */
/* ====================================================================== */
static uint8_t  demo_first_try = 1U;
static uint8_t  demo_err_count = 0U;			/* số lần lỗi liên tiếp khi hỏi GetImage */
static uint32_t demo_last_try = 0U;				/* lần handshake gần nhất */
static uint32_t demo_last_poll = 0U;			/* lần GetImage gần nhất */
static uint32_t demo_last_nofinger_log = 0U;	/* lần báo NO_FINGER gần nhất */
static uint32_t demo_last_err_log = 0U;			/* lần báo lỗi gần nhất */
static uint8_t  demo_last_err_kind = 0xFFU;		/* loại lỗi gần nhất (để không in lặp) */

/* Đặt lại các biến trạng thái của demo */
static void r502f_reset_state(void)
{
	demo_first_try = 1U;
	demo_err_count = 0U;
	demo_last_err_kind = 0xFFU;
}

/* In thông tin cấu hình lúc khởi động */
static void r502f_log_startup(void)
{
	fp_logf("SYSTEM_START");
	fp_logf("INFO sensor_baud=%lu expected=%lu%s",
			(unsigned long)FP_UART_SENSOR->Init.BaudRate, (unsigned long)FP_EXPECTED_BAUD,
			(FP_UART_SENSOR->Init.BaudRate == FP_EXPECTED_BAUD) ? "" : " (KHAC!)");
	fp_logf("INFO led_wait_mode=%d pctrl=%d", FP_LED_WAIT_MODE, FP_USE_PCTRL);
}

void r502f_init(void)
{
	r502f_reset_debug();
	r502f_setup_log();
	r502f_prepare_uart();
	r502f_power_on();
	r502f_reset_state();
	r502f_log_startup();
}

/* Có nên in thông báo lỗi lúc này không? In khi loại lỗi đổi, hoặc đã qua FP_ERR_LOG_MS. */
static uint8_t r502f_error_due(uint8_t kind, uint32_t now)
{
	if (kind == demo_last_err_kind && (uint32_t)(now - demo_last_err_log) < FP_ERR_LOG_MS) return 0U;

	demo_last_err_kind = kind;
	demo_last_err_log = now;
	return 1U;
}

/* ---- Bước 1: handshake cho đến khi cảm biến trả lời đúng ---- */
static void r502f_step_handshake(uint32_t now)
{
	uint8_t code = 0xFFU;
	r502f_status_t st;

	if (!demo_first_try && (uint32_t)(now - demo_last_try) < FP_HANDSHAKE_RETRY_MS) return;
	demo_first_try = 0U;
	demo_last_try = now;

	st = r502f_handshake(&code);
	if (st == R502F_OK && code == R502F_CODE_OK) {
		g_r502f.sensor_ready = 1U;
		g_r502f.finger_state = 2U;
		demo_err_count = 0U;
		demo_last_err_kind = 0xFFU;
		fp_logf("SENSOR_READY");
	} else if (st == R502F_OK) {
		fp_logf("SENSOR_COMM_ERROR handshake_code=0x%02X", (unsigned)code);
	} else if (r502f_error_due((uint8_t)st, now)) {
		fp_log_comm_error(st);
	}
}

/* ---- Bước 2: các tình huống khi hỏi GetImage ---- */

/* Lỗi giao tiếp: báo (có giãn cách), lỗi liên tiếp nhiều lần thì quay lại handshake */
static void r502f_handle_comm_error(r502f_status_t st, uint32_t now)
{
	if (r502f_error_due((uint8_t)st, now)) fp_log_comm_error(st);

	if (++demo_err_count >= FP_ERR_BEFORE_RESET) {
		g_r502f.sensor_ready = 0U;
		g_r502f.finger_state = 2U;
	}
}

/* Cảm biến báo không có ngón tay. Chỉ đổi LED khi trạng thái thay đổi. */
static void r502f_handle_no_finger(uint32_t now)
{
	if (g_r502f.finger_state != 0U) {
		g_r502f.finger_state = 0U;
		fp_logf("NO_FINGER");
		demo_last_nofinger_log = now;
		r502f_led_wait();
	} else if ((uint32_t)(now - demo_last_nofinger_log) >= FP_NO_FINGER_LOG_MS) {
		demo_last_nofinger_log = now;
		fp_logf("NO_FINGER");
	}
}

/* Cảm biến báo đã đọc được ảnh vân tay. Chỉ báo và đổi LED một lần cho mỗi lần đặt tay. */
static void r502f_handle_finger(void)
{
	if (g_r502f.finger_state == 1U) return;

	g_r502f.finger_state = 1U;
	fp_logf("FINGER_DETECTED");
	fp_logf("IMAGE_CAPTURED");
	r502f_led_finger();
}

/* Mã lạ (ví dụ 0x01 lỗi gói, 0x03 chụp ảnh thất bại): báo ra để biết cảm biến trả gì */
static void r502f_handle_other_code(uint8_t code, uint32_t now)
{
	if (r502f_error_due(code, now)) fp_logf("IMAGE_FAIL code=0x%02X", (unsigned)code);
}

/* Hỏi GetImage mỗi FP_POLL_MS rồi xử lý kết quả */
static void r502f_step_poll(uint32_t now)
{
	uint8_t code = 0xFFU;
	r502f_status_t st;

	if ((uint32_t)(now - demo_last_poll) < FP_POLL_MS) return;
	demo_last_poll = now;

	st = r502f_get_image(&code);
	if (st != R502F_OK) {
		r502f_handle_comm_error(st, now);
		return;
	}
	demo_err_count = 0U;

	if (code == R502F_CODE_NO_FINGER) r502f_handle_no_finger(now);
	else if (code == R502F_CODE_OK)   r502f_handle_finger();
	else                              r502f_handle_other_code(code, now);
}

/* Gọi liên tục trong while(1) */
void r502f_run(void)
{
	uint32_t now = HAL_GetTick();

	if (!g_r502f.sensor_ready) r502f_step_handshake(now);
	else                       r502f_step_poll(now);
}
