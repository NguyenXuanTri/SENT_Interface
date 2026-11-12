/*
Giao tiếp lệnh qua UART với cảm biến vân tay R502F. 
Nó nhận lệnh dạng văn bản, điều khiển cảm biến và gửi kết quả/trạng thái trở lại.

Các lệnh chính gồm:

- PING: kiểm tra kết nối cảm biến.
- VERIFY: quét và xác minh vân tay đã lưu.
- ENROLL: đăng ký vân tay mới; yêu cầu đặt ngón tay, nhấc ra rồi đặt lại.
- DELETE:<id>, EMPTY, SHOW: xóa một mẫu, xóa toàn bộ mẫu, hoặc liệt kê ID đã lưu.
- LED:BLUE, LED:WHITE, LED:OFF: điều khiển đèn cảm biến.
- CANCEL: hủy thao tác đang chạy.

File cũng quản lý các bước quét, timeout, lỗi 
và phản hồi như VERIFY_OK, ENROLL_OK hoặc NO_FINGER.
*/
#include "fp_ttl_cmd.h"
#include "fp_sensor.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;

#define FP_UART_TTL             	(&huart2)
#define FP_MAX_ID               	200U
#define FP_LINE_SIZE            	40U

#define FP_ERROR_HOLD_MS    		3000U
#define FP_SUCCESS_HOLD_MS    		3000U
#define FP_VERIFY_TIMEOUT_MS    	10000U
#define FP_ENROLL_STEP_TIMEOUT  	15000U
#define FP_ENROLL_REMOVE_TIMEOUT    15000U
#define FP_ENROLL_PLACE_TIMEOUT     15000U
typedef enum
{
    OP_IDLE = 0,
    OP_VERIFY,
    OP_ENROLL_FIRST,
    OP_ENROLL_REMOVE,
    OP_ENROLL_SECOND,
	OP_ERROR_WAIT,
	OP_SUCCESS_WAIT
} fp_operation_t;

static UART_HandleTypeDef *ttl_uart;

static char line[FP_LINE_SIZE];
static uint8_t line_len;
static uint8_t line_overflow;

static fp_operation_t operation = OP_IDLE;

static uint16_t enroll_id;
static uint32_t operation_tick;
static uint32_t poll_tick;

static uint8_t no_finger_reported;
static uint8_t operation_error_count;
static uint32_t error_hold_tick = 0U;

/* ---------- UART TTL output ---------- */

static void TTL_Send(const char *s)
{
    if (ttl_uart == NULL || s == NULL)
        return;

    (void)HAL_UART_Transmit(
        ttl_uart,
        (uint8_t *)s,
        (uint16_t)strlen(s),
        300U
    );
}

static void TTL_Sendf(const char *fmt, ...)
{
    char buf[96];
    va_list args;

    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0 && n < (int)sizeof(buf))
        TTL_Send(buf);
}


/* ---------- Initialization ---------- */

uint8_t FP_TTL_Init(void)
{
    ttl_uart = FP_UART_TTL;

    /* UART cảm biến và UART TTL phải khác nhau */
    if (ttl_uart == FP_UART_SENSOR)
        return 1U;

    line_len = 0U;
    line_overflow = 0U;
    operation = OP_IDLE;

    TTL_Send("FP_TTL_READY\r\n");
    return 0U;
}


/* ---------- Helpers ---------- */
static void TTL_SetLED(uint8_t control, uint8_t speed, uint8_t color, uint8_t times)
{
    uint8_t code = 0xFFU;

    r502f_status_t st =
        r502f_aura(&code, control, speed, color, times);

    if (st != R502F_OK)
    {
        TTL_Sendf("LED_ERROR status=%s\r\n",
                  r502f_status_name(st));
    }
    else if (code != R502F_CODE_OK)
    {
        TTL_Sendf("LED_ERROR code=0x%02X\r\n", code);
    }

}

/* Tìm ID trống đầu tiên trong phạm vi 0..FP_MAX_ID */
static uint8_t TTL_FindFreeID(uint16_t *free_id)
{
    uint8_t table[32];
    uint8_t code = 0xFFU;

    r502f_status_t st =
        r502f_read_index_table(0U, table, &code);

    if (st != R502F_OK || code != R502F_CODE_OK)
        return 0U;

    for (uint16_t id = 0U; id <= FP_MAX_ID; id++)
    {
        uint8_t byte_index = (uint8_t)(id / 8U);
        uint8_t bit_index = (uint8_t)(id % 8U);

        /* Bit 1: ID đã có mẫu; bit 0: ID còn trống */
        if ((table[byte_index] &
             (uint8_t)(1U << bit_index)) == 0U)
        {
            *free_id = id;
            return 1U;
        }
    }

    return 0U;
}

static uint8_t TTL_ParseDelete(const char *cmd, uint16_t *id)
{
    if (strncmp(cmd, "DELETE:", 7U) != 0)
        return 0U;

    const char *p = cmd + 7U;
    uint32_t value = 0U;

    if (*p == '\0')
        return 0U;

    while (*p != '\0')
    {
        if (*p < '0' || *p > '9')
            return 0U;

        value = value * 10U + (uint32_t)(*p - '0');

        if (value > FP_MAX_ID)
            return 0U;

        p++;
    }

    *id = (uint16_t)value;
    return 1U;
}

static uint8_t TTL_CommandResult(r502f_status_t st, uint8_t code)
{
    if (st != R502F_OK)
    {
        TTL_Sendf("SENSOR_COMM_ERROR status=%s\r\n",
                  r502f_status_name(st));
        return 0U;
    }

    if (code != R502F_CODE_OK)
    {
        TTL_Sendf("SENSOR_COMMAND_FAILED code=0x%02X\r\n",
                  code);
        return 0U;
    }

    return 1U;
}

static void TTL_StopOperation(void)
{
    operation = OP_IDLE;
    operation_error_count = 0U;
    no_finger_reported = 0U;

	/* Đưa LED về trạng thái chờ */
	TTL_SetLED(
		R502F_AURA_ALWAYS_OFF,
		0U,
		R502F_COLOR_BLUE,
		0U
	);
}

static void TTL_StartErrorWait(const char *message)
{
	uint8_t code = 0xFFU;
    TTL_Send(message);

    r502f_aura(&code,
			   R502F_AURA_ALWAYS_ON,
			   0U,
			   R502F_COLOR_RED,
			   0U);

    error_hold_tick = HAL_GetTick();
    operation = OP_ERROR_WAIT;

}

static void TTL_StartSuccessWait(const char *message)
{
	uint8_t code = 0xFFU;

    TTL_Send(message);

    r502f_aura(&code,
                   R502F_AURA_ALWAYS_ON,
                   0U,
                   R502F_COLOR_GREEN,
                   0U);

    error_hold_tick = HAL_GetTick();
    operation = OP_ERROR_WAIT;
}

/* ---------- Command dispatch ---------- */
static void TTL_Dispatch(const char *cmd)
{
    uint8_t code = 0xFFU;
    uint16_t id = 0U;
    r502f_status_t st;

    /* Đang chạy thao tác thì chỉ chấp nhận CANCEL */
    if (operation != OP_IDLE)
    {
		if (operation != OP_IDLE && strcmp(cmd, "CANCEL") == 0)
		{
			TTL_StopOperation();
			TTL_Send("CANCEL_OK\r\n");
		}else{
			TTL_Send("BUSY:OPERATION_RUNNING\r\n");
		}
			return;
    }

    if (strcmp(cmd, "CANCEL") == 0)
        {
            operation = OP_IDLE;

            TTL_SetLED(
                R502F_AURA_ALWAYS_OFF,
                0U,
                R502F_COLOR_BLUE,
                0U
            );

            TTL_Send("CANCEL_OK\r\n");
        }
    else if (strcmp(cmd, "PING") == 0)
    {
        st = r502f_handshake(&code);

        if (TTL_CommandResult(st, code))
            TTL_Send("SENSOR_READY\r\n");
    }

    else if (strcmp(cmd, "VERIFY") == 0)
    {
        st = r502f_handshake(&code);

        if (!TTL_CommandResult(st, code))
            return;

        operation = OP_VERIFY;
        operation_tick = HAL_GetTick();
        poll_tick = operation_tick - FP_POLL_MS;
        no_finger_reported = 0U;
        operation_error_count = 0U;

        r502f_led_wait();
        TTL_Send("\nVERIFY_STARTED\r\n");
    }

    else if (strcmp(cmd, "ENROLL") == 0)
    {
        if (!TTL_FindFreeID(&enroll_id))
        {
            TTL_Send("ENROLL_NO_FREE_ID_OR_SENSOR_ERROR\r\n");
            return;
        }

        operation = OP_ENROLL_FIRST;
        operation_tick = HAL_GetTick();
        poll_tick = operation_tick - FP_POLL_MS;
        no_finger_reported = 0U;
        operation_error_count = 0U;

        r502f_led_wait();
        TTL_Sendf("\nENROLL_STARTED id=%u\r\n",
                  (unsigned)enroll_id);
        TTL_Send("ENROLL_PLACE_FINGER\r\n");
    }

    else if (strncmp(cmd, "DELETE:", 7U) == 0)
    {
        if (!TTL_ParseDelete(cmd, &id))
        {
            TTL_Send("ERR_INVALID_DELETE_ID\r\n");
            return;
        }

        st = r502f_delete_template(id, 1U, &code);

        if (TTL_CommandResult(st, code))
            TTL_Sendf("\nDELETE_OK id=%u\r\n", (unsigned)id);
    }

    else if (strcmp(cmd, "EMPTY") == 0)
    {
        st = r502f_empty(&code);

        if (TTL_CommandResult(st, code))
            TTL_Send("\nEMPTY_OK\r\n");
    }

    else if (strcmp(cmd, "SHOW") == 0)
    {
        uint8_t table[32];

        st = r502f_read_index_table(0U, table, &code);

        if (!TTL_CommandResult(st, code))
            return;

        TTL_Send("\nSHOW_IDS:");

        uint8_t found = 0U;

        for (uint16_t value = 0U;
             value <= FP_MAX_ID;
             value++)
        {
            uint8_t byte_index = (uint8_t)(value / 8U);
            uint8_t bit_index = (uint8_t)(value % 8U);

            if (table[byte_index] &
                (uint8_t)(1U << bit_index))
            {
                TTL_Sendf("%s%u",
                          found ? "," : "",
                          (unsigned)value);
                found = 1U;
            }
        }

        if (!found)
            TTL_Send("NONE");

        TTL_Send("\r\n");
    }


    else if (strcmp(cmd, "LED:BLUE") == 0)
    {
        TTL_SetLED(
            R502F_AURA_ALWAYS_ON,
            0U,
            R502F_COLOR_BLUE,
            0U
        );
    }
    else if (strcmp(cmd, "LED:WHITE") == 0)
        {
            TTL_SetLED(
                R502F_AURA_ALWAYS_ON,
                0U,
                R502F_COLOR_WHITE,
                0U
            );
        }

    else if (strcmp(cmd, "LED:OFF") == 0)
    {
        TTL_SetLED(
            R502F_AURA_ALWAYS_OFF,
            0U,
            R502F_COLOR_BLUE,
            0U
        );
    }

    else
    {
        TTL_Send("ERR_INVALID_COMMAND\r\n");
    }
}

/* ---------- Verification and enrollment ---------- */
static void TTL_HandleImageError(r502f_status_t st)
{
    operation_error_count++;

    TTL_Sendf("SENSOR_COMM_ERROR status=%s\r\n",
              r502f_status_name(st));

    if (operation_error_count >= 3U)
    {
        TTL_StopOperation();
        TTL_Send("OPERATION_ERROR\r\n");
    }
}

static void TTL_VerifyFinish(void)
{
    uint8_t code = 0xFFU;
    uint16_t id = 0U;
    uint16_t score = 0U;
    r502f_status_t st;

    /* Bước 1: Tạo đặc trưng vân tay */
    TTL_Send("STEP: GENERATE_CHAR\r\n");

    st = r502f_generate_char(1U, &code);

    if (!TTL_CommandResult(st, code))
    {
        TTL_StartErrorWait("VERIFY_ERROR\r\n");
        return;
    }

    /* Bước 2: Tìm kiếm vân tay trong bộ nhớ */
    code = 0xFFU;

    st = r502f_search(
        1U,
        0U,
        (uint16_t)(FP_MAX_ID + 1U),
        &code,
        &id,
        &score
    );

    if (st != R502F_OK)
    {
        TTL_StartErrorWait("VERIFY_COMM_ERROR\r\n");
        return;
    }

    /* Bước 3: Xử lý kết quả */
    if (code == R502F_CODE_OK)
    {
        TTL_Sendf(
            "VERIFY_OK id=%u score=%u\r\n",
            (unsigned)id,
            (unsigned)score
        );

        TTL_StartSuccessWait("VERIFY_SUCCESS\r\n");
    }
    else if (code == R502F_CODE_NOT_FOUND)
    {
        TTL_StartErrorWait("VERIFY_NOT_FOUND\r\n");
    }
    else
    {
        TTL_StartErrorWait("VERIFY_FAILED\r\n");
    }
}

static void TTL_EnrollFinish(void)
{
    uint8_t code = 0xFFU;
    r502f_status_t st;

    /* Bước 1: Tạo đặc trưng từ ảnh vân tay lần 2 */
    TTL_Send("STEP: ENROLL_GENERATE_CHAR_2\r\n");

    st = r502f_generate_char(2U, &code);

    TTL_Sendf("GEN_CHAR_2 result=%s code=0x%02X\r\n",
              r502f_status_name(st), code);

    if (st != R502F_OK || code != R502F_CODE_OK)
    {
        TTL_StartErrorWait("ENROLL_FAILED\r\n");
        return;
    }

    /* Bước 2: Ghép hai đặc trưng thành mẫu vân tay */
    TTL_Send("STEP: ENROLL_GENERATE_TEMPLATE\r\n");

    code = 0xFFU;
    st = r502f_generate_template(&code);

    TTL_Sendf("GEN_TEMPLATE result=%s code=0x%02X\r\n",
              r502f_status_name(st), code);

    if (st != R502F_OK || code != R502F_CODE_OK)
    {
        TTL_StartErrorWait("ENROLL_FAILED:GENERATE_TEMPLATE\r\n");
        return;
    }

    /* Bước 3: Lưu mẫu vào ID đã chọn */
    TTL_Sendf("STEP: ENROLL_STORE id=%u\r\n",
              (unsigned)enroll_id);

    code = 0xFFU;
    st = r502f_store_template(1U, enroll_id, &code);

    TTL_Sendf("STORE result=%s code=0x%02X\r\n",
              r502f_status_name(st), code);

    if (st != R502F_OK || code != R502F_CODE_OK)
    {
    	TTL_StartErrorWait("ENROLL_FAILED:STORE\r\n");
        return;
    }

    /* Bước 4: Đăng ký thành công, giữ LED xanh 3 giây */
    TTL_Sendf("ENROLL_OK id=%u\r\n", (unsigned)enroll_id);

    TTL_StartSuccessWait("ENROLL_COMPLETE\r\n");
}
/* Chỉ chạy khi đang có một phiên VERIFY hoặc ENROLL */

static void TTL_OperationProcess(void)
{

    if (operation == OP_IDLE)
        return;

    if (operation == OP_ERROR_WAIT)
    {
    	if ((uint32_t)(HAL_GetTick() - error_hold_tick) >= FP_ERROR_HOLD_MS)
        {
    		 uint8_t code = 0xFFU;

    		    r502f_aura(&code,
    		               R502F_AURA_ALWAYS_OFF,
    		               0U,
    		               R502F_COLOR_RED,
    		               0U);
    		    TTL_StopOperation();
        }
        return;
    }
    uint32_t now = HAL_GetTick();
    uint32_t timeout;

    /* Timeout riêng cho từng thao tác/bước ENROLL */
    switch (operation)
    {
        case OP_VERIFY:
            timeout = FP_VERIFY_TIMEOUT_MS;
            break;

        case OP_ENROLL_REMOVE:
            timeout = FP_ENROLL_REMOVE_TIMEOUT;
            break;

        case OP_ENROLL_SECOND:
            timeout = FP_ENROLL_PLACE_TIMEOUT;
            break;

        case OP_ENROLL_FIRST:
        default:
            timeout = FP_ENROLL_STEP_TIMEOUT;
            break;
    }

    if ((uint32_t)(now - operation_tick) >= timeout)
    {
        if (operation == OP_VERIFY)
            TTL_Send("VERIFY_TIMEOUT\r\n");
        else if (operation == OP_ENROLL_REMOVE)
            TTL_Send("ENROLL_REMOVE_TIMEOUT\r\n");
        else if (operation == OP_ENROLL_SECOND)
            TTL_Send("ENROLL_PLACE_TIMEOUT\r\n");
        else
            TTL_Send("ENROLL_TIMEOUT\r\n");

        TTL_StopOperation();
        r502f_led_wait();
        return;
    }

    if ((uint32_t)(now - poll_tick) < FP_POLL_MS)
        return;

    poll_tick = now;

    uint8_t code = 0xFFU;
    r502f_status_t st = r502f_get_image(&code);

    if (st != R502F_OK)
    {
        TTL_HandleImageError(st);
        return;
    }

    operation_error_count = 0U;

    /* ---------- VERIFY ---------- */
    if (operation == OP_VERIFY)
    {
        if (code == R502F_CODE_NO_FINGER)
        {
            if (!no_finger_reported)
            {
                TTL_Send("NO_FINGER\r\n");
                no_finger_reported = 1U;
            }
            return;
        }

        if (code == R502F_CODE_OK)
            {
                uint8_t led_code = 0xFFU;

                r502f_aura(
                    &led_code,
                    R502F_AURA_ALWAYS_ON,
                    0U,
                    R502F_COLOR_BLUE,
                    0U
                );

                TTL_Send("FINGER_DETECTED\r\n");

                TTL_VerifyFinish();
                return;
            }
    }

    /* ---------- ENROLL: lần quét đầu ---------- */
    if (operation == OP_ENROLL_FIRST)
    {
        if (code == R502F_CODE_NO_FINGER)
            return;

        if (code != R502F_CODE_OK)
        {
            TTL_Sendf("ENROLL_IMAGE_ERROR code=0x%02X\r\n",
                      code);
            return;
        }

        st = r502f_generate_char(1U, &code);

        if (!TTL_CommandResult(st, code))
        {
            TTL_SetLED(R502F_AURA_ALWAYS_ON, 0U,
                       R502F_COLOR_RED, 0U);
            TTL_StopOperation();
            return;
        }

        operation = OP_ENROLL_REMOVE;
        operation_tick = HAL_GetTick();
        poll_tick = operation_tick - FP_POLL_MS;

        r502f_led_finger();
        TTL_Send("ENROLL_LIFT_FINGER\r\n");
        return;
    }

    /* ---------- ENROLL: chờ nhấc tay ---------- */
    if (operation == OP_ENROLL_REMOVE)
    {
        if (code == R502F_CODE_NO_FINGER)
        {
            operation = OP_ENROLL_SECOND;
            operation_tick = HAL_GetTick();
            poll_tick = operation_tick - FP_POLL_MS;

            no_finger_reported = 0U;
            r502f_led_wait();

            TTL_Send("ENROLL_PLACE_AGAIN\r\n");
        }
        return;
    }

    /* ---------- ENROLL: lần quét thứ hai ---------- */
    if (operation == OP_ENROLL_SECOND)
    {
        if (code == R502F_CODE_NO_FINGER)
            return;

        if (code != R502F_CODE_OK)
        {
            TTL_Sendf("ENROLL_IMAGE_ERROR code=0x%02X\r\n",
                      code);
            return;
        }

        TTL_EnrollFinish();
    }
}

/* ---------- Main polling function ---------- */
void FP_TTL_Process(void)
{
    uint8_t byte;

    /*
     * Không dùng UART callback.
     * Timeout 0: chỉ đọc nếu byte đã sẵn sàng.
     */
    if (HAL_UART_Receive(ttl_uart, &byte, 1U, 0U) == HAL_OK)
    {
        if (byte == '\r' || byte == '\n')
        {
            if (line_overflow)
            {
                TTL_Send("ERR_LINE_TOO_LONG\r\n");
            }
            else if (line_len > 0U)
            {
                line[line_len] = '\0';
                TTL_Dispatch(line);
            }

            line_len = 0U;
            line_overflow = 0U;
        }
        else if (!line_overflow)
        {
            if (line_len < FP_LINE_SIZE - 1U)
            {
                line[line_len++] = (char)byte;
            }
            else
            {
                line_overflow = 1U;
            }
        }
    }

    /* Không đọc cảm biến nếu không có lệnh đang hoạt động */
    TTL_OperationProcess();
}
