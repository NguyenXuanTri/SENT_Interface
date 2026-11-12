/*
cung cấp các hàm tiện ích để gửi dữ liệu qua UART đã dành cho cảm biến:

- SensorUART_Init(...): lưu UART handle để các hàm khác dùng.
- SensorUART_Send(...): gửi chuỗi ký tự qua UART.
- SensorUART_SendBytes(...): gửi một vùng dữ liệu byte có độ dài chỉ định.

Các hàm gửi trả về trạng thái HAL để bên gọi biết gửi thành công hay thất bại. 
Nếu UART chưa được khởi tạo hoặc tham số không hợp lệ, hàm trả về HAL_ERROR.
*/

#include "sensor_uart.h"
#include <string.h>

static UART_HandleTypeDef *sensor_uart = NULL;

void SensorUART_Init(UART_HandleTypeDef *huart)
{
    sensor_uart = huart;
}

HAL_StatusTypeDef SensorUART_Send(const char *data)
{
    if (sensor_uart == NULL || data == NULL)
        return HAL_ERROR;

    uint16_t len = (uint16_t)strlen(data);

    if (len == 0U)
        return HAL_OK;

    return HAL_UART_Transmit(sensor_uart,
                             (uint8_t *)data,
                             len,
                             100U);
}

HAL_StatusTypeDef SensorUART_SendBytes(uint8_t *data, uint16_t len)
{
    if (sensor_uart == NULL || data == NULL || len == 0U)
        return HAL_ERROR;

    return HAL_UART_Transmit(sensor_uart, data, len, 100U);
}
