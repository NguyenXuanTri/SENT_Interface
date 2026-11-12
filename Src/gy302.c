/*
khởi tạo cảm biến ánh sáng qua I²C, đọc độ sáng và gửi kết quả qua UART.

Các bước chính:

- GY302_Init(...):      kiểm tra kết nối, bật và cấu hình cảm biến ở chế độ đo liên tục.

- GY302_ReadLux(...):   đọc dữ liệu thô 2 byte, chuyển thành đơn vị lux theo công thức raw / 1.2. 
                        Sau khi khởi động, cảm biến cần khoảng 180 ms để sẵn sàng.

- GY302_Task(...):      được gọi lặp lại trong vòng lặp chính; 
                        cứ khoảng 500 ms đọc một mẫu và gửi độ sáng cùng giá trị thô qua UART. 
                        Nếu đọc lỗi, gửi thông báo lỗi.

Giúp chương trình biết môi trường đang sáng bao nhiêu lux
*/

#include "gy302.h"
#include <stdio.h>
#include <string.h>

#define GY302_POWER_ON       0x01U
#define GY302_RESET          0x07U
#define GY302_CONT_H_MODE    0x10U

#define GY302_MEASURE_TIME_MS 180U
#define GY302_SAMPLE_MS       500U
#define GY302_I2C_TIMEOUT_MS  100U
#define GY302_UART_TIMEOUT_MS 100U

static void GY302_SendText(GY302_Handle_t *sensor,
                           const char *text)
{
    if (sensor == NULL || sensor->uart == NULL)
        return;

    HAL_UART_Transmit(sensor->uart,
                      (uint8_t *)text,
                      (uint16_t)strlen(text),
                      GY302_UART_TIMEOUT_MS);
}

static HAL_StatusTypeDef GY302_WriteCommand(
    GY302_Handle_t *sensor,
    uint8_t command)
{
    return HAL_I2C_Master_Transmit(
        sensor->i2c,
        sensor->address,
        &command,
        1U,
        GY302_I2C_TIMEOUT_MS);
}

GY302_Status_t GY302_Init(GY302_Handle_t *sensor,
                          I2C_HandleTypeDef *i2c,
                          UART_HandleTypeDef *uart,
                          uint8_t address_7bit)
{
    char msg[64];

    if (sensor == NULL || i2c == NULL ||
        address_7bit > 0x7FU)
    {
        return GY302_ERROR;
    }

    memset(sensor, 0, sizeof(*sensor));

    sensor->i2c = i2c;
    sensor->uart = uart;
    sensor->address = (uint16_t)(address_7bit << 1);
    sensor->sample_interval_ms = GY302_SAMPLE_MS;
    sensor->status = GY302_NOT_READY;

    if (HAL_I2C_IsDeviceReady(
            i2c, sensor->address, 3U,
            GY302_I2C_TIMEOUT_MS) != HAL_OK)
    {
        GY302_SendText(sensor,
                       "GY302: I2C_DEVICE_NOT_FOUND\r\n");

        sensor->status = GY302_ERROR;
        return GY302_ERROR;
    }

    if (GY302_WriteCommand(sensor, GY302_POWER_ON) != HAL_OK)
        goto error;

    if (GY302_WriteCommand(sensor, GY302_RESET) != HAL_OK)
        goto error;

    if (GY302_WriteCommand(sensor, GY302_CONT_H_MODE) != HAL_OK)
        goto error;

    sensor->start_tick = HAL_GetTick();
    sensor->last_sample_tick = sensor->start_tick;
    sensor->initialized = 1U;
    sensor->status = GY302_NOT_READY;

    GY302_SendText(sensor, "GY302: INIT_OK\r\n");
    return GY302_OK;

error:
    sensor->status = GY302_ERROR;
    GY302_SendText(sensor, "GY302: INIT_ERROR\r\n");
    return GY302_ERROR;
}

GY302_Status_t GY302_ReadLux(GY302_Handle_t *sensor,
                             float *lux)
{
    uint8_t data[2];

    if (sensor == NULL || lux == NULL ||
        sensor->initialized == 0U)
    {
        return GY302_ERROR;
    }

    if ((uint32_t)(HAL_GetTick() - sensor->start_tick)
        < GY302_MEASURE_TIME_MS)
    {
        sensor->status = GY302_NOT_READY;
        return GY302_NOT_READY;
    }

    if (HAL_I2C_Master_Receive(
            sensor->i2c,
            sensor->address,
            data,
            2U,
            GY302_I2C_TIMEOUT_MS) != HAL_OK)
    {
        sensor->status = GY302_ERROR;
        return GY302_ERROR;
    }

    sensor->raw_value =
        ((uint16_t)data[0] << 8) | data[1];

    // BH1750 High Resolution Mode: lux = raw / 1.2
    sensor->lux = (float)sensor->raw_value / 1.2f;

    *lux = sensor->lux;
    sensor->status = GY302_OK;

    return GY302_OK;
}

void GY302_Task(GY302_Handle_t *sensor)
{
    uint32_t now;
    float lux;
    char msg[80];
    int len;
    GY302_Status_t result;

    if (sensor == NULL || sensor->initialized == 0U)
        return;

    now = HAL_GetTick();

    if ((uint32_t)(now - sensor->last_sample_tick)
        < sensor->sample_interval_ms)
    {
        return;
    }

    sensor->last_sample_tick = now;

    result = GY302_ReadLux(sensor, &lux);

    if (result == GY302_NOT_READY)
        return;

    if (result != GY302_OK)
    {
        GY302_SendText(sensor,
                       "GY302: READ_ERROR\r\n");
        return;
    }

    len = snprintf(msg, sizeof(msg),
                   "GY302: %.1f lux, RAW=%u\r\n",
                   (double)lux,
                   (unsigned int)sensor->raw_value);

    if (len > 0 && len < (int)sizeof(msg))
        GY302_SendText(sensor, msg);
}
