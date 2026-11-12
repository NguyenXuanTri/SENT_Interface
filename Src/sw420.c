/*
xử lý tín hiệu từ cảm biến rung SW-420:

- SW420_Init(...): lưu chân GPIO, UART và mức logic được xem là có rung; 
đồng thời đọc trạng thái ban đầu và gửi INIT_OK cùng trạng thái VIBRATION hoặc NORMAL.

- SW420_Read(...): đọc trạng thái hiện tại và trả về SW420_VIBRATION hoặc SW420_NORMAL.
- SW420_Task(...): nên được gọi lặp trong vòng lặp chính. Hàm kiểm tra chân GPIO mỗi 10 ms 
                    và gửi thông báo UART khi tín hiệu đổi trạng thái.
                    
*/
#include "sw420.h"
#include <stdio.h>
#include <string.h>

#define SW420_SAMPLE_INTERVAL_MS  10U
#define SW420_DEBOUNCE_MS         20U
#define SW420_UART_TIMEOUT_MS     100U

static void SW420_SendMessage(SW420_Handle_t *sensor,
                              const char *message)
{
    if (sensor == NULL || sensor->uart == NULL)
        return;

    HAL_UART_Transmit(sensor->uart,
                      (uint8_t *)message,
                      (uint16_t)strlen(message),
                      SW420_UART_TIMEOUT_MS);
}

void SW420_Init(SW420_Handle_t *sensor,
                GPIO_TypeDef *gpio_port,
                uint16_t gpio_pin,
                UART_HandleTypeDef *uart,
                GPIO_PinState active_level)
{
    if (sensor == NULL)
        return;

    sensor->gpio_port = gpio_port;
    sensor->gpio_pin = gpio_pin;
    sensor->uart = uart;
    sensor->active_level = active_level;

    sensor->last_level = HAL_GPIO_ReadPin(gpio_port, gpio_pin);
    sensor->last_sample_tick = HAL_GetTick();
    sensor->debounce_ms = SW420_DEBOUNCE_MS;
    sensor->initialized = 1U;

    SW420_SendMessage(sensor, "SW420: INIT_OK\r\n");

    if (sensor->last_level == active_level)
        SW420_SendMessage(sensor, "SW420: VIBRATION\r\n");
    else
        SW420_SendMessage(sensor, "SW420: NORMAL\r\n");
}

SW420_State_t SW420_Read(SW420_Handle_t *sensor)
{
    if (sensor == NULL || sensor->initialized == 0U)
        return SW420_NORMAL;

    if (HAL_GPIO_ReadPin(sensor->gpio_port,
                         sensor->gpio_pin) == sensor->active_level)
    {
        return SW420_VIBRATION;
    }

    return SW420_NORMAL;
}

void SW420_Task(SW420_Handle_t *sensor)
{
    uint32_t now;
    GPIO_PinState current_level;

    if (sensor == NULL || sensor->initialized == 0U)
        return;

    now = HAL_GetTick();

    if ((uint32_t)(now - sensor->last_sample_tick)
        < SW420_SAMPLE_INTERVAL_MS)
    {
        return;
    }

    sensor->last_sample_tick = now;

    current_level = HAL_GPIO_ReadPin(sensor->gpio_port,
                                     sensor->gpio_pin);

    if (current_level != sensor->last_level)
    {
        sensor->last_level = current_level;

        if (current_level == sensor->active_level)
        {
            SW420_SendMessage(sensor,
                              "SW420: VIBRATION\r\n");
        }
        else
        {
            SW420_SendMessage(sensor,
                              "SW420: NORMAL\r\n");
        }
    }
}
