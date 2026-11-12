/*
xử lý tín hiệu từ cảm biến quang GP1S093. 
Nó cho phép khởi tạo cảm biến, đọc mức GPIO, lấy hoặc đặt lại bộ đếm, 
và đếm tín hiệu trong callback ngắt.
*/

#include "gp1s093.h"


void GP1S093_Init(GP1S093_HandleTypeDef *sensor,
                  GPIO_TypeDef *port,
                  uint16_t pin,
                  uint32_t debounce_ms)
{
    if (sensor == NULL)
        return;

    sensor->port = port;
    sensor->pin = pin;
    sensor->count = 0U;
    sensor->last_tick = 0U;
    sensor->debounce_ms = debounce_ms;

}

GPIO_PinState GP1S093_Read(GP1S093_HandleTypeDef *sensor)
{
    if (sensor == NULL || sensor -> port == NULL)
        return GPIO_PIN_RESET;

    return HAL_GPIO_ReadPin(sensor->port, sensor->pin);
}

uint32_t GP1S093_GetCount(GP1S093_HandleTypeDef *sensor)
{
    if (sensor == NULL)
        return 0U;

    return sensor->count;
}

void GP1S093_ResetCount(GP1S093_HandleTypeDef *sensor)
{
    if (sensor != NULL)
        sensor->count = 0U;
}

void GP1S093_EXTI_Callback(GP1S093_HandleTypeDef *sensor, uint16_t GPIO_Pin)
{

    if (sensor == NULL || GPIO_Pin != sensor->pin)
        return;

    if (GPIO_Pin != sensor->pin)
        return;

    if (HAL_GPIO_ReadPin(sensor->port, sensor->pin)
        != GPIO_PIN_SET)
        return;

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - sensor->last_tick)
        < sensor->debounce_ms)
        return;

    sensor->last_tick = now;
    sensor->count++;
}
