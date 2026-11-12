#ifndef GP1S093_H
#define GP1S093_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;

    volatile uint32_t count;
    volatile uint32_t last_tick;

    uint32_t debounce_ms;
} GP1S093_HandleTypeDef;

void GP1S093_Init(GP1S093_HandleTypeDef *sensor,
                  GPIO_TypeDef *port,
                  uint16_t pin,
                  uint32_t debounce_ms);

GPIO_PinState GP1S093_Read(GP1S093_HandleTypeDef *sensor);

uint32_t GP1S093_GetCount(GP1S093_HandleTypeDef *sensor);

void GP1S093_ResetCount(GP1S093_HandleTypeDef *sensor);

/* Gọi từ HAL_GPIO_EXTI_Callback() */
void GP1S093_EXTI_Callback(GP1S093_HandleTypeDef *sensor, uint16_t GPIO_Pin);

#endif
