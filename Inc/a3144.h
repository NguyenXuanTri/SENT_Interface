#ifndef A3144_H
#define A3144_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    volatile uint32_t count;
    volatile uint32_t last_tick;
    uint32_t debounce_ms;
} A3144_HandleTypeDef;

void A3144_Init(A3144_HandleTypeDef *sensor,
                GPIO_TypeDef *port,
                uint16_t pin,
                uint32_t debounce_ms);

GPIO_PinState A3144_Read(A3144_HandleTypeDef *sensor);
uint32_t A3144_GetCount(A3144_HandleTypeDef *sensor);
void A3144_ResetCount(A3144_HandleTypeDef *sensor);

/* Gọi từ HAL_GPIO_EXTI_Callback() */
void A3144_EXTI_Callback(uint16_t GPIO_Pin);

#endif
