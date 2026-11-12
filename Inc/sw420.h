
#ifndef SW420_H
#define SW420_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    SW420_NORMAL = 0,
    SW420_VIBRATION = 1
} SW420_State_t;

typedef struct
{
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;

    UART_HandleTypeDef *uart;

    GPIO_PinState active_level;
    GPIO_PinState last_level;

    uint32_t last_sample_tick;
    uint32_t debounce_ms;

    uint8_t initialized;
} SW420_Handle_t;

void SW420_Init(SW420_Handle_t *sensor,
                GPIO_TypeDef *gpio_port,
                uint16_t gpio_pin,
                UART_HandleTypeDef *uart,
                GPIO_PinState active_level);

void SW420_Task(SW420_Handle_t *sensor);

SW420_State_t SW420_Read(SW420_Handle_t *sensor);

#endif
