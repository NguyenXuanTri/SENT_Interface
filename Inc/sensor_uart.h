#ifndef SENSOR_UART_H
#define SENSOR_UART_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void SensorUART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef SensorUART_Send(const char *data);
HAL_StatusTypeDef SensorUART_SendBytes(uint8_t *data, uint16_t len);

#endif
