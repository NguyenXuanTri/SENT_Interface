#ifndef GY302_H
#define GY302_H

#include "main.h"
#include <stdint.h>

typedef enum
{
    GY302_OK = 0,
    GY302_ERROR,
    GY302_NOT_READY
} GY302_Status_t;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    UART_HandleTypeDef *uart;

    uint16_t address;       // Dia chi HAL I2C, da dich trai
    uint16_t raw_value;
    float lux;

    uint32_t last_sample_tick;
    uint32_t sample_interval_ms;
    uint32_t start_tick;

    uint8_t initialized;
    uint8_t first_read;
    GY302_Status_t status;
} GY302_Handle_t;

GY302_Status_t GY302_Init(GY302_Handle_t *sensor,
                          I2C_HandleTypeDef *i2c,
                          UART_HandleTypeDef *uart,
                          uint8_t address_7bit);

GY302_Status_t GY302_ReadLux(GY302_Handle_t *sensor,
                             float *lux);

void GY302_Task(GY302_Handle_t *sensor);

#endif
