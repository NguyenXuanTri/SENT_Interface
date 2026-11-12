#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { volatile uint32_t CR1, PSC, ARR, EGR, CNT; } TIM_TypeDef;
extern TIM_TypeDef g_tim4;
#define TIM4 (&g_tim4)
typedef struct { uint32_t CFGR; } RCC_TypeDef;
extern RCC_TypeDef g_rcc;
#define RCC (&g_rcc)
#define RCC_CFGR_PPRE1 0x700U
#define RCC_CFGR_PPRE1_DIV1 0U
#define TIM_EGR_UG 1U
#define TIM_CR1_CEN 1U
typedef struct { int d; } UART_HandleTypeDef;
typedef struct { int d; } GPIO_TypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
extern GPIO_TypeDef g_gpiob;
#define IFB_GPIO_Port (&g_gpiob)
#define IFB_Pin 0x1000U
#define GPIO_MODE_IT_FALLING 1U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_HIGH 3U
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
#define EXTI15_10_IRQn 40
#define __HAL_RCC_TIM4_CLK_ENABLE()
#define __HAL_RCC_GPIOB_CLK_ENABLE()
#define __HAL_GPIO_EXTI_CLEAR_IT(x) ((void)(x))
#define __DMB() __asm__ volatile("" ::: "memory")
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin);
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i);
void HAL_NVIC_DisableIRQ(int);
void HAL_NVIC_EnableIRQ(int);
void HAL_NVIC_SetPriority(int, uint32_t, uint32_t);