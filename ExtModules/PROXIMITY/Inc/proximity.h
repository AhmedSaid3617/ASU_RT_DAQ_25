#ifndef PROXIMTY_H
#define PROXIMTY_H

/* =========================================== STM32HAL INCLUDES =========================================== */
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

/* ============================================= DAQ INCLUDES ============================================== */
#include "COMM.h"

/* =========================================== FREERTOS INCLUDES =========================================== */
#include "FreeRTOS.h"

void PROXIMITY_init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
void PROXIMITY_task();

#endif