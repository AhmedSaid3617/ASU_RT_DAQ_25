#ifndef PROXIMTY_H
#define PROXIMTY_H

/* =========================================== STM32HAL INCLUDES =========================================== */
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

void PROXIMITY_init(TIM_HandleTypeDef* htim);
void PROXIMITY_task();

#endif