#ifndef ENCODER_H
#define ENCODER_H

/* =========================================== STM32HAL INCLUDES =========================================== */
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

void ENCODER_init(TIM_HandleTypeDef *htim);
float ENCODER_get_angle();

#endif