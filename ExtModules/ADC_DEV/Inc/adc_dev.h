#ifndef ADC_DEV
#define ADC_DEV

/* =========================================== STM32HAL INCLUDES =========================================== */
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

#include "config.h"

/**
 * @brief Starts ADC conversions for all the devices connected using ADC (travel sensors).
 * 
 * @param hdma_travel DMA handler.
 * @param hadc ADC handler.
 * @param adc_travel_buffer Buffer for the DMA to write ADC readings in. 
 */
void ADC_DEV_start(DMA_InitTypeDef* hdma, ADC_InitTypeDef* hadc);

void ADC_DEV_task();

#endif