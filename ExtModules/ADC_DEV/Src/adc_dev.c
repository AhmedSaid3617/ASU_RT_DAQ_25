#include "adc_dev.h"

void ADC_DEV_start(DMA_InitTypeDef* hdma, ADC_InitTypeDef* hadc, uint16_t* adc_readings_buffer){

    HAL_DMA_Start(hdma, &ADC1->DR, adc_readings_buffer, CONFIG_TRAVEL_SENSOR_NUM); // Begin DMA data transfers.

    HAL_ADC_Start_DMA(hadc, adc_readings_buffer, CONFIG_TRAVEL_SENSOR_NUM); // Start ADC conversions, which will continue continuously.
}