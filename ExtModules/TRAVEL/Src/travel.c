#include "stm32f4xx.h"
#include "travel.h"
#include <stdint.h>

void TRAVEL_process_adc_readings(uint16_t* adc_readings, double* travel_output_buffer){
	for (uint8_t i = 0; i < CONFIG_TRAVEL_SENSOR_NUM; i++)
	travel_output_buffer[i] = (uint8_t)((adc_readings[i] * LINEAR_POT_PEAK) / CONFIG_ADC_MAX_VALUE);
}
