#include "stm32f4xx.h"
#include "travel.h"
#include "COMM.h"
#include <stdint.h>

static uint16_t TRAVEL_ADC_readings[CONFIG_TRAVEL_SENSOR_NUM];

// TODO: needs input buffer.
void TRAVEL_process_adc_readings(double* travel_output_buffer){
	for (uint8_t i = 0; i < CONFIG_TRAVEL_SENSOR_NUM; i++)
	travel_output_buffer[i] = (uint8_t)((TRAVEL_ADC_readings[i] * LINEAR_POT_PEAK) / CONFIG_ADC_MAX_VALUE);
}