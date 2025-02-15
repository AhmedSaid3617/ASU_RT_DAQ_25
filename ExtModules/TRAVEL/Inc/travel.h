#ifndef TRAVEL_H
#define TRAVEL_H

#include "config.h"

/**
 * @brief Convert the ADC readings from adc buffer to actual travel sensor readings in cm.
 * 
 * @param adc_readings Input buffer that contains the readings from ADC.
 * @param travel_output_buffer Output buffer where the travel sensor readings in cm will be written.
 */
void TRAVEL_process_adc_readings(uint16_t* adc_readings, double* travel_output_buffer);

#endif
