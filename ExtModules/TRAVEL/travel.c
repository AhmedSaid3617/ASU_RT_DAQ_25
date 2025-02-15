#include "stm32f4xx.h"
#include "travel.h"
#include <stdint.h>

double suspension_val[NO_OF_TRAVEL_SENSORS];

void Travel_voidProcessAdcReadings(uint16_t *Add_u16AdcData)
{
	for (uint8_t i = 0; i < NO_OF_TRAVEL_SENSORS; i++)
	suspension_val[i] = (uint8_t)((Add_u16AdcData[i] * LINEAR_POT_PEAK) / MAX_VALUE_TRAVEL_SENSOR);
}
