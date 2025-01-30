#include "stm32f4xx.h"
#include "Travel.h"
double Global_f64Suspension[NO_OF_TRAVEL_SENSORS];
double Global_f64Pressure[NO_OF_PRESSURE_SENSORS];

void Travel_voidProcessAdcReadings(uint16_t * Add_u16AdcData){
		
		for(uint8_t loc_u8counter = 0 ; loc_u8counter < NO_OF_TRAVEL_SENSORS ; loc_u8counter++)
			Global_f64Suspension[loc_u8counter]  =  (uint8_t)((Add_u16AdcData[loc_u8counter] * LINEAR_POT_PEAK) / MAX_VALUE_TRAVEL_SENSOR);
		for(uint8_t loc_u8counter = 4 ; loc_u8counter < 6 ; loc_u8counter++)
			Global_f64Pressure[loc_u8counter - 4] = (uint8_t)((Add_u16AdcData[loc_u8counter] * 40) / 4095);

}
