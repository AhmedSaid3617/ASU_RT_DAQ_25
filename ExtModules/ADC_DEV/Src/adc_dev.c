#include "adc_dev.h"
#include "COMM.h"

static uint16_t ADC_readings_buffer[CONFIG_TRAVEL_SENSOR_NUM + CONFIG_PRESSURE_SENSOR_NUM];

// TODO: ADC multiple channels to DMA.
void ADC_DEV_start(DMA_InitTypeDef* hdma, ADC_InitTypeDef* hadc){

    //HAL_DMA_Start(hdma, &ADC1->DR, TRAVEL_ADC_readings, CONFIG_TRAVEL_SENSOR_NUM); // Begin DMA data transfers.
    HAL_ADC_Start_DMA(hadc, ADC_readings_buffer, CONFIG_TRAVEL_SENSOR_NUM+CONFIG_PRESSURE_SENSOR_NUM); // Start ADC conversions, which will continue continuously.
}


void ADC_DEV_task(){

	COMM_can_message_t can_message = {};
    COMM_message_ADC_t adc_message = {};
    can_message.size = 8;
    can_message.id = COMM_CAN_ID_TRAVEL;
	while (1)
	{
		vTaskDelay(500);
        // TODO: this is temporary until we use travel_process_readings
        adc_message.SUS_1 = ADC_readings_buffer[0] >> 2;
        adc_message.SUS_2 = ADC_readings_buffer[1] >> 2;
        adc_message.SUS_3 = ADC_readings_buffer[2] >> 2;
        adc_message.SUS_4 = ADC_readings_buffer[3] >> 2;

        adc_message.PRESSURE_1 = ADC_readings_buffer[4] >> 2;
        adc_message.PRESSURE_2 = ADC_readings_buffer[5] >> 2;

        can_message.data = *((uint64_t*)(&adc_message));
        COMM_can_enqueue(&can_message);
	}
	
}
