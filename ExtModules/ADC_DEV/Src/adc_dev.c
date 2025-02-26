#include "adc_dev.h"
#include "COMM.h"

static uint16_t TRAVEL_ADC_readings[CONFIG_TRAVEL_SENSOR_NUM];

void ADC_DEV_start(DMA_InitTypeDef* hdma, ADC_InitTypeDef* hadc){

    //HAL_DMA_Start(hdma, &ADC1->DR, TRAVEL_ADC_readings, CONFIG_TRAVEL_SENSOR_NUM); // Begin DMA data transfers.
    HAL_ADC_Start_DMA(hadc, TRAVEL_ADC_readings, 1); // Start ADC conversions, which will continue continuously.
}

void ADC_DEV_task(){

	COMM_can_message_t can_message = {};
    can_message.size = 2;
    can_message.id = COMM_CAN_ID_TRAVEL;
	while (1)
	{
		vTaskDelay(500);
        can_message.data = TRAVEL_ADC_readings[0];   
        COMM_can_enqueue(&can_message);

	}
	
}
