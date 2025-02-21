#include "proximity.h"

uint16_t PROXIMITY_DMA_capture_buff[16];
double PROXIMITY_fr_speed; /* Speed of the front-right wheel. */
double PROXIMITY_fl_speed; /* Speed of the front-left wheel. */

void PROXIMITY_init(TIM_HandleTypeDef* htim){
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_2, &PROXIMITY_DMA_capture_buff, 16);
}

void PROXIMITY_task(void* param){
    
}
