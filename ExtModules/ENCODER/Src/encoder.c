#include "encoder.h"
#include "config.h"

TIM_HandleTypeDef encoder_timer_handle;
uint32_t encoder_value;

void ENCODER_init(TIM_HandleTypeDef *htim)
{
    encoder_timer_handle = *htim;
    HAL_TIM_Encoder_Start(&encoder_timer_handle, TIM_CHANNEL_ALL);
}

float ENCODER_get_angle(){
    static float angle = 0;
    encoder_value = (uint32_t)__HAL_TIM_GET_COUNTER(&encoder_timer_handle);
    angle = (float)encoder_value * 360.0f / CONFIG_ENCODER_MAX_VALUE;
    return angle;
}