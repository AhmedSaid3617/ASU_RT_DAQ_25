#include "proximity.h"
#include "config.h"
#include "proximity_priv.h"

uint16_t PROXIMITY_DMA_capture_buff[4][16];
double PROXIMITY_wheel_rpm[4];
double PROXIMITY_dashboard_speed = 0;
static TIM_HandleTypeDef proximity_timer_handle;
static DMA_HandleTypeDef proximity_dma_handles[4];

// TODO: fix 1 and 3.
static uint32_t timer_counters[4] = {&TIM1->CCR2, &TIM1->CCR4, &TIM1->CCR3, &TIM1->CCR1};


void PROXIMITY_init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef* hdma[4])
{
    proximity_timer_handle = *htim;
    for (int i = 0; i < 4; i++)
    {
        proximity_dma_handles[i] = *hdma[i];
    }
    
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_2, &PROXIMITY_DMA_capture_buff[FRONT_LEFT_BUFF], 16);
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_4, &PROXIMITY_DMA_capture_buff[FRONT_RIGHT_BUFF], 16);
}

void PROXIMITY_task()
{
    COMM_can_message_t can_message = {};
    COMM_message_PROX_encoder_t proximity_encoder_message = {};
    uint8_t slow_counter = 0;

    // TODO: There might be more IDs in the future.
    can_message.id = COMM_CAN_ID_PROX;
    can_message.size = 8;
    while (1)
    {
        // TODO: needs testing.
        vTaskDelay(1000);

        for (int wheel_no = 0; wheel_no < 2; wheel_no++)
        {
            int last_reading_index = 0;

            for (last_reading_index = 0; last_reading_index < 16; last_reading_index++)
            {
                if (PROXIMITY_DMA_capture_buff[wheel_no][last_reading_index] == 0)
                {
                    break;
                }
            }

            // If the wheel is moving fast.
            if (last_reading_index > 1)
            {
                slow_counter = 0;
                // Calculate the speed.
                PROXIMITY_wheel_rpm[wheel_no] = (last_reading_index - 1) * 0.25 * (CONFIG_PROXIMITY_TIMER_FREQ / (PROXIMITY_DMA_capture_buff[wheel_no][last_reading_index - 1] - PROXIMITY_DMA_capture_buff[wheel_no][0]));

                // Pause DMA and Timer
                HAL_DMA_Abort(&proximity_dma_handles[wheel_no]);
                HAL_TIM_Base_Stop(&proximity_timer_handle);

                // Reset the DMA pointer so that it starts at the begining of the array.
                proximity_dma_handles[wheel_no].Instance->M0AR = &PROXIMITY_DMA_capture_buff[wheel_no];
                proximity_dma_handles[wheel_no].Instance->NDTR = 16;
                HAL_DMA_Start(&proximity_dma_handles[wheel_no], timer_counters[wheel_no], &PROXIMITY_DMA_capture_buff[wheel_no], 16);

                // Erase the array.
                for (int i = 0; i < 16; i++)
                {
                    PROXIMITY_DMA_capture_buff[wheel_no][i] = 0;
                }

                // Reset the timer so that the readings in the next cycle will be relative to 0.
                __HAL_TIM_SET_COUNTER(&proximity_timer_handle, 0);
                HAL_TIM_Base_Start(&proximity_timer_handle);
            }
            else
            {
                // TODO: macro this magic number and try playing with it.
                if (slow_counter <= 14) // The car is slow
                {
                    slow_counter++;
                }
                else
                { // The car is at rest.
                    PROXIMITY_wheel_rpm[wheel_no] = 0;
                    slow_counter = 0;

                    // Pause DMA and Timer
                    HAL_DMA_Abort(&proximity_dma_handles[wheel_no]);
                    HAL_TIM_Base_Stop(&proximity_timer_handle);

                    // Reset the DMA pointer so that it starts at the begining of the array.
                    proximity_dma_handles[wheel_no].Instance->M0AR = &PROXIMITY_DMA_capture_buff[wheel_no];
                    proximity_dma_handles[wheel_no].Instance->NDTR = 16;
                    HAL_DMA_Start(&proximity_dma_handles[wheel_no], timer_counters[wheel_no], &PROXIMITY_DMA_capture_buff[wheel_no], 16);

                    // Erase the array.
                    for (int i = 0; i < 16; i++)
                    {
                        PROXIMITY_DMA_capture_buff[wheel_no][i] = 0;
                    }

                    // Reset the timer so that the readings in the next cycle will be relative to 0.
                    __HAL_TIM_SET_COUNTER(&proximity_timer_handle, 0);
                    HAL_TIM_Base_Start(&proximity_timer_handle);
                }
            }
        }

        // TODO: calculate back wheels too.

        proximity_encoder_message.RPM_front_left = PROXIMITY_wheel_rpm[FRONT_LEFT_BUFF];
        proximity_encoder_message.RPM_front_right = PROXIMITY_wheel_rpm[FRONT_RIGHT_BUFF];
        proximity_encoder_message.RPM_rear_left = 1234;
        proximity_encoder_message.RPM_rear_right = 4095;
        proximity_encoder_message.ENCODER_angle = 123;
        can_message.data = *((uint64_t*)(&proximity_encoder_message));
        COMM_can_enqueue(&can_message);

    }
}