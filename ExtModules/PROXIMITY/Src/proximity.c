#include "proximity.h"
#include "config.h"

uint16_t PROXIMITY_DMA_capture_buff[16];
double PROXIMITY_fr_speed; /* Speed of the front-right wheel. */
double PROXIMITY_fl_speed; /* Speed of the front-left wheel. */

static TIM_HandleTypeDef proximity_timer_handle;
static DMA_HandleTypeDef proximity_dma_handle;

void PROXIMITY_init(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma)
{
    proximity_timer_handle = *htim;
    proximity_dma_handle = *hdma;
    HAL_TIM_IC_Start_DMA(htim, TIM_CHANNEL_2, &PROXIMITY_DMA_capture_buff, 16);
}

void PROXIMITY_task()
{
    COMM_can_message_t can_message = {};
    double speed = 0;
    uint8_t slow_counter = 0;

    // TODO: There might be more IDs in the future.
    can_message.id = COMM_CAN_ID_PROX;
    while (1)
    {
        vTaskDelay(67);

        int last_reading_index = 0;

        for (last_reading_index = 0; last_reading_index < 16; last_reading_index++)
        {
            if (PROXIMITY_DMA_capture_buff[last_reading_index] == 0)
            {
                break;
            }
        }

        // If the car is moving fast.
        if (last_reading_index > 1)
        {
            slow_counter = 0;
            // Calculate the speed.
            speed = (last_reading_index - 1) * CONFIG_TIRE_CIRCUMFERENCE * 0.25 * (CONFIG_PROXIMITY_TIMER_FREQ / (PROXIMITY_DMA_capture_buff[last_reading_index - 1] - PROXIMITY_DMA_capture_buff[0])) * 3.6;

            // Pause DMA and Timer
            HAL_DMA_Abort(&proximity_dma_handle);
            HAL_TIM_Base_Stop(&proximity_timer_handle);

            // Reset the DMA pointer so that it starts at the begining of the array.
            proximity_dma_handle.Instance->M0AR = PROXIMITY_DMA_capture_buff;
            proximity_dma_handle.Instance->NDTR = 16;
            HAL_DMA_Start(&proximity_dma_handle, &TIM1->CCR2, &PROXIMITY_DMA_capture_buff, 16);

            // Erase the array.
            for (int i = 0; i < 16; i++)
            {
                PROXIMITY_DMA_capture_buff[i] = 0;
            }

            // Reset the timer so that the readings in the next cycle will be relative to 0.
            __HAL_TIM_SET_COUNTER(&proximity_timer_handle, 0);
            HAL_TIM_Base_Start(&proximity_timer_handle);
        }
        else
        {
            if (slow_counter <= 10) // The car is slow
            {
                slow_counter++;
            }
            else
            { // The car is at rest.
                speed = 0;
                slow_counter = 0;

                // Pause DMA and Timer
                HAL_DMA_Abort(&proximity_dma_handle);
                HAL_TIM_Base_Stop(&proximity_timer_handle);

                // Reset the DMA pointer so that it starts at the begining of the array.
                proximity_dma_handle.Instance->M0AR = PROXIMITY_DMA_capture_buff;
                proximity_dma_handle.Instance->NDTR = 16;
                HAL_DMA_Start(&proximity_dma_handle, &TIM1->CCR2, &PROXIMITY_DMA_capture_buff, 16);

                // Erase the array.
                for (int i = 0; i < 16; i++)
                {
                    PROXIMITY_DMA_capture_buff[i] = 0;
                }

                // Reset the timer so that the readings in the next cycle will be relative to 0.
                __HAL_TIM_SET_COUNTER(&proximity_timer_handle, 0);
                HAL_TIM_Base_Start(&proximity_timer_handle);
            }
        }

        // TODO: adjust to send speed as double instead of truncating.
        can_message.data = (uint64_t)speed;
        COMM_can_enqueue(&can_message);
    }
}