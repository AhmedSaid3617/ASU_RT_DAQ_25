#include "proximity.h"
#include "config.h"
#include "proximity_priv.h"

uint16_t PROXIMITY_DMA_capture_buff[4][16];
double PROXIMITY_wheel_rpm[4];
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
    COMM_can_message_t can_message_rpm = {};
    COMM_can_message_t can_message_speed = {};
    COMM_message_PROX_encoder_t proximity_encoder_message = {};
    COMM_dashboard_speed_t dashboard_speed_message = 0;
    uint8_t slow_counter[4] = {0, 0, 0, 0}; // Changed to an array of size 4

    // TODO: There might be more IDs in the future.
    can_message_rpm.id = COMM_CAN_ID_PROX_ENCODER;
    can_message_rpm.size = 8;
    while (1)
    {
        // TODO: needs testing.
        vTaskDelay(50);

        for (int wheel_no = 0; wheel_no < 2; wheel_no++) // Only front wheels for now
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
                slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter
                // Calculate the speed.
                uint16_t difference = PROXIMITY_DMA_capture_buff[wheel_no][last_reading_index - 1] - PROXIMITY_DMA_capture_buff[wheel_no][last_reading_index - 2];
                PROXIMITY_wheel_rpm[wheel_no] = 0.25 * 60 * (CONFIG_PROXIMITY_TIMER_FREQ / difference);

                // Pause DMA.
                HAL_DMA_Abort(&proximity_dma_handles[wheel_no]);

                // Erase the array.
                for (int i = 0; i < 16; i++)
                {
                    PROXIMITY_DMA_capture_buff[wheel_no][i] = 0;
                }

                // Reset the DMA pointer so that it starts at the beginning of the array.
                proximity_dma_handles[wheel_no].Instance->M0AR = &PROXIMITY_DMA_capture_buff[wheel_no];
                proximity_dma_handles[wheel_no].Instance->NDTR = 16;
                HAL_DMA_Start(&proximity_dma_handles[wheel_no], timer_counters[wheel_no], &PROXIMITY_DMA_capture_buff[wheel_no], 16);
            }
            else
            {
                // TODO: macro this magic number and try playing with it.
                if (slow_counter[wheel_no] <= 14) // The car is slow
                {
                    slow_counter[wheel_no]++; // Increment the corresponding slow_counter
                }
                else
                { // The car is at rest.
                    PROXIMITY_wheel_rpm[wheel_no] = 0;
                    slow_counter[wheel_no] = 0; // Reset the corresponding slow_counter

                    // Pause DMA.
                    HAL_DMA_Abort(&proximity_dma_handles[wheel_no]);

                    // Erase the array.
                    for (int i = 0; i < 16; i++)
                    {
                        PROXIMITY_DMA_capture_buff[wheel_no][i] = 0;
                    }

                    // Reset the DMA pointer so that it starts at the beginning of the array.
                    proximity_dma_handles[wheel_no].Instance->M0AR = &PROXIMITY_DMA_capture_buff[wheel_no];
                    proximity_dma_handles[wheel_no].Instance->NDTR = 16;
                    HAL_DMA_Start(&proximity_dma_handles[wheel_no], timer_counters[wheel_no], &PROXIMITY_DMA_capture_buff[wheel_no], 16);
                }
            }
        }

        // TODO: calculate back wheels too.

        proximity_encoder_message.RPM_front_left = (uint16_t)PROXIMITY_wheel_rpm[FRONT_LEFT_BUFF];
        proximity_encoder_message.RPM_front_right = (uint16_t)PROXIMITY_wheel_rpm[FRONT_RIGHT_BUFF];
        proximity_encoder_message.RPM_rear_left = (uint16_t)PROXIMITY_wheel_rpm[REAR_LEFT_BUFF];
        proximity_encoder_message.RPM_rear_right = (uint16_t)PROXIMITY_wheel_rpm[REAR_RIGHT_BUFF];
        proximity_encoder_message.ENCODER_angle = 123;
        can_message_rpm.data = *((uint64_t*)(&proximity_encoder_message));

        dashboard_speed_message = (uint8_t) PROXIMITY_CALCULATE_SPEED(PROXIMITY_wheel_rpm[FRONT_LEFT_BUFF], PROXIMITY_wheel_rpm[FRONT_RIGHT_BUFF]);
        can_message_speed.id = COMM_CAN_ID_DASHBOARD_SPEED;
        can_message_speed.data = dashboard_speed_message;
        can_message_speed.size = 1;

        COMM_can_enqueue(&can_message_rpm);
        COMM_can_enqueue(&can_message_speed);
    }
}