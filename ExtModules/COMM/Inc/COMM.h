#ifndef COMM_H
#define COMM_H

#include <stdint.h>
#include "DAQ.h"

/* =========================================== FREERTOS INCLUDES =========================================== */
#include "FreeRTOS.h"
#include "queue.h"

/* =========================================== STM32HAL INCLUDES =========================================== */
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"

/**
 * @brief Message to be enqueued in the CAN queue.
 * 
 */
typedef struct{
    uint64_t data; /*!< Data to be sent. Maximum of 8 bytes (as specified by CAN protocol). */
    uint16_t id;   /*!< 11-bit CAN message id. Must be the least-significant 11 bits. */
    uint8_t size;  /*!< Size of the data in bytes. */
} COMM_can_message_t;


/**
 * @brief Format for messages from IMU to be included in a COMM_can_message_t object.
 * Can include accleration or angles.
 * 
 */
typedef struct
{
    int16_t x;     /*!< X-axis value. */
    int16_t y;     /*!< Y-axis value. */
    int16_t z;     /*!< Z-axis value. */
} COMM_message_IMU_t;


// TODO: this is temporary until I figure out the accuracy and range on the travel and pressure sensors.
/**
 * @brief Format for messages from ADC_DEV to be included in a COMM_can_message_t object.
 * Includes travel sensor and pressure sensor values.
 * 
 */
typedef struct
{
    uint64_t SUS_1 : 10;
    uint64_t SUS_2 : 10;
    uint64_t SUS_3 : 10;
    uint64_t SUS_4 : 10;

    uint64_t PRESSURE_1 : 10;
    uint64_t PRESSURE_2 : 10;
} COMM_message_ADC_t;


/**
 * @brief Format for messages from proximity to be included in a COMM_can_message_t object.
 * Contains RPM for the 4 wheels and the angle of the steering wheel from rotary encoder.
 * 
 */
typedef struct
{
    uint64_t RPM_front_left : 12;
    uint64_t RPM_front_right : 12;
    uint64_t RPM_rear_left : 12;
    uint64_t RPM_rear_right : 12;

    uint64_t ENCODER_angle: 10;
} COMM_message_PROX_encoder_t;
 

/**
 * @brief ID for CAN messages.
 * 
 */
typedef enum {
    COMM_CAN_ID_LED = 3,
    COMM_CAN_ID_IMU_ANGLE,
    COMM_CAN_ID_IMU_ACCEL,
    COMM_CAN_ID_TRAVEL,
    COMM_CAN_ID_PROX
} COMM_can_id;

/**
 * @brief Adds one CAN message to the FreeRTOS queue, to be transmitted on CAN bus by the CAN task.
 * @param message Pointer to CAN message object to be enqueued.
 */
void COMM_can_enqueue(COMM_can_message_t* message);

/**
 * @brief Initializes the COMM task by starting CAN communication and setting the can_tx_header.
 * 
 * @param can_handle Pointer to CAN handle object.
 * @param can_tx_header Pointer to CAN TX header object.
 */
void COMM_init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header);

/**
 * @brief Dequeues one COMM_can_message_t from the FreeRTOS CAN queue.
 * 
 * @return COMM_can_message_t: the dequeued message.
 * @attention This function should only be used in CAN task. If the queue is empty, it will block the task until the queue is not empty.
 */
COMM_can_message_t COMM_can_dequeue();

#endif