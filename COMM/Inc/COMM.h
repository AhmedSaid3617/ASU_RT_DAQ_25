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
    uint8_t size;  /*!< Size of the data in bytes. */
    uint16_t id;   /*!< 11-bit CAN message id. Must be the least-significant 11 bits. */
    uint64_t data; /*!< Data to be sent. Maximum of 8 bytes (as specified by CAN protocol). */
} COMM_can_message_t;

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