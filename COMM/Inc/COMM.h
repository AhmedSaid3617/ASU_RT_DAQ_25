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
    uint8_t size;
    uint16_t id;
    uint64_t data;
} COMM_can_message_t;

// TODO: clean these comments up.
/**
 * @brief Adds one CAN message to the FreeRTOS queue, to be transmitted using CAN by the CAN task.
 * 
 * @param can_handle SLKDJVNKD
 * @param message Can message to be sent, with data and message ID.
 */
void COMM_can_enqueue(COMM_can_message_t* message);

/**
 * @brief Initializes the COMM task by
 * 
 */
void COMM_init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header);

/**
 * @brief Get and remove a message from the queue.
 * 
 * @return COMM_can_message_t 
 */
COMM_can_message_t COMM_can_dequeue();

#endif