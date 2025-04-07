#include "COMM.h"
#include "COMM_priv.h"

/* =========================================== PRIVATE VARIABLES =========================================== */
QueueHandle_t CAN_queue;
/* =========================================== PRIVATE VARIABLES END =========================================== */

void COMM_init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header)
{
    CAN_queue = xQueueCreate(sizeof(COMM_can_message_t), 10);
    HAL_CAN_Start(can_handle);
    can_tx_header->IDE = CAN_ID_STD;
    can_tx_header->RTR = CAN_RTR_DATA;
    can_tx_header->TransmitGlobalTime = DISABLE;
}

void COMM_can_enqueue(COMM_can_message_t* message)
{
    /* can_tx_header.DLC = message->size;
    can_tx_header.StdId = message->id; */
    xQueueSend(CAN_queue, message, 2);
}

COMM_can_message_t COMM_can_dequeue(){
    COMM_can_message_t message = {};
    xQueueReceive(CAN_queue, &message, portMAX_DELAY);
    return message;
}
