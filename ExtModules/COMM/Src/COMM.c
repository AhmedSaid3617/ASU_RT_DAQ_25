#include "COMM.h"
#include "COMM_priv.h"

/* =========================================== PRIVATE VARIABLES =========================================== */
QueueHandle_t CAN_queue;
CAN_HandleTypeDef COMM_can_handle;
uint32_t tx_mailbox;
CAN_TxHeaderTypeDef* ptr_tx_header;
/* =========================================== PRIVATE VARIABLES END =========================================== */

void COMM_init(CAN_HandleTypeDef *can_handle, CAN_TxHeaderTypeDef* can_tx_header)
{
    CAN_queue = xQueueCreate(10, sizeof(COMM_can_message_t));
    COMM_can_handle = *can_handle;
    HAL_CAN_Start(can_handle);
    can_tx_header->IDE = CAN_ID_STD;
    can_tx_header->RTR = CAN_RTR_DATA;
    can_tx_header->TransmitGlobalTime = DISABLE;
    ptr_tx_header = can_tx_header;
}

void COMM_can_enqueue(COMM_can_message_t* message)
{
    xQueueSend(CAN_queue, message, 2);
}

COMM_can_message_t COMM_can_dequeue(){
    COMM_can_message_t message = {};
    xQueueReceive(CAN_queue, &message, portMAX_DELAY);
    return message;
}

void CAN_task()
{
  COMM_can_message_t can_message;
  while (1)
  {
    can_message = COMM_can_dequeue();
    ptr_tx_header->DLC = can_message.size;
    ptr_tx_header->StdId = can_message.id;

    // TODO: figure out if this is needed.
    taskENTER_CRITICAL();
    if (HAL_CAN_AddTxMessage(&COMM_can_handle, ptr_tx_header, &can_message.data, &tx_mailbox) == HAL_ERROR)
    {

      // Error_Handler();
    }
    taskEXIT_CRITICAL();
  }
}
