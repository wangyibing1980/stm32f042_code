#ifndef __CAN_H
#define __CAN_H
#include "sys.h"


#define CANx                       CAN
#define CAN_CLK                    RCC_APB1Periph_CAN
#define CAN_RX_PIN                 GPIO_Pin_11
#define CAN_TX_PIN                 GPIO_Pin_12
#define CAN_GPIO_PORT              GPIOA
#define CAN_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define CAN_AF_PORT                GPIO_AF_4
#define CAN_RX_SOURCE              GPIO_PinSource11
#define CAN_TX_SOURCE              GPIO_PinSource12   
extern CanRxMsg RxMessage;
extern CanTxMsg TxMessage;

volatile extern uint8_t can_recv_flag;
volatile extern uint8_t Receive_Buffer_Len;
volatile extern uint8_t ReceiveBuffer[255];
volatile extern uint8_t *pReceiveBuffer;

void CAN_Config(void);
#endif
