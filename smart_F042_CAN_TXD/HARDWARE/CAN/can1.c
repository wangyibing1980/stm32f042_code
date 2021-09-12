#include "can1.h"

CanRxMsg RxMessage = {0};
CanTxMsg TxMessage = {0};

void can_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  SYSCFG->CFGR1 |=SYSCFG_CFGR1_PA11_PA12_RMP;

  //这里一定要先映射GPIO
  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* Connect CAN pins to AF7 */
  /* CAN GPIOs configuration **************************************************/
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4);

  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_PinSource11 | GPIO_PinSource12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* NVIC configuration *******************************************************/
  NVIC_InitStructure.NVIC_IRQChannel = CEC_CAN_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* CAN configuration ********************************************************/
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE);

  /* CAN register init */
  CAN_DeInit(CAN);
  CAN_StructInit(&CAN_InitStructure);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  //CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

  //200k的配置
  //48M/12/(CAN_SJW_1tq + CAN_BS1_4tq+CAN_BS2_5tq) == 200k
  /* CAN Baudrate = 1MBps (CAN clocked at 36 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler = 3;
  CAN_Init(CAN, &CAN_InitStructure);

  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 1;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  /* Transmit Structure preparation */

  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE);
}