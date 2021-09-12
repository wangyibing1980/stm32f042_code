#ifndef __CAN1_H
#define __CAN1_H
#include "sys.h"
extern CanRxMsg RxMessage;
extern CanTxMsg TxMessage;
void can_init(void);
#endif