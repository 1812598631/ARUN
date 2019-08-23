#ifndef CAN_H
#define CAN_H
#include "CAN_Receive.h"
#include "stm32f10x.h"

void CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
#endif
