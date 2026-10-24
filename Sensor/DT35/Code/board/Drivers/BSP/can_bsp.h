#ifndef __CAN_BSP_H
#define __CAN_BSP_H

#include "main.h"
#include "can.h"

void CAN_Init(void);
void CAN_Send_Message(uint32_t ID, uint8_t* data, uint8_t len);

#endif
