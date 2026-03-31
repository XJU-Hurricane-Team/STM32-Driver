/**
 * @file    CANSPI.c
 * @author  Dominate0017
 * @brief   MCP2515 SPI CAN driver (TX/RX, filter, status).
 * @version 1.0
 * @date    2026-3-15
 * @note    Provides MCP2515 initialization, transmit/receive and helper functions.
 */

#ifndef __CAN_SPI_H
#define	__CAN_SPI_H

#include <cubemx.h>
#include "MCP2515.h"
#include <stdbool.h>

typedef union {
  struct {
    uint8_t idType;
    uint32_t id;
    uint8_t dlc;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
    uint8_t data7;
  } frame;
  uint8_t array[14];
} uCAN_MSG;

#define dSTANDARD_CAN_MSG_ID_2_0B 0
#define dEXTENDED_CAN_MSG_ID_2_0B 1

bool CANSPI_Initialize_Ext(MCP2515_DevId_t dev_id);
void CANSPI_Sleep_Ext(MCP2515_DevId_t dev_id);
uint8_t CANSPI_Transmit_Ext(MCP2515_DevId_t dev_id, uCAN_MSG *tempCanMsg);
uint8_t CANSPI_Receive_Ext(MCP2515_DevId_t dev_id, uCAN_MSG *tempCanMsg);
uint8_t CANSPI_messagesInBuffer_Ext(MCP2515_DevId_t dev_id);
uint8_t CANSPI_isBussOff(MCP2515_DevId_t dev_id);
uint8_t CANSPI_isRxErrorPassive(MCP2515_DevId_t dev_id);
uint8_t CANSPI_isTxErrorPassive(MCP2515_DevId_t dev_id);

#endif	/* __CAN_SPI_H */



