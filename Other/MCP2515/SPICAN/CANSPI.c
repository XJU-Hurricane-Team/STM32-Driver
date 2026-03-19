/**
 * @file    CANSPI.c
 * @author  Dominate0017
 * @brief   MCP2515 SPI CAN driver (TX/RX, filter, status).
 * @version 1.0
 * @date    2026-3-15
 */

#include "SPICAN/CANSPI.h"

/** Local Function Prototypes */  
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL);
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) ;
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg);

/** Local Variables */ 
ctrl_status_t ctrlStatus;
ctrl_error_status_t errorStatus;
id_reg_t idReg;

/** CAN SPI APIs */ 

/**
 * @brief 将 MCP2515 置入睡眠模式（低功耗）。
 *
 * @details
 * 清除总线唤醒中断标志、启用总线活动唤醒，然后切换 MCP2515 至睡眠模式。
 *
 * @param dev_id MCP2515 设备 ID。
 */
void CANSPI_Sleep_Ext(MCP2515_DevId_t dev_id)
{
  /* Clear CAN bus wakeup interrupt */
  MCP2515_BitModify_Ext(dev_id, MCP2515_CANINTF, 0x40, 0x00);        
  
  /* Enable CAN bus activity wakeup */
  MCP2515_BitModify_Ext(dev_id, MCP2515_CANINTE, 0x40, 0x40);        
  
  MCP2515_SetSleepMode_Ext(dev_id);
}

/**
 * @brief 初始化 MCP2515 并配置接收过滤与波特率。
 *
 * @details
 * 初始化 MCP2515, 将其置于配置模式，设置接收滤波器和掩码（默认接收所有消息），
 * 并配置 CAN 波特率寄存器（1MBaud），然后切换到正常模式。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 初始化成功返回 true，否则返回 false。
 */
bool CANSPI_Initialize_Ext(MCP2515_DevId_t dev_id)
{
  RXF0 RXF0reg;
  RXF1 RXF1reg;
  RXF2 RXF2reg;
  RXF3 RXF3reg;
  RXF4 RXF4reg;
  RXF5 RXF5reg;
  RXM0 RXM0reg;
  RXM1 RXM1reg;
      
  /* Intialize Rx Mask values */
  RXM0reg.RXM0SIDH = 0x00;
  RXM0reg.RXM0SIDL = 0x00;
  RXM0reg.RXM0EID8 = 0x00;
  RXM0reg.RXM0EID0 = 0x00;
  
  RXM1reg.RXM1SIDH = 0x00;
  RXM1reg.RXM1SIDL = 0x00;
  RXM1reg.RXM1EID8 = 0x00;
  RXM1reg.RXM1EID0 = 0x00;
  
  /* Intialize Rx Filter values */
  RXF0reg.RXF0SIDH = 0x00;      
  RXF0reg.RXF0SIDL = 0x00;      //Starndard Filter
  RXF0reg.RXF0EID8 = 0x00;
  RXF0reg.RXF0EID0 = 0x00;
  
  RXF1reg.RXF1SIDH = 0x00;
  RXF1reg.RXF1SIDL = 0x08;      //Exntended Filter
  RXF1reg.RXF1EID8 = 0x00;
  RXF1reg.RXF1EID0 = 0x00;
  
  RXF2reg.RXF2SIDH = 0x00;
  RXF2reg.RXF2SIDL = 0x00;
  RXF2reg.RXF2EID8 = 0x00;
  RXF2reg.RXF2EID0 = 0x00;
  
  RXF3reg.RXF3SIDH = 0x00;
  RXF3reg.RXF3SIDL = 0x00;
  RXF3reg.RXF3EID8 = 0x00;
  RXF3reg.RXF3EID0 = 0x00;
  
  RXF4reg.RXF4SIDH = 0x00;
  RXF4reg.RXF4SIDL = 0x00;
  RXF4reg.RXF4EID8 = 0x00;
  RXF4reg.RXF4EID0 = 0x00;
  
  RXF5reg.RXF5SIDH = 0x00;
  RXF5reg.RXF5SIDL = 0x08;
  RXF5reg.RXF5EID8 = 0x00;
  RXF5reg.RXF5EID0 = 0x00;
  
  /* Intialize MCP2515, check SPI */
  if(!MCP2515_Initialize()) 
  {
    return false;
  }
    
  /* Change mode as configuration mode */
  if(!MCP2515_SetConfigMode_Ext(dev_id)) 
  {
    return false;
  }
  
  /* Configure filter & mask, all filters and masks set to 0, accept all */
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXM0SIDH, MCP2515_RXM0EID0, &(RXM0reg.RXM0SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXM1SIDH, MCP2515_RXM1EID0, &(RXM1reg.RXM1SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXF0SIDH, MCP2515_RXF0EID0, &(RXF0reg.RXF0SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXF1SIDH, MCP2515_RXF1EID0, &(RXF1reg.RXF1SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXF2SIDH, MCP2515_RXF2EID0, &(RXF2reg.RXF2SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXF3SIDH, MCP2515_RXF3EID0, &(RXF3reg.RXF3SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXF4SIDH, MCP2515_RXF4EID0, &(RXF4reg.RXF4SIDH));
  MCP2515_WriteByteSequence_Ext(dev_id, MCP2515_RXF5SIDH, MCP2515_RXF5EID0, &(RXF5reg.RXF5SIDH));
  
  /* Accept All (Standard + Extended) */
  MCP2515_WriteByte_Ext(dev_id, MCP2515_RXB0CTRL, 0x04);    //Enable BUKT, Accept Filter 0
  MCP2515_WriteByte_Ext(dev_id, MCP2515_RXB1CTRL, 0x01);    //Accept Filter 1
  
  /* 00(SJW 1tq) 000000 */    
  MCP2515_WriteByte_Ext(dev_id, MCP2515_CNF1, 0x00);
  /* 1 1 010(3tq) 001(2tq) */  
  MCP2515_WriteByte_Ext(dev_id, MCP2515_CNF2, 0xD1);
  /* 1 0 000 001(2tq) */  
  MCP2515_WriteByte_Ext(dev_id, MCP2515_CNF3, 0x81);
  /* Normal mode */
  if(!MCP2515_SetNormalMode_Ext(dev_id)) 
    return false;
  
  return true;
}

/**
 * @brief 通过 MCP2515 发送一条 CAN 消息。
 *
 * @details
 * 检查可用发送缓冲区（TXB0/TXB1/TXB2），将报文加载到第一个可用缓冲区并发出
 * 请求发送指令。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param tempCanMsg 指向要发送的 CAN 消息结构。
 * @return 发送请求是否成功（1 成功，0 失败）。
 */
uint8_t CANSPI_Transmit_Ext(MCP2515_DevId_t dev_id, uCAN_MSG *tempCanMsg) 
{
    uint8_t returnValue = 0;
    id_reg_t idReg = {0}; 

    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);

    ctrl_status_t ctrlStatus;
    ctrlStatus.ctrl_status = MCP2515_ReadStatus_Ext(dev_id);

    if (ctrlStatus.TXB0REQ != 1)
    {
        convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);
        MCP2515_LoadTxSequence_Ext(dev_id, MCP2515_LOAD_TXB0SIDH, &(idReg.tempSIDH), 
                                   tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
        MCP2515_RequestToSend_Ext(dev_id, MCP2515_RTS_TX0);
        returnValue = 1;
    }
    else if (ctrlStatus.TXB1REQ != 1)
    {
        convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);
        MCP2515_LoadTxSequence_Ext(dev_id, MCP2515_LOAD_TXB1SIDH, &(idReg.tempSIDH), 
                                   tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
        MCP2515_RequestToSend_Ext(dev_id, MCP2515_RTS_TX1);
        returnValue = 1;
    }
    else if (ctrlStatus.TXB2REQ != 1)
    {
        convertCANid2Reg(tempCanMsg->frame.id, tempCanMsg->frame.idType, &idReg);
        MCP2515_LoadTxSequence_Ext(dev_id, MCP2515_LOAD_TXB2SIDH, &(idReg.tempSIDH), 
                                   tempCanMsg->frame.dlc, &(tempCanMsg->frame.data0));
        MCP2515_RequestToSend_Ext(dev_id, MCP2515_RTS_TX2);
        returnValue = 1;
    }

     MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);

    return returnValue;
}

/**
 * @brief 从 MCP2515 接收一条 CAN 消息。
 *
 * @details
 * 检查 RX 状态寄存器，用0x90和0x94指令读取与接收相关的一系列寄存器，
 * 读取Rx0和Rx1相应接收缓冲区的数据，并将其转换为
 * uCAN_MSG 结构体。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param tempCanMsg 输出接收消息的结构体指针。
 * @return 是否成功接收到一条消息（1 成功，0 无消息或错误）。
 */
uint8_t CANSPI_Receive_Ext(MCP2515_DevId_t dev_id, uCAN_MSG *tempCanMsg) 
{
  uint8_t returnValue = 0;
  rx_reg_t rxReg = {0}; 
  ctrl_rx_status_t rxStatus;
  
  rxStatus.ctrl_rx_status = MCP2515_GetRxStatus_Ext(dev_id);

  if (rxStatus.rxBuffer != 0)
  {
    if ((rxStatus.rxBuffer == MSG_IN_RXB0) || (rxStatus.rxBuffer == MSG_IN_BOTH_BUFFERS))
    {
      MCP2515_ReadRxSequence_Ext(dev_id, MCP2515_READ_RXB0SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
      MCP2515_ClearIntFlag_Ext(dev_id, MCP2515_INTF_RX0 | MCP2515_INTF_RX1);
      MCP2515_ReadByte_Ext(dev_id, MCP2515_CANINTF);
    }
    else if (rxStatus.rxBuffer == MSG_IN_RXB1)
    {
      MCP2515_ReadRxSequence_Ext(dev_id, MCP2515_READ_RXB1SIDH, rxReg.rx_reg_array, sizeof(rxReg.rx_reg_array));
      MCP2515_ClearIntFlag_Ext(dev_id, MCP2515_INTF_RX0 | MCP2515_INTF_RX1);
    }

    if (rxStatus.msgType == dEXTENDED_CAN_MSG_ID_2_0B)
    {
      tempCanMsg->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
      tempCanMsg->frame.id = convertReg2ExtendedCANid(rxReg.RXBnEID8, rxReg.RXBnEID0, rxReg.RXBnSIDH, rxReg.RXBnSIDL);
    } 
    if (rxStatus.msgType == dSTANDARD_CAN_MSG_ID_2_0B)
    {
      tempCanMsg->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
      tempCanMsg->frame.id = convertReg2StandardCANid(rxReg.RXBnSIDH, rxReg.RXBnSIDL);
    }
    else
    {
      return 0; /* error idtype*/
    }
    
    tempCanMsg->frame.dlc   = rxReg.RXBnDLC & 0x0F; 
    tempCanMsg->frame.data0 = rxReg.RXBnD0;
    tempCanMsg->frame.data1 = rxReg.RXBnD1;
    tempCanMsg->frame.data2 = rxReg.RXBnD2;
    tempCanMsg->frame.data3 = rxReg.RXBnD3;
    tempCanMsg->frame.data4 = rxReg.RXBnD4;
    tempCanMsg->frame.data5 = rxReg.RXBnD5;
    tempCanMsg->frame.data6 = rxReg.RXBnD6;
    tempCanMsg->frame.data7 = rxReg.RXBnD7;
    
    returnValue = 1;
  }
  
  return (returnValue);
}

/**
 * @brief 查询接收缓冲区中待处理消息数量。
 *
 * @details
 * 读取 MCP2515 状态寄存器并统计 RX0/RX1 中的消息标志。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 缓冲区中消息数量（0~2）。
 */
uint8_t CANSPI_messagesInBuffer_Ext(MCP2515_DevId_t dev_id)
{
  uint8_t messageCount = 0;
  ctrl_status_t ctrlStatus;

  ctrlStatus.ctrl_status = MCP2515_ReadStatus_Ext(dev_id);
  
  if(ctrlStatus.RX0IF != 0)
  {
    messageCount++;
  }
  
  if(ctrlStatus.RX1IF != 0)
  {
    messageCount++;
  }
  
  return (messageCount);
}

/**
 * @brief 检查 MCP2515 是否进入总线关闭（Bus-Off）状态。
 *
 * @details
 * 读取 EFLG 寄存器并检查 TXBO 标志。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 如果总线关闭则返回 1，否则返回 0。
 */
uint8_t CANSPI_isBussOff(MCP2515_DevId_t dev_id)
{
  uint8_t returnValue = 0;
  ctrl_error_status_t errorStatus;
  
  errorStatus.error_flag_reg = MCP2515_ReadByte_Ext(dev_id, MCP2515_EFLG);
  
  if(errorStatus.TXBO == 1)
  {
    returnValue = 1;
  }
  
  return (returnValue);
}

/**
 * @brief 检查 MCP2515 是否处于接收错误被动（RX error passive）状态。
 *
 * @details
 * 读取 EFLG 寄存器并检查 RXEP 标志。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 如果接收错误被动则返回 1，否则返回 0。
 */
uint8_t CANSPI_isRxErrorPassive(MCP2515_DevId_t dev_id)
{
  uint8_t returnValue = 0;
  ctrl_error_status_t errorStatus; 

  errorStatus.error_flag_reg = MCP2515_ReadByte_Ext(dev_id, MCP2515_EFLG);

  if(errorStatus.RXEP == 1)
  {
    returnValue = 1;
  }
  
  return (returnValue);
}

/**
 * @brief 检查 MCP2515 是否处于发送错误被动（TX error passive）状态。
 *
 * @details
 * 读取 EFLG 寄存器并检查 TXEP 标志。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 如果发送错误被动则返回 1，否则返回 0。
 */
uint8_t CANSPI_isTxErrorPassive(MCP2515_DevId_t dev_id)
{
  uint8_t returnValue = 0;
  ctrl_error_status_t errorStatus;
  
  errorStatus.error_flag_reg = MCP2515_ReadByte_Ext(dev_id, MCP2515_EFLG);
  
  if(errorStatus.TXEP == 1)
  {
    returnValue = 1;
  }
  
  return (returnValue);
}

/**
 * @brief 将 MCP2515 接收寄存器内容转换为扩展 CAN ID。
 *
 * @details
 * 根据 MCP2515 接收缓冲区寄存器编码格式，合成 29 位扩展 CAN ID。
 */
static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) 
{
  uint32_t returnValue = 0;
  uint32_t ConvertedID = 0;
  uint8_t CAN_standardLo_ID_lo2bits;
  uint8_t CAN_standardLo_ID_hi3bits;
  
  CAN_standardLo_ID_lo2bits = (tempRXBn_SIDL & 0x03);
  CAN_standardLo_ID_hi3bits = (tempRXBn_SIDL >> 5);
  ConvertedID = (tempRXBn_SIDH << 3);
  ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
  ConvertedID = (ConvertedID << 2);
  ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
  ConvertedID = (ConvertedID << 8);
  ConvertedID = ConvertedID + tempRXBn_EIDH;
  ConvertedID = (ConvertedID << 8);
  ConvertedID = ConvertedID + tempRXBn_EIDL;
  returnValue = ConvertedID;    
  return (returnValue);
}

/**
 * @brief 将 MCP2515 接收寄存器内容转换为标准 CAN ID。
 *
 * @details
 * 根据 MCP2515 接收缓冲区寄存器编码格式，合成 11 位标准 CAN ID。
 */
static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) 
{
  uint32_t returnValue = 0;
  uint32_t ConvertedID;
  
  ConvertedID = (tempRXBn_SIDH << 3);
  ConvertedID = ConvertedID + (tempRXBn_SIDL >> 5);
  returnValue = ConvertedID;
  
  return (returnValue);
}

/**
 * @brief 将 CAN ID 转换成 MCP2515 寄存器格式用于发送。
 *
 * @details
 * 根据标准/扩展 ID 格式，将传入的 CAN ID 编码到 MCP2515 的 SIDH/SIDL/EID8/EID0 寄存器值。
 */
static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, id_reg_t *passedIdReg) 
{
  uint8_t wipSIDL = 0;
  
  if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B) 
  {
    //EID0
    passedIdReg->tempEID0 = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    
    //EID8
    passedIdReg->tempEID8 = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    
    //SIDL
    wipSIDL = 0x03 & tempPassedInID;
    tempPassedInID = tempPassedInID << 3;
    wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
    wipSIDL = wipSIDL + 0x08;
    passedIdReg->tempSIDL = 0xEB & wipSIDL;
    
    //SIDH
    tempPassedInID = tempPassedInID >> 8;
    passedIdReg->tempSIDH = 0xFF & tempPassedInID;
  } 
  else
  {
    passedIdReg->tempEID8 = 0;
    passedIdReg->tempEID0 = 0;
    tempPassedInID = tempPassedInID << 5;
    passedIdReg->tempSIDL = 0xFF & tempPassedInID;
    tempPassedInID = tempPassedInID >> 8;
    passedIdReg->tempSIDH = 0xFF & tempPassedInID;
  }
}


