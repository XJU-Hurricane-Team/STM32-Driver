/**
 * @file    MCP2515.h
 * @author  Dominate0017
 * @brief   MCP2515 SPI interface and interrupt helper routines.
 * @version 1.0
 * @date    2026-3-15
 * @note    Provides MCP2515 register definitions, SPI communication functions, and interrupt handling support.
 * @note    Origin Driver: https://github.com/eziya/STM32_SPI_MCP2515
 * @note    MCP2515 Datasheet: https://ww1.microchip.com/downloads/cn/DeviceDoc/21801D_CN.pdf
 */

#ifndef __MCP2515_H
#define	__MCP2515_H

#include <stdbool.h>
#include <CSP_Config.h>

#define MCP2515_RESET           0xC0

#define MCP2515_READ            0x03
#define MCP2515_READ_RXB0SIDH   0x90
#define MCP2515_READ_RXB0D0     0x92
#define MCP2515_READ_RXB1SIDH   0x94
#define MCP2515_READ_RXB1D0     0x96

#define MCP2515_WRITE           0x02
#define MCP2515_LOAD_TXB0SIDH   0x40    /* TX0 ID location */
#define MCP2515_LOAD_TXB0D0     0x41    /* TX0 Data location */
#define MCP2515_LOAD_TXB1SIDH   0x42    /* TX1 ID location */
#define MCP2515_LOAD_TXB1D0     0x43    /* TX1 Data location */
#define MCP2515_LOAD_TXB2SIDH   0x44    /* TX2 ID location */
#define MCP2515_LOAD_TXB2D0     0x45    /* TX2 Data location */

#define MCP2515_RTS_TX0         0x81
#define MCP2515_RTS_TX1         0x82
#define MCP2515_RTS_TX2         0x84
#define MCP2515_RTS_ALL         0x87
#define MCP2515_READ_STATUS     0xA0
#define MCP2515_RX_STATUS       0xB0
#define MCP2515_BIT_MOD         0x05

#define MCP2515_RXF0SIDH	0x00
#define MCP2515_RXF0SIDL	0x01
#define MCP2515_RXF0EID8	0x02
#define MCP2515_RXF0EID0	0x03
#define MCP2515_RXF1SIDH	0x04
#define MCP2515_RXF1SIDL	0x05
#define MCP2515_RXF1EID8	0x06
#define MCP2515_RXF1EID0	0x07
#define MCP2515_RXF2SIDH	0x08
#define MCP2515_RXF2SIDL	0x09
#define MCP2515_RXF2EID8	0x0A
#define MCP2515_RXF2EID0	0x0B
#define MCP2515_CANSTAT		0x0E
#define MCP2515_CANCTRL		0x0F

#define MCP2515_RXF3SIDH	0x10
#define MCP2515_RXF3SIDL	0x11
#define MCP2515_RXF3EID8	0x12
#define MCP2515_RXF3EID0	0x13
#define MCP2515_RXF4SIDH	0x14
#define MCP2515_RXF4SIDL	0x15
#define MCP2515_RXF4EID8	0x16
#define MCP2515_RXF4EID0	0x17
#define MCP2515_RXF5SIDH	0x18
#define MCP2515_RXF5SIDL	0x19
#define MCP2515_RXF5EID8	0x1A
#define MCP2515_RXF5EID0	0x1B
#define MCP2515_TEC		0x1C
#define MCP2515_REC		0x1D

#define MCP2515_RXM0SIDH	0x20
#define MCP2515_RXM0SIDL	0x21
#define MCP2515_RXM0EID8	0x22
#define MCP2515_RXM0EID0	0x23
#define MCP2515_RXM1SIDH	0x24
#define MCP2515_RXM1SIDL	0x25
#define MCP2515_RXM1EID8	0x26
#define MCP2515_RXM1EID0	0x27
#define MCP2515_CNF3		0x28
#define MCP2515_CNF2		0x29
#define MCP2515_CNF1		0x2A
#define MCP2515_CANINTE		0x2B
#define MCP2515_CANINTF		0x2C
#define MCP2515_EFLG		0x2D

#define MCP2515_TXB0CTRL	0x30
#define MCP2515_TXB1CTRL	0x40
#define MCP2515_TXB2CTRL	0x50
#define MCP2515_RXB0CTRL	0x60
#define MCP2515_RXB0SIDH	0x61
#define MCP2515_RXB1CTRL	0x70
#define MCP2515_RXB1SIDH	0x71

#define MSG_IN_RXB0             0x01
#define MSG_IN_RXB1             0x02
#define MSG_IN_BOTH_BUFFERS     0x03

#define MCP2515_DEVICE_CNT    3 
#define SPI_TIMEOUT             10

typedef enum {
    MCP2515_DEV_1 = 0U,      // 对应DEV1
    MCP2515_DEV_2,          // 对应DEV2
    MCP2515_DEV_3             // 对应DEV3
} MCP2515_DevId_t;

typedef union{
  struct{
    unsigned RX0IF      : 1;
    unsigned RX1IF      : 1;
    unsigned TXB0REQ    : 1;
    unsigned TX0IF      : 1;
    unsigned TXB1REQ    : 1;
    unsigned TX1IF      : 1;
    unsigned TXB2REQ    : 1;
    unsigned TX2IF      : 1;
  };
  uint8_t ctrl_status;  
}ctrl_status_t;

typedef union{
  struct{
    unsigned filter     : 3;
    unsigned msgType    : 2;
    unsigned unusedBit  : 1;
    unsigned rxBuffer   : 2;
  };
  uint8_t ctrl_rx_status;
}ctrl_rx_status_t;

typedef union{
  struct{
    unsigned EWARN      :1;
    unsigned RXWAR      :1;
    unsigned TXWAR      :1;
    unsigned RXEP       :1;
    unsigned TXEP       :1;
    unsigned TXBO       :1;
    unsigned RX0OVR     :1;
    unsigned RX1OVR     :1;  
  };
  uint8_t error_flag_reg;
}ctrl_error_status_t;

typedef union{
  struct{
    uint8_t RXBnSIDH;
    uint8_t RXBnSIDL;
    uint8_t RXBnEID8;
    uint8_t RXBnEID0;
    uint8_t RXBnDLC;
    uint8_t RXBnD0;
    uint8_t RXBnD1;
    uint8_t RXBnD2;
    uint8_t RXBnD3;
    uint8_t RXBnD4;
    uint8_t RXBnD5;
    uint8_t RXBnD6;
    uint8_t RXBnD7;
  };
  uint8_t rx_reg_array[13];
}rx_reg_t;

typedef struct{
  uint8_t RXF0SIDH;
  uint8_t RXF0SIDL;
  uint8_t RXF0EID8;
  uint8_t RXF0EID0;
}RXF0;

typedef struct{
  uint8_t RXF1SIDH;
  uint8_t RXF1SIDL;
  uint8_t RXF1EID8;
  uint8_t RXF1EID0;
}RXF1;

typedef struct{
  uint8_t RXF2SIDH;
  uint8_t RXF2SIDL;
  uint8_t RXF2EID8;
  uint8_t RXF2EID0;
}RXF2;

typedef struct{
  uint8_t RXF3SIDH;
  uint8_t RXF3SIDL;
  uint8_t RXF3EID8;
  uint8_t RXF3EID0;
}RXF3;

typedef struct{
  uint8_t RXF4SIDH;
  uint8_t RXF4SIDL;
  uint8_t RXF4EID8;
  uint8_t RXF4EID0;
}RXF4;

typedef struct{
  uint8_t RXF5SIDH;
  uint8_t RXF5SIDL;
  uint8_t RXF5EID8;
  uint8_t RXF5EID0;
}RXF5;

typedef struct{
  uint8_t RXM0SIDH;
  uint8_t RXM0SIDL;
  uint8_t RXM0EID8;
  uint8_t RXM0EID0;
}RXM0;

typedef struct{
  uint8_t RXM1SIDH;
  uint8_t RXM1SIDL;
  uint8_t RXM1EID8;
  uint8_t RXM1EID0;
}RXM1;

typedef struct{
  uint8_t tempSIDH;
  uint8_t tempSIDL;
  uint8_t tempEID8;
  uint8_t tempEID0;
}id_reg_t;

bool MCP2515_Initialize(void);
void MCP2515_WriteByteSequence_Ext(MCP2515_DevId_t dev_id, uint8_t startAddress, uint8_t endAddress, uint8_t *data);
void MCP2515_LoadTxBuffer_Ext(MCP2515_DevId_t dev_id, uint8_t instruction, uint8_t data);
void MCP2515_Reset_Ext(MCP2515_DevId_t dev_id);
uint8_t MCP2515_ReadByte_Ext(MCP2515_DevId_t dev_id, uint8_t address);
void MCP2515_WriteByte_Ext(MCP2515_DevId_t dev_id, uint8_t address, uint8_t data);
void MCP2515_ReadRxSequence_Ext(MCP2515_DevId_t dev_id, uint8_t instruction, uint8_t *data, uint8_t length);
void MCP2515_LoadTxSequence_Ext(MCP2515_DevId_t dev_id, uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data);
void MCP2515_BitModify_Ext(MCP2515_DevId_t dev_id, uint8_t address, uint8_t mask, uint8_t data);
uint8_t MCP2515_ReadStatus_Ext(MCP2515_DevId_t dev_id);
uint8_t MCP2515_GetRxStatus_Ext(MCP2515_DevId_t dev_id);
bool MCP2515_SetConfigMode_Ext(MCP2515_DevId_t dev_id);
bool MCP2515_SetNormalMode_Ext(MCP2515_DevId_t dev_id);
bool MCP2515_SetSleepMode_Ext(MCP2515_DevId_t dev_id);
void MCP2515_RequestToSend_Ext(MCP2515_DevId_t dev_id, uint8_t instruction);
void MCP2515_SetCSPin(MCP2515_DevId_t dev_id, GPIO_PinState state);
void MCP2515_INT_Init(void);
void MCP2515_ClearIntFlag_Ext(MCP2515_DevId_t dev_id, uint8_t intMask);
void MCP2515_EnableInt_Ext(MCP2515_DevId_t dev_id, uint8_t intMask);
void MCP2515_DisableInt_Ext(MCP2515_DevId_t dev_id, uint8_t intMask);

// 启用的设备开关（1=启用，0=禁用）
#define MCP2515_DEV1_ENABLE 1   // 设备1
#define MCP2515_DEV2_ENABLE 0   // 设备2
#define MCP2515_DEV3_ENABLE 0   // 设备3

#if MCP2515_DEV1_ENABLE
#define MCP2515_DEV1_INT_PIN    GPIO_PIN_13
#define MCP2515_DEV1_INT_PORT   GPIOB
#define MCP2515_DEV1_SPI_HANDLE &spi1_handle
#define MCP2515_DEV1_CS_PIN     GPIO_PIN_12
#define MCP2515_DEV1_CS_PORT    GPIOB
#define MCP2515_DEV1_IRQn       EXTI15_10_IRQn
#define MCP2515_DEV1_IRQ_PRIO   3
#define MCP2515_DEV1_IRQ_SUBPRIO 0
#endif /* MCP2515_DEV1_ENABLE */

#if MCP2515_DEV2_ENABLE
#define MCP2515_DEV2_INT_PIN    GPIO_PIN_14
#define MCP2515_DEV2_INT_PORT   GPIOB
#define MCP2515_DEV2_SPI_HANDLE &spi2_handle
#define MCP2515_DEV2_CS_PIN     GPIO_PIN_13
#define MCP2515_DEV2_CS_PORT    GPIOB
#define MCP2515_DEV2_IRQn       EXTI15_10_IRQn
#define MCP2515_DEV2_IRQ_PRIO   3
#define MCP2515_DEV2_IRQ_SUBPRIO 0
#endif /* MCP2515_DEV2_ENABLE */

#if MCP2515_DEV3_ENABLE
#define MCP2515_DEV3_INT_PIN    GPIO_PIN_15
#define MCP2515_DEV3_INT_PORT   GPIOB
#define MCP2515_DEV3_SPI_HANDLE &spi3_handle
#define MCP2515_DEV3_CS_PIN     GPIO_PIN_14
#define MCP2515_DEV3_CS_PORT    GPIOB
#define MCP2515_DEV3_IRQn       EXTI15_10_IRQn
#define MCP2515_DEV3_IRQ_PRIO   3
#define MCP2515_DEV3_IRQ_SUBPRIO 0
#endif /* MCP2515_DEV3_ENABLE */

#if MCP2515_DEV1_ENABLE
#define MCP2515_DEV1_CS_HIGH() HAL_GPIO_WritePin(MCP2515_DEV1_CS_PORT, MCP2515_DEV1_CS_PIN, GPIO_PIN_SET)
#define MCP2515_DEV1_CS_LOW()  HAL_GPIO_WritePin(MCP2515_DEV1_CS_PORT, MCP2515_DEV1_CS_PIN, GPIO_PIN_RESET)
#endif /* MCP2515_DEV1_ENABLE */

#if MCP2515_DEV2_ENABLE
#define MCP2515_DEV2_CS_HIGH() HAL_GPIO_WritePin(MCP2515_DEV2_CS_PORT, MCP2515_DEV2_CS_PIN, GPIO_PIN_SET)
#define MCP2515_DEV2_CS_LOW()  HAL_GPIO_WritePin(MCP2515_DEV2_CS_PORT, MCP2515_DEV2_CS_PIN, GPIO_PIN_RESET)
#endif /* MCP2515_DEV2_ENABLE */

#if MCP2515_DEV3_ENABLE
#define MCP2515_DEV3_CS_HIGH() HAL_GPIO_WritePin(MCP2515_DEV3_CS_PORT, MCP2515_DEV3_CS_PIN, GPIO_PIN_SET)
#define MCP2515_DEV3_CS_LOW()  HAL_GPIO_WritePin(MCP2515_DEV3_CS_PORT, MCP2515_DEV3_CS_PIN, GPIO_PIN_RESET)
#endif /* MCP2515_DEV3_ENABLE */

#define MCP2515_INT_USE    1

#define MCP2515_INTE_RX0    (1 << 0)  // RX0接收中断
#define MCP2515_INTE_RX1    (1 << 1)  // RX1接收中断
#define MCP2515_INTE_TX0    (1 << 2)  // TX0发送完成中断
#define MCP2515_INTE_TX1    (1 << 3)  // TX1发送完成中断
#define MCP2515_INTE_TX2    (1 << 4)  // TX2发送完成中断
#define MCP2515_INTE_ERR    (1 << 5)  // 错误中断
#define MCP2515_INTE_WAKE   (1 << 6)  // 唤醒中断
#define MCP2515_INTE_MERR   (1 << 7)  // 总线错误中断

#define MCP2515_INTF_RX0    (1 << 0)  
#define MCP2515_INTF_RX1    (1 << 1)
#define MCP2515_INTF_TX0    (1 << 2)
#define MCP2515_INTF_TX1    (1 << 3)
#define MCP2515_INTF_TX2    (1 << 4)
#define MCP2515_INTF_ERR    (1 << 5)
#define MCP2515_INTF_WAKE   (1 << 6)
#define MCP2515_INTF_MERR   (1 << 7)

#endif /*__MCP2515_H */