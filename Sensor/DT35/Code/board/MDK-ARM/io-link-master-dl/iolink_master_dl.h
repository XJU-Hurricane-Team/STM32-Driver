/**
 ******************************************************************************
 * @file    iolink_master_dl.h
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V2.0.0
 * @date    2024-04-24
 * @brief   IO-Link master data link layer
 * @note    COPYRIGHT 2024 UESTC-LIMITI
 ******************************************************************************
 */

#ifndef __IOLINK_MASTER_DL_H
#define __IOLINK_MASTER_DL_H

#include "main.h"
#include "l6360.h"

#define CHECKSUM_SEED (0x52U)

typedef uint8_t OD_t;
typedef uint8_t PD_t;

typedef struct
{
	uint8_t data[5];
	uint8_t counter;
	uint8_t total;
	uint8_t complete;
} Recv_t;

typedef struct
{
	uint8_t data[5];
	uint8_t counter;
	uint8_t total;
	uint8_t complete;
} Trans_t;

typedef enum
{
	DL_OK,
	DL_ERROR,
	DL_TIMEOUT,
} Dl_Status_t;

/************************** MC **************************/
typedef enum
{
	Process		= 0U,
	Page		= 1U,
	Diagnosis	= 2U,
	ISDU		= 3U,
} MC_Communication_Channel_t;

typedef enum
{
	Write	= 0U,
	Read	= 1U,
} MC_RW_t;

typedef struct
{
	MC_RW_t rw;
	MC_Communication_Channel_t communication_channel;
	uint8_t address;
	uint8_t buffer;
} MC_t;

/************************** CKT **************************/
typedef enum
{
	Type_0	= 0U,
	Type_1	= 1U,
	Type_2	= 2U,
	Type_3	= 3U,
} CKT_Type_t;

typedef struct
{
	CKT_Type_t type;
	uint8_t checksum;
	uint8_t buffer;
} CKT_t;

/************************** CKS **************************/
typedef enum
{
	No_Event	= 0U,
	Event		= 1U,
} CKS_Event_t;

typedef enum
{
	Process_Data_Valid		= 0U,
	Process_Data_Invalid	= 1U,
} CKS_Status_t;

typedef struct
{
	CKS_Event_t event;
	CKS_Status_t status;
	uint8_t checksum;
	uint8_t buffer;
} CKS_t;

/************************** M_Sequence_Type **************************/
typedef struct
{
	MC_t mc;
	CKT_t ckt;
	OD_t od;
	CKS_t cks;
} M_Sequence_Type_0;

typedef struct
{
	MC_t mc;
	CKT_t ckt;
	OD_t od;
	PD_t pd0;
	PD_t pd1;
	CKS_t cks;
} M_Sequence_Type_2_2;

/************************** I-Service **************************/
typedef enum
{
	No_Service							= 0U,
	Write_Request_8_Bit_Index			= 1U,
	Write_Request_8_Bit_Index_Subindex	= 2U,
	Write_Request_16_Bit_Index_Subindex	= 3U,
	Write_Response_Negative				= 4U,
	Write_Response_Positive				= 5U,
	Read_Request_8_Bit_Index			= 9U,
	Read_Request_8_Bit_Index_Subindex	= 10U,
	Read_Request_16_Bit_Index_Subindex	= 11U,
	Read_Response_Negative				= 12U,
	Read_Response_Positive				= 13U,
} I_Service_t;

typedef enum
{
	Count		= 0U,
	Count_Last	= 0x0FU,
	Start		= 0x10U,
	Idle_1		= 0x11U,
	Idle_2		= 0x12U, // Reserved
	Abort		= 0x1FU,
} Flow_Control_t;

typedef struct
{
	uint8_t I_Service_Octect;
	uint8_t Index;
	uint8_t data;
	uint8_t CHKPDU;
} ISDU_Uint8_t;

Dl_Status_t dl_wake_up(void);
//void dl_receive_handler(uint8_t portId, uint8_t rxData);
void delay_us(uint32_t us);
Dl_Status_t dl_read_type_0(M_Sequence_Type_0* m_sequence, uint8_t* data_out, MC_RW_t RW, MC_Communication_Channel_t CC, uint8_t Address, CKT_Type_t Type);
Dl_Status_t dl_write_type_0(M_Sequence_Type_0* m_sequence, uint8_t data_in, uint8_t Address);
Dl_Status_t dl_read_type_2_2(M_Sequence_Type_2_2* m_sequence, uint16_t* data_out, OD_t* od_out,  uint8_t Address);
Dl_Status_t dl_write_type_2_2(M_Sequence_Type_2_2* m_sequence, OD_t data_in, uint16_t* data_out, uint8_t Address);
Dl_Status_t dl_write_ISDU_uint8(uint8_t index, uint8_t data);
Dl_Status_t dl_reconnection_handler(void);

#endif
