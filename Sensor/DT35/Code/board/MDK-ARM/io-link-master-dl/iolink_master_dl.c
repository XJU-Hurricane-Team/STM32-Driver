/**
 ******************************************************************************
 * @file    iolink_master_dl.c
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V2.0.0
 * @date    2024-04-24
 * @brief   IO-Link master data link layer
 * @note    COPYRIGHT 2024 UESTC-LIMITI
 ******************************************************************************
 */

#include "iolink_master_dl.h"

#define BIT_IN_BYTE(bit, byte) (((byte) >> (bit)) & 1U)

M_Sequence_Type_0 m_sequence0;
M_Sequence_Type_2_2 m_sequence2;

uint8_t rxData;

Recv_t recv = {0};
Trans_t trans = {0};

extern uint16_t processData;

/**
 * @brief Wakes up the IO-Link device.
 *
 * This function wakes up the IO-Link device by sending a wake-up sequence.
 * It checks the state of the CQ line and sends the appropriate data based on the state.
 * After sending the wake-up sequence, it reads the response from the device to verify the wake-up.
 *
 * @return Dl_Status_t The status of the wake-up operation. Possible values are:
 *         - DL_OK: Wake-up successful.
 *         - DL_ERROR: Error occurred during wake-up.
 */
Dl_Status_t dl_wake_up(void)
{
	HAL_StatusTypeDef status;
	uint8_t outData;

	trans.data[0] = 0xA2;
	trans.data[1] = 0x00;
	L6360_Set_Buadrate(COMX);

	// disable enCQ to get the state of the CQ line
	L6360_Set_EN_CQ(0U);
	L6360_Write_Register((uint8_t)L6360_CONFIGURATION, L6360_CONFIGURATION_CQ_OFF);
	HAL_Delay(1);
	L6360_Write_Register((uint8_t)L6360_CONFIGURATION, L6360_CONFIGURATION_CQ_PUSH_PULL);

	HAL_Delay(1);

	if (HAL_GPIO_ReadPin(L6360_OUT_CQ_GPIO_Port, L6360_OUT_CQ_Pin) == GPIO_PIN_SET)
	{
		// outCQ is high, CQ is low, ↑↓
		L6360_Set_EN_CQ(1U); // enable
		trans.total = 1;
		trans.counter = 0;

		status = L6360_Send_IN_CQ_Data(&trans.data[1], 1);
		if (status != HAL_OK)
		{
			return DL_ERROR;
		}
	}
	else
	{
		// outCQ is low, CQ is high, ↓
		L6360_Set_EN_CQ(1U);
	}
	delay_us(300);
//	L6360_Set_Buadrate(COM3);
//	trans.total = 2;
//	trans.counter = 0;
//	status = L6360_Send_IN_CQ_Data(trans.data, trans.total);

//	if (status != HAL_OK)
//	{
//		return DL_ERROR;
//	}

	L6360_Set_Buadrate(COM2);
	HAL_Delay(0);
	if (dl_read_type_0(&m_sequence0, &outData, Read, Page, 0x02, Type_0) != DL_OK)
	{
		return DL_ERROR;
	}
	return DL_OK;
}

/*
set the M-seqence control octet
	R/W		|Communication channel	|Address
	1 bit	|2 bits					|5 bits
*/
void set_MC(MC_t *MC, MC_RW_t RW, MC_Communication_Channel_t CC, uint8_t Address)
{
	MC->rw = RW; // read / write
	MC->communication_channel = CC;
	MC->address = ((Address) & 0x1F); // five bits
	MC->buffer = 0;
	MC->buffer |= (RW << 7);
	MC->buffer |= (CC << 5);
	MC->buffer |= ((Address) & 0x1F);
}

/*
set the Checksum/M-sequence type octet
	M-seqence type	|Checksum
	2 bits			|6 bits
*/
void set_CKT(CKT_t *CKT, CKT_Type_t Type)
{
	CKT->type = Type;
	CKT->buffer = 0;
	CKT->buffer |= (Type << 6);
}

uint8_t checksum_8_to_6(uint8_t cs8)
{
	uint8_t cs6 = 0;
	cs6 |= (BIT_IN_BYTE(7, cs8) ^
			BIT_IN_BYTE(5, cs8) ^
			BIT_IN_BYTE(3, cs8) ^
			BIT_IN_BYTE(1, cs8))
		   << 5;

	cs6 |= (BIT_IN_BYTE(6, cs8) ^
			BIT_IN_BYTE(4, cs8) ^
			BIT_IN_BYTE(2, cs8) ^
			BIT_IN_BYTE(0, cs8))
		   << 4;

	cs6 |= (BIT_IN_BYTE(7, cs8) ^
			BIT_IN_BYTE(6, cs8))
		   << 3;

	cs6 |= (BIT_IN_BYTE(5, cs8) ^
			BIT_IN_BYTE(4, cs8))
		   << 2;

	cs6 |= (BIT_IN_BYTE(3, cs8) ^
			BIT_IN_BYTE(2, cs8))
		   << 1;

	cs6 |= (BIT_IN_BYTE(1, cs8) ^
			BIT_IN_BYTE(0, cs8))
		   << 0;

	return cs6;
}


/**
 * @brief Reads data of Type 0 from the IO-Link device.
 *
 * This function reads data of Type 0 from the IO-Link device. It sets the MC and CKT octets,
 * calculates the checksum, transmits the data, receives the response, processes the response, and verifies the checksum.
 *
 * @param m_sequence Pointer to the M_Sequence_Type_0 structure that holds the MC and CKT octets.
 * @param data_out Pointer to the variable where the received data will be stored.
 * @param RW MC_RW_t value indicating the read or write operation.
 * @param CC MC_Communication_Channel_t value indicating the communication channel.
 * @param Address Address of the device.
 * @param Type CKT_Type_t value indicating the CKT type.
 * @return Dl_Status_t value indicating the status of the IO-Link communication.
 *         - DL_OK: Communication successful.
 *         - DL_TIMEOUT: Communication timeout.
 *         - DL_ERROR: Communication error.
 */
Dl_Status_t dl_read_type_0(M_Sequence_Type_0 *m_sequence,
						   uint8_t *data_out,
						   MC_RW_t RW,
						   MC_Communication_Channel_t CC,
						   uint8_t Address,
						   CKT_Type_t Type)
{
	HAL_StatusTypeDef status;

	// set the MC and CKT octets
	set_MC(&(m_sequence->mc), RW, CC, Address);
	set_CKT(&(m_sequence->ckt), Type);

	// calculate the checksum
	uint8_t checksum_8 = CHECKSUM_SEED ^
						 (m_sequence->mc.buffer) ^
						 (m_sequence->ckt.buffer);
	uint8_t checksum_6 = checksum_8_to_6(checksum_8);
	m_sequence->ckt.buffer |= checksum_6;

	// transmit
	L6360_Set_EN_CQ(1U); // enable
	trans.total = 2;
	trans.counter = 0;
	trans.data[0] = m_sequence->mc.buffer;
	trans.data[1] = m_sequence->ckt.buffer;
	status = L6360_Send_IN_CQ_Data(trans.data, trans.total);

	if (status == HAL_TIMEOUT)
	{
		return DL_TIMEOUT;
	}

	// receive
	L6360_Set_EN_CQ(0U); // disable
	recv.total = 2;
	recv.counter = 0;
	status = L6360_Recv_OUT_CQ_Data(recv.data, recv.total);

	if (status == HAL_TIMEOUT)
	{
		return DL_TIMEOUT;
	}

	// process
	m_sequence->od = recv.data[0];
	m_sequence->cks.buffer = recv.data[1];
	m_sequence->cks.event = (CKS_Event_t)((m_sequence->cks.buffer >> 7) & 0x01);
	m_sequence->cks.status = (CKS_Status_t)((m_sequence->cks.buffer >> 6) & 0x01);
	m_sequence->cks.checksum = ((m_sequence->cks.buffer >> 0) & 0x3F);

	// verify the checksum
	checksum_8 = CHECKSUM_SEED ^
				 (m_sequence->od) ^
				 (m_sequence->cks.buffer & 0xC0); // 1100 0000
	if (checksum_8_to_6(checksum_8) == m_sequence->cks.checksum)
	{
		*data_out = m_sequence->od;
	}
	else
	{
		return DL_ERROR;
	}

	return DL_OK;
}

/**
 * @brief Writes data of Type 0 to the IO-Link device.
 *
 * This function writes data of Type 0 to the IO-Link device. It sets the MC and CKT octets,
 * calculates the checksum, transmits the data, receives the response, processes the response,
 * and verifies the checksum.
 *
 * @param m_sequence Pointer to the M_Sequence_Type_0 structure containing the MC, CKT, and OD octets.
 * @param data_in The data to be written to the IO-Link master device.
 * @param Address The address of the IO-Link master device.
 * @return Dl_Status_t The status of the write operation. Possible values are:
 *         - DL_OK: Write operation successful.
 *         - DL_ERROR: Error occurred during the write operation.
 *         - DL_TIMEOUT: Timeout occurred during the write operation.
 */
Dl_Status_t dl_write_type_0(M_Sequence_Type_0 *m_sequence, uint8_t data_in, uint8_t Address)
{
	HAL_StatusTypeDef status;

	// set the MC and CKT octets
	set_MC(&(m_sequence->mc), Write, Page, Address);
	set_CKT(&(m_sequence->ckt), Type_0);
	m_sequence->od = data_in;

	// calculate the checksum
	uint8_t checksum_8 = CHECKSUM_SEED ^
						 (m_sequence->mc.buffer) ^
						 (m_sequence->ckt.buffer) ^
						 (m_sequence->od);
	uint8_t checksum_6 = checksum_8_to_6(checksum_8);
	m_sequence->ckt.buffer |= checksum_6;

	// transmit
	L6360_Set_EN_CQ(1U); // enable
	trans.total = 3;
	trans.counter = 0;
	trans.data[0] = m_sequence->mc.buffer;
	trans.data[1] = m_sequence->ckt.buffer;
	trans.data[2] = m_sequence->od;
	status = L6360_Send_IN_CQ_Data(trans.data, trans.total);

	if (status != HAL_OK)
	{
		return DL_ERROR;
	}

	// receive
	L6360_Set_EN_CQ(0U); // disable
	recv.total = 1;
	recv.counter = 0;
	status = L6360_Recv_OUT_CQ_Data(recv.data, recv.total);

	if (status == HAL_TIMEOUT)
	{
		return DL_TIMEOUT;
	}

	// process
	m_sequence->cks.buffer = recv.data[0];
	m_sequence->cks.event = (CKS_Event_t)((m_sequence->cks.buffer >> 7) & 0x01);
	m_sequence->cks.status = (CKS_Status_t)((m_sequence->cks.buffer >> 6) & 0x01);
	m_sequence->cks.checksum = ((m_sequence->cks.buffer >> 0) & 0x3F);

	// verify the checksum
	checksum_8 = CHECKSUM_SEED ^
				 (m_sequence->cks.buffer & 0xC0); // 1100 0000
	if (checksum_8_to_6(checksum_8) == m_sequence->cks.checksum)
	{
		// correct data
		HAL_Delay(1);
	}
	else
	{
		// error data
		HAL_Delay(1);
		return DL_ERROR;
	}
	return DL_OK;
}

Dl_Status_t dl_read_type_2_2(M_Sequence_Type_2_2 *m_sequence, uint16_t *data_out, OD_t *od_out, uint8_t Address)
{
	HAL_StatusTypeDef status;

	// set the MC and CKT octets
	set_MC(&(m_sequence->mc), Read, ISDU, Address);
	set_CKT(&(m_sequence->ckt), Type_2);

	// calculate the checksum
	uint8_t checksum_8 = CHECKSUM_SEED ^
						 (m_sequence->mc.buffer) ^
						 (m_sequence->ckt.buffer);
	uint8_t checksum_6 = checksum_8_to_6(checksum_8);
	m_sequence->ckt.buffer |= checksum_6;

	// transmit
	L6360_Set_EN_CQ(1U); // enable
	trans.total = 2;
	trans.counter = 0;
	trans.data[0] = m_sequence->mc.buffer;
	trans.data[1] = m_sequence->ckt.buffer;
	status = L6360_Send_IN_CQ_Data(trans.data, trans.total);

	if (status != HAL_OK)
	{
		return DL_ERROR;
	}

	// receive
	L6360_Set_EN_CQ(0U); // disable
	recv.total = 4;
	recv.counter = 0;
	status = L6360_Recv_OUT_CQ_Data(recv.data, recv.total);

	if (status == HAL_TIMEOUT)
	{
		return DL_TIMEOUT;
	}

	// process
	m_sequence->od = recv.data[0];
	m_sequence->pd0 = recv.data[1];
	m_sequence->pd1 = recv.data[2];
	m_sequence->cks.buffer = recv.data[3];
	m_sequence->cks.event = (CKS_Event_t)((m_sequence->cks.buffer >> 7) & 0x01);
	m_sequence->cks.status = (CKS_Status_t)((m_sequence->cks.buffer >> 6) & 0x01);
	m_sequence->cks.checksum = ((m_sequence->cks.buffer >> 0) & 0x3F);

	// verify the checksum
	checksum_8 = CHECKSUM_SEED ^
				 (m_sequence->od) ^
				 (m_sequence->pd0) ^
				 (m_sequence->pd1) ^
				 (m_sequence->cks.buffer & 0xC0); // 1100 0000
	if (checksum_8_to_6(checksum_8) == m_sequence->cks.checksum)
	{
		if (data_out != NULL)
		{
			*data_out = (uint16_t)((uint16_t)(m_sequence->pd0) << 8U) | ((uint16_t)(m_sequence->pd1));
		}
		if (od_out != NULL)
		{
			*od_out = m_sequence->od;
		}
	}
	else
	{
		// error data
		return DL_ERROR;
	}
	return DL_OK;
}

Dl_Status_t dl_write_type_2_2(M_Sequence_Type_2_2 *m_sequence, OD_t data_in, uint16_t *data_out, uint8_t Address)
{
	HAL_StatusTypeDef status;

	// set the MC and CKT octets
	set_MC(&(m_sequence->mc), Write, ISDU, Address);
	set_CKT(&(m_sequence->ckt), Type_2);
	m_sequence->od = data_in;

	// calculate the checksum
	uint8_t checksum_8 = CHECKSUM_SEED ^
						 (m_sequence->mc.buffer) ^
						 (m_sequence->ckt.buffer) ^
						 (m_sequence->od);
	uint8_t checksum_6 = checksum_8_to_6(checksum_8);
	m_sequence->ckt.buffer |= checksum_6;

	// transmit
	L6360_Set_EN_CQ(1U); // enable
	trans.total = 3;
	trans.counter = 0;
	trans.data[0] = m_sequence->mc.buffer;
	trans.data[1] = m_sequence->ckt.buffer;
	trans.data[2] = m_sequence->od;
	status = L6360_Send_IN_CQ_Data(trans.data, trans.total);

	if (status != HAL_OK)
	{
		return DL_ERROR;
	}

	// receive
	L6360_Set_EN_CQ(0U); // disable
	recv.total = 3;
	recv.counter = 0;
	status = L6360_Recv_OUT_CQ_Data(recv.data, recv.total);

	if (status == HAL_TIMEOUT)
	{
		return DL_TIMEOUT;
	}

	// process
	m_sequence->pd0 = recv.data[0];
	m_sequence->pd1 = recv.data[1];
	m_sequence->cks.buffer = recv.data[2];
	m_sequence->cks.event = (CKS_Event_t)((m_sequence->cks.buffer >> 7) & 0x01);
	m_sequence->cks.status = (CKS_Status_t)((m_sequence->cks.buffer >> 6) & 0x01);
	m_sequence->cks.checksum = ((m_sequence->cks.buffer >> 0) & 0x3F);

	// verify the checksum
	checksum_8 = CHECKSUM_SEED ^
				 (m_sequence->pd0) ^
				 (m_sequence->pd1) ^
				 (m_sequence->cks.buffer & 0xC0); // 1100 0000
	if (checksum_8_to_6(checksum_8) == m_sequence->cks.checksum)
	{
		*data_out = (uint16_t)((uint16_t)(m_sequence->pd0) << 8U) | ((uint16_t)(m_sequence->pd1));
	}
	else
	{
		// error data
		return DL_ERROR;
	}
	return DL_OK;
}

/**
 * @brief Writes an 8-bit data value to the IO-Link device.
 * 
 * This function writes an 8-bit data value to the IO-Link device using the specified index.
 * It performs the necessary sequence of write operations and checks for successful completion.
 * 
 * @param index The index of the data value to be written.
 * @param data The 8-bit data value to be written.
 * @return Dl_Status_t The status of the write operation. Returns DL_OK if successful, DL_ERROR otherwise.
 */
Dl_Status_t dl_write_ISDU_uint8(uint8_t index, uint8_t data)
{
	Dl_Status_t status;
	ISDU_Uint8_t isdu_uint8;
	OD_t od_temp = 0x01;
	isdu_uint8.I_Service_Octect = (Write_Request_8_Bit_Index << 4) | 0x04;
	isdu_uint8.Index = index;
	isdu_uint8.data = data;
	isdu_uint8.CHKPDU = isdu_uint8.I_Service_Octect ^
						isdu_uint8.Index ^
						isdu_uint8.data;
	status = dl_write_type_2_2(&m_sequence2, isdu_uint8.I_Service_Octect, &processData, Start);
	if (status != DL_OK)
	{
		return DL_ERROR;
	}
	HAL_Delay(0);
	dl_write_type_2_2(&m_sequence2, isdu_uint8.Index, &processData, 0x01);
	if (status != DL_OK)
	{
		return DL_ERROR;
	}
	HAL_Delay(0);
	dl_write_type_2_2(&m_sequence2, isdu_uint8.data, &processData, 0x02);
	if (status != DL_OK)
	{
		return DL_ERROR;
	}
	HAL_Delay(0);
	dl_write_type_2_2(&m_sequence2, isdu_uint8.CHKPDU, &processData, 0x03);
	if (status != DL_OK)
	{
		return DL_ERROR;
	}
	HAL_Delay(0);
	while (od_temp == 0x01)
	{
		status = dl_read_type_2_2(&m_sequence2, &processData, &od_temp, Start);
		if (status != DL_OK)
		{
			return DL_ERROR;
		}
		HAL_Delay(0);
	}
	if (od_temp == ((Write_Response_Positive << 4) | 0x02))
	{
		status = dl_read_type_2_2(&m_sequence2, &processData, &od_temp, 0x01);
		if (status != DL_OK)
		{
			return DL_ERROR;
		}
		// CHKPDU, since there is only one octect of received data, the CHKPDU is the data itself
		if (od_temp == ((Write_Response_Positive << 4) | 0x02))
		{
			return DL_OK;
		}
	}
	else
	{
		return DL_ERROR;
	}
	return DL_ERROR;
}

void delay_us(uint32_t us)
{
	uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
	while (delay--)
	{
		;
	}
}

/**
 * @brief Handles the reconnection process for the IO-Link master.
 * 
 * This function performs the necessary steps to handle the reconnection process for the IO-Link master.
 * It sets the EN_LPlus pin to 0, waits for 500 milliseconds, sets the EN_LPlus pin to 1, and waits for another 500 milliseconds.
 * Then, it performs a fallback operation by writing a type 0 command with the sequence 0x5A and data 0x00.
 * After a delay of 10 milliseconds, it performs a re-wake-up operation by calling the dl_wake_up() function.
 * If the wake-up operation fails, it returns DL_ERROR.
 * Otherwise, it sets the IO-Link master to operate mode by writing a type 0 command with the sequence 0x99 and data 0x00.
 * If the write operation fails, it returns DL_ERROR.
 * Finally, it returns DL_OK to indicate a successful reconnection process.
 * 
 * @return Dl_Status_t The status of the reconnection process (DL_OK if successful, DL_ERROR otherwise).
 */
Dl_Status_t dl_reconnection_handler(void)
{
	Dl_Status_t status;
//	L6360_Reset();
	L6360_Set_EN_LPlus(0);
	HAL_Delay(500);
	L6360_Set_EN_LPlus(1);
	HAL_Delay(500);

	// Fallback
	dl_write_type_0(&m_sequence0, 0x5A, 0x00);
	HAL_Delay(10);

	// ReWakeUp
	status = dl_wake_up();
	if (status != DL_OK)
	{
		return DL_ERROR;
	}
	HAL_Delay(1);

	// Set to operate mode
	status = dl_write_type_0(&m_sequence0, 0x99, 0x00);
	if (status != DL_OK)
	{
		return DL_ERROR;
	}
	HAL_Delay(1);
	return DL_OK;
     }
