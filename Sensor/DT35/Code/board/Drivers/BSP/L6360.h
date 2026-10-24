/**
 ******************************************************************************
 * @file    L6360.h
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V1.0.0
 * @date    2024-04-24
 * @brief   L6360 driver (IO-Link communication master transceiver IC)
 * @note    COPYRIGHT 2024 UESTC-LIMITI
 ******************************************************************************
 */

#ifndef __L6360_H
#define __L6360_H

#include "main.h"
#include "usart.h"
#include "i2c.h"

#define L6360_UART (huart1)

#define L6360_I2C (hi2c1)

#define L6360_I2C_ADD (0x60U)

#define L6360_I2C_ADD_OFFSET (0x0U)

#define PARITY(x)      ((uint8_t)(((((uint8_t)(x)&0x80U)>>7U)^(((uint8_t)(x)&0x40U)>>6U)^(((uint8_t)(x)&0x20U)>>5U)^(((uint8_t)(x)&0x10U)>>4U)^(((uint8_t)(x)&0x08U)>>3U)^(((uint8_t)(x)&0x04U)>>2U)^(((uint8_t)(x)&0x02U)>>1U)^((uint8_t)(x)&0x01U))))
#define ODD_PARITY(x)  ((uint8_t)((((uint8_t)(x)&0x80U)>>7U)^(((uint8_t)(x)&0x20U)>>5U)^(((uint8_t)(x)&0x08U)>>3U)^(((uint8_t)(x)&0x02U)>>1U)))
#define EVEN_PARITY(x) ((uint8_t)((((uint8_t)(x)&0x40U)>>6U)^(((uint8_t)(x)&0x10U)>>4U)^(((uint8_t)(x)&0x04U)>>2U)^((uint8_t)(x)&0x01U)))

/*******************  Bit definition for STATUS register  *******************/
#define L6360_STATUS_PE_Pos             (0U)                                         
#define L6360_STATUS_PE_Msk             (0x1U << L6360_STATUS_PE_Pos)           /*!< 0x01 */
#define L6360_STATUS_PE                 L6360_STATUS_PE_Msk                     /*!<Parity Check error */

#define L6360_STATUS_REG_LN_Pos         (1U)                                         
#define L6360_STATUS_REG_LN_Msk         (0x1U << L6360_STATUS_REG_LN_Pos)       /*!< 0x02 */
#define L6360_STATUS_REG_LN             L6360_STATUS_REG_LN_Msk                 /*!<Linear regulator undervoltage fault */

#define L6360_STATUS_IQOL_Pos           (3U)                                         
#define L6360_STATUS_IQOL_Msk           (0x1U << L6360_STATUS_IQOL_Pos)          /*!< 0x08 */
#define L6360_STATUS_IQOL               L6360_STATUS_IQOL_Msk                    /*!<L+ overload */

#define L6360_STATUS_CQOL_Pos           (4U)                                         
#define L6360_STATUS_CQOL_Msk           (0x1U << L6360_STATUS_CQOL_Pos)         /*!< 0x10 */
#define L6360_STATUS_CQOL               L6360_STATUS_CQOL_Msk                   /*!<C/Q overload */

#define L6360_STATUS_OVT_Pos            (5U)                                         
#define L6360_STATUS_OVT_Msk            (0x1U << L6360_STATUS_OVT_Pos)          /*!< 0x20 */
#define L6360_STATUS_OVT                L6360_STATUS_OVT_Msk                    /*!<Overtemperature fault */

#define L6360_STATUS_PO_Pos            (7U)                                         
#define L6360_STATUS_PO_Msk            (0x1U << L6360_STATUS_PO_Pos)            /*!< 0x80 */
#define L6360_STATUS_PO                L6360_STATUS_PO_Msk                      /*!<Power-on (L+ line) */

/*******************  Bit definition for CONTROL 1 register  *******************/
#define L6360_CONTROL_1_TDBQ_Pos              (0U)                                         
#define L6360_CONTROL_1_TDBQ_Msk              (0x3U << L6360_CONTROL_1_TDBQ_Pos)           /*!< 0x03 */
#define L6360_CONTROL_1_TDBQ                  L6360_CONTROL_1_TDBQ_Msk                     /*!<TDBQ[1:0] bits (C/Qi debounce time) */
#define L6360_CONTROL_1_TDBQ_0_US             (0x0U << L6360_CONTROL_1_TDBQ_Pos)           /*!< 0 us */
#define L6360_CONTROL_1_TDBQ_5_US             (0x1U << L6360_CONTROL_1_TDBQ_Pos)           /*!< 5 us */
#define L6360_CONTROL_1_TDBQ_20_US            (0x2U << L6360_CONTROL_1_TDBQ_Pos)           /*!< 20 us */
#define L6360_CONTROL_1_TDBQ_100_US           (030U << L6360_CONTROL_1_TDBQ_Pos)           /*!< 100 us */

#define L6360_CONTROL_1_TRCOQ_Pos             (2U)                                         
#define L6360_CONTROL_1_TRCOQ_Msk             (0x1U << L6360_CONTROL_1_TRCOQ_Pos)           /*!< 0x04 */
#define L6360_CONTROL_1_TRCOQ                 L6360_CONTROL_1_TRCOQ_Msk                     /*!<C/Qo restart delay time */
#define L6360_CONTROL_1_TRCOQ_255X            (0x0U << L6360_CONTROL_1_TRCOQ_Pos)           /*!< 255 x Tdcoq */
#define L6360_CONTROL_1_TRCOQ_LATCHED         (0x1U << L6360_CONTROL_1_TRCOQ_Pos)           /*!< Latched */

#define L6360_CONTROL_1_TDCOQ_Pos             (3U)                                         
#define L6360_CONTROL_1_TDCOQ_Msk             (0x3U << L6360_CONTROL_1_TDCOQ_Pos)           /*!< 0x18 */
#define L6360_CONTROL_1_TDCOQ                 L6360_CONTROL_1_TDCOQ_Msk                     /*!<TDCOQ[1:0] bits (C/Qo HS and LS cut-off current delay time) */
#define L6360_CONTROL_1_TDCOQ_100_US          (0x0U << L6360_CONTROL_1_TDCOQ_Pos)           /*!< 100 us */
#define L6360_CONTROL_1_TDCOQ_150_US          (0x1U << L6360_CONTROL_1_TDCOQ_Pos)           /*!< 150 us */
#define L6360_CONTROL_1_TDCOQ_200_US          (0x2U << L6360_CONTROL_1_TDCOQ_Pos)           /*!< 200 us */
#define L6360_CONTROL_1_TDCOQ_250_US          (0x3U << L6360_CONTROL_1_TDCOQ_Pos)           /*!< 250 us */

#define L6360_CONTROL_1_ICOQ_Pos             (5U)                                         
#define L6360_CONTROL_1_ICOQ_Msk             (0x3U << L6360_CONTROL_1_ICOQ_Pos)             /*!< 0x60 */
#define L6360_CONTROL_1_ICOQ                 L6360_CONTROL_1_ICOQ_Msk                       /*!<ICOQ[1:0] bits (C/QO HS and LS cut-off current) */
#define L6360_CONTROL_1_ICOQ_115_MA          (0x0U << L6360_CONTROL_1_ICOQ_Pos)             /*!< 115 mA */
#define L6360_CONTROL_1_ICOQ_220_MA          (0x1U << L6360_CONTROL_1_ICOQ_Pos)             /*!< 220 mA */
#define L6360_CONTROL_1_ICOQ_350_MA          (0x2U << L6360_CONTROL_1_ICOQ_Pos)             /*!< 350 mA */
#define L6360_CONTROL_1_ICOQ_580_MA          (0x3U << L6360_CONTROL_1_ICOQ_Pos)             /*!< 580 mA */

#define L6360_CONTROL_1_ENCGQ_Pos             (7U)                                         
#define L6360_CONTROL_1_ENCGQ_Msk             (0x1U << L6360_CONTROL_1_ENCGQ_Pos)           /*!< 0x80 */
#define L6360_CONTROL_1_ENCGQ                 L6360_CONTROL_1_ENCGQ_Msk                     /*!<C/Qi pull-down generator enable */
#define L6360_CONTROL_1_ENCGQ_OFF             (0x0U << L6360_CONTROL_1_ENCGQ_Pos)           /*!< Always OFF*/
#define L6360_CONTROL_1_ENCGQ_ENCQ            (0x1U << L6360_CONTROL_1_ENCGQ_Pos)           /*!< According to ENc/q  */


/*******************  Bit definition for CONTROL 2 register  *******************/

#define L6360_CONTROL_2_TDBI_Pos             (0)                                         
#define L6360_CONTROL_2_TDBI_Msk             (0x3U << L6360_CONTROL_2_TDBI_Pos)             /*!< 0x03 */
#define L6360_CONTROL_2_TDBI                 L6360_CONTROL_2_TDBI_Msk                       /*!<TDBI[1:0] bits (I/Q debounce time) */
#define L6360_CONTROL_2_TDBI_0_US            (0x0U << L6360_CONTROL_2_TDBI_Pos)             /*!< 0 us */
#define L6360_CONTROL_2_TDBI_5_US            (0x1U << L6360_CONTROL_2_TDBI_Pos)             /*!< 5 us */
#define L6360_CONTROL_2_TDBI_20_US           (0x2U << L6360_CONTROL_2_TDBI_Pos)             /*!< 20 us */
#define L6360_CONTROL_2_TDBI_100_US          (0x3U << L6360_CONTROL_2_TDBI_Pos)             /*!< 100 us */

#define L6360_CONTROL_2_TRCOI_Pos             (2U)                                         
#define L6360_CONTROL_2_TRCOI_Msk             (0x1U << L6360_CONTROL_2_TRCOI_Pos)           /*!< 0x04 */
#define L6360_CONTROL_2_TRCOI                 L6360_CONTROL_2_TRCOI_Msk                     /*!<L+ restart delay */
#define L6360_CONTROL_2_TRCOI_64_MS           (0x0U << L6360_CONTROL_2_TRCOI_Pos)           /*!< 64 ms */
#define L6360_CONTROL_2_TRCOI_LATCHED         (0x1U << L6360_CONTROL_2_TRCOI_Pos)           /*!< Latched */

#define L6360_CONTROL_2_TDCOI_Pos             (3U)                                         
#define L6360_CONTROL_2_TDCOI_Msk             (0x1U << L6360_CONTROL_2_TDCOI_Pos)           /*!< 0x08 */
#define L6360_CONTROL_2_TDCOI                 L6360_CONTROL_2_TDCOI_Msk                     /*!<L+ cut-off current delay time */
#define L6360_CONTROL_2_TDCOI_500_US          (0x0U << L6360_CONTROL_2_TDCOI_Pos)           /*!< 500 us */
#define L6360_CONTROL_2_TDCOI_0_US            (0x1U << L6360_CONTROL_2_TDCOI_Pos)           /*!< 0 us */

#define L6360_CONTROL_2_LCOD_Pos             (4U)                                         
#define L6360_CONTROL_2_LCOD_Msk             (0x1U << L6360_CONTROL_2_LCOD_Pos)             /*!< 0x10 */
#define L6360_CONTROL_2_LCOD                 L6360_CONTROL_2_LCOD_Msk                       /*!<L+ cut-off disable */
#define L6360_CONTROL_2_LCOD_ENABLE          (0x0U << L6360_CONTROL_2_LCOD_Pos)             /*!< L+ cut-off current enabled */
#define L6360_CONTROL_2_LCOD_DISABLE         (0x1U << L6360_CONTROL_2_LCOD_Pos)             /*!< L+ cut-off current disabled */

#define L6360_CONTROL_2_CQPDG_Pos             (5U)                                         
#define L6360_CONTROL_2_CQPDG_Msk             (0x1U << L6360_CONTROL_2_CQPDG_Pos)           /*!< 0x20 */
#define L6360_CONTROL_2_CQPDG                 L6360_CONTROL_2_CQPDG_Msk                     /*!<C/Q pull-down generator switching */
#define L6360_CONTROL_2_CQPDG_2_5_MA          (0x0U << L6360_CONTROL_2_CQPDG_Pos)           /*!< IINI/Qi (input current C/QI pin) = 2.5 mA */
#define L6360_CONTROL_2_CQPDG_5_5_MA          (0x1U << L6360_CONTROL_2_CQPDG_Pos)           /*!< IINC/Qi (input current C/QI pin) = 5.5 mA */

#define L6360_CONTROL_2_ENCGI_Pos             (7U)                                         
#define L6360_CONTROL_2_ENCGI_Msk             (0x1U << L6360_CONTROL_2_ENCGI_Pos)           /*!< 0x80 */
#define L6360_CONTROL_2_ENCGI                 L6360_CONTROL_2_ENCGI_Msk                     /*!<C/Q pull-down generator switching */
#define L6360_CONTROL_2_ENCGI_OFF             (0x0U << L6360_CONTROL_2_ENCGI_Pos)           /*!< Pull-down generator always off*/
#define L6360_CONTROL_2_ENCGI_ON              (0x1U << L6360_CONTROL_2_ENCGI_Pos)           /*!< Pull-down generator always on */

/*******************  Bit definition for Configuration register  *******************/

#define L6360_CONFIGURATION_CQ_Pos               (5)                                         
#define L6360_CONFIGURATION_CQ_Msk               (0x7U << L6360_CONFIGURATION_CQ_Pos)             /*!< 0xE0 */
#define L6360_CONFIGURATION_CQ                   L6360_CONFIGURATION_CQ_Msk                       /*!<CQ[2:0] bits  (C/Q output stage configuration)*/
#define L6360_CONFIGURATION_CQ_OFF               (0x0U << L6360_CONFIGURATION_CQ_Pos)             /*!< Off */
#define L6360_CONFIGURATION_CQ_LOW_SIDE          (0x1U << L6360_CONFIGURATION_CQ_Pos)             /*!< Low-side */
#define L6360_CONFIGURATION_CQ_HIGH_SIDE         (0x2U << L6360_CONFIGURATION_CQ_Pos)             /*!< High-side */
#define L6360_CONFIGURATION_CQ_PUSH_PULL         (0x3U << L6360_CONFIGURATION_CQ_Pos)             /*!< Push-pull */
#define L6360_CONFIGURATION_CQ_TRI_STATE         (0x4U << L6360_CONFIGURATION_CQ_Pos)             /*!< Tri-state */
#define L6360_CONFIGURATION_CQ_LOW_SIDE_ON       (0x5U << L6360_CONFIGURATION_CQ_Pos)             /*!< Low-side ON */
#define L6360_CONFIGURATION_CQ_HIGH_SIDE_ON      (0x6U << L6360_CONFIGURATION_CQ_Pos)             /*!< High-side ON  */
#define L6360_CONFIGURATION_CQ_INDUCTIVE_LOAD    (0x7U << L6360_CONFIGURATION_CQ_Pos)             /*!< Push-pull inductive load */

/*******************  Bit definition for LED1 MSB register  *******************/

#define L6360_LED1_MSB_Pos                    (0)                                         
#define L6360_LED1_MSB_Msk                    (0xFFU << L6360_LED1_MSB_Pos)                  /*!< 0xFF */
#define L6360_LED1_MSB                        L6360_LED1_MSB_Msk                            /*!<LED1 MSB bits */

/*******************  Bit definition for LED1 LSB register  *******************/

#define L6360_LED1_LSB_Pos                    (0)                                         
#define L6360_LED1_LSB_Msk                    (0xFFU << L6360_LED1_LSB_Pos)                  /*!< 0xFF */
#define L6360_LED1_LSB                        L6360_LED1_LSB_Msk                            /*!<LED1 LSB bits */

/*******************  Bit definition for LED2 MSB register  *******************/

#define L6360_LED2_MSB_Pos                    (0)                                         
#define L6360_LED2_MSB_Msk                    (0xFFU << L6360_LED2_MSB_Pos)                  /*!< 0xFF */
#define L6360_LED2_MSB                        L6360_LED2_MSB_Msk                            /*!<LED2 MSB bits */

/*******************  Bit definition for LED2 LSB register  *******************/

#define L6360_LED2_LSB_Pos                    (0)                                         
#define L6360_LED2_LSB_Msk                    (0xFFU << L6360_LED2_LSB_Pos)                  /*!< 0xFF */
#define L6360_LED2_LSB                        L6360_LED2_LSB_Msk                            /*!<LED2 LSB bits */

/*******************  Bit definition for Parity register  *******************/

#define L6360_PARITY_L2L_Pos               (0)                                         
#define L6360_PARITY_L2L_Msk               (0x1U << L6360_PARITY_L2L_Pos)             /*!< 0x01 */
#define L6360_PARITY_L2L                   L6360_PARITY_L2L_Msk                       /*!<LED2 low register parity1) */

#define L6360_PARITY_L2H_Pos               (1)                                         
#define L6360_PARITY_L2H_Msk               (0x1U << L6360_PARITY_L2H_Pos)             /*!< 0x02 */
#define L6360_PARITY_L2H                   L6360_PARITY_L2H_Msk                       /*!<LED2 high register parity1) */

#define L6360_PARITY_L1L_Pos               (2)                                         
#define L6360_PARITY_L1L_Msk               (0x1U << L6360_PARITY_L1L_Pos)             /*!< 0x04 */
#define L6360_PARITY_L1L                   L6360_PARITY_L1L_Msk                       /*!<LED1 low register parity1) */

#define L6360_PARITY_L1H_Pos               (3)                                         
#define L6360_PARITY_L1H_Msk               (0x1U << L6360_PARITY_L1H_Pos)             /*!< 0x08 */
#define L6360_PARITY_L1H                   L6360_PARITY_L1H_Msk                       /*!<LED1 high register parity1) */

#define L6360_PARITY_CT2_Pos               (4)                                         
#define L6360_PARITY_CT2_Msk               (0x1U << L6360_PARITY_CT2_Pos)             /*!< 0x10 */
#define L6360_PARITY_CT2                   L6360_PARITY_CT2_Msk                       /*!<control register 2 parity) */

#define L6360_PARITY_CT1_Pos               (5)                                         
#define L6360_PARITY_CT1_Msk               (0x1U << L6360_PARITY_CT1_Pos)             /*!< 0x20 */
#define L6360_PARITY_CT1                   L6360_PARITY_CT1_Msk                       /*!<control register 1 parity) */

#define L6360_PARITY_CR_Pos                (6)                                         
#define L6360_PARITY_CR_Msk                (0x1U << L6360_PARITY_CR_Pos)              /*!< 0x40 */
#define L6360_PARITY_CR                    L6360_PARITY_CR_Msk                        /*!<configuration register parity) */

#define L6360_PARITY_SR_Pos                (7)                                         
#define L6360_PARITY_SR_Msk                (0x1U << L6360_PARITY_SR_Pos)              /*!< 0x80 */
#define L6360_PARITY_SR                    L6360_PARITY_SR_Msk                        /*!<status register parity) */

typedef enum
{
  L6360_STATUS            = ((uint8_t)0x00),
  L6360_CONFIGURATION     = ((uint8_t)0x01),
  L6360_CONTROL_1         = ((uint8_t)0x02),
  L6360_CONTROL_2         = ((uint8_t)0x03), 
  L6360_LED_1_MSB         = ((uint8_t)0x04),
  L6360_LED_1_LSB         = ((uint8_t)0x05),
  L6360_LED_2_MSB         = ((uint8_t)0x06),
  L6360_LED_2_LSB         = ((uint8_t)0x07),
  L6360_PARITY            = ((uint8_t)0x08)
} L6360_Registers_t;

typedef enum
{
	LED_ON = 0x00U,
	LED_OFF,
	LED_SLOW_BLINK,
	LED_FAST_BLINK
} LED_Mode_t;

typedef enum
{
	COM1 = (uint32_t)4800U,
	COM2 = (uint32_t)38400U,
	COM3 = (uint32_t)230400U,
	COMX = (uint32_t)122000U, // to generate the wake-up pulse
} IO_Link_Buadrate_t;

void L6360_Init(void);
//uint8_t L6360_UART_DeInit(void);
//uint8_t L6360_UART_Init(void);
uint8_t L6360_Set_Buadrate(IO_Link_Buadrate_t buadrate);
uint8_t L6360_Set_EN_CQ(uint8_t enable);
uint8_t L6360_Set_EN_LPlus(uint8_t enable);
uint8_t L6360_Set_RST(uint8_t enable);
void L6360_Reset(void);
//uint8_t L6360_Send_IN_CQ_Data(uint8_t data);
HAL_StatusTypeDef L6360_Send_IN_CQ_Data(uint8_t* pdata, uint8_t len);
HAL_StatusTypeDef L6360_Recv_OUT_CQ_Data(uint8_t* pdata, uint8_t len);
uint8_t L6360_Read_Register(uint8_t regAdd);
uint16_t L6360_Current_Read_Register(void);
uint8_t L6360_Write_Register(uint8_t regAdd, uint8_t data);
uint8_t L6360_Set_Led(uint8_t id, LED_Mode_t mode);

#endif
