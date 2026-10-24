/**
 ******************************************************************************
 * @file    DT35.h
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V1.0.0
 * @date    2024-04-24
 * @brief   DT35 IO-Link driver
 * @note    COPYRIGHT 2024 UESTC-LIMITI
 ******************************************************************************
 */

#ifndef __DT35_H
#define __DT35_H

#include "main.h"
#include "iolink_master_dl.h"

Dl_Status_t DT35_Reset_Parameter(void);
Dl_Status_t DT35_Set_Resolution(uint8_t mode);
Dl_Status_t DT35_Set_Laser(uint8_t mode);
Dl_Status_t DT35_Set_Response_Time(uint8_t mode);
Dl_Status_t DT35_Set_Output_Rate(uint8_t mode);
Dl_Status_t DT35_Set_Filter_Depth(uint8_t mode);

#endif
