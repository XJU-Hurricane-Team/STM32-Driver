/**
 ******************************************************************************
 * @file    DT35.c
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V1.0.0
 * @date    2024-04-24
 * @brief   DT35 IO-Link driver
 * @note    COPYRIGHT 2024 UESTC-LIMITI
 ******************************************************************************
 */

#include "DT35.h"

Dl_Status_t DT35_Reset_Parameter(void)
{
	return dl_write_ISDU_uint8(0x02, 130);
}

// 0:0.1mm  1:1mm  2:10mm
Dl_Status_t DT35_Set_Resolution(uint8_t mode)
{
	return dl_write_ISDU_uint8(0x69, mode);
}

// 0:OFF  1:ON
Dl_Status_t DT35_Set_Laser(uint8_t mode)
{
	return dl_write_ISDU_uint8(0x44, mode);
}

// 0:Expert  1:Super-slow  2:Slow  3:Medium  4:Fast  5:Super-fast
// Index 64, 67 and 66 are only available if "0: Expert" has been selected.
Dl_Status_t DT35_Set_Response_Time(uint8_t mode)
{
	return dl_write_ISDU_uint8(0x67, mode);
}

// 2^n * 2ms
// 0 ~ 8
// Writable only if "0:Expert" was selected for index "103"
Dl_Status_t DT35_Set_Output_Rate(uint8_t mode)
{
	return dl_write_ISDU_uint8(0x40, mode);
}

// 1:OFF  2:depth2  4:depth4  8:depth8  16:depth16
// Writable only if "0:Expert" was selected for index "103"
Dl_Status_t DT35_Set_Filter_Depth(uint8_t mode)
{
	return dl_write_ISDU_uint8(0x43, mode);
}
