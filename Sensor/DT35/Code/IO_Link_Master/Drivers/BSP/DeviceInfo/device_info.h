/**
 ******************************************************************************
 * @file    device_info.h
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V1.0.0
 * @date    2026-10-26
 * @brief   STM32F103 型号识别：读取出厂 FLASH 容量寄存器判定 C6T6 / C8T6
 * @note    0x1FFFF7E0 是 ST 在生产测试时写入的 16bit FLASH 容量（单位 KB），
 *          与芯片丝印无关，因此比看丝印/看代码配置更可靠：
 *              0x0020 (32) -> STM32F103C6T6   (32KB FLASH / 10KB SRAM)
 *              0x0040 (64) -> STM32F103C8T6   (64KB FLASH / 20KB SRAM)
 ******************************************************************************
 */

#ifndef __DEVICE_INFO_H
#define __DEVICE_INFO_H

#include "main.h"

/* 出厂 FLASH 容量寄存器（16bit，单位 KB，STM32F1 全系列都有） */
#define DEVICE_FLASH_SIZE_ADDR      (0x1FFFF7E0UL)

/* 96bit 唯一 ID 基址（可用于区分正品 / 国产兼容芯片批次） */
#define DEVICE_UID_ADDR             (0x1FFFF7E8UL)

typedef enum
{
	DEVICE_MODEL_UNKNOWN = 0,       /* 寄存器读不到有效值，编译配置也没有信息 */
	DEVICE_MODEL_STM32F103C6T6,     /* 32KB FLASH / 10KB SRAM */
	DEVICE_MODEL_STM32F103C8T6,     /* 64KB FLASH / 20KB SRAM */
	DEVICE_MODEL_OTHER_F103         /* 其它容量（128KB 及以上） */
} Device_Model_t;

typedef struct
{
	uint16_t       flash_size_kb;       /* 判定用的容量，寄存器无效时为编译配置的假设值 */
	uint16_t       raw_flash_size_kb;   /* 寄存器原值：0x20/0x40/0xFFFF... 便于排查兼容芯片 */
	uint16_t       ram_size_kb;         /* 由 FLASH 容量推断，0 表示未知 */
	uint8_t        flash_size_ok;       /* 1: 容量来自芯片寄存器；0: 只是编译配置假设 */
	Device_Model_t model;
	const char    *name;                /* 型号名字符串，调试器里可直接查看 */
} Device_Info_t;

/* 结果放在全局变量里：Keil 调试时 Watch 窗口直接添加 g_device_info 即可 */
extern Device_Info_t g_device_info;

/* 上电调用一次（main 里已调用）；重复调用只会重新读一次寄存器 */
void DeviceInfo_Init(void);

/* 未调用 Init 时，这几个接口会自动初始化一次 */
const Device_Info_t *DeviceInfo_Get(void);
uint16_t DeviceInfo_GetFlashSizeKB(void);
Device_Model_t DeviceInfo_GetModel(void);

/* 读取 96bit 唯一 ID（UID[0] 为低 32 位） */
void DeviceInfo_GetUID(uint32_t uid[3]);

#endif /* __DEVICE_INFO_H */
