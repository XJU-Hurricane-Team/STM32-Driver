/**
 ******************************************************************************
 * @file    device_info.c
 * @author  SkylerLiang  Skyler_Liang@163.com
 * @version V1.0.0
 * @date    2026-10-26
 * @brief   STM32F103 型号识别：读取出厂 FLASH 容量寄存器判定 C6T6 / C8T6
 ******************************************************************************
 */

#include "device_info.h"

Device_Info_t g_device_info;

static uint8_t s_device_info_ready = 0u;

/* 本工程编译时假设的容量：仅在 0x1FFFF7E0 读不到有效值时兜底 */
#if defined(STM32F103x6)
  #define DEVICE_BUILD_FLASH_KB   32u
#elif defined(STM32F103x8) || defined(STM32F103xB)
  #define DEVICE_BUILD_FLASH_KB   64u
#elif defined(STM32F103xC)
  #define DEVICE_BUILD_FLASH_KB   256u
#elif defined(STM32F103xD) || defined(STM32F103xE)
  #define DEVICE_BUILD_FLASH_KB   512u
#elif defined(STM32F103xF) || defined(STM32F103xG)
  #define DEVICE_BUILD_FLASH_KB   1024u
#else
  #define DEVICE_BUILD_FLASH_KB   0u
#endif

/**
 * @brief  读取 FLASH 容量寄存器
 * @note   该寄存器只有低 16bit 有效，必须按 16bit 访问；
 *         部分国产兼容芯片不实现该寄存器，会返回 0x0000 / 0xFFFF。
 * @retval 容量，单位 KB（未校验）
 */
static uint16_t device_read_flash_size_raw(void)
{
	return *((volatile uint16_t *)DEVICE_FLASH_SIZE_ADDR);
}

/**
 * @brief  判断寄存器值是否是合理的 FLASH 容量
 * @retval 1: 合理  0: 无效（0x0000、0xFFFF 之类的垃圾值）
 */
static uint8_t device_flash_size_is_sane(uint16_t kb)
{
	if ((kb < 8u) || (kb > 1024u))
	{
		return 0u;
	}
	/* STM32F1 的容量都是 8KB 的整数倍 */
	if ((kb % 8u) != 0u)
	{
		return 0u;
	}
	return 1u;
}

/**
 * @brief  FLASH 容量 -> 型号
 * @note   这里只按容量分类：LQFP48 封装下 32KB=C6T6、64KB=C8T6；
 *         128KB 及以上属于 CB/CC 等其它型号，统一归到 OTHER。
 */
static Device_Model_t device_classify(uint16_t flash_kb)
{
	if (flash_kb == 32u)
	{
		return DEVICE_MODEL_STM32F103C6T6;
	}
	if (flash_kb == 64u)
	{
		return DEVICE_MODEL_STM32F103C8T6;
	}
	if (flash_kb >= 128u)
	{
		return DEVICE_MODEL_OTHER_F103;
	}
	return DEVICE_MODEL_UNKNOWN;
}

/**
 * @brief  由 FLASH 容量推断 SRAM 容量（C6/C8 的 SRAM 与 FLASH 一一对应）
 * @retval SRAM 容量，单位 KB；0 表示不做猜测
 */
static uint16_t device_ram_from_flash(uint16_t flash_kb)
{
	switch (flash_kb)
	{
		case 32u:
			return 10u;     /* STM32F103C6T6: 10KB SRAM */
		case 64u:
		case 128u:
			return 20u;     /* STM32F103C8T6 / CBT6: 20KB SRAM */
		default:
			return 0u;
	}
}

static const char *device_name_from_model(Device_Model_t model, uint8_t from_register)
{
	switch (model)
	{
		case DEVICE_MODEL_STM32F103C6T6:
			return from_register ? "STM32F103C6T6 (32KB, reg)"
			                     : "STM32F103C6T6 (32KB, build cfg)";
		case DEVICE_MODEL_STM32F103C8T6:
			return from_register ? "STM32F103C8T6 (64KB, reg)"
			                     : "STM32F103C8T6 (64KB, build cfg)";
		case DEVICE_MODEL_OTHER_F103:
			return from_register ? "STM32F103 other (reg)" : "STM32F103 other (build cfg)";
		default:
			return "UNKNOWN";
	}
}

void DeviceInfo_Init(void)
{
	uint16_t      raw_kb;
	uint16_t      flash_kb;
	uint8_t       size_ok;
	Device_Model_t model;

	raw_kb   = device_read_flash_size_raw();
	size_ok  = device_flash_size_is_sane(raw_kb);
	flash_kb = size_ok ? raw_kb : (uint16_t)DEVICE_BUILD_FLASH_KB;
	model    = device_classify(flash_kb);

	g_device_info.raw_flash_size_kb = raw_kb;
	g_device_info.flash_size_ok     = size_ok;
	g_device_info.flash_size_kb     = flash_kb;
	g_device_info.ram_size_kb       = device_ram_from_flash(flash_kb);
	g_device_info.model             = model;
	g_device_info.name              = device_name_from_model(model, size_ok);

	s_device_info_ready = 1u;
}

const Device_Info_t *DeviceInfo_Get(void)
{
	if (s_device_info_ready == 0u)
	{
		DeviceInfo_Init();
	}
	return &g_device_info;
}

uint16_t DeviceInfo_GetFlashSizeKB(void)
{
	if (s_device_info_ready == 0u)
	{
		DeviceInfo_Init();
	}
	return g_device_info.flash_size_kb;
}

Device_Model_t DeviceInfo_GetModel(void)
{
	if (s_device_info_ready == 0u)
	{
		DeviceInfo_Init();
	}
	return g_device_info.model;
}

void DeviceInfo_GetUID(uint32_t uid[3])
{
	if (uid == NULL)
	{
		return;
	}

	uid[0] = *((volatile uint32_t *)(DEVICE_UID_ADDR + 0u));
	uid[1] = *((volatile uint32_t *)(DEVICE_UID_ADDR + 4u));
	uid[2] = *((volatile uint32_t *)(DEVICE_UID_ADDR + 8u));
}
