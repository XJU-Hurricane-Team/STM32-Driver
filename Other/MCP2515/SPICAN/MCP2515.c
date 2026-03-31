/**
 * @file    MCP2515.c
 * @author  Dominate0017
 * @brief   MCP2515 SPI interface and interrupt helper routines.
 * @version 1.0
 * @date    2026-3-15
 */

#include "SPICAN/MCP2515.h"

/* SPI related variables */ 
SPI_HandleTypeDef *spicanx_selected[] = {
#if MCP2515_DEV1_ENABLE 
    MCP2515_DEV1_SPI_HANDLE
#endif /* MCP2515_DEV1_ENABLE */
#if MCP2515_DEV2_ENABLE
    , MCP2515_DEV2_SPI_HANDLE
#endif /* MCP2515_DEV2_ENABLE */
#if MCP2515_DEV3_ENABLE
    , MCP2515_DEV3_SPI_HANDLE
#endif /* MCP2515_DEV3_ENABLE */
};
    
/* Prototypes */
static HAL_StatusTypeDef SPI_WaitForDMA(SPI_HandleTypeDef *hspi);
static void SPI_Tx_Ext(MCP2515_DevId_t dev_id, uint8_t data);
static void SPI_TxBuffer_Ext(MCP2515_DevId_t dev_id, uint8_t *buffer, uint8_t length);
static uint8_t SPI_Rx_Ext(MCP2515_DevId_t dev_id);
static void SPI_RxBuffer_Ext(MCP2515_DevId_t dev_id, uint8_t *buffer, uint8_t length);
static SPI_HandleTypeDef* MCP2515_GetSPIHandle(MCP2515_DevId_t dev_id);
void MCP2515_SetCSPin(MCP2515_DevId_t dev_id, GPIO_PinState state);

/**
 * @brief 初始化 MCP2515 设备及相关 GPIO/SPI 接口。
 *
 * @details
 * 为启用的 MCP2515 设备配置片选引脚，可选配置中断引脚，
 * 并检查 SPI 句柄是否就绪。只要任意启用的 MCP2515 SPI 外设就绪，
 * 即返回 true。
 */
bool MCP2515_Initialize(void)
{

   GPIO_InitTypeDef gpio_init_struct = {0};

#if MCP2515_DEV1_ENABLE

  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  gpio_init_struct.Pin = MCP2515_DEV1_CS_PIN;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCP2515_DEV1_CS_PORT, &gpio_init_struct);

  MCP2515_DEV1_CS_HIGH();
#endif /* MCP2515_DEV1_ENABLE */

#if MCP2515_DEV2_ENABLE

   __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio_init_struct.Pin = MCP2515_DEV2_CS_PIN;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCP2515_DEV2_CS_PORT, &gpio_init_struct);

  MCP2515_DEV1_CS_HIGH();
#endif /* MCP2515_DEV2_ENABLE */

#if MCP2515_DEV3_ENABLE

  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio_init_struct.Pin = MCP2515_DEV3_CS_PIN;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init_struct.Pull = GPIO_NOPULL;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MCP2515_DEV3_CS_PORT, &gpio_init_struct);

  MCP2515_DEV1_CS_HIGH();
#endif /* MCP2515_DEV3_ENABLE */

#if MCP2515_INT_USE
  MCP2515_INT_Init();
#endif /* MCP2515_INT_USE */

  uint8_t loop = 10;
  do {
#if MCP2515_DEV1_ENABLE
    if(HAL_SPI_GetState(MCP2515_DEV1_SPI_HANDLE) == HAL_SPI_STATE_READY)
      return true;
#endif /* MCP2515_DEV1_ENABLE */
#if MCP2515_DEV2_ENABLE
    if(HAL_SPI_GetState(MCP2515_DEV2_SPI_HANDLE) == HAL_SPI_STATE_READY)
      return true;
#endif /* MCP2515_DEV2_ENABLE */
#if MCP2515_DEV3_ENABLE
    if(HAL_SPI_GetState(MCP2515_DEV3_SPI_HANDLE) == HAL_SPI_STATE_READY)
      return true;
#endif /* MCP2515_DEV3_ENABLE */
    loop--;
  } while(loop > 0); 

  return false;
}

/**
 * @brief 初始化 MCP2515 中断引脚并配置默认中断掩码。
 *
 * @details
 * 为每个启用的 MCP2515 设备配置 GPIO 中断线，设置 NVIC 优先级，
 * 并初始化 MCP2515 中断掩码，使默认只启用 RX 中断。
 */
void MCP2515_INT_Init(void)
{
      GPIO_InitTypeDef gpio_init_struct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

#if MCP2515_DEV1_ENABLE

    gpio_init_struct.Pin = MCP2515_DEV1_INT_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(MCP2515_DEV1_INT_PORT, &gpio_init_struct);

    HAL_NVIC_SetPriority(MCP2515_DEV1_IRQn, MCP2515_DEV1_IRQ_PRIO, MCP2515_DEV1_IRQ_SUBPRIO);
    HAL_NVIC_EnableIRQ(MCP2515_DEV1_IRQn);

    MCP2515_DEV1_CS_LOW();
    /* 八个中断只开启了两个接收缓存器的接收中断 */
    MCP2515_DisableInt_Ext(MCP2515_DEV_1, MCP2515_INTE_TX0 | MCP2515_INTE_TX1 | MCP2515_INTE_TX2 | MCP2515_INTE_ERR |
                            MCP2515_INTE_WAKE | MCP2515_INTE_MERR);
    MCP2515_EnableInt_Ext(MCP2515_DEV_1, MCP2515_INTE_RX0 | MCP2515_INTE_RX1);
    MCP2515_ClearIntFlag_Ext(MCP2515_DEV_1, MCP2515_INTF_RX0 | MCP2515_INTF_RX1);
    MCP2515_DEV1_CS_HIGH();
#endif /* MCP2515_DEV1_ENABLE */

#if MCP2515_DEV2_ENABLE

    gpio_init_struct.Pin = MCP2515_DEV2_INT_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MCP2515_DEV2_INT_PORT, &gpio_init_struct);

    HAL_NVIC_SetPriority(MCP2515_DEV2_IRQn, MCP2515_DEV2_IRQ_PRIO, MCP2515_DEV2_IRQ_SUBPRIO);
    HAL_NVIC_EnableIRQ(MCP2515_DEV2_IRQn);

    MCP2515_DEV2_CS_LOW();
    MCP2515_DisableInt_Ext(MCP2515_DEV_2, MCP2515_INTE_TX0 | MCP2515_INTE_TX1 | MCP2515_INTE_TX2 | MCP2515_INTE_ERR |
                            MCP2515_INTE_WAKE | MCP2515_INTE_MERR);
    MCP2515_EnableInt_Ext(MCP2515_DEV_2, MCP2515_INTE_RX0 | MCP2515_INTE_RX1 | MCP2515_INTE_ERR);
    MCP2515_ClearIntFlag_Ext(MCP2515_DEV_2, MCP2515_INTF_RX0 | MCP2515_INTF_RX1);
    MCP2515_DEV2_CS_HIGH();
#endif /* MCP2515_DEV2_ENABLE */

#if MCP2515_DEV3_ENABLE

    gpio_init_struct.Pin = MCP2515_DEV3_INT_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MCP2515_DEV3_INT_PORT, &gpio_init_struct);

    HAL_NVIC_SetPriority(MCP2515_DEV3_IRQn, MCP2515_DEV3_IRQ_PRIO, MCP2515_DEV3_IRQ_SUBPRIO);
    HAL_NVIC_EnableIRQ(MCP2515_DEV3_IRQn);

    MCP2515_DEV3_CS_LOW();
    MCP2515_DisableInt_Ext(MCP2515_DEV_3, MCP2515_INTE_TX0 | MCP2515_INTE_TX1 | MCP2515_INTE_TX2 | MCP2515_INTE_ERR |
                            MCP2515_INTE_WAKE | MCP2515_INTE_MERR);
    MCP2515_EnableInt_Ext(MCP2515_DEV_3, MCP2515_INTE_RX0 | MCP2515_INTE_RX1 | MCP2515_INTE_ERR);
    MCP2515_ClearIntFlag_Ext(MCP2515_DEV_3, MCP2515_INTF_RX0 | MCP2515_INTF_RX1);
    MCP2515_DEV3_CS_HIGH();
#endif /* MCP2515_DEV3_ENABLE */
}

/**
 * @brief 将 MCP2515 置于配置模式。
 *
 * @details
 * 通过向 CANCTRL 寄存器写入命令请求配置模式，并轮询 CANSTAT 寄存器
 * 直到确认进入该模式或超时。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 成功进入配置模式返回 true，超时返回 false。
 */
bool MCP2515_SetConfigMode_Ext(MCP2515_DevId_t dev_id)
{
    /* configure CANCTRL Register */
    MCP2515_WriteByte_Ext(dev_id, MCP2515_CANCTRL, 0x80);
  
    uint8_t loop = 10;
  
    do {    
        /* confirm mode configuration */
        if((MCP2515_ReadByte_Ext(dev_id, MCP2515_CANSTAT) & 0xE0) == 0x80)
            return true;
    
        loop--;
    } while(loop > 0); 
  
    return false;
}

/**
 * @brief 将 MCP2515 置于正常工作模式。
 *
 * @details
 * 通过向 CANCTRL 寄存器写入命令选择正常模式，并轮询 CANSTAT 寄存器
 * 以确认模式切换完成。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 成功进入正常模式返回 true，超时返回 false。
 */
bool MCP2515_SetNormalMode_Ext(MCP2515_DevId_t dev_id)
{
    /* configure CANCTRL Register */
    MCP2515_WriteByte_Ext(dev_id, MCP2515_CANCTRL, 0x00);
  
    uint8_t loop = 10;
  
    do {    
        /* confirm mode configuration */
        if((MCP2515_ReadByte_Ext(dev_id, MCP2515_CANSTAT) & 0xE0) == 0x00)
            return true;
    
        loop--;
    } while(loop > 0);
  
    return false;
}

/**
 * @brief 将 MCP2515 置于睡眠模式。
 *
 * @details
 * 通过向 CANCTRL 寄存器写入命令进入睡眠模式，并轮询 CANSTAT 寄存器
 * 直到确认进入该模式或超时。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 成功进入睡眠模式返回 true，超时返回 false。
 */
bool MCP2515_SetSleepMode_Ext(MCP2515_DevId_t dev_id)
{
    /* configure CANCTRL Register */
    MCP2515_WriteByte_Ext(dev_id, MCP2515_CANCTRL, 0x20);
  
    uint8_t loop = 10;
  
    do {    
        /* confirm mode configuration */
        if((MCP2515_ReadByte_Ext(dev_id, MCP2515_CANSTAT) & 0xE0) == 0x20)
            return true;
    
        loop--;
    } while(loop > 0);
  
    return false;
}

/**
 * @brief 向 MCP2515 发送复位命令。
 *
 * @details
 * 将片选拉低，发送 MCP2515 RESET 指令，然后释放片选。
 *
 * @param dev_id MCP2515 设备 ID。
 */
void MCP2515_Reset_Ext(MCP2515_DevId_t dev_id)
{    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
 
    SPI_Tx_Ext(dev_id, MCP2515_RESET);

    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
}

/**
 * @brief 读取 MCP2515 寄存器的一个字节。
 *
 * @details
 * 通过 SPI 发送 READ 指令和目标地址，然后读取返回的数据字节。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param address 目标寄存器地址。
 * @return 从寄存器中读取到的字节值。
 */
uint8_t MCP2515_ReadByte_Ext(MCP2515_DevId_t dev_id, uint8_t address)
{
    uint8_t retVal;
  
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET); 
  
    SPI_Tx_Ext(dev_id, MCP2515_READ);
    SPI_Tx_Ext(dev_id, address);
    retVal = SPI_Rx_Ext(dev_id);
      
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET); 
  
    return retVal;
}

/**
 * @brief 读取 MCP2515 的连续字节序列（通常用于接收缓冲区）。
 *
 * @details
 * 发送指定指令后，通过 SPI 接收指定长度的数据并存入提供的缓冲区。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param instruction MCP2515 读取指令（例如 READ_RX_BUFFER）。
 * @param data 目标数据缓冲区。
 * @param length 要读取的字节数。
 */
void MCP2515_ReadRxSequence_Ext(MCP2515_DevId_t dev_id, uint8_t instruction, uint8_t *data, uint8_t length)
{
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
  
    SPI_Tx_Ext(dev_id, instruction);        
    SPI_RxBuffer_Ext(dev_id, data, length);
    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
}

/**
 * @brief 向 MCP2515 指定寄存器写入一个字节。
 *
 * @details
 * 发送 WRITE 指令、目标地址和数据字节，然后释放片选。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param address 目标寄存器地址。
 * @param data 要写入的字节。
 */
void MCP2515_WriteByte_Ext(MCP2515_DevId_t dev_id, uint8_t address, uint8_t data)
{    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET); 
  
    SPI_Tx_Ext(dev_id, MCP2515_WRITE);
    SPI_Tx_Ext(dev_id, address);
    SPI_Tx_Ext(dev_id, data);  
    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET); 
}

/**
 * @brief 向 MCP2515 指定地址范围写入一段字节序列。
 *
 * @details
 * 发送 WRITE 指令与起始地址，然后将提供的数据缓冲区写入连续寄存器。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param startAddress 起始寄存器地址。
 * @param endAddress 结束寄存器地址。
 * @param data 要写入的数据缓冲区。
 */
void MCP2515_WriteByteSequence_Ext(MCP2515_DevId_t dev_id, uint8_t startAddress, uint8_t endAddress, uint8_t *data)
{
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
    
    SPI_Tx_Ext(dev_id, MCP2515_WRITE);
    SPI_Tx_Ext(dev_id, startAddress);
    // 计算数据长度并发送字节序列
    uint8_t len = endAddress - startAddress + 1;
    SPI_TxBuffer_Ext(dev_id, data, len);
    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET); 
}

/**
 * @brief 将 CAN 报文写入 MCP2515 发送缓存。
 *
 * @details
 * 发送指定的指令（例如 LOAD_TX_BUFFER），写入标识符寄存器、DLC 和数据。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param instruction 发送缓冲区加载指令。
 * @param idReg 指向 4 字节 ID 寄存器数据（标准/扩展 ID）。
 * @param dlc 数据长度码。
 * @param data 指向要发送的数据缓冲区。
 */
void MCP2515_LoadTxSequence_Ext(MCP2515_DevId_t dev_id, uint8_t instruction, uint8_t *idReg, uint8_t dlc, uint8_t *data)
{    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
  
    SPI_Tx_Ext(dev_id, instruction);
    SPI_TxBuffer_Ext(dev_id, idReg, 4);
    SPI_Tx_Ext(dev_id, dlc);
    SPI_TxBuffer_Ext(dev_id, data, dlc);
       
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
}

/**
 * @brief 向 MCP2515 发送缓存写入一个字节。
 *
 * @details
 * 发送指定的指令和数据字节到发送缓存。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param instruction 发送缓存写入指令。
 * @param data 要写入的字节。
 */
void MCP2515_LoadTxBuffer_Ext(MCP2515_DevId_t dev_id, uint8_t instruction, uint8_t data)
{
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET); 
    
    SPI_Tx_Ext(dev_id, instruction);
    SPI_Tx_Ext(dev_id, data);
    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET); 
}

/**
 * @brief 发送请求发送（RTS）指令给 MCP2515。
 *
 * @details
 * 将片选拉低，发送指定的 RTS 指令以触发 MCP2515 将对应的发送缓冲区
 * 中的数据发送到 CAN 总线，然后释放片选。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param instruction RTS 指令（例如 RTS TXB0、TXB1、TXB2）。
 */
void MCP2515_RequestToSend_Ext(MCP2515_DevId_t dev_id, uint8_t instruction)
{
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
  
    SPI_Tx_Ext(dev_id, instruction);
      
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
}

/**
 * @brief 读取 MCP2515 的状态寄存器。
 *
 * @details
 * 发送 READ_STATUS 指令并读取返回的状态字节，用于检测发送/接收缓冲区状态
 * 和错误标志。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 状态寄存器的字节值。
 */
uint8_t MCP2515_ReadStatus_Ext(MCP2515_DevId_t dev_id)
{
    uint8_t retVal;
  
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
  
    SPI_Tx_Ext(dev_id, MCP2515_READ_STATUS);
    retVal = SPI_Rx_Ext(dev_id);
        
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
  
    return retVal;
}

/**
 * @brief 读取 MCP2515 的接收状态寄存器。
 *
 * @details
 * 发送 RX_STATUS 指令并读取返回的状态字节，用于判断是否有接收到报文
 * 以及报文来源（RX0/RX1）。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 接收状态寄存器的字节值。
 */
uint8_t MCP2515_GetRxStatus_Ext(MCP2515_DevId_t dev_id)
{
    uint8_t retVal;
  
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
  
    SPI_Tx_Ext(dev_id, MCP2515_RX_STATUS);
    retVal = SPI_Rx_Ext(dev_id);
        
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
  
    return retVal;
}

/**
 * @brief 对 MCP2515 寄存器指定位进行修改。
 *
 * @details
 * 发送 BIT_MOD 指令，并提供掩码和数据字节，可在不影响其他位的情况下
 * 修改寄存器的特定位。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param address 目标寄存器地址。
 * @param mask 位掩码，1 表示修改的位。
 * @param data 要写入的新位值。
 */
void MCP2515_BitModify_Ext(MCP2515_DevId_t dev_id, uint8_t address, uint8_t mask, uint8_t data)
{    
    MCP2515_SetCSPin(dev_id, GPIO_PIN_RESET);
  
    SPI_Tx_Ext(dev_id, MCP2515_BIT_MOD);
    SPI_Tx_Ext(dev_id, address);
    SPI_Tx_Ext(dev_id, mask);
    SPI_Tx_Ext(dev_id, data);
      
    MCP2515_SetCSPin(dev_id, GPIO_PIN_SET);
}

/**
 * @brief 发送一个字节到 MCP2515（SPI）。
 *
 * @details
 * 使用 HAL SPI 接口将单个字节发送到指定 MCP2515 设备。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param data 要发送的字节。
 */
static HAL_StatusTypeDef SPI_WaitForDMA(SPI_HandleTypeDef *hspi)
{
    uint32_t tickstart = HAL_GetTick();

    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > SPI_TIMEOUT)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static void SPI_Tx_Ext(MCP2515_DevId_t dev_id, uint8_t data)
{
    SPI_HandleTypeDef *hspi = MCP2515_GetSPIHandle(dev_id);

    if (hspi == NULL)
        return;

    if (HAL_SPI_Transmit_DMA(hspi, &data, 1) != HAL_OK)
    {
        return;
    }

    uint32_t tickstart = HAL_GetTick();
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > SPI_TIMEOUT)
        {
            HAL_SPI_DMAStop(hspi);
            return;
        }
    }
}

/**
 * @brief 从 MCP2515 读取一个字节（SPI）。
 *
 * @details
 * 使用 HAL SPI 接口接收一个字节并返回。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 接收到的字节。
 */
static uint8_t SPI_Rx_Ext(MCP2515_DevId_t dev_id)
{
    uint8_t retVal = 0;
    SPI_HandleTypeDef *hspi = MCP2515_GetSPIHandle(dev_id);

    if (hspi == NULL)
        return retVal;

    if (HAL_SPI_Receive_DMA(hspi, &retVal, 1) != HAL_OK)
    {
        return retVal;
    }

    uint32_t tickstart = HAL_GetTick();
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > SPI_TIMEOUT)
        {
            HAL_SPI_DMAStop(hspi);
            return retVal;
        }
    }

    return retVal;
}

/**
 * @brief 发送一段字节缓冲区到 MCP2515（SPI）。
 *
 * @details
 * 使用 HAL SPI 接口发送指定长度的数据缓冲区。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param buffer 数据缓冲区。
 * @param length 要发送的字节数。
 */
static void SPI_TxBuffer_Ext(MCP2515_DevId_t dev_id, uint8_t *buffer, uint8_t length)
{
    SPI_HandleTypeDef *hspi = MCP2515_GetSPIHandle(dev_id);

    if (hspi == NULL || buffer == NULL || length == 0)
        return;

    if (HAL_SPI_Transmit_DMA(hspi, buffer, length) != HAL_OK)
    {
        return;
    }

    uint32_t tickstart = HAL_GetTick();
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > SPI_TIMEOUT)
        {
            HAL_SPI_DMAStop(hspi);
            return;
        }
    }
}

/**
 * @brief 从 MCP2515 接收一段字节到缓冲区（SPI）。
 *
 * @details
 * 使用 HAL SPI 接口接收指定长度的数据并存入提供的缓冲区。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param buffer 目标数据缓冲区。
 * @param length 要接收的字节数。
 */
static void SPI_RxBuffer_Ext(MCP2515_DevId_t dev_id, uint8_t *buffer, uint8_t length)
{
    SPI_HandleTypeDef *hspi = MCP2515_GetSPIHandle(dev_id);

    if (hspi == NULL || buffer == NULL || length == 0)
        return;

    if (HAL_SPI_Receive_DMA(hspi, buffer, length) != HAL_OK)
    {
        return;
    }

    uint32_t tickstart = HAL_GetTick();
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - tickstart) > SPI_TIMEOUT)
        {
            HAL_SPI_DMAStop(hspi);
            return;
        }
    }
}

/**
 * @brief 启用 MCP2515 中断。
 *
 * @details
 * 通过 BIT_MOD 指令修改 CANINTE 寄存器，将指定掩码位设置为 1，从而
 * 启用对应的中断源。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param intMask 中断掩码位。
 */
void MCP2515_EnableInt_Ext(MCP2515_DevId_t dev_id, uint8_t intMask)
{
    MCP2515_BitModify_Ext(dev_id, MCP2515_CANINTE, intMask, intMask);
}

/**
 * @brief 禁用 MCP2515 中断。
 *
 * @details
 * 通过 BIT_MOD 指令修改 CANINTE 寄存器，将指定掩码位清零，从而禁用
 * 对应的中断源。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param intMask 中断掩码位。
 */
void MCP2515_DisableInt_Ext(MCP2515_DevId_t dev_id, uint8_t intMask)
{
    MCP2515_BitModify_Ext(dev_id, MCP2515_CANINTE, intMask, 0x00);
}

/**
 * @brief 清除 MCP2515 中断标志。
 *
 * @details
 * 通过 BIT_MOD 指令将 CANINTF 寄存器中指定的标志位清零，以便重新识别
 * 新的中断事件。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param intMask 中断标志掩码。
 */
void MCP2515_ClearIntFlag_Ext(MCP2515_DevId_t dev_id, uint8_t intMask)
{
    MCP2515_BitModify_Ext(dev_id, MCP2515_CANINTF, intMask, 0x00);
}

/**
 * @brief 设置 MCP2515 的片选引脚状态。
 *
 * @details
 * 根据传入的设备 ID，选择对应的 CS 引脚并写入指定状态。
 * 如果设备 ID 无效，则使用 DEV1 的 CS 引脚作为默认。
 *
 * @param dev_id MCP2515 设备 ID。
 * @param state 片选引脚状态（GPIO_PIN_RESET/SET）。
 */
void MCP2515_SetCSPin(MCP2515_DevId_t dev_id, GPIO_PinState state)
{
    switch(dev_id)
    {
#if MCP2515_DEV1_ENABLE
        case MCP2515_DEV_1:
            HAL_GPIO_WritePin(MCP2515_DEV1_CS_PORT, MCP2515_DEV1_CS_PIN, state);
            break;
#endif /* MCP2515_DEV1_ENABLE */
#if MCP2515_DEV2_ENABLE
        case MCP2515_DEV_2:
            HAL_GPIO_WritePin(MCP2515_DEV2_CS_PORT, MCP2515_DEV2_CS_PIN, state);
            break;
#endif /* MCP2515_DEV2_ENABLE */
#if MCP2515_DEV3_ENABLE
        case MCP2515_DEV_3:
            HAL_GPIO_WritePin(MCP2515_DEV3_CS_PORT, MCP2515_DEV3_CS_PIN, state);
            break;
#endif /* MCP2515_DEV3_ENABLE */
        default:
            // 非法设备ID，默认操作DEV1
#if MCP2515_DEV1_ENABLE
            HAL_GPIO_WritePin(MCP2515_DEV1_CS_PORT, MCP2515_DEV1_CS_PIN, state);
#endif /* MCP2515_DEV1_ENABLE */
            break;
    }
}

/**
 * @brief 根据设备 ID 获取对应的 SPI 句柄。
 *
 * @details
 * 返回与指定 MCP2515 设备关联的 HAL SPI 句柄，用于后续 SPI 传输。
 * 如果设备 ID 无效，则返回 DEV1 的句柄（若启用），否则返回 NULL。
 *
 * @param dev_id MCP2515 设备 ID。
 * @return 对应的 SPI 句柄或 NULL。
 */
static SPI_HandleTypeDef* MCP2515_GetSPIHandle(MCP2515_DevId_t dev_id)
{
    switch(dev_id)
    {
#if MCP2515_DEV1_ENABLE
        case MCP2515_DEV_1:
            return MCP2515_DEV1_SPI_HANDLE;
#endif  /* MCP2515_DEV1_ENABLE */
#if MCP2515_DEV2_ENABLE
        case MCP2515_DEV_2: 
            return MCP2515_DEV2_SPI_HANDLE;
#endif  /* MCP2515_DEV2_ENABLE */
#if MCP2515_DEV3_ENABLE 
        case MCP2515_DEV_3:
            return MCP2515_DEV3_SPI_HANDLE;
#endif  /* MCP2515_DEV3_ENABLE */
        default:
            // 非法设备ID，默认返回DEV1的SPI句柄
#if MCP2515_DEV1_ENABLE
            return MCP2515_DEV1_SPI_HANDLE;
#else 
            return NULL;
#endif /* MCP2515_DEV1_ENABLE */
    }
}

