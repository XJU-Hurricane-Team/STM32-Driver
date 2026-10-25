/**
 * @file    w5500.c
 * @author  xinglu
 * @brief   W5500 以太网芯片驱动
 * @version 1.1
 * @date    2026-10-25
 * @copyright Copyright (c) 2026
 *
 * @par 版本更迭
 * - 1.0 (2026-10-11)：初始版本，提供 W5500 初始化、链路查询及 TCP/UDP 通信接口。
 * - 1.1 (2026-10-25)：初始化时软件接管 PHY，配置 10/100M 全能力自动协商，
 *   执行 PHY 软件复位并校验配置；新增 W5500_GetPhyConfig() 接口及
 *   W5500_ERROR_PHY 错误码。
 */

#include "w5500.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* 单次 HAL SPI 操作的超时时间，以及等待芯片命令/发送完成的最长时间 */
#define W5500_SPI_TIMEOUT_MS  100U
#define W5500_POLL_TIMEOUT_MS 1000U

/* W5500 有 8 个硬件 socket；每个 socket 的 TX/RX 空间配置为 2 KiB */
#define W5500_SOCKETS         8U
#define W5500_SOCK_BUF_MASK   0x07FFU /* 2 KiB 缓冲区偏移掩码：2048 - 1 */

/* 通用寄存器地址（SPI 控制字节中的 BSB=0） */
#define REG_MR                0x0000U
#define REG_GAR               0x0001U
#define REG_SUBR              0x0005U
#define REG_SHAR              0x0009U
#define REG_SIPR              0x000FU
#define REG_PHYCFGR           0x002EU
#define REG_VERSIONR          0x0039U

/* PHYCFGR：软件接管模式，启用 10/100M 全能力自动协商，释放复位。 */
#define PHY_AUTONEG_CONFIG    0xF8U
#define PHY_RESET_RELEASE     0x80U

/* Socket 寄存器偏移。实际寄存器块编号由 socket_reg_block() 计算 */
#define SN_MR                 0x0000U
#define SN_CR                 0x0001U
#define SN_IR                 0x0002U
#define SN_SR                 0x0003U
#define SN_PORT               0x0004U
#define SN_DIPR               0x000CU
#define SN_DPORT              0x0010U
#define SN_TX_FSR             0x0020U
#define SN_TX_WR              0x0024U
#define SN_RX_RSR             0x0026U
#define SN_RX_RD              0x0028U
#define SN_RXBUF_SIZE         0x001EU
#define SN_TXBUF_SIZE         0x001FU

/* 写入 SN_CR 的命令值；命令执行后芯片会自动清零该寄存器 */
#define CMD_OPEN              0x01U
#define CMD_LISTEN            0x02U
#define CMD_DISCON            0x08U
#define CMD_CLOSE             0x10U
#define CMD_SEND              0x20U
#define CMD_RECV              0x40U

/* socket 模式寄存器值和状态寄存器状态码 */
#define MODE_TCP              0x01U
#define MODE_UDP              0x02U
#define SR_CLOSED             0x00U
#define SR_INIT               0x13U
#define SR_LISTEN             0x14U
#define SR_ESTABLISHED        0x17U
#define SR_CLOSE_WAIT         0x1CU
#define SR_UDP                0x22U

/* Socket 中断寄存器 SN_IR 的中断标志位；写 1 清除对应标志 */
#define IR_CON                0x01U
#define IR_DISCON             0x02U
#define IR_RECV               0x04U
#define IR_TIMEOUT            0x08U
#define IR_SEND_OK            0x10U

typedef enum {
    ACCESS_READ = 0U,
    ACCESS_WRITE = 1U
} W5500_Access;

/* 记录每个 TCP 服务端 socket 的监听端口，供断线后自动重新监听 */
static uint16_t tcp_server_ports[W5500_SOCKETS];

/** 拉低片选，开始一笔 W5500 SPI 帧 */
static void cs_select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

/** 拉高片选，结束当前 SPI 帧 */
static void cs_deselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 将逻辑 socket 编号转换为 W5500 的寄存器块编号
 * @details 每个 socket 占用四个 BSB 块：寄存器、TX 缓冲区及 RX 缓冲区块
 *          按芯片定义排列；寄存器块为 1 + 4 * socket
 */
static uint8_t socket_reg_block(uint8_t socket) {
    return (uint8_t)(1U + (socket * 4U));
}

/**
 * @brief 执行一笔 W5500 可变长度 SPI 读写
 * @param address W5500 16 位寄存器/缓冲区地址
 * @param block BSB 块编号，放入 SPI 控制字节 bit[7:3]
 * @param access 读或写；控制字节 bit2 为 1 表示写
 * @param data 数据缓冲区；不能为 NUL
 * @param length 数据字节数；不能为 0
 * @return W5500_OK 表示操作成功，其他值表示错误
 * @details SPI 帧由 16 位地址、8 位控制字节和数据组成。整个帧期间 CS 保持低
 */
static W5500_Status transfer(uint16_t address, uint8_t block,
                             W5500_Access access, uint8_t *data,
                             uint16_t length) {
    uint8_t header[3];

    if ((data == NULL) || (length == 0U)) {
        return W5500_ERROR_ARGUMENT;
    }

    header[0] = (uint8_t)(address >> 8U);
    header[1] = (uint8_t)address;
    /* 操作模式 00 表示可变长度；BSB 在 bit[7:3]，R/W 在 bit2 */
    header[2] =
        (uint8_t)((block << 3U) | ((access == ACCESS_WRITE) ? 0x04U : 0U));

    cs_select();
    if (HAL_SPI_Transmit(&hspi1, header, sizeof(header),
                         W5500_SPI_TIMEOUT_MS) != HAL_OK) {
        cs_deselect();
        return W5500_ERROR_SPI;
    }

    HAL_StatusTypeDef result;
    if (access == ACCESS_WRITE) {
        result = HAL_SPI_Transmit(&hspi1, data, length, W5500_SPI_TIMEOUT_MS);
    } else {
        result = HAL_SPI_Receive(&hspi1, data, length, W5500_SPI_TIMEOUT_MS);
    }
    cs_deselect();
    return (result == HAL_OK) ? W5500_OK : W5500_ERROR_SPI;
}

/** 读取一个字节寄存器 */
static W5500_Status read8(uint16_t address, uint8_t block, uint8_t *value) {
    return transfer(address, block, ACCESS_READ, value, 1U);
}

/** 写入一个字节寄存器 */
static W5500_Status write8(uint16_t address, uint8_t block, uint8_t value) {
    return transfer(address, block, ACCESS_WRITE, &value, 1U);
}

/** 读取 16 位大端寄存器，并转换为 MCU 本机 uint16_t 数值 */
static W5500_Status read16(uint16_t address, uint8_t block, uint16_t *value) {
    uint8_t bytes[2];
    W5500_Status status =
        transfer(address, block, ACCESS_READ, bytes, sizeof(bytes));
    if (status == W5500_OK) {
        *value = (uint16_t)(((uint16_t)bytes[0] << 8U) | bytes[1]);
    }
    return status;
}

/** 将 16 位数值按网络字节顺序写入芯片 */
static W5500_Status write16(uint16_t address, uint8_t block, uint16_t value) {
    uint8_t bytes[2] = {(uint8_t)(value >> 8U), (uint8_t)value};
    return transfer(address, block, ACCESS_WRITE, bytes, sizeof(bytes));
}

/** 连续写入缓冲区数据；仅作为 transfer() 的写方向封装 */
static W5500_Status write_buf(uint16_t address, uint8_t block,
                              const uint8_t *data, uint16_t length) {
    return transfer(address, block, ACCESS_WRITE, (uint8_t *)data, length);
}

/** 连续读取缓冲区数据；仅作为 transfer() 的读方向封装 */
static W5500_Status read_buf(uint16_t address, uint8_t block, uint8_t *data,
                             uint16_t length) {
    return transfer(address, block, ACCESS_READ, data, length);
}

/**
 * @brief 对 socket 发出命令，并轮询 SN_CR 直到芯片清除命令值
 * @note 轮询有超时，避免芯片异常时程序永久卡在此处
 */
static W5500_Status socket_command(uint8_t socket, uint8_t command) {
    uint8_t block = socket_reg_block(socket);
    W5500_Status status = write8(SN_CR, block, command);
    uint32_t started = HAL_GetTick();
    uint8_t value = command;

    while ((status == W5500_OK) && (value != 0U)) {
        status = read8(SN_CR, block, &value);
        if ((uint32_t)(HAL_GetTick() - started) > W5500_POLL_TIMEOUT_MS) {
            return W5500_ERROR_TIMEOUT;
        }
    }
    return status;
}

/**
 * @brief 等待 socket 中断标志（如 SEND_OK），同时检查发送超时标志
 * @param wanted 需要等待的 SN_IR 标志
 * @param observed 可选输出，返回触发等待结束时读到的完整 SN_IR 值
 * @details SN_IR 为写 1 清零寄存器；检测到标志后将读取到的值写回以清除
 */
static W5500_Status wait_socket_ir(uint8_t socket, uint8_t wanted,
                                   uint8_t *observed) {
    uint8_t block = socket_reg_block(socket);
    uint8_t value = 0U;
    uint32_t started = HAL_GetTick();

    do {
        W5500_Status status = read8(SN_IR, block, &value);
        if (status != W5500_OK) {
            return status;
        }
        if ((value & (uint8_t)(wanted | IR_TIMEOUT)) != 0U) {
            (void)write8(SN_IR, block, value);
            if (observed != NULL) {
                *observed = value;
            }
            return ((value & IR_TIMEOUT) != 0U) ? W5500_ERROR_TIMEOUT
                                                : W5500_OK;
        }
    } while ((uint32_t)(HAL_GetTick() - started) <= W5500_POLL_TIMEOUT_MS);

    return W5500_ERROR_TIMEOUT;
}

/** 读取芯片版本寄存器；SPI 失败时返回错误码且不应使用输出值 */
W5500_Status W5500_GetVersion(uint8_t *version) {
    if (version == NULL) {
        return W5500_ERROR_ARGUMENT;
    }
    return read8(REG_VERSIONR, 0U, version);
}

/** 读取完整 PHY 配置和状态；低三位是实时的双工、速率与链路状态 */
W5500_Status W5500_GetPhyConfig(uint8_t *phy_config) {
    if (phy_config == NULL) {
        return W5500_ERROR_ARGUMENT;
    }
    return read8(REG_PHYCFGR, 0U, phy_config);
}

/** 读取 PHYCFGR 的 LINK 位，转换为 0/1 输出 */
W5500_Status W5500_GetLinkStatus(uint8_t *link_up) {
    uint8_t phy = 0U;
    if (link_up == NULL) {
        return W5500_ERROR_ARGUMENT;
    }
    W5500_Status status = W5500_GetPhyConfig(&phy);
    if (status == W5500_OK) {
        *link_up = (uint8_t)((phy & 0x01U) != 0U);
    }
    return status;
}

/**
 * @brief 复位芯片，配置 PHY 自动协商、socket 缓冲区和静态网络参数
 * @details 分别将每个 socket 的 Sn_RXBUF_SIZE/Sn_TXBUF_SIZE 设为 2（单位 KiB），
 *          与后续使用 0x07FF 掩码和 2 KiB 边界的环形缓冲区读写逻辑相匹配
 */
W5500_Status W5500_Init(const W5500_NetworkConfig *config) {
    if (config == NULL) {
        return W5500_ERROR_ARGUMENT;
    }

    cs_deselect();
    HAL_Delay(2U);

    /* 软件复位：MR bit7 置 1；复位完成后芯片清除此位 */
    W5500_Status status = write8(REG_MR, 0U, 0x80U);
    if (status != W5500_OK) {
        return status;
    }
    HAL_Delay(2U);

    uint32_t started = HAL_GetTick();
    uint8_t version = 0U;
    for (;;) {
        status = W5500_GetVersion(&version);
        if (status != W5500_OK) {
            return status;
        }
        if (version == 0x04U) {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - started) > W5500_POLL_TIMEOUT_MS) {
            return W5500_ERROR_VERSION;
        }
        HAL_Delay(1U);
    }

    /* 先软件接管 PHY 并设置自动协商，再通过 RST 位复位使配置生效。
     * 不依赖模块 PMODE 引脚；MR 软件复位不能代替这里的 PHY 复位。
     */
    status = write8(REG_PHYCFGR, 0U, PHY_AUTONEG_CONFIG);
    if (status != W5500_OK) return status;
    status = write8(REG_PHYCFGR, 0U,
                    (uint8_t)(PHY_AUTONEG_CONFIG & ~PHY_RESET_RELEASE));
    if (status != W5500_OK) return status;
    HAL_Delay(1U);
    status = write8(REG_PHYCFGR, 0U, PHY_AUTONEG_CONFIG);
    if (status != W5500_OK) return status;
    HAL_Delay(1U);

    /* 只校验可写配置位，忽略会随协商结果变化的低三位。 */
    uint8_t phy = 0U;
    status = W5500_GetPhyConfig(&phy);
    if (status != W5500_OK) return status;
    if ((phy & PHY_AUTONEG_CONFIG) != PHY_AUTONEG_CONFIG) {
        return W5500_ERROR_PHY;
    }

    /* 每路 RX/TX 各分配 2 KiB；大小寄存器位于各 socket 自己的寄存器块 */
    for (uint8_t socket = 0U; (socket < W5500_SOCKETS) && (status == W5500_OK);
         ++socket) {
        uint8_t block = socket_reg_block(socket);
        status = write8(SN_RXBUF_SIZE, block, 2U);
        if (status == W5500_OK) {
            status = write8(SN_TXBUF_SIZE, block, 2U);
        }
    }
    /* 依次写入网关、子网掩码、MAC 和本机 IPv4 地址 */
    if (status == W5500_OK) {
        status = write_buf(REG_GAR, 0U, config->gateway, 4U);
    }
    if (status == W5500_OK) {
        status = write_buf(REG_SUBR, 0U, config->subnet, 4U);
    }
    if (status == W5500_OK) {
        status = write_buf(REG_SHAR, 0U, config->mac, 6U);
    }
    if (status == W5500_OK) {
        status = write_buf(REG_SIPR, 0U, config->ip, 4U);
    }
    return status;
}

/** 关闭 socket、清中断、配置 TCP 模式和本地端口，然后开始监听 */
W5500_Status W5500_TcpServerStart(uint8_t socket, uint16_t port) {
    if ((socket >= W5500_SOCKETS) || (port == 0U)) {
        return W5500_ERROR_ARGUMENT;
    }

    uint8_t block = socket_reg_block(socket);
    W5500_Status status = socket_command(socket, CMD_CLOSE);
    if (status == W5500_OK) {
        status = write8(SN_IR, block, 0xFFU);
    }
    if (status == W5500_OK) {
        status = write8(SN_MR, block, MODE_TCP);
    }
    if (status == W5500_OK) {
        status = write16(SN_PORT, block, port);
    }
    if (status == W5500_OK) {
        status = socket_command(socket, CMD_OPEN);
    }
    if (status != W5500_OK) {
        return status;
    }

    uint8_t state = 0U;
    status = read8(SN_SR, block, &state);
    if ((status != W5500_OK) || (state != SR_INIT)) {
        return W5500_ERROR_SOCKET;
    }
    status = socket_command(socket, CMD_LISTEN);
    if (status != W5500_OK) {
        return status;
    }
    status = read8(SN_SR, block, &state);
    if ((status == W5500_OK) && (state == SR_LISTEN)) {
        tcp_server_ports[socket] = port;
        return W5500_OK;
    }
    return W5500_ERROR_SOCKET;
}

/**
 * @brief 查询 TCP 状态，并按状态维持服务端监听。
 * @details ESTABLISHED/LISTEN 视作正常；CLOSE_WAIT/CLOSED 时尝试重建；
 *          INIT 时补发 LISTEN；其他状态返回 socket 错误
 */
W5500_Status W5500_TcpServerPoll(uint8_t socket) {
    uint8_t state;
    uint8_t block;
    W5500_Status status;
    if (socket >= W5500_SOCKETS) {
        return W5500_ERROR_ARGUMENT;
    }
    block = socket_reg_block(socket);
    status = read8(SN_SR, block, &state);
    if (status != W5500_OK) {
        return status;
    }
    if ((state == SR_ESTABLISHED) || (state == SR_LISTEN)) {
        return W5500_OK;
    }
    if (state == SR_CLOSE_WAIT) {
        (void)socket_command(socket, CMD_CLOSE);
        return (tcp_server_ports[socket] != 0U)
                   ? W5500_TcpServerStart(socket, tcp_server_ports[socket])
                   : W5500_ERROR_SOCKET;
    }
    if (state == SR_CLOSED) {
        return (tcp_server_ports[socket] != 0U)
                   ? W5500_TcpServerStart(socket, tcp_server_ports[socket])
                   : W5500_ERROR_SOCKET;
    }
    if (state == SR_INIT) {
        return socket_command(socket, CMD_LISTEN);
    }
    return W5500_ERROR_SOCKET;
}

/**
 * @brief 从 socket RX 环形区读取可用 TCP 字节
 * @details 读取最多 capacity 字节，更新 RX_RD 后发 RECV 命令释放芯片缓冲区
 *          若数据从 2 KiB 环形区尾部跨回开头，则拆成两次 SPI 读取
 */
int32_t W5500_TcpReceive(uint8_t socket, uint8_t *buffer, uint16_t capacity) {
    uint8_t block;
    uint16_t available;
    uint16_t pointer;
    uint16_t count;
    if ((socket >= W5500_SOCKETS) || (buffer == NULL) || (capacity == 0U)) {
        return -(int32_t)W5500_ERROR_ARGUMENT;
    }

    block = socket_reg_block(socket);
    W5500_Status status = read16(SN_RX_RSR, block, &available);
    if (status != W5500_OK) {
        return -(int32_t)status;
    }
    if (available == 0U) {
        return 0;
    }
    count = (available < capacity) ? available : capacity;
    status = read16(SN_RX_RD, block, &pointer);
    if (status != W5500_OK) {
        return -(int32_t)status;
    }

    uint16_t offset = (uint16_t)(pointer & W5500_SOCK_BUF_MASK);
    uint16_t first = (uint16_t)(2048U - offset);
    if (first > count) {
        first = count;
    }
    uint8_t rx_block = (uint8_t)(3U + (socket * 4U));
    status = read_buf((uint16_t)(pointer & W5500_SOCK_BUF_MASK), rx_block,
                      buffer, first);
    if ((status == W5500_OK) && (count > first)) {
        status =
            read_buf(0U, rx_block, &buffer[first], (uint16_t)(count - first));
    }
    if (status != W5500_OK) {
        return -(int32_t)status;
    }

    pointer = (uint16_t)(pointer + count);
    status = write16(SN_RX_RD, block, pointer);
    if (status == W5500_OK) {
        status = socket_command(socket, CMD_RECV);
    }
    if (status != W5500_OK) {
        return -(int32_t)status;
    }
    return (int32_t)count;
}

/**
 * @brief 向 TCP 发送缓冲区写入数据并启动发送
 * @details 先等待 TX_FSR 足够，再按 TX_WR 写入（必要时跨环形区边界拆写），
 *          更新 TX_WR、发 SEND 命令，最后等待 SEND_OK
 */
W5500_Status W5500_TcpSend(uint8_t socket, const uint8_t *data,
                           uint16_t length) {
    if ((socket >= W5500_SOCKETS) || ((data == NULL) && (length != 0U))) {
        return W5500_ERROR_ARGUMENT;
    }
    if (length > 2048U) {
        return W5500_ERROR_ARGUMENT;
    }
    if (length == 0U) {
        return W5500_OK;
    }

    uint8_t block = socket_reg_block(socket);
    uint16_t free_size = 0U;
    uint32_t started = HAL_GetTick();
    do {
        W5500_Status status = read16(SN_TX_FSR, block, &free_size);
        if (status != W5500_OK) {
            return status;
        }
        if (free_size >= length) {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - started) > W5500_POLL_TIMEOUT_MS) {
            return W5500_ERROR_TIMEOUT;
        }
    } while (true);

    uint16_t pointer;
    W5500_Status status = read16(SN_TX_WR, block, &pointer);
    if (status != W5500_OK) {
        return status;
    }
    uint16_t offset = (uint16_t)(pointer & W5500_SOCK_BUF_MASK);
    uint16_t first = (uint16_t)(2048U - offset);
    if (first > length) {
        first = length;
    }
    uint8_t tx_block = (uint8_t)(2U + (socket * 4U));
    status = write_buf((uint16_t)(pointer & W5500_SOCK_BUF_MASK), tx_block,
                       data, first);
    if ((status == W5500_OK) && (length > first)) {
        status =
            write_buf(0U, tx_block, &data[first], (uint16_t)(length - first));
    }
    if (status != W5500_OK) {
        return status;
    }

    pointer = (uint16_t)(pointer + length);
    status = write16(SN_TX_WR, block, pointer);
    if (status == W5500_OK) {
        status = socket_command(socket, CMD_SEND);
    }
    if (status == W5500_OK) {
        status = wait_socket_ir(socket, IR_SEND_OK, NULL);
    }
    return status;
}

/** 尝试主动断开连接并关闭指定 TCP socket */
void W5500_TcpClose(uint8_t socket) {
    if (socket < W5500_SOCKETS) {
        (void)socket_command(socket, CMD_DISCON);
        (void)socket_command(socket, CMD_CLOSE);
    }
}

/**
 * @brief 从 W5500 socket 的 RX 环形缓冲区读取指定字节数
 * @details pointer 是芯片的 16 位 RX_RD 指针；低 11 位为 2 KiB 缓冲区偏移。
 *          当请求跨越缓冲区末尾时，先读末尾部分，再从偏移 0 继续读
 */
static W5500_Status read_rx_ring(uint8_t socket, uint16_t pointer,
                                 uint8_t *buffer, uint16_t length) {
    uint8_t block = (uint8_t)(3U + (socket * 4U));
    uint16_t offset = (uint16_t)(pointer & W5500_SOCK_BUF_MASK);
    uint16_t first = (uint16_t)(2048U - offset);
    if (first > length) {
        first = length;
    }

    W5500_Status status = read_buf(offset, block, buffer, first);
    if ((status == W5500_OK) && (length > first)) {
        status =
            read_buf(0U, block, &buffer[first], (uint16_t)(length - first));
    }
    return status;
}

/** 将数据写入 socket TX 环形区；处理跨越 2 KiB 末尾的回绕 */
static W5500_Status write_tx_ring(uint8_t socket, uint16_t pointer,
                                  const uint8_t *data, uint16_t length) {
    uint8_t block = (uint8_t)(2U + (socket * 4U));
    uint16_t offset = (uint16_t)(pointer & W5500_SOCK_BUF_MASK);
    uint16_t first = (uint16_t)(2048U - offset);
    if (first > length) {
        first = length;
    }

    W5500_Status status = write_buf(offset, block, data, first);
    if ((status == W5500_OK) && (length > first)) {
        status = write_buf(0U, block, &data[first], (uint16_t)(length - first));
    }
    return status;
}

/** 关闭并清理 socket，然后以 UDP 模式绑定本地端口 */
W5500_Status W5500_UdpOpen(uint8_t socket, uint16_t local_port) {
    if ((socket >= W5500_SOCKETS) || (local_port == 0U)) {
        return W5500_ERROR_ARGUMENT;
    }

    uint8_t block = socket_reg_block(socket);
    W5500_Status status = socket_command(socket, CMD_CLOSE);
    if (status == W5500_OK) {
        status = write8(SN_IR, block, 0xFFU);
    }
    if (status == W5500_OK) {
        status = write8(SN_MR, block, MODE_UDP);
    }
    if (status == W5500_OK) {
        status = write16(SN_PORT, block, local_port);
    }
    if (status == W5500_OK) {
        status = socket_command(socket, CMD_OPEN);
    }
    if (status != W5500_OK) {
        return status;
    }

    uint8_t state = 0U;
    status = read8(SN_SR, block, &state);
    return ((status == W5500_OK) && (state == SR_UDP)) ? W5500_OK
                                                       : W5500_ERROR_SOCKET;
}

/**
 * @brief 读取一个完整 UDP 数据包
 * @details RX 缓冲区中的 UDP 记录格式为 4 字节源 IP、2 字节源端口、
 *          2 字节负载长度，后跟负载。函数先检查完整记录是否已到齐，再复制
 *          负载、更新 RX_RD 并发 RECV 释放整个数据包
 */
int32_t W5500_UdpReceiveFrom(uint8_t socket, uint8_t *buffer, uint16_t capacity,
                             uint8_t source_ip[4], uint16_t *source_port) {
    if ((socket >= W5500_SOCKETS) || (buffer == NULL) || (capacity == 0U) ||
        (source_ip == NULL) || (source_port == NULL)) {
        return -(int32_t)W5500_ERROR_ARGUMENT;
    }

    uint8_t block = socket_reg_block(socket);
    uint16_t available = 0U;
    uint16_t pointer = 0U;
    if (read16(SN_RX_RSR, block, &available) != W5500_OK) {
        return -(int32_t)W5500_ERROR_SPI;
    }
    if (available < 8U) {
        return 0;
    }
    if (read16(SN_RX_RD, block, &pointer) != W5500_OK) {
        return -(int32_t)W5500_ERROR_SPI;
    }

    uint8_t udp_header[8];
    W5500_Status status =
        read_rx_ring(socket, pointer, udp_header, sizeof(udp_header));
    if (status != W5500_OK) {
        return -(int32_t)status;
    }

    uint16_t datagram_length =
        (uint16_t)(((uint16_t)udp_header[6] << 8U) | udp_header[7]);
    uint32_t packet_length = 8UL + datagram_length;
    if (packet_length > available) {
        return 0;
    }

    uint16_t copy_length =
        (datagram_length < capacity) ? datagram_length : capacity;
    if (copy_length > 0U) {
        status =
            read_rx_ring(socket, (uint16_t)(pointer + 8U), buffer, copy_length);
        if (status != W5500_OK) {
            return -(int32_t)status;
        }
    }
    memcpy(source_ip, udp_header, 4U);
    *source_port = (uint16_t)(((uint16_t)udp_header[4] << 8U) | udp_header[5]);

    pointer = (uint16_t)(pointer + packet_length);
    status = write16(SN_RX_RD, block, pointer);
    if (status == W5500_OK) {
        status = socket_command(socket, CMD_RECV);
    }
    if (status != W5500_OK) {
        return -(int32_t)status;
    }
    return (int32_t)copy_length;
}

/** 设定目标 IP/端口，将数据写入 TX 环形缓冲区并等待发送完成 */
W5500_Status W5500_UdpSendTo(uint8_t socket, const uint8_t *data,
                             uint16_t length, const uint8_t destination_ip[4],
                             uint16_t destination_port) {
    if ((socket >= W5500_SOCKETS) || (data == NULL) ||
        (destination_ip == NULL) || (destination_port == 0U) ||
        (length == 0U) || (length > 2048U)) {
        return W5500_ERROR_ARGUMENT;
    }

    uint8_t block = socket_reg_block(socket);
    W5500_Status status = write_buf(SN_DIPR, block, destination_ip, 4U);
    if (status == W5500_OK) {
        status = write16(SN_DPORT, block, destination_port);
    }
    if (status != W5500_OK) {
        return status;
    }

    uint16_t free_size = 0U;
    uint32_t started = HAL_GetTick();
    do {
        status = read16(SN_TX_FSR, block, &free_size);
        if (status != W5500_OK) {
            return status;
        }
        if (free_size >= length) {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - started) > W5500_POLL_TIMEOUT_MS) {
            return W5500_ERROR_TIMEOUT;
        }
    } while (true);

    uint16_t pointer = 0U;
    status = read16(SN_TX_WR, block, &pointer);
    if (status == W5500_OK) {
        status = write_tx_ring(socket, pointer, data, length);
    }
    if (status == W5500_OK) {
        status = write16(SN_TX_WR, block, (uint16_t)(pointer + length));
    }
    if (status == W5500_OK) {
        status = socket_command(socket, CMD_SEND);
    }
    if (status == W5500_OK) {
        status = wait_socket_ir(socket, IR_SEND_OK, NULL);
    }
    return status;
}

/** 关闭指定 UDP socket */
void W5500_UdpClose(uint8_t socket) {
    if (socket < W5500_SOCKETS) {
        (void)socket_command(socket, CMD_CLOSE);
    }
}
