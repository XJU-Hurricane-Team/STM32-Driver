/**
 * @file    w5500.h
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

#ifndef W5500_H
#define W5500_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** 驱动接口返回码 */
typedef enum {
    W5500_OK = 0,
    W5500_ERROR_ARGUMENT,
    W5500_ERROR_SPI,
    W5500_ERROR_TIMEOUT,
    W5500_ERROR_VERSION,
    W5500_ERROR_SOCKET,
    W5500_ERROR_PHY
} W5500_Status;

/** W5500 静态网络配置 */
typedef struct {
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t subnet[4];
    uint8_t gateway[4];
} W5500_NetworkConfig;

/** 初始化芯片、PHY 自动协商、socket 缓冲区和静态网络参数；不等待链路 */
W5500_Status W5500_Init(const W5500_NetworkConfig *config);
/** 获取芯片版本 */
W5500_Status W5500_GetVersion(uint8_t *version);
/** 获取完整 PHYCFGR，便于检查模式配置与链路状态 */
W5500_Status W5500_GetPhyConfig(uint8_t *phy_config);
/** 获取以太网物理链路状态 */
W5500_Status W5500_GetLinkStatus(uint8_t *link_up);

/** 启动指定 socket 上的 TCP 服务端 */
W5500_Status W5500_TcpServerStart(uint8_t socket, uint16_t port);
/** 查询并维护 TCP 服务端 socket 状态，适合由应用任务周期调用 */
W5500_Status W5500_TcpServerPoll(uint8_t socket);
/** 接收 TCP 数据；返回 0 表示无数据，正数表示长度，负数表示错误码的相反数 */
int32_t W5500_TcpReceive(uint8_t socket, uint8_t *buffer, uint16_t capacity);
/** 发送 TCP 数据 */
W5500_Status W5500_TcpSend(uint8_t socket, const uint8_t *data,
                           uint16_t length);
/** 关闭 TCP socket */
void W5500_TcpClose(uint8_t socket);

/** 在指定 socket 上打开 UDP 并绑定本地端口 */
W5500_Status W5500_UdpOpen(uint8_t socket, uint16_t local_port);
/** 接收一个 UDP 数据报，同时返回发送方 IP 和端口 */
int32_t W5500_UdpReceiveFrom(uint8_t socket, uint8_t *buffer, uint16_t capacity,
                             uint8_t source_ip[4], uint16_t *source_port);
/** 向指定 IPv4 地址和端口发送 UDP 数据报 */
W5500_Status W5500_UdpSendTo(uint8_t socket, const uint8_t *data,
                             uint16_t length, const uint8_t destination_ip[4],
                             uint16_t destination_port);
/** 关闭 UDP socket */
void W5500_UdpClose(uint8_t socket);

#endif /* W5500_H */
