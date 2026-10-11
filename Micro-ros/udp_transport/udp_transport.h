/**
 * @file    udp_transport.h
 * @author  xinglu
 * @brief   micro-ROS W5500 UDP 传输接口
 * @version 1.0
 * @date    2026-10-11
 * @copyright Copyright (c) 2026
 */

#ifndef W5500_UDP_TRANSPORT_H
#define W5500_UDP_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

#ifdef __cplusplus
extern "C" {
#endif

/** UDP 传输配置；W5500 初始化后再打开传输，配置对象须在传输期间保持有效。 */
typedef struct {
    uint8_t agent_ip[4]; /**< micro-ROS Agent 的 IPv4 地址。 */
    uint16_t agent_port; /**< micro-ROS Agent 的 UDP 端口。 */
    uint16_t local_port; /**< 本机绑定的 UDP 端口。 */
    uint8_t socket_num;  /**< 使用的 W5500 socket 编号（0–7）。 */
} w5500_udp_transport_args_t;

/** 打开并绑定配置指定的 W5500 UDP socket。 */
bool w5500_udp_transport_open(struct uxrCustomTransport *transport);
/** 关闭传输使用的 W5500 UDP socket。 */
bool w5500_udp_transport_close(struct uxrCustomTransport *transport);

/** 将数据报发送至配置中的 Agent 地址，返回成功发送的字节数。 */
size_t w5500_udp_transport_write(struct uxrCustomTransport *transport,
                                 const uint8_t *buffer, size_t length,
                                 uint8_t *error_code);
/** 在指定超时时间内接收来自配置中 Agent 的数据报。 */
size_t w5500_udp_transport_read(struct uxrCustomTransport *transport,
                                uint8_t *buffer, size_t length, int timeout_ms,
                                uint8_t *error_code);

#ifdef __cplusplus
}
#endif

#endif /* W5500_UDP_TRANSPORT_H */
