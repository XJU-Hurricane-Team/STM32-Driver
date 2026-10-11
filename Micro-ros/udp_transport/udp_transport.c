/**
 * @file    udp_transport.c
 * @author  xinglu
 * @brief   micro-ROS W5500 UDP 传输实现
 * @version 1.0
 * @date    2026-10-11
 * @copyright Copyright (c) 2026
 */

#include "udp_transport.h"

#include <rmw_microxrcedds_c/config.h>

#include <string.h>

#include "w5500/w5500.h"
#include "FreeRTOS.h"
#include "task.h"

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

/* 收到无数据时每次轮询之间的最小让出时间，避免忙等占满 CPU。 */
#ifndef W5500_UDP_RETRY_DELAY_TICKS
#define W5500_UDP_RETRY_DELAY_TICKS 1U
#endif

/** 从 Micro XRCE-DDS transport 对象中取出用户提供的 W5500 配置。 */
static w5500_udp_transport_args_t *transport_args(
    struct uxrCustomTransport *transport) {
    return (w5500_udp_transport_args_t *)transport->args;
}

bool w5500_udp_transport_open(struct uxrCustomTransport *transport) {
    w5500_udp_transport_args_t *args;

    /* transport->args 指向由调用方创建并持续持有的配置结构。 */
    if (transport == NULL || transport->args == NULL) {
        return false;
    }

    args = transport_args(transport);
    /* W5500 共提供 8 个 socket；端口为 0 时配置无效。 */
    if (args->agent_port == 0U || args->local_port == 0U ||
        args->socket_num >= 8U) {
        return false;
    }

    /* 绑定本地端口并打开指定 socket，底层返回成功才报告打开成功。 */
    return W5500_UdpOpen(args->socket_num, args->local_port) == W5500_OK;
}

bool w5500_udp_transport_close(struct uxrCustomTransport *transport) {
    /* 参数无效时无法确定要关闭的 socket。 */
    if (transport == NULL || transport->args == NULL) {
        return false;
    }

    /* 关闭由配置指定的 socket；底层关闭接口不返回状态。 */
    W5500_UdpClose(transport_args(transport)->socket_num);
    return true;
}

size_t w5500_udp_transport_write(struct uxrCustomTransport *transport,
                                 const uint8_t *buffer, size_t length,
                                 uint8_t *error_code) {
    w5500_udp_transport_args_t *args;
    W5500_Status status;

    /* 默认认为无错误；后续检测到参数或硬件错误时再置错误码。 */
    if (error_code != NULL) {
        *error_code = 0U;
    }
    /* W5500 单个 socket 的 UDP 缓冲区为 2 KiB，长度还须可装入 uint16_t。 */
    if (transport == NULL || transport->args == NULL || buffer == NULL ||
        length == 0U || length > UINT16_MAX || length > 2048U) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
        return 0U;
    }

    args = transport_args(transport);
    /* 将数据报发往配置的 Agent IPv4 地址和端口。 */
    status = W5500_UdpSendTo(args->socket_num, buffer, (uint16_t)length,
                             args->agent_ip, args->agent_port);
    if (status != W5500_OK) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
        return 0U;
    }

    /* 底层发送成功时返回完整数据报长度。 */
    return length;
}

size_t w5500_udp_transport_read(struct uxrCustomTransport *transport,
                                uint8_t *buffer, size_t length, int timeout_ms,
                                uint8_t *error_code) {
    w5500_udp_transport_args_t *args;
    uint8_t source_ip[4];
    uint16_t source_port;
    TickType_t start_tick;
    TickType_t timeout_ticks;

    /* 超时返回 0 字节且不视为传输错误；错误码只表示参数或底层故障。 */
    if (error_code != NULL) {
        *error_code = 0U;
    }
    /* 检查接收所需参数，并确保长度可转换为 W5500 接口使用的 uint16_t。 */
    if (transport == NULL || transport->args == NULL || buffer == NULL ||
        length == 0U || length > UINT16_MAX) {
        if (error_code != NULL) {
            *error_code = 1U;
        }
        return 0U;
    }

    args = transport_args(transport);
    /* 以 FreeRTOS tick 计时，保留起始值用于处理 tick 计数回绕。 */
    start_tick = xTaskGetTickCount();
    timeout_ticks =
        (timeout_ms <= 0) ? 0U : pdMS_TO_TICKS((uint32_t)timeout_ms);
    if (timeout_ms > 0 && timeout_ticks == 0U) {
        timeout_ticks = 1U;
    }

    /* 轮询 W5500：忽略非 Agent 来源的数据报，只交付匹配的包。 */
    for (;;) {
        int32_t received =
            W5500_UdpReceiveFrom(args->socket_num, buffer, (uint16_t)length,
                                 source_ip, &source_port);

        if (received > 0) {
            /* 只有来源 IP 和端口都匹配配置的 Agent 才是有效 XRCE 数据。 */
            if (source_port == args->agent_port &&
                memcmp(source_ip, args->agent_ip, sizeof(source_ip)) == 0) {
                return (size_t)received;
            }
            /* 非目标来源的数据报已被底层读出，丢弃后继续等待。 */
        } else if (received < 0) {
            /* 底层接收报告硬件/驱动错误，立即结束本次读取。 */
            if (error_code != NULL) {
                *error_code = 1U;
            }
            return 0U;
        }

        /* timeout_ms <= 0 表示非阻塞读取；正超时则检查经过的 tick。 */
        if (timeout_ticks == 0U ||
            (TickType_t)(xTaskGetTickCount() - start_tick) >= timeout_ticks) {
            return 0U;
        }

        /* 暂停少量 tick 后重试，让出 CPU 给其他 FreeRTOS 任务。 */
        vTaskDelay(W5500_UDP_RETRY_DELAY_TICKS);
    }
}

#endif /* RMW_UXRCE_TRANSPORT_CUSTOM */
