# W5500 驱动说明

本文档对应本目录中的 `w5500.c` / `w5500.h`，适用于仓库的 STM32G474 工程。驱动使用 STM32 HAL 的 SPI1 与 W5500 通信，提供静态 IPv4 配置、TCP 服务端和 UDP 收发接口。当前 micro-ROS UDP transport 也基于此驱动。

## 文件与依赖

- `w5500.h`：公开状态码、网络配置结构和驱动 API。
- `w5500.c`：SPI 帧、寄存器访问、芯片初始化、socket 操作和收发缓冲区管理。
- 依赖工程生成的 `main.h`（CS GPIO 定义）和 `spi.h`（`hspi1`）。
- CS 使用 `W5500_CS_GPIO_Port` / `W5500_CS_Pin`，由驱动在每笔 SPI 帧开始时拉低、结束时拉高。
- SPI 必须在调用驱动前完成 HAL 初始化，并配置为与板上 W5500 兼容的 SPI 模式和时钟。

## 通信实现

每个 W5500 SPI 帧由 16 位地址、8 位控制字节和数据组成，驱动采用可变长度数据模式。控制字节携带块选择编号（BSB）和读写方向；整笔帧期间 CS 保持有效。底层操作使用 HAL SPI 阻塞接口，单次 HAL SPI 超时为 100 ms。芯片命令和发送完成等待最长轮询 1000 ms。

W5500 的 8 个 socket 各自配置 2 KiB TX 和 2 KiB RX 缓冲区。数据读写按 2 KiB 环形缓冲区边界拆分，处理指针回绕。缓冲区配置总量为每方向 16 KiB。

## 初始化

调用 `W5500_Init()` 前，应确保 SPI1 和 CS GPIO 已初始化。传入 MAC、IP、子网掩码和网关；驱动软件复位芯片，等待版本寄存器读到 `0x04`，通过 `PHYCFGR` 软件接管 PHY 并启用 10/100M 全能力自动协商，再设置所有 socket 缓冲区大小并写入静态网络参数。该函数不执行 DHCP，也不等待或要求网线链路已经建立。

PHY 配置按 `0xF8 → 0x78 → 0xF8` 顺序写入，设置模式后拉低并释放 PHY 复位位，不依赖模块的 `PMODE` 引脚。初始化会读回并校验高五位，低三位为实时状态，不参与配置校验；配置不匹配返回 `W5500_ERROR_PHY`。

```c
static const W5500_NetworkConfig net = {
    .mac     = {0x02, 0x08, 0xDC, 0x55, 0x00, 0x01},
    .ip      = {192, 168, 10, 2},
    .subnet  = {255, 255, 255, 0},
    .gateway = {192, 168, 10, 1}
};

W5500_Status status = W5500_Init(&net);
if (status != W5500_OK) {
    /* 处理初始化失败 */
}
```

之后可用 `W5500_GetVersion()` 读取芯片版本，用 `W5500_GetLinkStatus()` 查询 PHY 链路状态；后者成功时输出 `0`（断开）或 `1`（已连接）。链路状态是物理连接状态，不代表远端服务可达。

当前 micro-ROS 启动任务每 100 ms 读取一次 `PHYCFGR`，最多等待 10 秒建立链路。可在调试器中监视 `W5500_TestPhyConfig`（建议按十六进制显示）和 `W5500_TestLink`。超时会打印完整 PHY 状态，设置 `W5500_TestStatus = W5500_ERROR_TIMEOUT` 并挂起任务，不继续创建 micro-ROS 会话。接通链路后需复位开发板重新启动。

## API 概览

socket 编号范围为 `0..7`。端口参数使用主机序整数，例如十进制 `8888`。

| API | 用途与行为 |
| --- | --- |
| `W5500_Init(config)` | 复位、检查版本、配置 PHY 自动协商、缓冲区和静态 IPv4 参数。 |
| `W5500_GetVersion(&version)` | 读取芯片版本寄存器。 |
| `W5500_GetPhyConfig(&phy_config)` | 读取完整 PHYCFGR 配置和状态。 |
| `W5500_GetLinkStatus(&link_up)` | 查询 PHY 链路状态。 |
| `W5500_TcpServerStart(socket, port)` | 关闭并清理 socket，配置 TCP、本地端口，然后进入监听。成功以状态寄存器确认监听状态。 |
| `W5500_TcpServerPoll(socket)` | 检查服务端 socket 状态；连接断开或 socket 关闭后按记录端口重新监听。建议由应用周期调用。 |
| `W5500_TcpReceive(socket, buffer, capacity)` | 非阻塞读取当前可用 TCP 数据，最多写入 `capacity` 字节。 |
| `W5500_TcpSend(socket, data, length)` | 等待发送缓冲区可用，写入并发送数据，等待发送完成或超时。 |
| `W5500_TcpClose(socket)` | 关闭指定 TCP socket。 |
| `W5500_UdpOpen(socket, local_port)` | 关闭并重新打开 socket，绑定本地 UDP 端口。 |
| `W5500_UdpReceiveFrom(socket, buffer, capacity, source_ip, source_port)` | 非阻塞接收一个 UDP 数据报，并返回发送方地址。 |
| `W5500_UdpSendTo(socket, data, length, destination_ip, destination_port)` | 向 IPv4 目的地址和端口发送一个 UDP 数据报。 |
| `W5500_UdpClose(socket)` | 关闭指定 UDP socket。 |

### 返回值和错误处理

大多数控制接口返回 `W5500_Status`：

| 状态 | 含义 |
| --- | --- |
| `W5500_OK` | 操作成功。 |
| `W5500_ERROR_ARGUMENT` | 空指针、无效 socket、零端口或其他无效参数。 |
| `W5500_ERROR_SPI` | HAL SPI 读写失败。 |
| `W5500_ERROR_TIMEOUT` | 等待芯片命令、发送完成或缓冲区资源超时。 |
| `W5500_ERROR_VERSION` | 初始化等待版本号 `0x04` 超时。 |
| `W5500_ERROR_SOCKET` | socket 状态未达到接口要求，或状态无法恢复。 |
| `W5500_ERROR_PHY` | PHY 软件配置读回值不匹配。 |

`W5500_TcpReceive()` 和 `W5500_UdpReceiveFrom()` 返回 `int32_t`：正数是收到的字节数，`0` 表示当前没有可读数据，负数为对应 `W5500_Status` 的相反数（例如 `-W5500_ERROR_SPI`）。UDP 接收一次处理一个数据报；若数据报长度超过调用方 `capacity`，驱动读取并丢弃超出部分，以保持 RX 队列同步。

发送接口的长度必须适合单个 socket 的 2 KiB TX 缓冲区；micro-ROS transport 另外将单次写入限制为 2048 字节。`W5500_TcpSend()` 对长度为 0 的发送返回成功但不产生数据；UDP 发送要求有效数据指针和非零长度。UDP 接收若数据报长度超过 `capacity`，只复制前 `capacity` 字节，丢弃该数据报剩余内容，并返回实际复制长度。

## TCP 服务端用法

```c
uint8_t rx[256];
if (W5500_TcpServerStart(1U, 5000U) == W5500_OK) {
    for (;;) {
        W5500_Status state = W5500_TcpServerPoll(1U);
        int32_t n = W5500_TcpReceive(1U, rx, sizeof(rx));
        if (n > 0) {
            (void)W5500_TcpSend(1U, rx, (uint16_t)n); /* 示例：回显 */
        }
        /* 在应用任务中安排适当的周期/延时，并处理 state 与 n < 0。 */
    }
}
```

`W5500_TcpServerPoll()` 应定期调用，以便发现连接关闭并重启监听。接收接口是流式 TCP 接口：一次读取可能只拿到部分应用消息，应用协议需要自行定义分帧方式。

## UDP 用法

```c
uint8_t rx[512];
uint8_t peer_ip[4];
uint16_t peer_port;

if (W5500_UdpOpen(0U, 5000U) == W5500_OK) {
    int32_t n = W5500_UdpReceiveFrom(0U, rx, sizeof(rx), peer_ip, &peer_port);
    if (n > 0) {
        (void)W5500_UdpSendTo(0U, rx, (uint16_t)n, peer_ip, peer_port);
    }
}
```

接收为轮询式、非阻塞调用，没有数据时立即返回 `0`。在 FreeRTOS 任务中调用时，应在应用侧安排延时或任务阻塞，避免空转。

## 线程安全与使用约束

驱动直接使用共享的 `hspi1` 和 CS GPIO，且没有互斥锁。不要让多个任务同时调用 W5500 API；如确需多任务访问，应在应用层用同一把互斥锁保护所有 W5500 操作。避免在中断服务程序中调用这些阻塞式 API。TCP 和 UDP socket 也应由单一业务所有者管理，避免不同协议重复打开同一 socket。

## 排查建议

1. 初始化返回 `W5500_ERROR_SPI`：检查 SPI1 初始化、引脚复用、CS 极性/引脚宏和接线。
2. 初始化返回 `W5500_ERROR_VERSION`：检查供电、复位/片选、SPI 模式和时序，并确认芯片返回版本 `0x04`。
3. 初始化成功但链路为 0：检查网线、PHY 对端和模块供电；链路状态可用 `W5500_GetLinkStatus()` 重复查询。
4. UDP 无数据：确认设备 IP、Agent IP/端口、子网和本地端口一致，并确保网络中没有地址冲突或防火墙阻断。
5. 收发偶发失败：先确认同一时刻只有一个任务访问 SPI/W5500，再检查应用是否及时读取数据及是否超过单次 2 KiB 缓冲区限制。

