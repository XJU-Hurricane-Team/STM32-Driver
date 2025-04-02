# STM32 驱动

存放一些经过验证的驱动代码。主要是 STM32 外设和电机的驱动程序。

每个文件夹包含如何添加到工程中、API 参考和代码示例的文档。

代码注释采用 doxygen 格式。后面会考虑将代码中的 doxygen 转换成 markdown，以方便查阅。

某些代码即使验证了也可能存在 bug，而且 README 中的 demo 不保证 100% 可用。欢迎 Issue 或 PR。

目前已有的模块：

| 模块      | 说明                                                       | 是否验证 |
| --------- | ---------------------------------------------------------- | -------- |
| CAN       | CAN 底层驱动以及回调链表，所有 CAN 通信的电机都需要依赖此模块 | 是       |
| AK-Motor  | CubeMars AK 系列电机                                        | 是       |
| DJI-Motor | 大疆 M3508/2006/GM6020                                      | 是       |
| VESC      | vesc 电调                                                   | 是       |
| MLDS      | 铭朗科技 伺服电机                                             | 否       |
| Myantenna-L1 | 激光测距模块代码                                          | 见readme |