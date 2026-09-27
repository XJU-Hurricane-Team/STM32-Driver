# 一阶与二阶 LADRC

一阶对象采用二状态 ESO：`x1_dot = x2 + b*u_prev + beta1*(y-x1)`，`x2_dot = beta2*(y-x1)`；控制律为 `u = (kp*(r-x1)-x2)/b`。

二阶对象采用三状态 ESO：`x1_dot = x2 + beta1*e`，`x2_dot = x3 + b*u_prev + beta2*e`，`x3_dot = beta3*e`；控制律为 `u = [kp*(r1-x1)+kd*(r2-x2)-x3]/b`。`u_prev` 必须是上周期**实际交给执行器且已限幅**的控制命令，否则观测器的输入模型不成立。

带宽函数按角频率 rad/s 计算：一阶 ESO `beta1=2w0, beta2=w0²`；二阶 ESO `beta1=3w0, beta2=3w0², beta3=w0³`。这里的带宽参数不是 Hz。显式欧拉积分要求调用周期与 `dt` 一致；高带宽、采样抖动和量化噪声需要在项目里评估。

新接口移除了旧 `k_aw` 参数和重复的 `pre_out` 字段，这是一次接口不兼容变更。现有线性状态反馈没有积分状态；原实现的“抗积分饱和”修正既不是积分反算，符号还会增大饱和控制量。现在只限幅输出，并将该限幅值作为下周期 ESO 输入。若执行器还会在模块外再次裁剪，调用方需要扩展接口回传实际施加的控制量。

无效 `dt`、过小 `b`、非正输出上限或非有限输入使本周期返回零。`reset` 清空 ESO、TD 与输出状态；运行中重新配置后，应调用 `*_set_state(measure, target)` 以当前测量值和目标预置观测器与 TD。主机侧公式、限幅和复位测试见 `../tests/test_control_algorithms.c`；目前不能宣称实车验证。

公式核对参考：[线性 ESO 与一阶速度环模型](https://www.nature.com/articles/s41598-025-34362-z)、[线性 ADRC 的观测器及带宽设计](https://link.springer.com/chapter/10.1007/978-3-031-72687-3_3)。
