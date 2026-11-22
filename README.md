# 任务01 Frameworks —— STM32 分层框架

> **仓库地址**：https://github.com/cloudsdff/f103_w1
> **移植版**：https://github.com/cloudsdff/f407_port
> **演示视频**：https://github.com/cloudsdff/f103_w1/releases/tag/v1.0

一套 C++17 写的 STM32 三层框架（Application / Module / BSP），
在 **STM32F103C8T6** 上完成开发与实测，并**零改动业务代码**移植到 **STM32F407VET6**，
两块板通过 500 kbps CAN 总线双向互通。

```
app/app_main.cpp        ← 业务逻辑（看不到 HAL）
modules/mpu6050/        ← 传感器语义（看不到 HAL）
bsp/uart|i2c|can        ← 纯虚接口 + HAL 实现（HAL 只在 .cpp 里）
common/error_code|log   ← 统一错误码 + 异步日志
```

## 一、五项要求的完成情况

| 要求 | 做法 | 实测证据 |
|---|---|---|
| **分层** | BSP/Module/App 三层，接口纯虚类 | `app/` `modules/` 搜不到 `HAL_`；移植时业务层改动 **0 行** |
| **FreeRTOS 任务划分** | 5 个任务，优先级/周期/栈逐一论证 | 实测剩余栈 Comm=352 B、Sensor=360 B、task_log=1296 B |
| **每外设 ≥2 种方式** | UART 阻塞+DMA、I2C 阻塞+中断、CAN 阻塞发+中断收 | 25600 字节零错位；I2C 拔线 4 ms 自愈；CAN 1800+ 帧无错 |
| **统一错误码 + 日志** | `Err` 10 种错误码 + 异步日志（队列+低优先级任务） | 六条失败路径实测，`logdrop` 可观测 |
| **移植到不同型号** | F103 → F407，只改 3 个文件 | `PCLK1=42000000`，跨型号 CAN 双向互通 |

## 二、硬件

| 器件 | 型号 | 用途 |
|---|---|---|
| 主控 A | STM32F103C8T6（蓝药丸） | 主开发板，Cortex-M3 @72 MHz |
| 主控 B | STM32F407VET6（黑卡板） | 移植验证，Cortex-M4F @168 MHz |
| 传感器 | MPU6050 | I2C1（PB6/PB7），地址 0x68 |
| 收发器 | TJA1050 ×2 | CAN1（PA11/PA12），500 kbps，两端各 120 Ω |
| 调试器 | J-Link（含 CDC 串口） | SWD + 115200 串口 |

## 三、目录

```
f103_w1/
├── app/            应用层（任务划分、业务循环）
├── bsp/            板级支持包（uart / i2c / can）
├── common/         错误码、异步日志、RTOS 钩子
├── modules/        模块层（mpu6050）
├── Core/ Drivers/ Middlewares/    CubeMX 生成（HAL + FreeRTOS）
├── docs/           全部设计文档与实测记录
└── CMakeLists.txt  构建脚本
```

业务代码共 **1012 行**（不含生成代码）。

## 四、编译与烧录

### 环境

| 工具 | 版本 |
|---|---|
| arm-none-eabi-gcc | 14.3.Rel1 |
| CMake | 4.x |
| Ninja | 1.13 |
| STM32CubeMX | 6.18 |
| STM32Cube FW_F1 | V1.8.7 |

### 编译

```powershell
cd D:\stm32\f103_w1
cmake --preset Debug
cmake --build --preset Debug
```

产物：`build/Debug/f103_w1.elf`

### 编译对端节点（B 节点，发 0x456）

```powershell
cmake -B build/B -G Ninja -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/gcc-arm-none-eabi.cmake -DCAN_NODE_B=ON
cmake --build build/B
```

### 烧录（J-Link Commander）

```text
loadfile build\Debug\f103_w1.elf
r
g
qc
```

## 五、运行现象

```text
===== D2 boot test =====
SYSCLK=72000000 HCLK=72000000 PCLK1=36000000 PCLK2=72000000
[I][12] app layer ready (C++17, HAL-free)
[I][12] can: started (filter id=0x456, FIFO0 irq on)
[I][1023] acc[-1048,220,18336] gyro[-140,-4,0]
[I][1037] rx id=0x456 len=8 seq=13799
[I][5016] stats: tx=49 rx=49 drop=0 err=0 busoff=0 logsent=21 logdrop=0
[I][5016] stack free: Comm=352 Sensor=360
```

## 六、演示视频

`桌面\Helios 任务01\演示视频-任务01.mp4`（3 分 30 秒，1920x1080，H.264，含中文字幕）

内容：① 双板 CAN 互通 ② 启动与时钟 ③ 任务与栈水线 ④ 总线故障与恢复 ⑤ 分层证据 ⑥ 移植到 F407

录制流程见 [11-录制操作单](docs/11-录制操作单.md)，提交清单见 [SUBMISSION.md](SUBMISSION.md)。

## 七、文档索引

| 文档 | 内容 |
|---|---|
| [01-阅读笔记](docs/01-阅读笔记.md) | 精读开源框架 `HNUYueLuRM/basic_framework` 的记录 |
| [01-开源框架分析](docs/01-开源框架分析.md) | 四条具体批判（带文件名+行号） |
| [02-UART实测记录](docs/02-UART实测记录.md) | 阻塞 vs DMA，25600 字节压测，96.5% 带宽利用率 |
| [03-I2C与MPU6050实测](docs/03-I2C与MPU6050实测.md) | 六轴数据、1.119g 偏差分析、拔线 4ms 自愈 |
| [04-CAN双节点实测](docs/04-CAN双节点实测.md) | 500 kbps 参数推导、双板互通、三个坑 |
| [05-任务设计](docs/05-任务设计.md) | 5 个任务的优先级/周期/栈论证与实测 |
| [06-失败路径矩阵](docs/06-失败路径矩阵.md) | 六条故障的实测证据 |
| [07-移植说明](docs/07-移植说明.md) | F103→F407 的 6 处改动与跨型号互通 |
| [08-架构与目录说明](docs/08-架构与目录说明.md) | 分层结构、依赖规则、资源占用 |
| [09-接口说明](docs/09-接口说明.md) | 四个接口 + 返回值语义 |
| [10-演示视频脚本](docs/10-演示视频脚本.md) | 分镜脚本 |
| [ai_log/](ai_log/) | AI 助手协作记录 |

## 八、移植版工程

F407 版本是独立仓库：`D:\stm32\f407_port`（结构相同，`docs/` 内含同一套文档）。

差异总结：`bsp_can.cpp` 改了 3 行句柄名（`hcan`→`hcan1`），
`main.c` 加 3 处 USER CODE，CubeMX 里改 CAN 位时序（PCLK1 从 36→42 MHz）
并勾上 `CAN1_SCE interrupt`（F4 把 CAN 错误中断拆成了独立向量）。

## 九、已知限制

- F103 的 Flash 已用 83%，继续加模块需先换 `-Os`
- I2C 上拉依赖从机模块，模块断电时会报 `Busy` 而不是 `NoAck`（建议主机侧加 4.7k）
- F407 未开 FPU（当前用软浮点，对现有功能无影响）
- 日志丢弃策略为"丢最新"，可改为"丢最旧"以保留崩溃前现场
