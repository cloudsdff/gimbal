/**
 * @file    bluetooth.hpp
 * @brief   蓝牙串口（USART2，PA2=TX / PA3=RX）对外入口
 *
 * 为什么单独一条串口：
 *   USART1 是调试口（日志 + 角度帧已经占满），
 *   蓝牙用作"指令通道"，两条链路互不干扰。
 *
 * 为什么直接配寄存器而不走 CubeMX：
 *   只为加一个 USART2 就重新生成工程，会把 FreeRTOSConfig.h 里的堆大小重置
 *   （任务01 踩过两次），而且生成后还要手工加中断服务函数。直接配置更可控。
 */
#ifndef BSP_BLUETOOTH_HPP
#define BSP_BLUETOOTH_HPP

#include <cstdint>

#include "uart.hpp"

namespace bsp {

/// 全局唯一的 USART2 实例（接 HC-05）
extern Uart& uart2;

/// 启动中断接收与串口（app_init 里调一次）
common::Err uart2_start();

/// 运行时修改蓝牙串口波特率（HC-05 出厂多为 9600，可用 `B,<baud>` 指令切换）
common::Err uart2_set_baud(std::uint32_t baud);

/// 累计收到的字节数（诊断用）
std::uint32_t uart2_rx_count();

/// 由中断服务函数调用（内部使用）
void uart2_on_rx_byte();

}  // namespace bsp

#endif  // BSP_BLUETOOTH_HPP
