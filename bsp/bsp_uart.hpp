/**
 * @file    bsp_uart.hpp
 * @brief   串口板级实现的对外入口
 *
 * 上层（app / modules）只用这里的 uart1，不直接见 HAL。
 */
#ifndef BSP_UART_IMPL_HPP
#define BSP_UART_IMPL_HPP

#include <cstdint>

#include "uart.hpp"

namespace bsp {

/// 全局唯一的 USART1 实例（对应 CubeMX 生成的 huart1）
extern Uart& uart1;

/// 由 HAL 的发送完成回调转调（内部使用）
void uart1_on_tx_cplt();

// ---- 发送串行化 ----
// USART1 上有两个写者：异步日志（阻塞写）和角度帧（DMA）。
// 一条物理链路只能有一个"当前所有者"，否则字节流会交织（实测过）。
void uart1_tx_lock();
void uart1_tx_unlock();
bool uart1_wait_idle(std::uint32_t timeout_ms);

// ---- 中断接收（串口指令用）----
// 轮询式 HAL_UART_Receive(timeout=0) 读不到字节，必须用中断 + 环形缓冲。
void uart1_start_rx();
void uart1_on_rx_byte();

// (comment cleaned)
std::uint32_t uart1_rx_count();

}  // namespace bsp

/// 给 C 层用的 printf 落地点（main.c 的 __io_putchar 调用它）
extern "C" int bsp_uart1_putchar(int ch);

#endif  // BSP_UART_IMPL_HPP
