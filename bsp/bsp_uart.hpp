/**
 * @file    bsp_uart.hpp
 * @brief   串口板级实现的对外入口
 *
 * 上层（app / modules）只用这里的 uart1，不直接见 HAL。
 */
#ifndef BSP_UART_IMPL_HPP
#define BSP_UART_IMPL_HPP

#include "uart.hpp"

namespace bsp {

/// 全局唯一的 USART1 实例（对应 CubeMX 生成的 huart1）
extern Uart& uart1;

/// 由 HAL 的发送完成回调转调（内部使用）
void uart1_on_tx_cplt();

}  // namespace bsp

/// 给 C 层用的 printf 落地点（main.c 的 __io_putchar 调用它）
extern "C" int bsp_uart1_putchar(int ch);

#endif  // BSP_UART_IMPL_HPP
