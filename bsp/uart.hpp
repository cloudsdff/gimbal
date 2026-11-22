/**
 * @file    uart.hpp
 * @brief   串口抽象接口 —— 任务01 工程要求 1（业务不直接调 HAL）+ 要求 3（≥2 种方式）
 *
 * 设计意图：
 *   - application / modules 只看到这个接口，看不到 HAL_UART_xxx
 *   - 具体实现见 bsp_uart.cpp：用 huart1 等 HAL 句柄
 *   - 两种发送方式各自独立暴露：
 *       write()       阻塞（简单场景：调试打印、小数据）
 *       write_async() DMA  （大块/高频数据：日志、图像，不能卡住业务）
 *
 * 移植提示：换芯片时只需重写 bsp_uart.cpp 的实现，上层零改动。
 */
#ifndef BSP_UART_HPP
#define BSP_UART_HPP

#include <cstddef>
#include <cstdint>

#include "error_code.hpp"

namespace bsp {

class Uart {
 public:
  virtual ~Uart() = default;

  /// 阻塞发送：等全部发完或超时才返回
  /// @retval Ok / Timeout / InvalidParam / NotInit
  virtual common::Err write(const std::uint8_t* data, std::size_t len,
                            std::uint32_t timeout_ms) = 0;

  /// 非阻塞发送（DMA）：立即返回，完成后由回调清标志
  /// @retval Ok（已启动）/ Busy（上一笔未完成）/ Overflow / InvalidParam / NotInit
  virtual common::Err write_async(const std::uint8_t* data, std::size_t len) = 0;

  /// 阻塞接收
  /// @retval Ok / Timeout / InvalidParam / NotInit
  virtual common::Err read(std::uint8_t* buf, std::size_t len,
                           std::uint32_t timeout_ms) = 0;

  /// 是否空闲（上一笔 DMA 已发完）
  virtual bool is_idle() const = 0;
};

}  // namespace bsp

#endif  // BSP_UART_HPP
