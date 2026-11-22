/**
 * @file    bsp_uart.cpp
 * @brief   USART1 的板级实现（阻塞 + DMA 两种发送方式）
 *
 * 这个文件是整个工程里**唯一**允许出现 HAL_UART_xxx 的地方。
 * 上层看到的只有 bsp::Uart 接口和 common::Err 返回值。
 *
 * 对应任务01 的两条工程要求：
 *   要求 1：外设只通过统一接口暴露，业务代码不直接调用 HAL
 *   要求 3：至少支持阻塞 / 中断 / DMA 三种方式中的两种（这里是阻塞 + DMA）
 *
 * 设计上刻意补掉了参考框架的三个问题：
 *   - 不再 while(1) 死等，失败返回错误码（Err）
 *   - DMA 忙时返回 Err::Busy，而不是悄悄把上一帧覆盖掉
 *   - 数据先拷进内部缓冲，避免调用方提前改内存导致发出脏数据
 */
#include "bsp_uart.hpp"

#include <cstring>

#include "main.h"           // huart1 / HAL 声明
#include "log.hpp"          // common/ 已在 include 路径里

// CubeMX 把 huart1 定义在 Core/Src/main.c 里（没有单独的头文件），
// 这里用 extern "C" 声明一下，链接时才能找到那个 C 符号。
extern "C" {
extern UART_HandleTypeDef huart1;
}

namespace bsp {
namespace {

constexpr std::size_t kTxBufSize = 256;   // 内部发送缓冲（占 256 B RAM）

UART_HandleTypeDef* const kHuart = &huart1;

std::uint8_t g_tx_buf[kTxBufSize];
volatile bool g_tx_busy = false;          // DMA 是否正在发送

}  // namespace

class Uart1 final : public Uart {
 public:
  common::Err write(const std::uint8_t* data, std::size_t len,
                    std::uint32_t timeout_ms) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    if (len > 0xFFFFu) return common::Err::InvalidParam;   // HAL 长度参数是 uint16_t
    if (g_tx_busy) return common::Err::Busy;

    const auto st = HAL_UART_Transmit(kHuart, const_cast<std::uint8_t*>(data),
                                      static_cast<std::uint16_t>(len), timeout_ms);
    if (st == HAL_OK) return common::Err::Ok;
    if (st == HAL_TIMEOUT) return common::Err::Timeout;
    return common::Err::BusError;
  }

  common::Err write_async(const std::uint8_t* data, std::size_t len) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    if (len > kTxBufSize) return common::Err::Overflow;    // 明确告知装不下，不静默截断
    if (kHuart->hdmatx == nullptr) return common::Err::NotInit;  // CubeMX 里没开 DMA
    if (g_tx_busy) return common::Err::Busy;               // 上一帧没发完

    // 先拷进内部缓冲：DMA 在后台读这块内存，调用方的缓冲随时可能被复用
    std::memcpy(g_tx_buf, data, len);
    g_tx_busy = true;

    const auto st = HAL_UART_Transmit_DMA(kHuart, g_tx_buf,
                                          static_cast<std::uint16_t>(len));
    if (st != HAL_OK) {
      g_tx_busy = false;                                   // 启动失败要把标志还原
      return common::Err::BusError;
    }
    return common::Err::Ok;
  }

  common::Err read(std::uint8_t* buf, std::size_t len,
                   std::uint32_t timeout_ms) override
  {
    if (buf == nullptr || len == 0) return common::Err::InvalidParam;
    if (len > 0xFFFFu) return common::Err::InvalidParam;

    const auto st = HAL_UART_Receive(kHuart, buf, static_cast<std::uint16_t>(len),
                                     timeout_ms);
    if (st == HAL_OK) return common::Err::Ok;
    if (st == HAL_TIMEOUT) return common::Err::Timeout;
    return common::Err::BusError;
  }

  bool is_idle() const override { return !g_tx_busy; }

  /// 由 HAL 的发送完成回调调用（中断上下文，只清标志）
  void on_tx_cplt() { g_tx_busy = false; }
};

namespace {
Uart1 g_uart1;
}  // namespace

Uart& uart1 = g_uart1;

void uart1_on_tx_cplt() { g_uart1.on_tx_cplt(); }

}  // namespace bsp

/* -------------------------------------------------------------------------
 * C 接口部分：让 main.c 的 printf 也能走这层，Core 里不必再出现 HAL_UART_xxx
 * ---------------------------------------------------------------------- */

extern "C" int bsp_uart1_putchar(int ch)
{
  const std::uint8_t b = static_cast<std::uint8_t>(ch);
  // timeout 给 10 ms：115200 下一字节约 87 us，10 ms 足够且不会卡死太久
  if (HAL_UART_Transmit(&huart1, const_cast<std::uint8_t*>(&b), 1, 10) == HAL_OK) {
    return ch;
  }
  return -1;
}

/// 覆盖 HAL 的弱函数：DMA 发送完成时清忙标志
extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == &huart1) {
    bsp::uart1_on_tx_cplt();
  }
}
