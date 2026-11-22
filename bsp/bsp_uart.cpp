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

#include "cmsis_os2.h"
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
volatile bool g_tx_busy = false;   // ?????(DMA ????)
volatile bool g_dma_active = false; // DMA ??????(?????)

// ????:????????? DMA ?? USART1
osMutexId_t g_tx_mutex = nullptr;

void ensure_mutex()
{
  if (g_tx_mutex == nullptr) {
    g_tx_mutex = osMutexNew(nullptr);
  }
}

// ---- 中断接收环形缓冲（串口指令用，非阻塞读取）----
constexpr std::size_t kRxRingSize = 128;
volatile std::uint8_t g_rx_ring[kRxRingSize];
volatile std::uint32_t g_rx_head = 0;
volatile std::uint32_t g_rx_tail = 0;
std::uint8_t g_rx_byte = 0;
bool g_rx_armed = false;
volatile std::uint32_t g_rx_count = 0;          // DMA 是否正在发送

}  // namespace

class Uart1 final : public Uart {
 public:
  common::Err write(const std::uint8_t* data, std::size_t len,
                    std::uint32_t timeout_ms) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    if (len > 0xFFFFu) return common::Err::InvalidParam;   // HAL 长度参数是 uint16_t

    // 串行化：等上一帧 DMA 发完，再拿锁做阻塞写
    uart1_tx_lock();
    if (!uart1_wait_idle(timeout_ms)) {
      uart1_tx_unlock();
      return common::Err::Busy;
    }

    // 关键：阻塞写期间同样置"总线占用"，否则 write_async 会在这段时间插进来
    g_tx_busy = true;
    const auto st = HAL_UART_Transmit(kHuart, const_cast<std::uint8_t*>(data),
                                      static_cast<std::uint16_t>(len), timeout_ms);
    g_tx_busy = false;
    uart1_tx_unlock();
    if (st == HAL_OK) return common::Err::Ok;
    if (st == HAL_TIMEOUT) return common::Err::Timeout;
    return common::Err::BusError;
  }

  common::Err write_async(const std::uint8_t* data, std::size_t len) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    if (len > kTxBufSize) return common::Err::Overflow;    // 明确告知装不下，不静默截断
    if (kHuart->hdmatx == nullptr) return common::Err::NotInit;  // CubeMX 里没开 DMA

    uart1_tx_lock();
    if (g_tx_busy) {                                       // 上一帧没发完
      uart1_tx_unlock();
      return common::Err::Busy;
    }

    // 先拷进内部缓冲：DMA 在后台读这块内存，调用方的缓冲随时可能被复用
    std::memcpy(g_tx_buf, data, len);
    g_tx_busy = true;
    g_dma_active = true;

    const auto st = HAL_UART_Transmit_DMA(kHuart, g_tx_buf,
                                          static_cast<std::uint16_t>(len));
    uart1_tx_unlock();
    if (st != HAL_OK) {
      g_dma_active = false;
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

    // 优先从中断接收环形缓冲取（非阻塞，timeout=0 也能用）
    const std::uint32_t t0 = HAL_GetTick();
    std::size_t got = 0;
    while (got < len) {
      if (g_rx_head != g_rx_tail) {
        buf[got++] = g_rx_ring[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1) % kRxRingSize;
        continue;
      }
      if ((HAL_GetTick() - t0) >= timeout_ms) break;
      osDelay(1);
    }
    if (got == len) return common::Err::Ok;

    // 环形缓冲没数据时，退化为阻塞读（保留原语义）
    if (got == 0 && timeout_ms > 0) {
      const auto st = HAL_UART_Receive(kHuart, buf, static_cast<std::uint16_t>(len),
                                       timeout_ms);
      if (st == HAL_OK) return common::Err::Ok;
      if (st == HAL_TIMEOUT) return common::Err::Timeout;
      return common::Err::BusError;
    }
    return common::Err::Timeout;
  }

  bool is_idle() const override { return !g_tx_busy; }

  /// 由 HAL 的发送完成回调调用（中断上下文，只清标志）
  void on_tx_cplt()
  {
    // ??????? DMA ??,??????;
    // ????'?????'????????,??????(???)
    if (g_dma_active) {
      g_dma_active = false;
      g_tx_busy = false;
    }
  }
};

namespace {
Uart1 g_uart1;
}  // namespace

Uart& uart1 = g_uart1;

void uart1_on_tx_cplt() { g_uart1.on_tx_cplt(); }


void uart1_tx_lock()
{
  ensure_mutex();
  if (g_tx_mutex) (void)osMutexAcquire(g_tx_mutex, 200);
}

void uart1_tx_unlock()
{
  if (g_tx_mutex) (void)osMutexRelease(g_tx_mutex);
}

bool uart1_wait_idle(std::uint32_t timeout_ms)
{
  const std::uint32_t t0 = HAL_GetTick();
  while (g_tx_busy) {
    if ((HAL_GetTick() - t0) > timeout_ms) return false;
    for (volatile int i = 0; i < 200; ++i) {}   // ???(???????????)
  }
  return true;
}

/// 启动中断接收（在 app_init 里调一次）
void uart1_start_rx()
{
  if (!g_rx_armed) {
    g_rx_armed = true;
    const HAL_StatusTypeDef st = HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1);
    LOG_I("uart rx armed: st=%d", (int)st);
  }
}

/// 由中断回调调用：存一个字节并重新武装
void uart1_on_rx_byte()
{
  ++g_rx_count;
  const std::uint32_t next = (g_rx_head + 1) % kRxRingSize;
  if (next != g_rx_tail) {           // ??????,??????
    g_rx_ring[g_rx_head] = g_rx_byte;
    g_rx_head = next;
  }
  (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1);
}
std::uint32_t uart1_rx_count() { return g_rx_count; }

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
/* -------------------- HAL ??(?????) -------------------- */

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  if (huart == &huart1) bsp::uart1_on_rx_byte();
}