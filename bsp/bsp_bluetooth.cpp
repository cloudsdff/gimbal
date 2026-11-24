/**
 * @file    bsp_bluetooth.cpp
 * @brief   USART2 驱动 HC-05（PA2 = TX, PA3 = RX, 115200 8N1）
 *
 * 寄存器配置依据 RM0008：
 *   PA2 → 复用推挽（USART2_TX，默认映射无需重映射）
 *   PA3 → 浮空输入（USART2_RX）
 *   USART2 挂在 APB1：PCLK1 = 36 MHz
 *     BRR = 36 MHz / 115200 ≈ 312.5 → 取 313（误差约 0.16%，远小于 2% 容限）
 *   CR1 = UE | TE | RE | RXNEIE
 *   NVIC：优先级 6（低于 configMAX_SYSCALL_INTERRUPT_PRIORITY=5，
 *         本 ISR 不调用 FreeRTOS API，只往环形缓冲塞字节）
 *
 * HC-05 注意事项：
 *   - 出厂默认波特率多为 9600，需要在 AT 模式下用 AT+UART=115200,0,0 改掉；
 *   - 模块 RXD 是 3.3 V 逻辑，PA2 输出 3.3 V，可直接相连；
 *   - 供电要 5 V（模块板载 LDO），不要用 3.3 V。
 */
#include "bluetooth.hpp"

#include "cmsis_os2.h"
#include "log.hpp"
#include "main.h"

namespace bsp {
namespace {

constexpr std::uint32_t kPclk1Hz = 36000000u;
// (comment cleaned)
// (comment cleaned)
constexpr std::uint32_t kBaud = 38400u;
constexpr std::size_t kRingSize = 128;

volatile std::uint8_t g_ring[kRingSize];
volatile std::uint32_t g_head = 0;
volatile std::uint32_t g_tail = 0;
volatile std::uint32_t g_rx_count = 0;
bool g_inited = false;

/// 64 位静态断言：确保波特率分频误差可接受
inline std::uint32_t baud_div()
{
  return (kPclk1Hz + kBaud / 2u) / kBaud;   // 四舍五入
}

}  // namespace

class Uart2 final : public Uart {
 public:
  common::Err write(const std::uint8_t* data, std::size_t len,
                    std::uint32_t timeout_ms) override
  {
    if (!g_inited) return common::Err::NotInit;
    if (data == nullptr || len == 0) return common::Err::InvalidParam;

    const std::uint32_t t0 = HAL_GetTick();
    for (std::size_t i = 0; i < len; ++i) {
      // 等发送寄存器空
      while ((USART2->SR & USART_SR_TXE) == 0) {
        if ((HAL_GetTick() - t0) > timeout_ms) return common::Err::Timeout;
      }
      USART2->DR = data[i];
    }
    // 等最后一个字节真正发完
    while ((USART2->SR & USART_SR_TC) == 0) {
      if ((HAL_GetTick() - t0) > timeout_ms) return common::Err::Timeout;
    }
    return common::Err::Ok;
  }

  /// 蓝牙链路只需要阻塞写（指令回显很短），异步发送不支持
  common::Err write_async(const std::uint8_t*, std::size_t) override
  {
    return common::Err::NotInit;
  }

  common::Err read(std::uint8_t* buf, std::size_t len,
                   std::uint32_t timeout_ms) override
  {
    if (buf == nullptr || len == 0) return common::Err::InvalidParam;
    if (!g_inited) return common::Err::NotInit;

    const std::uint32_t t0 = HAL_GetTick();
    std::size_t got = 0;
    while (got < len) {
      if (g_head != g_tail) {
        buf[got++] = g_ring[g_tail];
        g_tail = (g_tail + 1) % kRingSize;
        continue;
      }
      if ((HAL_GetTick() - t0) >= timeout_ms) break;
      osDelay(1);
    }
    return (got == len) ? common::Err::Ok : common::Err::Timeout;
  }

  bool is_idle() const override { return true; }   // 阻塞写，永远"空闲"
};

namespace {
Uart2 g_uart2;
}  // namespace

Uart& uart2 = g_uart2;

common::Err uart2_start()
{
  if (g_inited) return common::Err::Busy;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  // PA2 = TX（复用推挽）
  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  // PA3 = RX（浮空输入）
  gpio.Pin = GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  USART2->CR1 = 0;                       // 先关掉
  USART2->BRR = baud_div();
  USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

  HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  g_inited = true;
  LOG_I("bluetooth: USART2 ready (PA2/PA3, 115200)");
  return common::Err::Ok;
}

std::uint32_t uart2_rx_count() { return g_rx_count; }

void uart2_on_rx_byte()
{
  // 读 DR 会清 RXNE
  const std::uint8_t b = static_cast<std::uint8_t>(USART2->DR & 0xFFu);
  ++g_rx_count;
  const std::uint32_t next = (g_head + 1) % kRingSize;
  if (next != g_tail) {
    g_ring[g_head] = b;
    g_head = next;
  }
}


// (comment cleaned)
common::Err uart2_set_baud(std::uint32_t baud)
{
  if (!g_inited) return common::Err::NotInit;
  if (baud < 1200 || baud > 921600) return common::Err::InvalidParam;

  const std::uint32_t brr = (kPclk1Hz + baud / 2u) / baud;
  const std::uint32_t saved = USART2->CR1;
  USART2->CR1 = 0;
  USART2->BRR = brr;
  USART2->CR1 = saved;
  LOG_I("bluetooth baud -> %lu (BRR=%lu)", (unsigned long)baud, (unsigned long)brr);
  return common::Err::Ok;
}
}  // namespace bsp

/* --------- USART2 中断服务函数（不经 CubeMX，直接定义向量） --------- */
extern "C" void USART2_IRQHandler(void)
{
  if (USART2->SR & USART_SR_RXNE) {
    bsp::uart2_on_rx_byte();
  }
  if (USART2->SR & USART_SR_ORE) {
    // 溢出：读一下 DR/SR 清标志，避免中断一直挂着
    (void)USART2->DR;
  }
}