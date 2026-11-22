/**
 * @file    bsp_pwm.cpp
 * @brief   TIM3 CH1/CH2 输出 50 Hz 舵机 PWM（PA6 / PA7）
 *
 * 为什么直接写寄存器，而不走 CubeMX：
 *   1. 只为加一个定时器就重新生成工程，会把 FreeRTOSConfig.h 里的堆大小重置（踩过两次）；
 *   2. 代码更集中，评审看得到"这个模块到底配了什么"；
 *   3. TIM3 的 PWM 配置在 RM0008 第 15 章写得很清楚，照着手册配即可。
 *
 * 配置（PCLK1 = 72 MHz，定时器时钟 = 72 MHz）：
 *   PSC = 72 - 1        → 计数频率 1 MHz（1 tick = 1 µs）
 *   ARR = 20000 - 1     → 周期 20 ms（50 Hz）
 *   CCR = 脉宽 µs       → 500–2500 µs 对应 0–180°
 */
#include "pwm.hpp"

#include "main.h"
#include "log.hpp"

namespace bsp {
namespace {

constexpr std::uint32_t kTimerClockHz = 72000000u;   ///< TIM3 挂在 APB1，定时器时钟 72 MHz
constexpr std::uint32_t kPsc = kTimerClockHz / 1000000u - 1u;   // 1 MHz
constexpr std::uint32_t kArr = 20000u - 1u;                     // 20 ms

std::uint32_t g_pulse[2] = {1500, 1500};   ///< 默认中位
bool g_inited = false;

/// CH1/CH2 使能 + PWM 模式 1（CNT < CCR 时输出高）
void configure_channels()
{
  // CCMR1 低 8 位管 CH1，高 8 位管 CH2
  // OCxM = 110 (PWM mode 1)，OCxPE = 1（预装载使能）
  TIM3->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
  // CCER: CC1E=1, CC2E=1（高电平有效）
  TIM3->CCER = (1u << 0) | (1u << 4);
  TIM3->CCR1 = g_pulse[0];
  TIM3->CCR2 = g_pulse[1];
}

}  // namespace

class ServoPwm final : public Pwm {
 public:
  common::Err init() override
  {
    if (g_inited) return common::Err::Busy;

    // 1) 时钟：TIM3 与 GPIOA
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2) 引脚：PA6 / PA7 → 复用推挽输出（TIM3_CH1/CH2 的默认映射，无需重映射）
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    // 3) 定时器时基
    TIM3->CR1 = 0;                 // 先停
    TIM3->PSC = kPsc;
    TIM3->ARR = kArr;
    TIM3->CNT = 0;
    configure_channels();
    TIM3->EGR = 1;                 // UG：立即把 PSC/ARR 生效
    TIM3->CR1 = 1;                 // CEN：启动计数

    g_inited = true;
    LOG_I("pwm: TIM3 CH1/CH2 @50Hz ready (PA6/PA7)");
    return common::Err::Ok;
  }

  common::Err set_pulse_us(std::uint8_t ch, std::uint32_t us) override
  {
    if (!g_inited) return common::Err::NotInit;
    if (ch < 1 || ch > 2) return common::Err::InvalidParam;

    common::Err ret = common::Err::Ok;
    if (us < kMinUs) { us = kMinUs; ret = common::Err::InvalidParam; }
    if (us > kMaxUs) { us = kMaxUs; ret = common::Err::InvalidParam; }

    g_pulse[ch - 1] = us;
    if (ch == 1) {
      TIM3->CCR1 = us;
    } else {
      TIM3->CCR2 = us;
    }
    return ret;
  }

  std::uint32_t get_pulse_us(std::uint8_t ch) const override
  {
    if (ch < 1 || ch > 2) return 0;
    return g_pulse[ch - 1];
  }
};

namespace {
ServoPwm g_pwm;
}  // namespace

Pwm& servo_pwm = g_pwm;

}  // namespace bsp
