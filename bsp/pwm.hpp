/**
 * @file    pwm.hpp
 * @brief   舵机 PWM 抽象接口（TIM3_CH1/CH2，50 Hz）
 *
 * 为什么是 50 Hz：
 *   SG90 这类模拟舵机的标准控制信号是 20 ms 周期（50 Hz），
 *   高电平宽度 0.5–2.5 ms 对应 0–180°。改成更高频率会让舵机内部
 *   比较电路误判，出现抖动、发热、甚至烧毁。
 */
#ifndef BSP_PWM_HPP
#define BSP_PWM_HPP

#include <cstdint>

#include "error_code.hpp"

namespace bsp {

class Pwm {
 public:
  virtual ~Pwm() = default;

  /// 初始化定时器与引脚（上电后调用一次）
  virtual common::Err init() = 0;

  /// 设置高电平宽度（微秒）。超出 500–2500 会被夹紧并返回 InvalidParam
  virtual common::Err set_pulse_us(std::uint8_t channel, std::uint32_t us) = 0;

  /// 读取当前设置值（微秒）
  virtual std::uint32_t get_pulse_us(std::uint8_t channel) const = 0;

  static constexpr std::uint32_t kMinUs = 500;    ///< 0°
  static constexpr std::uint32_t kMaxUs = 2500;   ///< 180°
};

/// 两路舵机 PWM（channel 1 → PA6，channel 2 → PA7）
extern Pwm& servo_pwm;

}  // namespace bsp

#endif  // BSP_PWM_HPP
