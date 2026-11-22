/**
 * @file    servo.hpp
 * @brief   舵机模块：角度语义 + 限位 + 上电缓启动
 *
 * 分两层：
 *   bsp::Pwm    只管"输出多宽的脉冲"（500–2500 µs）
 *   module::Servo 管"角度语义"（0–180°）、机械限位、缓启动斜率
 *
 * 为什么需要缓启动：
 *   SG90 上电瞬间如果直接给 180° 的目标，会以最大速度撞向机位限位，
 *   轻则抖动，重则扫齿。缓启动把"跳变"变成"斜坡"，按 deg/s 推进。
 */
#ifndef MODULE_SERVO_HPP
#define MODULE_SERVO_HPP

#include <cstdint>

#include "error_code.hpp"
#include "pwm.hpp"

namespace module {

class Servo {
 public:
  Servo(bsp::Pwm& pwm, std::uint8_t channel) : pwm_(pwm), ch_(channel) {}

  /// @param center_deg 上电时停在哪（一般 90 = 中位）
  common::Err init(float center_deg = 90.0f);

  /// 设置目标角度（0–180°）。会做限位夹紧 + 斜率限制
  common::Err set_target(float deg);

  /// 直接跳到某个角度（调试用，绕过斜率限制）
  common::Err set_now(float deg);

  /// 每个控制周期调用一次，把当前角度朝目标推进 dt 对应的量
  common::Err update(float dt_s);

  float target() const { return target_; }
  float angle() const { return angle_; }
  std::uint32_t pulse_us() const;

  /// 机械限位（默认 20–160°，留出余量避免撞死）
  void set_limits(float min_deg, float max_deg);
  /// 「缓启动」：默认 60 °/s，上电后 1.5 秒内走完 90°
  void set_slew_rate(float deg_per_s) { slew_ = deg_per_s; }
  /// 死区：目标与当前差小于该值就不动，避免舵机来回抖
  void set_deadband(float deg) { deadband_ = deg; }

  void enable(bool on) { enabled_ = on; }

 private:
  common::Err write_angle(float deg);

  bsp::Pwm& pwm_;
  std::uint8_t ch_;
  float angle_ = 90.0f;
  float target_ = 90.0f;
  float min_deg_ = 20.0f;
  float max_deg_ = 160.0f;
  float slew_ = 60.0f;      ///< 最大角速度（deg/s）
  float deadband_ = 0.3f;   ///< 角度死区（deg）
  bool enabled_ = false;
  bool inited_ = false;
};

}  // namespace module

#endif  // MODULE_SERVO_HPP
