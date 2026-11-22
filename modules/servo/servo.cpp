/**
 * @file    servo.cpp
 * @brief   舵机角度语义实现
 *
 * 角度 → 脉宽的线性映射（SG90 典型值）：
 *   0°   → 500 µs
 *   90°  → 1500 µs
 *   180° → 2500 µs
 *   即 us = 500 + deg / 180 * 2000
 *
 * 注意：不同批次舵机的实际映射略有差异（有的 0° 是 600 µs），
 * 到货后应该实测标定，把 kUsPerDeg 按实测结果修正。
 */
#include "servo.hpp"

#include "log.hpp"

namespace module {
namespace {

constexpr float kUsAtZeroDeg = static_cast<float>(bsp::Pwm::kMinUs);   // 500
constexpr float kUsPerDeg =
    (static_cast<float>(bsp::Pwm::kMaxUs - bsp::Pwm::kMinUs)) / 180.0f;  // ≈11.11

inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

common::Err Servo::init(float center_deg)
{
  if (inited_) return common::Err::Busy;

  angle_ = clampf(center_deg, min_deg_, max_deg_);
  target_ = angle_;
  const common::Err e = write_angle(angle_);
  if (e != common::Err::Ok) return e;

  inited_ = true;
  enabled_ = true;
  LOG_I("servo ch%u init at %d(0.1deg) %lu us", (unsigned)ch_, (int)(angle_ * 10.0f),
        (unsigned long)pulse_us());
  return common::Err::Ok;
}

common::Err Servo::set_target(float deg)
{
  if (!inited_) return common::Err::NotInit;
  target_ = clampf(deg, min_deg_, max_deg_);
  return common::Err::Ok;
}

common::Err Servo::set_now(float deg)
{
  if (!inited_) return common::Err::NotInit;
  angle_ = clampf(deg, min_deg_, max_deg_);
  target_ = angle_;
  return write_angle(angle_);
}

common::Err Servo::update(float dt_s)
{
  if (!inited_) return common::Err::NotInit;
  if (!enabled_) return common::Err::Ok;

  const float diff = target_ - angle_;
  if (diff > -deadband_ && diff < deadband_) return common::Err::Ok;   // 死区内不动

  // 斜率限制：这一周期最多走 slew_ * dt
  const float max_step = slew_ * (dt_s > 0.0f ? dt_s : 0.005f);
  float step = diff;
  if (step > max_step) step = max_step;
  if (step < -max_step) step = -max_step;

  angle_ += step;
  return write_angle(angle_);
}

std::uint32_t Servo::pulse_us() const
{
  return pwm_.get_pulse_us(ch_);
}

void Servo::set_limits(float min_deg, float max_deg)
{
  if (min_deg < 0.0f) min_deg = 0.0f;
  if (max_deg > 180.0f) max_deg = 180.0f;
  if (min_deg > max_deg) {
    const float t = min_deg;
    min_deg = max_deg;
    max_deg = t;
  }
  min_deg_ = min_deg;
  max_deg_ = max_deg;
}

common::Err Servo::write_angle(float deg)
{
  const float us_f = kUsAtZeroDeg + deg * kUsPerDeg;
  std::uint32_t us = static_cast<std::uint32_t>(us_f + 0.5f);
  return pwm_.set_pulse_us(ch_, us);
}

}  // namespace module
