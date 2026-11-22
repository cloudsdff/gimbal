/**
 * @file    attitude.cpp
 * @brief   互补滤波实现（含上电自校准与零漂抑制）
 *
 * 算法（每帧）：
 *   1) 把原始值换成物理量：加速度 → g，陀螺 → dps
 *   2) 加速度计给出"绝对角度"（低频准、高频抖）：
 *        roll_acc  = atan2(ay, az)
 *        pitch_acc = atan2(-ax, sqrt(ay^2 + az^2))
 *   3) 陀螺积分给出"增量角度"（高频准、低频漂）
 *        roll_g  = roll  + gx * dt
 *        pitch_g = pitch + gy * dt
 *   4) 加权融合（互补滤波）：
 *        roll  = alpha * roll_g  + (1-alpha) * roll_acc
 *        pitch = alpha * pitch_g + (1-alpha) * pitch_acc
 *
 * 为什么这样能"取长补短"：
 *   陀螺在短时间内积分非常准（噪声被积分平滑），但零偏会随时间累积成漂移；
 *   加速度计没有累积误差，但受振动和线性加速度影响，高频噪声大。
 *   两者在频域上互补，加权平均后高频跟陀螺、低频跟加速度计。
 *
 * 零漂抑制的两道措施：
 *   ① 上电自校准求零偏，之后每帧减掉（消除系统性漂移）
 *   ② 小角速度死区：|rate| < deadband 时视为 0（消除静止时的随机游走）
 *
 * yaw 的说明：
 *   MPU6050 没有磁力计，yaw 只能靠 gz 积分，**必然缓慢漂移**。
 *   本工程的处理：上电时把 yaw 清零，之后靠"死区 + 零偏"把漂移压到可接受范围；
 *   要真正消除，需要磁力计（如 HMC5883L）或外部参考（编码器/视觉）。
 */
#include "attitude.hpp"

#include <cmath>

namespace module {
namespace {

constexpr float kRadToDeg = 57.29577951308232f;

/// 死区处理：小幅角速度归零
inline float deadband(float v, float band)
{
  if (v > band) return v - band;
  if (v < -band) return v + band;
  return 0.0f;
}

}  // namespace

void Attitude::reset()
{
  euler_.yaw = 0.0f;
  euler_.pitch = 0.0f;
  euler_.roll = 0.0f;
}

void Attitude::begin_calibration(std::uint16_t samples)
{
  cal_state_ = CalState::Sampling;
  cal_target_ = (samples == 0) ? 1 : samples;
  cal_count_ = 0;
  cal_sum_x_ = 0.0f;
  cal_sum_y_ = 0.0f;
  cal_sum_z_ = 0.0f;
  bias_.x = bias_.y = bias_.z = 0.0f;
  reset();
}

void Attitude::set_alpha(float a)
{
  if (a < 0.5f) a = 0.5f;
  if (a > 0.999f) a = 0.999f;
  alpha_ = a;
}

common::Err Attitude::update(const Mpu6050::Raw& raw, float dt_s)
{
  if (dt_s <= 0.0f) dt_s = 0.005f;    // 兜底：默认 200 Hz
  if (dt_s > 0.2f) dt_s = 0.2f;       // 兜底：防止一次卡顿把积分打飞

  const float gx = to_dps(raw.gx);
  const float gy = to_dps(raw.gy);
  const float gz = to_dps(raw.gz);

  const float ax = to_g(raw.ax);
  const float ay = to_g(raw.ay);
  const float az = to_g(raw.az);

  // ---------- 上电自校准：静止采样求零偏 ----------
  if (cal_state_ == CalState::Sampling) {
    cal_sum_x_ += gx;
    cal_sum_y_ += gy;
    cal_sum_z_ += gz;
    ++cal_count_;
    if (cal_count_ >= cal_target_) {
      const float inv = 1.0f / static_cast<float>(cal_count_);
      bias_.x = cal_sum_x_ * inv;
      bias_.y = cal_sum_y_ * inv;
      bias_.z = cal_sum_z_ * inv;
      cal_state_ = CalState::Done;
      reset();
    }
    return common::Err::Busy;   // 校准期间不给角度
  }

  // ---------- 去零偏 + 死区 ----------
  const float gx_d = deadband(gx - bias_.x, deadband_dps_);
  const float gy_d = deadband(gy - bias_.y, deadband_dps_);
  const float gz_d = deadband(gz - bias_.z, deadband_dps_);

  // ---------- 加速度计角度 ----------
  const float roll_acc = std::atan2(ay, az) * kRadToDeg;
  const float pitch_acc = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg;

  // ---------- 互补滤波 ----------
  euler_.roll = alpha_ * (euler_.roll + gx_d * dt_s) + (1.0f - alpha_) * roll_acc;
  euler_.pitch = alpha_ * (euler_.pitch + gy_d * dt_s) + (1.0f - alpha_) * pitch_acc;
  euler_.yaw += gz_d * dt_s;   // 无磁力计，只能积分（会缓慢漂移）

  return common::Err::Ok;
}

}  // namespace module
