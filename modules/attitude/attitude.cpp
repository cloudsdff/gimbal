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
  cal_total_frames_ = 0;
  cal_sum_x_ = 0.0f;
  cal_sum_y_ = 0.0f;
  cal_sum_z_ = 0.0f;
  bias_.x = bias_.y = bias_.z = 0.0f;
  reset();
}

void Attitude::zero_here()
{
  offset_roll_ = last_roll_raw_;
  offset_pitch_ = last_pitch_raw_;
  euler_.roll = 0.0f;
  euler_.pitch = 0.0f;
  euler_.yaw = 0.0f;
  has_prev_ = false;
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

  float gx = to_dps(raw.gx);
  float gy = to_dps(raw.gy);
  float gz = to_dps(raw.gz);

  float ax = to_g(raw.ax);
  float ay = to_g(raw.ay);
  float az = to_g(raw.az);

  // (comment cleaned)
  if (flip_) {
    ay = -ay;
    az = -az;
    gy = -gy;
    gz = -gz;
  }

  // ---------- 上电自校准：静止采样求零偏 ----------
  if (cal_state_ == CalState::Sampling) {
    ++cal_total_frames_;

    // 静止校验：角速度太大说明板子在被碰，这一帧的加速度角不可信
    const float rate_mag = std::sqrt(gx * gx + gy * gy + gz * gz);
    // 振动只统计、不丢弃采样：振动是零均值的，直接对窗口求平均就是零偏估计。
    // 之前这里一旦超阈值就清空累加重来 —— 云台一开始闭环微调就永远凑不满
    // 采样数，零偏没扣掉，-8 dps 积分 10 秒漂 80°，姿态直接发散（实测踩过）。
    if (rate_mag > kCalMaxRateDps) {
      ++cal_rejected_;
    }

    cal_sum_x_ += gx;
    cal_sum_y_ += gy;
    cal_sum_z_ += gz;
    // 同时累积"加速度角"，用于记录零姿态（板子未必水平 + 模块自身零偏）
    cal_sum_roll_ += std::atan2(ay, az) * kRadToDeg;
    cal_sum_pitch_ += std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg;
    ++cal_count_;

    // 超时强制完成：如果因为持续振动一直凑不满目标帧数，
    // 累计到 1500 帧（7.5 秒）且已有 20 帧以上，就用现有数据完成校准。
    // 不加这个保护的话，稍微有点振动就会"永远在校准中"，角度完全不更新（踩过）。
    const bool timeout_ok = (cal_total_frames_ >= 200 && cal_count_ >= 20);

    if (cal_count_ >= cal_target_ || timeout_ok) {
      const float inv = 1.0f / static_cast<float>(cal_count_);
      bias_.x = cal_sum_x_ * inv;
      bias_.y = cal_sum_y_ * inv;
      bias_.z = cal_sum_z_ * inv;
      offset_roll_ = cal_sum_roll_ * inv;
      offset_pitch_ = cal_sum_pitch_ * inv;
      cal_state_ = CalState::Done;
      reset();
    }
    return common::Err::Busy;   // 校准期间不给角度
  }

  // 存下修正后的角速度供控制器使用（注意 flip 已经在上面处理过了）
  rate_.x = gx - bias_.x;
  rate_.y = gy - bias_.y;
  rate_.z = gz - bias_.z;

  // ---------- 去零偏 + 死区 ----------
  const float gx_d = deadband(gx - bias_.x, deadband_dps_);
  const float gy_d = deadband(gy - bias_.y, deadband_dps_);
  const float gz_d = deadband(gz - bias_.z, deadband_dps_);

  // ---------- 加速度计角度（扣除"零姿态"偏移）----------
  // 不扣的话：板子放得不平、模块自身有零偏，平放时 pitch 会是 8° 之类的假值
  last_roll_raw_ = std::atan2(ay, az) * kRadToDeg;
  const float roll_acc = std::atan2(ay, az) * kRadToDeg - offset_roll_;
  last_pitch_raw_ = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg;
  const float pitch_acc =
      std::atan2(-ax, std::sqrt(ay * ay + az * az)) * kRadToDeg - offset_pitch_;

  // 仅用加速度计模式：不做陀螺积分，只对加速度角做一阶低通。
  // 好处是永远不漂移、不需要陀螺零偏；代价是动态响应稍慢、对线加速度敏感。
  if (accel_only_) {
    const float k = 0.25f;   // 低通系数：越小越平滑、越滞后（100 Hz 下约 40 ms 时间常数）
    euler_.pitch += k * (pitch_acc - euler_.pitch);
    euler_.roll += k * (roll_acc - euler_.roll);
    // 加速计无法测 yaw，保持为 0（无磁力计时 yaw 本来就没有绝对基准）
    euler_.yaw = 0.0f;
    rate_.x = rate_.y = rate_.z = 0.0f;
    angle_rate_.x = angle_rate_.y = angle_rate_.z = 0.0f;
    has_prev_ = false;   // 切回互补滤波时不要用旧差分
    return common::Err::Ok;
  }


  // ---------- 互补滤波 ----------
  euler_.roll = alpha_ * (euler_.roll + gx_d * dt_s) + (1.0f - alpha_) * roll_acc;
  euler_.pitch = alpha_ * (euler_.pitch + gy_d * dt_s) + (1.0f - alpha_) * pitch_acc;
  euler_.yaw += gz_d * dt_s;   // 无磁力计，只能积分（会缓慢漂移）

  // 欧拉角微分：给控制器做阻尼用，符号必然和角度一致
  if (has_prev_ && dt_s > 0.0f) {
    angle_rate_.z = (euler_.yaw - prev_euler_.yaw) / dt_s;   // z 存 yaw 微分
    angle_rate_.y = (euler_.pitch - prev_euler_.pitch) / dt_s; // y 存 pitch 微分
    angle_rate_.x = (euler_.roll - prev_euler_.roll) / dt_s;  // x 存 roll 微分
  }
  prev_euler_ = euler_;
  has_prev_ = true;

  return common::Err::Ok;
}

}  // namespace module
