/**
 * @file    gimbal.hpp
 * @brief   二轴云台控制器：串级（角度环 + 角速度环）
 *
 * 结构：
 *
 *   目标角度 ──→(Σ)──→ 角度环 P ──→ 角速度参考 ──→(Σ)──→ 角速度环 PI ──→ 限幅 ──→ 舵机角度
 *              ↑(-)                                   ↑(-)
 *           当前角度                                当前角速度
 *
 * 为什么串级：
 *   单环（只有角度 P）在受到外力扰动时，要等角度先偏了才动作，响应慢且容易振荡。
 *   内环把陀螺的角速度用上——**扰动刚发生、角度还没变时，角速度已经变了**，
 *   内环立刻反向出力，等效于增加了阻尼。这就是云台"抗扰"的来源。
 *
 * 两个环的参数物理意义：
 *   kp   [1/s]   角度误差 1° 产生多少 °/s 的角速度参考
 *   kd   [-]     角速度误差 1°/s 产生多少 ° 的舵机角增量（等效微分/阻尼）
 *   ki   [1/s²]  积分，消除稳态误差（重力不平衡、舵机中立点偏差）
 */
#ifndef MODULE_GIMBAL_HPP
#define MODULE_GIMBAL_HPP

#include <cstdint>

#include "attitude.hpp"
#include "error_code.hpp"
#include "servo.hpp"

namespace module {

/// 角速度（度/秒）
struct Rates {
  float yaw;
  float pitch;
  float roll;
};

class Gimbal {
 public:
  struct Gains {
    float kp = 2.0f;      ///< 角度环 P
    float kd = 0.6f;      ///< 角速度环 D（阻尼）
    float ki = 0.05f;     ///< 角速度环 I
    float rate_ref_max = 200.0f;   ///< 角速度参考限幅（°/s）
    float out_max_deg = 35.0f;     ///< 输出到舵机的最大角度偏置
    float deadband_deg = 0.5f;     ///< 角度死区，抑制抖动
  };

  Gimbal(Servo& yaw, Servo& pitch) : yaw_(yaw), pitch_(pitch) {}

  common::Err init(float center_deg = 90.0f);

  /// 设置目标指向（相对零姿态的角度，单位度）
  void set_target(float yaw_deg, float pitch_deg);
  void set_gains(const Gains& g) { gains_ = g; }

  /// ???:+1 ? -1??????????????(???),????????,
  /// ????????????????
  void set_sign(float yaw_sign, float pitch_sign)
  {
    sign_[0] = (yaw_sign < 0.0f) ? -1.0f : 1.0f;
    sign_[1] = (pitch_sign < 0.0f) ? -1.0f : 1.0f;
  }
  Gains gains() const { return gains_; }

  /// 200 Hz 调用：att 是当前姿态，rates 是陀螺角速度（dps）
  common::Err update(const Euler& att, const Rates& rates, float dt_s);

  /// 急停：输出回中位并停止更新
  common::Err stop();

  float out_yaw() const { return out_[0]; }
  float out_pitch() const { return out_[1]; }

 private:
  struct AxisState {
    float integ = 0.0f;
    float out = 0.0f;
  };

  common::Err step_axis(AxisState& st, float target, float angle, float rate, float dt);

  Servo& yaw_;
  Servo& pitch_;
  Gains gains_{};
  AxisState ax_[2]{};      ///< 0 = yaw, 1 = pitch
  float target_[2]{};      ///< 目标角度（相对）
  float out_[2]{};         ///< 当前输出偏置（度）
  float center_deg_ = 90.0f;
  float sign_[2] = {1.0f, 1.0f};   ///< axis direction (+1/-1)
  bool inited_ = false;
  bool enabled_ = true;
};

}  // namespace module

#endif  // MODULE_GIMBAL_HPP
