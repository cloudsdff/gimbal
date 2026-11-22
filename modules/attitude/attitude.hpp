/**
 * @file    attitude.hpp
 * @brief   姿态解算：互补滤波 + 上电自校准 + 零漂抑制
 *
 * 层级：module 层，只依赖 Mpu6050 的原始数据结构，不碰 HAL。
 *
 * 为什么用互补滤波（而不是 Mahony / EKF）：
 *   - F103 没有 FPU，浮点是软件模拟的。互补滤波每帧只有十几次乘加，
 *     200 Hz 下 CPU 占用可以忽略；Mahony 要做四元数微分和归一化，
 *     EKF 更是矩阵运算，在 F103 上会吃掉大部分算力。
 *   - 任务02 的合格线是"闭环带宽 ≥2 Hz、稳态误差 ≤5%"，
 *     互补滤波在静态和低频段的精度完全够用。
 *   - 后续若要更高动态性能，可以在 F407（有 FPU）上换成 Mahony。
 *
 * 三个功能：
 *   1. 上电自校准：静止采 N 帧，求陀螺零偏（gyro bias）
 *   2. 零漂抑制：减去零偏 + 小角速度死区，抑制静止时的积分漂移
 *   3. 融合：陀螺积分（高频好）+ 加速度计角度（低频好），用 alpha 加权
 */
#ifndef MODULE_ATTITUDE_HPP
#define MODULE_ATTITUDE_HPP

#include <cstdint>

#include "error_code.hpp"
#include "mpu6050.hpp"

namespace module {

/// 欧拉角（单位：度）
struct Euler {
  float yaw;
  float pitch;
  float roll;
};

/// 陀螺零偏（单位：dps，度每秒）
struct GyroBias {
  float x;
  float y;
  float z;
};

class Attitude {
 public:
  /// alpha 越大越信陀螺（动态好、易漂），越小越信加速度计（静态稳、受振动影响）
  /// 0.96–0.99 是常见范围；本工程默认 0.98
  explicit Attitude(float alpha = 0.98f) : alpha_(alpha) {}

  /// 复位所有状态（不清零偏，零偏要靠校准）
  void reset();

  /// 开始上电自校准：接下来 samples 帧当作静止数据
  /// @param samples 采样帧数，200 Hz 下 200 帧 = 1 秒
  void begin_calibration(std::uint16_t samples = 200);

  /// 是否还在校准中
  bool calibrating() const { return cal_state_ == CalState::Sampling; }

  /// 校准是否已完成过
  bool calibrated() const { return cal_state_ == CalState::Done; }

  /**
   * @brief 送入一帧原始数据，内部完成融合
   * @param raw   MPU6050 的原始六轴数据
   * @param dt_s  距上一帧的时间（秒）
   * @return Err::Busy  正在校准，本帧只累积不输出角度
   *         Err::Ok    角度已更新
   */
  common::Err update(const Mpu6050::Raw& raw, float dt_s);

  /// 当前姿态（度）
  const Euler& euler() const { return euler_; }

  /// 当前零偏（dps）
  const GyroBias& bias() const { return bias_; }

  /// 在线调整滤波加权系数（上位机下发指令用）
  void set_alpha(float a);
  float alpha() const { return alpha_; }

  /// 死区（dps）：小于它的角速度视为静止，用于抑制零漂
  void set_gyro_deadband(float dps) { deadband_dps_ = dps; }
  float gyro_deadband() const { return deadband_dps_; }

  // ---- 传感器标度（±2g / ±2000dps）----
  static constexpr float kAccelLsbPerG = 16384.0f;   ///< ±2g 量程
  static constexpr float kGyroLsbPerDps = 16.4f;     ///< ±2000dps 量程

 private:
  enum class CalState : std::uint8_t { Idle, Sampling, Done };

  float to_dps(std::int16_t v) const { return static_cast<float>(v) / kGyroLsbPerDps; }
  float to_g(std::int16_t v) const { return static_cast<float>(v) / kAccelLsbPerG; }

  float alpha_;
  Euler euler_{};
  GyroBias bias_{};
  float deadband_dps_ = 0.3f;   ///< 静止死区，抑制零漂

  /// 加速度计"零姿态"偏移：上电时板子未必水平、模块本身也有零偏，
  /// 校准期间把两者的平均值记下来，之后从加速度角里减掉（否则平放时 pitch 不为 0）
  float offset_roll_ = 0.0f;
  float offset_pitch_ = 0.0f;

  CalState cal_state_ = CalState::Idle;
  std::uint16_t cal_target_ = 0;
  std::uint16_t cal_count_ = 0;
  float cal_sum_x_ = 0.0f;
  float cal_sum_y_ = 0.0f;
  float cal_sum_z_ = 0.0f;
  float cal_sum_roll_ = 0.0f;
  float cal_sum_pitch_ = 0.0f;
};

}  // namespace module

#endif  // MODULE_ATTITUDE_HPP
