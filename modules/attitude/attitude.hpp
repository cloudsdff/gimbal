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

  /// 角度的微分（dps），由滤波后的欧拉角差分得到。
  ///
  /// 为什么不用陀螺：陀螺在"翻面补偿"后符号和欧拉角不一致，
  /// 用它做阻尼会变成正反馈（实测：kd 越大越振荡，峰峰 50°+）。
  /// 直接微分欧拉角，符号必然和角度一致，怎么装都不会错。
  const GyroBias& angle_rate() const { return angle_rate_; }

  /// 修正后的角速度（已扣零偏、已按安装方向取反），单位 dps。
  /// 控制器必须用这个值，不能自己拿 raw 算 —— 否则和角度符号不一致，
  /// 阻尼项会变成正反馈（踩过：表现为"怎么调方向都振荡"）。
  const GyroBias& corrected_rates() const { return rate_; }

  /// 当前零偏（dps）
  const GyroBias& bias() const { return bias_; }

  /// 在线调整滤波加权系数（上位机下发指令用）
  void set_alpha(float a);
  float alpha() const { return alpha_; }

  /**
   * @brief 模块"翻面安装"补偿（绕 X 轴 180°）
   *
   * 现象：平台明明水平，roll 却读到 ±180° 附近 —— 说明模块的 +Z 轴朝下。
   * 数学上等价于把坐标系绕 X 轴转 180°：x 不变，y/z 取反（加速度与角速度都是）。
   * 有了这个开关就不用为了"翻个面"去拆胶带重装。
   */
  void set_mount_flip(bool flip)
  {
    flip_ = flip;
    reset();
  }
  bool mount_flip() const { return flip_; }

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
  // 默认开启：这块模块是倒装在上层平台上的（roll 读数接近 180°），
  // 之前用 F 指令临时打开，一重启就失效、控制方向反过来。
  // 装正之后把它改回 false 即可。
  bool flip_ = true;            ///< 模块翻面安装补偿（绕 X 轴 180°）
  Euler euler_{};
  GyroBias bias_{};
  GyroBias rate_{};   ///< 最近一帧修正后的角速度（dps）
  GyroBias angle_rate_{};   ///< 欧拉角微分（dps），供控制器做阻尼
  Euler prev_euler_{};      ///< 上一帧欧拉角
  bool has_prev_ = false;
  float deadband_dps_ = 0.3f;   ///< 静止死区，抑制零漂

  /// 加速度计"零姿态"偏移：上电时板子未必水平、模块本身也有零偏，
  /// 校准期间把两者的平均值记下来，之后从加速度角里减掉（否则平放时 pitch 不为 0）
  float offset_roll_ = 0.0f;
  float offset_pitch_ = 0.0f;

  /// 校准期间允许的最大角速度（dps）：超过说明板子在被碰，本帧作废
  static constexpr float kCalMaxRateDps = 20.0f;

  CalState cal_state_ = CalState::Idle;
  std::uint16_t cal_target_ = 0;
  std::uint16_t cal_count_ = 0;
  std::uint16_t cal_rejected_ = 0;   ///< 被拒的采样次数（板子动了）
  float cal_sum_x_ = 0.0f;
  float cal_sum_y_ = 0.0f;
  float cal_sum_z_ = 0.0f;
  float cal_sum_roll_ = 0.0f;
  float cal_sum_pitch_ = 0.0f;
};

}  // namespace module

#endif  // MODULE_ATTITUDE_HPP
