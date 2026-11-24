/**
 * @file    gimbal.cpp
 * @brief   二轴云台串级控制器实现
 *
 * 单个轴的算法（每 5 ms 步）：
 *
 *   e_ang    = target - angle                     角度误差
 *   |e_ang| < deadband → 输出保持（避免舵机抖）
 *   rate_ref = kp * e_ang                         角度环 P → 角速度参考
 *   rate_ref = clamp(rate_ref, ±rate_ref_max)
 *   e_rate   = rate_ref - rate                    角速度误差
 *   integ   += ki * e_ang * dt                    积分（用角度误差，保证稳态无差）
 *   integ    = clamp(integ, ±out_max)
 *   out      = kd * e_rate + integ                角速度环
 *   out      = clamp(out, ±out_max_deg)
 *   servo    = center + out
 *
 * 说明：这里内环用 kd 直接乘角速度误差（等价于 PD 的 D 项），
 * 没有单独再串一个 PI 内环 —— 对 SG90 这种带宽几 Hz 的舵机，
 * 再快的内环也跟不上，反而容易振荡。工程上够用且稳。
 */
#include "gimbal.hpp"

#include "log.hpp"

namespace module {
namespace {

/// 积分泄漏系数：误差为 0 时每步乘一次，200 Hz 下约 1 秒衰减到 2%
constexpr float kIntegLeak = 0.98f;

inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

}  // namespace

common::Err Gimbal::init(float center_deg)
{
  if (inited_) return common::Err::Busy;

  center_deg_ = center_deg;
  for (int i = 0; i < 2; ++i) {
    ax_[i] = AxisState{};
    target_[i] = 0.0f;
    out_[i] = 0.0f;
  }

  common::Err e = yaw_.init(center_deg_);
  if (e != common::Err::Ok) return e;
  e = pitch_.init(center_deg_);
  if (e != common::Err::Ok) return e;

  // 云台上电缓启动：先慢后快，避免上电瞬间甩头
  yaw_.set_slew_rate(60.0f);
  pitch_.set_slew_rate(60.0f);

  inited_ = true;
  // 整数打印：newlib-nano 的 printf 没有浮点支持
  LOG_I("gimbal init: center=%d(0.1deg) kp=%d kd=%d ki=%d (x100/x100/x1000)",
        (int)(center_deg_ * 10.0f), (int)(gains_.kp * 100.0f), (int)(gains_.kd * 100.0f),
        (int)(gains_.ki * 1000.0f));
  return common::Err::Ok;
}

void Gimbal::set_target(float yaw_deg, float pitch_deg)
{
  target_[0] = yaw_deg;
  target_[1] = pitch_deg;
}

common::Err Gimbal::update(const Euler& att, const Rates& rates, float dt_s)
{
  if (!inited_) return common::Err::NotInit;
  if (!enabled_) return common::Err::Ok;
  if (dt_s <= 0.0f) dt_s = 0.005f;

  // yaw 轴：用 yaw 角度 + gz 角速度；pitch 轴：用 pitch 角度 + gy 角速度
  (void)step_axis(ax_[0], target_[0], att.yaw, rates.yaw, dt_s, 0.0f);
  (void)step_axis(ax_[1], target_[1], att.pitch, rates.pitch, dt_s, gains_.ff_pitch);

  out_[0] = ax_[0].out;
  out_[1] = ax_[1].out;

  // 输出到舵机：中位 + 偏置（乘方向系数，装配后可在线取反）
  (void)yaw_.set_target(center_deg_ + sign_[0] * out_[0]);
  (void)pitch_.set_target(center_deg_ + sign_[1] * out_[1]);

  // 推动斜率限制（缓启动就在这里生效）
  (void)yaw_.update(dt_s);
  (void)pitch_.update(dt_s);

  return common::Err::Ok;
}

common::Err Gimbal::step_axis(AxisState& st, float target, float angle, float rate,
                              float dt, float ff)
{
  float e_ang = target - angle;

  // 死区：误差很小时把误差当作 0（而不是直接返回），
  // 否则输出会被"锁死"在上一次的值 —— 实测过：目标回 0 后舵机停在偏置位不动
  if (e_ang < gains_.deadband_deg && e_ang > -gains_.deadband_deg) {
    e_ang = 0.0f;
  }

  // 角度环 P → 角速度参考
  float rate_ref = gains_.kp * e_ang;
  rate_ref = clampf(rate_ref, -gains_.rate_ref_max, gains_.rate_ref_max);

  // 角速度误差（内环）
  const float e_rate = rate_ref - rate;

  // 内环比例输出
  const float out_p = gains_.kd * e_rate;

  // 抗积分饱和（conditional integration）：
  // 只有"输出还没顶到限幅"时才积分。
  // 不加这一条的话，舵机被机械限位挡住 / 负载卡住时，
  // 误差一直存在 → 积分一直加 → 一旦松开就猛冲过去（实测：积分顶到 35° 上限，
  // 云台回正后舵机仍停在偏 34° 的位置不肯回来）。
  const float pre = out_p + st.integ;
  if (pre < gains_.out_max_deg && pre > -gains_.out_max_deg) {
    st.integ += gains_.ki * e_ang * dt;
    st.integ = clampf(st.integ, -gains_.out_max_deg, gains_.out_max_deg);
  }

  // 积分泄漏：误差为 0 时让积分缓慢回零，
  // 否则目标回到 0 后舵机会永远停在"上一次的偏置"上（实测：停在偏 3° 处不下来）
  if (e_ang == 0.0f) {
    st.integ *= kIntegLeak;
  }

  // 内环输出
  float out = out_p + st.integ + ff;   // ff = 重力前馈
  out = clampf(out, -gains_.out_max_deg, gains_.out_max_deg);
  st.out = out;
  return common::Err::Ok;
}

common::Err Gimbal::stop()
{
  enabled_ = false;
  for (int i = 0; i < 2; ++i) {
    ax_[i] = AxisState{};
  }
  (void)yaw_.set_now(center_deg_);
  (void)pitch_.set_now(center_deg_);
  LOG_W("gimbal stopped, outputs to center");
  return common::Err::Ok;
}

}  // namespace module
