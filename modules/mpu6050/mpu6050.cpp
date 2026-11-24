/**
 * @file    mpu6050.cpp
 * @brief   MPU6050 初始化与数据读取
 *
 * 关键寄存器（数据手册 §4.1 / §6）：
 *   0x75 WHO_AM_I    固定 0x68，用来确认"这个地址上真的有芯片"
 *   0x6B PWR_MGMT_1  写 0x00 解除睡眠（默认是睡眠态！不写这一句读出来全是 0）
 *   0x1C ACCEL_CONFIG 0x00 = ±2g
 *   0x1B GYRO_CONFIG  0x18 = ±2000 dps
 *   0x3B ACCEL_XOUT_H 起 14 字节：加速度 6 + 温度 2 + 角速度 6
 */
#include "mpu6050.hpp"

#include <cstring>

#include "log.hpp"

namespace module {
namespace {

constexpr std::uint32_t kTimeoutMs = 10;   // 14 ?? @100kHz ??? 1.5 ms,10 ms ??

std::int16_t be16(const std::uint8_t* p)
{
  return static_cast<std::int16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

}  // namespace

common::Err Mpu6050::write_reg(std::uint8_t reg, std::uint8_t val,
                               std::uint32_t timeout_ms)
{
  return bus_.write_reg(addr_, reg, &val, 1, timeout_ms);
}

common::Err Mpu6050::read_regs(std::uint8_t reg, std::uint8_t* buf,
                               std::size_t len, std::uint32_t timeout_ms)
{
  return bus_.read_reg(addr_, reg, buf, len, timeout_ms);
}

common::Err Mpu6050::init()
{
  if (inited_) return common::Err::Busy;

  // 1) 先探测地址上有没有人应答
  common::Err e = bus_.probe(addr_, kTimeoutMs);
  if (e != common::Err::Ok) {
    LOG_E("mpu6050: probe failed (%s)", common::to_str(e));
    return e;
  }

  // 2) 核对芯片身份
  std::uint8_t who = 0;
  e = read_regs(kRegWhoAmI, &who, 1, kTimeoutMs);
  if (e != common::Err::Ok) {
    LOG_E("mpu6050: read WHO_AM_I failed (%s)", common::to_str(e));
    return e;
  }
  if (who != kWhoAmIValue) {
    LOG_E("mpu6050: WHO_AM_I=0x%02X, expect 0x68", who);
    return common::Err::NotFound;
  }
  LOG_I("mpu6050: WHO_AM_I=0x%02X ok", who);

  // 3) 唤醒 + 量程（顺序不重要，但唤醒必须在读数据之前）
  if ((e = write_reg(kRegPwrMgmt1, 0x00, kTimeoutMs)) != common::Err::Ok) {
    LOG_E("mpu6050: wake up failed (%s)", common::to_str(e));
    return e;
  }
  if ((e = write_reg(kRegAccelConfig, 0x00, kTimeoutMs)) != common::Err::Ok) {  // ±2g
    return e;
  }
  if ((e = write_reg(kRegGyroConfig, 0x18, kTimeoutMs)) != common::Err::Ok) {   // ±2000 dps
    return e;
  }

  inited_ = true;
  LOG_I("mpu6050: init done (accel +-2g, gyro +-2000dps)");
  return common::Err::Ok;
}

common::Err Mpu6050::read_raw(Raw& out)
{
  if (!inited_) return common::Err::NotInit;

  std::uint8_t buf[14] = {0};
  const common::Err e = read_regs(kRegAccelXoutH, buf, sizeof(buf), kTimeoutMs);
  if (e != common::Err::Ok) return e;

  out.ax = be16(&buf[0]);
  out.ay = be16(&buf[2]);
  out.az = be16(&buf[4]);
  out.temp = be16(&buf[6]);
  out.gx = be16(&buf[8]);
  out.gy = be16(&buf[10]);
  out.gz = be16(&buf[12]);
  return common::Err::Ok;
}

common::Err Mpu6050::read_raw_async(Raw& out)
{
  if (!inited_) return common::Err::NotInit;
  // 直接读进 raw_，完成后再解释。注意 out 和 raw_ 是同一块也可
  const common::Err e = bus_.read_reg_async(addr_, kRegAccelXoutH,
                                            reinterpret_cast<std::uint8_t*>(&raw_),
                                            sizeof(raw_));
  if (e != common::Err::Ok) return e;
  out = raw_;   // 数据在中断里落地，这里先把上一次的值给出去（调用方用 busy() 判断新鲜度）
  return common::Err::Ok;
}

}  // namespace module
