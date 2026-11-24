/**
 * @file    bsp_i2c_soft.hpp
 * @brief   软件模拟 I2C（bit-bang），PB6=SCL / PB7=SDA
 *
 * 为什么要这个：硬件 I2C1 在舵机动作（电流突变 + PWM 干扰）后，
 * 外设状态机会卡死在 BUSY，连从机线拔掉都恢复不了（实测过）。
 * bit-bang 直接控制引脚电平，没有"外设卡死"这个状态，
 * 读失败可以无限次重试，代价只是占一点 CPU。
 */
#ifndef BSP_I2C_SOFT_HPP
#define BSP_I2C_SOFT_HPP

#include "i2c.hpp"

namespace bsp {

class SoftI2c final : public I2c {
 public:
  /// 初始化 PB6/PB7 为开漏输出
  static void init_pins();

  common::Err write_reg(std::uint8_t dev, std::uint8_t reg,
                        const std::uint8_t* data, std::size_t len,
                        std::uint32_t timeout_ms) override;

  common::Err read_reg(std::uint8_t dev, std::uint8_t reg,
                       std::uint8_t* data, std::size_t len,
                       std::uint32_t timeout_ms) override;

  common::Err read_reg_async(std::uint8_t dev, std::uint8_t reg,
                             std::uint8_t* data, std::size_t len) override;

  bool busy() const override { return false; }

  common::Err probe(std::uint8_t dev, std::uint32_t timeout_ms) override;

  /// bit-bang 下"恢复"就是补 9 个时钟 + STOP，可以随便调
  common::Err recover_bus() override;

  /// 累计失败次数（调试用）
  static std::uint32_t fail_count() { return fail_count_; }

 private:
  static void scl_high();
  static void scl_low();
  static void sda_high();
  static void sda_low();
  static bool sda_read();
  static void delay();
  static void start();
  static void stop();
  static bool write_byte(std::uint8_t b);
  static std::uint8_t read_byte(bool ack);

  static std::uint32_t fail_count_;
};

/// 全局唯一的软件 I2C 实例
extern SoftI2c& soft_i2c;

}  // namespace bsp

#endif  // BSP_I2C_SOFT_HPP
