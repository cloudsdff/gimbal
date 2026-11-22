/**
 * @file    mpu6050.hpp
 * @brief   MPU6050 六轴传感器模块（module 层，只依赖 bsp::I2c 接口）
 *
 * 这里不允许出现 HAL_：数据怎么搬运是 bsp 的事，模块只关心寄存器语义。
 */
#ifndef MODULE_MPU6050_HPP
#define MODULE_MPU6050_HPP

#include <cstdint>

#include "error_code.hpp"
#include "i2c.hpp"

namespace module {

class Mpu6050 {
 public:
  /// 寄存器地址（只列用到的）
  enum Reg : std::uint8_t {
    kRegSmplrtDiv = 0x19,
    kRegConfig = 0x1A,
    kRegGyroConfig = 0x1B,
    kRegAccelConfig = 0x1C,
    kRegAccelXoutH = 0x3B,
    kRegPwrMgmt1 = 0x6B,
    kRegWhoAmI = 0x75,
  };

  static constexpr std::uint8_t kAddrDefault = 0x68;   // AD0 接 GND
  static constexpr std::uint8_t kWhoAmIValue = 0x68;

  /// 六轴原始值 + 片上温度（全是 ADC 码，未换算成物理量）
  struct Raw {
    std::int16_t ax, ay, az;
    std::int16_t temp;
    std::int16_t gx, gy, gz;
  };

  explicit Mpu6050(bsp::I2c& bus, std::uint8_t addr = kAddrDefault)
      : bus_(bus), addr_(addr) {}

  /// 探测 + 唤醒 + 设量程。失败返回具体 Err，不静默
  common::Err init();

  /// 阻塞方式读一帧
  common::Err read_raw(Raw& out);

  /// 中断方式：发起后立即返回，用 busy() 查询、last_result() 取结果
  common::Err read_raw_async(Raw& out);

  bool busy() const { return bus_.busy(); }
  bool inited() const { return inited_; }

 private:
  common::Err read_regs(std::uint8_t reg, std::uint8_t* buf, std::size_t len,
                        std::uint32_t timeout_ms);
  common::Err write_reg(std::uint8_t reg, std::uint8_t val, std::uint32_t timeout_ms);

  bsp::I2c& bus_;
  std::uint8_t addr_;
  bool inited_ = false;
  Raw raw_{};        // 异步读的落地缓冲（回调直接写这里）
};

}  // namespace module

#endif  // MODULE_MPU6050_HPP
