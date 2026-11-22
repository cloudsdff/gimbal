/**
 * @file    i2c.hpp
 * @brief   I2C 抽象接口（任务01 要求 1：上层不直接调 HAL；要求 3：≥2 种方式）
 *
 * 两种访问方式：
 *   read_reg()       阻塞式（初始化、低频读取）
 *   read_reg_async() 中断式（周期采样，发起后立即返回，由调用方轮询完成）
 *
 * 还有两个"工程化"接口：
 *   probe()       探测设备是否在线（对应 HAL_I2C_IsDeviceReady）
 *   recover_bus() 总线被从机拉死后的恢复流程（F103 I2C1 的经典 errata）
 */
#ifndef BSP_I2C_HPP
#define BSP_I2C_HPP

#include <cstddef>
#include <cstdint>

#include "error_code.hpp"

namespace bsp {

class I2c {
 public:
  virtual ~I2c() = default;

  /// 写寄存器（阻塞）
  virtual common::Err write_reg(std::uint8_t dev, std::uint8_t reg,
                                const std::uint8_t* data, std::size_t len,
                                std::uint32_t timeout_ms) = 0;

  /// 读寄存器（阻塞）
  virtual common::Err read_reg(std::uint8_t dev, std::uint8_t reg,
                               std::uint8_t* data, std::size_t len,
                               std::uint32_t timeout_ms) = 0;

  /// 读寄存器（中断方式）：发起后立刻返回，完成时 ok/err 被写回
  virtual common::Err read_reg_async(std::uint8_t dev, std::uint8_t reg,
                                     std::uint8_t* data, std::size_t len) = 0;

  /// 异步读是否还在进行
  virtual bool busy() const = 0;

  /// 探测设备（地址 7 位，函数内部左移）
  virtual common::Err probe(std::uint8_t dev, std::uint32_t timeout_ms) = 0;

  /// 总线恢复：从机把 SDA 拉低导致永久 BUSY 时，手动打 9 个时钟再重初始化
  virtual common::Err recover_bus() = 0;
};

}  // namespace bsp

#endif  // BSP_I2C_HPP
