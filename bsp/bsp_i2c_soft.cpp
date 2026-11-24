/**
 * @file    bsp_i2c_soft.cpp
 * @brief   软件模拟 I2C 实现（PB6=SCL / PB7=SDA，开漏 + 外部/内部上拉）
 *
 * 时序：每次电平变化后延时约 5 us，等效 SCL 约 100 kHz（比硬件 I2C 的
 * 100 kHz 略慢一点，但对干扰的容忍度高得多）。
 */

#include "bsp_i2c_soft.hpp"

#include <cstdint>

namespace bsp {

namespace {

constexpr std::uint32_t kGpioBBase = 0x40010C00u;
constexpr std::uint32_t kCrl = kGpioBBase + 0x00u;
constexpr std::uint32_t kCrh = kGpioBBase + 0x04u;
constexpr std::uint32_t kIdr = kGpioBBase + 0x08u;
constexpr std::uint32_t kBsrr = kGpioBBase + 0x10u;
constexpr std::uint32_t kBrr = kGpioBBase + 0x14u;
constexpr std::uint32_t kRccApb2Enr = 0x40021018u;
constexpr std::uint32_t kRccIopb = 1u << 3;

constexpr std::uint32_t kSclBit = 6u;   // PB6
constexpr std::uint32_t kSdaBit = 7u;   // PB7

inline void reg_write(std::uint32_t addr, std::uint32_t v) {
  *reinterpret_cast<volatile std::uint32_t*>(addr) = v;
}
inline std::uint32_t reg_read(std::uint32_t addr) {
  return *reinterpret_cast<volatile std::uint32_t*>(addr);
}

}  // namespace

std::uint32_t SoftI2c::fail_count_ = 0;

void SoftI2c::delay() {
  // 72 MHz 下大约 4 us。volatile 防止被优化掉。
  for (volatile int i = 0; i < 40; ++i) {
  }
}

void SoftI2c::scl_high() { reg_write(kBsrr, 1u << kSclBit); }
void SoftI2c::scl_low() { reg_write(kBrr, 1u << kSclBit); }
void SoftI2c::sda_high() { reg_write(kBsrr, 1u << kSdaBit); }
void SoftI2c::sda_low() { reg_write(kBrr, 1u << kSdaBit); }
bool SoftI2c::sda_read() { return (reg_read(kIdr) & (1u << kSdaBit)) != 0; }

void SoftI2c::init_pins() {
  // 使能 GPIOB 时钟（APB2），否则对 CRL/BSRR 的写全部无效
  reg_write(kRccApb2Enr, reg_read(kRccApb2Enr) | kRccIopb);
  // CRH 管 PB8..PB15，但 PB6/PB7 在 CRL 里（PB0..PB7）
  // CRL: 每引脚 4 位，PB6 -> bit[27:24]，PB7 -> bit[31:28]
  // MODE=10 (2 MHz 输出)，CNF=01 (开漏输出) → 0b0110 = 0x6
  std::uint32_t crl = reg_read(kCrl);
  crl &= ~(0xFu << 24);   // 清 PB6
  crl |= (0x6u << 24);
  crl &= ~(0xFu << 28);   // 清 PB7
  crl |= (0x6u << 28);
  reg_write(kCrl, crl);

  scl_high();
  sda_high();
  delay();
}

void SoftI2c::start() {
  sda_high();
  scl_high();
  delay();
  sda_low();
  delay();
  scl_low();
  delay();
}

void SoftI2c::stop() {
  sda_low();
  delay();
  scl_high();
  delay();
  sda_high();
  delay();
}

bool SoftI2c::write_byte(std::uint8_t b) {
  for (int i = 7; i >= 0; --i) {
    if ((b >> i) & 1u) {
      sda_high();
    } else {
      sda_low();
    }
    delay();
    scl_high();
    delay();
    scl_low();
    delay();
  }
  // 第 9 个时钟读 ACK
  sda_high();   // 释放 SDA
  delay();
  scl_high();
  delay();
  const bool ack = !sda_read();   // 低电平 = ACK
  scl_low();
  delay();
  return ack;
}

std::uint8_t SoftI2c::read_byte(bool ack) {
  std::uint8_t b = 0;
  sda_high();   // 释放，让从机驱动
  delay();
  for (int i = 0; i < 8; ++i) {
    scl_high();
    delay();
    b = static_cast<std::uint8_t>((b << 1) | (sda_read() ? 1u : 0u));
    scl_low();
    delay();
  }
  // 主机应答位
  if (ack) {
    sda_low();
  } else {
    sda_high();
  }
  delay();
  scl_high();
  delay();
  scl_low();
  sda_high();
  delay();
  return b;
}

common::Err SoftI2c::write_reg(std::uint8_t dev, std::uint8_t reg,
                               const std::uint8_t* data, std::size_t len,
                               std::uint32_t) {
  start();
  if (!write_byte(static_cast<std::uint8_t>(dev << 1))) {
    stop();
    ++fail_count_;
    return common::Err::BusError;
  }
  if (!write_byte(reg)) {
    stop();
    ++fail_count_;
    return common::Err::BusError;
  }
  for (std::size_t i = 0; i < len; ++i) {
    if (!write_byte(data[i])) {
      stop();
      ++fail_count_;
      return common::Err::BusError;
    }
  }
  stop();
  return common::Err::Ok;
}

common::Err SoftI2c::read_reg(std::uint8_t dev, std::uint8_t reg,
                              std::uint8_t* data, std::size_t len,
                              std::uint32_t) {
  start();
  if (!write_byte(static_cast<std::uint8_t>(dev << 1))) {
    stop();
    ++fail_count_;
    return common::Err::BusError;
  }
  if (!write_byte(reg)) {
    stop();
    ++fail_count_;
    return common::Err::BusError;
  }
  // 重复起始 + 读
  start();
  if (!write_byte(static_cast<std::uint8_t>((dev << 1) | 1u))) {
    stop();
    ++fail_count_;
    return common::Err::BusError;
  }
  for (std::size_t i = 0; i < len; ++i) {
    data[i] = read_byte(i + 1 < len);   // 最后一字节回 NACK
  }
  stop();
  return common::Err::Ok;
}

common::Err SoftI2c::read_reg_async(std::uint8_t dev, std::uint8_t reg,
                                    std::uint8_t* data, std::size_t len) {
  // bit-bang 没有异步模式，直接同步做完（调用方之后 busy() 恒为 false）
  return read_reg(dev, reg, data, len, 10);
}

common::Err SoftI2c::probe(std::uint8_t dev, std::uint32_t) {
  start();
  const bool ack = write_byte(static_cast<std::uint8_t>(dev << 1));
  stop();
  if (!ack) {
    ++fail_count_;
    return common::Err::NoAck;
  }
  return common::Err::Ok;
}

common::Err SoftI2c::recover_bus() {
  // SDA 被从机拉低时，补最多 9 个时钟让它把字节发完，然后补 STOP
  sda_high();
  for (int i = 0; i < 9; ++i) {
    scl_low();
    delay();
    scl_high();
    delay();
  }
  stop();
  sda_high();
  scl_high();
  delay();
  return common::Err::Ok;
}

namespace {
SoftI2c g_soft_i2c;   // 静态对象：不用动态分配（静态初始化期 new 会崩）
}  // namespace
SoftI2c& soft_i2c = g_soft_i2c;

}  // namespace bsp
