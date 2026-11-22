/**
 * @file    bsp_i2c.cpp
 * @brief   I2C1 板级实现（PB6 = SCL, PB7 = SDA）
 *
 * 工程要求兑现：
 *   要求 1：HAL 调用全部关在这个文件里
 *   要求 3：阻塞 + 中断两种读方式
 *   要求 4：任何失败都返回 Err，不 while(1)
 *
 * 特别处理：F103 的 I2C1 有一个著名的 errata —— 从机在传输中途被拔线时
 * 会把 SDA 一直拉低，HAL 从此永远返回 BUSY。做法是手动打 9 个 SCL 时钟
 * 让从机把字节吐完，再重新 Init。
 */
#include "bsp_i2c.hpp"

#include "main.h"
#include "log.hpp"

extern "C" {
extern I2C_HandleTypeDef hi2c1;
}

namespace bsp {
namespace {

I2C_HandleTypeDef* const kBus = &hi2c1;

volatile bool g_async_busy = false;
volatile int g_async_status = 0;          // 0 = OK，1 = 出错

constexpr std::uint16_t kAddrShift = 1;   // HAL 要 8 位地址

common::Err to_err(HAL_StatusTypeDef st)
{
  switch (st) {
    case HAL_OK:      return common::Err::Ok;
    case HAL_BUSY:    return common::Err::Busy;
    case HAL_TIMEOUT: return common::Err::Timeout;
    default:          return common::Err::BusError;
  }
}

}  // namespace

class I2c1 final : public I2c {
 public:
  common::Err write_reg(std::uint8_t dev, std::uint8_t reg,
                        const std::uint8_t* data, std::size_t len,
                        std::uint32_t timeout_ms) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    const auto st = HAL_I2C_Mem_Write(kBus, dev << kAddrShift, reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      const_cast<std::uint8_t*>(data),
                                      static_cast<std::uint16_t>(len), timeout_ms);
    return to_err(st);
  }

  common::Err read_reg(std::uint8_t dev, std::uint8_t reg, std::uint8_t* data,
                       std::size_t len, std::uint32_t timeout_ms) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    if (g_async_busy) return common::Err::Busy;

    const auto st = HAL_I2C_Mem_Read(kBus, dev << kAddrShift, reg,
                                     I2C_MEMADD_SIZE_8BIT, data,
                                     static_cast<std::uint16_t>(len), timeout_ms);
    return to_err(st);
  }

  common::Err read_reg_async(std::uint8_t dev, std::uint8_t reg,
                             std::uint8_t* data, std::size_t len) override
  {
    if (data == nullptr || len == 0) return common::Err::InvalidParam;
    if (g_async_busy) return common::Err::Busy;

    g_async_busy = true;
    g_async_status = 0;
    const auto st = HAL_I2C_Mem_Read_IT(kBus, dev << kAddrShift, reg,
                                        I2C_MEMADD_SIZE_8BIT, data,
                                        static_cast<std::uint16_t>(len));
    if (st != HAL_OK) {
      g_async_busy = false;
      return to_err(st);
    }
    return common::Err::Ok;
  }

  bool busy() const override { return g_async_busy; }

  common::Err probe(std::uint8_t dev, std::uint32_t timeout_ms) override
  {
    const auto st = HAL_I2C_IsDeviceReady(kBus, dev << kAddrShift, 3, timeout_ms);
    if (st == HAL_OK) return common::Err::Ok;
    if (st == HAL_BUSY) return common::Err::Busy;
    return common::Err::NoAck;
  }

  common::Err recover_bus() override
  {
    LOG_W("i2c: bus recovery start");
    HAL_I2C_DeInit(kBus);

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    for (int i = 0; i < 9; ++i) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
      for (volatile int d = 0; d < 200; ++d) {}
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      for (volatile int d = 0; d < 200; ++d) {}
    }

    const auto st = HAL_I2C_Init(kBus);
    LOG_W("i2c: recover done, re-init %s", (st == HAL_OK) ? "Ok" : "FAILED");
    return (st == HAL_OK) ? common::Err::Ok : common::Err::BusError;
  }

  void set_async_status(int s) { g_async_status = s; g_async_busy = false; }
};

namespace {
I2c1 g_i2c1;
}  // namespace

I2c& i2c1 = g_i2c1;

void i2c1_on_async_done(int status) { g_i2c1.set_async_status(status); }

}  // namespace bsp

/* -------------------- HAL 回调（覆盖弱符号） -------------------- */

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c == &hi2c1) bsp::i2c1_on_async_done(0);
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c)
{
  if (hi2c == &hi2c1) bsp::i2c1_on_async_done(1);
}
