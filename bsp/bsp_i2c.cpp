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
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(kBus, dev << kAddrShift, 3, timeout_ms);

    // (comment cleaned)
    if (st == HAL_BUSY) {
      I2C1->CR1 |= I2C_CR1_SWRST;
      for (volatile int d = 0; d < 100; ++d) {}
      I2C1->CR1 &= ~I2C_CR1_SWRST;
      for (volatile int d = 0; d < 100; ++d) {}
      (void)HAL_I2C_Init(kBus);
      st = HAL_I2C_IsDeviceReady(kBus, dev << kAddrShift, 3, timeout_ms);
    }

    if (st == HAL_OK) return common::Err::Ok;
    if (st == HAL_BUSY) return common::Err::Busy;
    return common::Err::NoAck;
  }

  /**
   * @brief 总线恢复：把卡死的 I2C 从"永远 BUSY"里救出来
   *
   * 为什么原来的实现不够：
   *   F103 的 I2C 外设只有在总线上检测到 **STOP 条件** 时才会清 BUSY 位。
   *   只打 9 个 SCL 时钟能让从机吐完字节，但**不产生 STOP**，
   *   于是外设永远认为自己还在传输中 —— 之后每次调用都返回 HAL_BUSY。
   *   实测：把 SCL/SDA 从模块上完全拔掉，BUSY 依然是 1，就是证据。
   *
   * 正确的四步：
   *   1) 把两根线切成普通 GPIO 开漏，先都释放为高
   *   2) 打 9 个 SCL 时钟，让可能卡住的从机把字节吐完
   *   3) **手动生成 STOP 条件**（SCL 高 → SDA 由低变高）
   *   4) 外设软复位（CR1.SWRST）+ 重新 Init
   */
  common::Err recover_bus() override
  {
    LOG_W("i2c: bus recovery start");
    HAL_I2C_DeInit(kBus);

    // 1) 两根线都切成开漏输出，先释放为高电平（开漏 + 外部上拉 = 高）
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    for (volatile int d = 0; d < 500; ++d) {}

    // 2) 9 个 SCL 时钟：让卡在"等时钟"状态的从机把剩余字节吐完
    for (int i = 0; i < 9; ++i) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      for (volatile int d = 0; d < 300; ++d) {}
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
      for (volatile int d = 0; d < 300; ++d) {}
    }

    // 3) 手动生成 STOP：SCL 保持高，SDA 由低变高
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile int d = 0; d < 300; ++d) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    for (volatile int d = 0; d < 300; ++d) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile int d = 0; d < 300; ++d) {}
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // ← 这一下产生 STOP
    for (volatile int d = 0; d < 500; ++d) {}

    // 4) 外设软复位：清掉可能残留的 BUSY/状态机
    I2C1->CR1 |= I2C_CR1_SWRST;
    for (volatile int d = 0; d < 100; ++d) {}
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    for (volatile int d = 0; d < 100; ++d) {}

    const auto st = HAL_I2C_Init(kBus);

    // 5) 引脚交回 I2C 外设（复用开漏）
    gpio.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(GPIOB, &gpio);

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

/**
 // (comment cleaned)
 *
 // (comment cleaned)
 // (comment cleaned)
 */
common::Err i2c1_set_speed(std::uint32_t hz)
{
  if (hz < 10000 || hz > 400000) return common::Err::InvalidParam;
  HAL_I2C_DeInit(&hi2c1);
  hi2c1.Init.ClockSpeed = hz;
  const auto st = HAL_I2C_Init(&hi2c1);
  LOG_I("i2c: clock -> %lu Hz (%s)", (unsigned long)hz, (st == HAL_OK) ? "ok" : "FAILED");
  return (st == HAL_OK) ? common::Err::Ok : common::Err::BusError;
}

/**
 // (comment cleaned)
 *
 // (comment cleaned)
 // (comment cleaned)
 // (comment cleaned)
 // (comment cleaned)
 // (comment cleaned)
 */
void i2c1_enable_internal_pullup()
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
}

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
