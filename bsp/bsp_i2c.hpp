/**
 * @file    bsp_i2c.hpp
 * @brief   I2C1 板级实现的对外入口
 */
#ifndef BSP_I2C_IMPL_HPP
#define BSP_I2C_IMPL_HPP

#include "i2c.hpp"

namespace bsp {

/// 全局唯一的 I2C1 实例（对应 CubeMX 生成的 hi2c1）
extern I2c& i2c1;

/// 由 HAL 回调转调（内部使用）：status 0 = 成功，1 = 出错
void i2c1_on_async_done(int status);

/// ?? PB6/PB7 ????(??/???)
void i2c1_enable_internal_pullup();

/// ?? I2C ??(?????? 50 kHz ?????)
common::Err i2c1_set_speed(std::uint32_t hz);

}  // namespace bsp

#endif  // BSP_I2C_IMPL_HPP
