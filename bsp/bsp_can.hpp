/**
 * @file    bsp_can.hpp
 * @brief   CAN1 板级实现的对外入口
 */
#ifndef BSP_CAN_IMPL_HPP
#define BSP_CAN_IMPL_HPP

#include "can.hpp"

namespace bsp {

/// 全局唯一的 CAN1 实例（对应 CubeMX 生成的 hcan）
extern Can& can1;

/// 由 HAL 回调转调（内部使用）
void can1_on_rx_pending();
void can1_on_error();

}  // namespace bsp

#endif  // BSP_CAN_IMPL_HPP
