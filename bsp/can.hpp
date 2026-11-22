/**
 * @file    can.hpp
 * @brief   CAN 抽象接口（任务01 要求 1：上层不调 HAL；要求 3：阻塞发送 + 中断接收）
 *
 * 为什么 CAN 只有两种方式（没有 DMA）：
 *   bxCAN 的发送硬件本身就是一个 3 级邮箱，HAL_CAN_AddTxMessage() 把报文丢进邮箱
 *   就返回了，CPU 不参与搬运，所以"用 DMA 发 CAN"在硬件上没有对应机制。
 *   接收侧则靠 FIFO0 中断，这是 CAN 的标准做法。
 */
#ifndef BSP_CAN_HPP
#define BSP_CAN_HPP

#include <cstddef>
#include <cstdint>

#include "error_code.hpp"

namespace bsp {

/// 一帧标准 CAN 数据帧（ID 11 位，0–8 字节）
struct CanFrame {
  std::uint32_t id;
  std::uint8_t len;
  std::uint8_t data[8];
};

class Can {
 public:
  virtual ~Can() = default;

  /// 启动控制器并打开 FIFO0 接收中断
  /// @param accept_std_id 只接收这个标准 ID（11 位）的帧；传 0xFFFFFFFF 表示全通
  ///        —— 单节点自测时模块可能把 TX 回环到 RX，硬件过滤可以把"自己发的"挡掉
  virtual common::Err start(std::uint32_t accept_std_id = 0xFFFFFFFFu) = 0;

  /// 阻塞发送：等邮箱空出来最多 timeout_ms
  /// @retval Ok / Timeout（邮箱满了）/ BusError / NotInit
  virtual common::Err send(const CanFrame& f, std::uint32_t timeout_ms) = 0;

  /// 从接收队列取一帧（由中断负责入队）
  /// @retval Ok / Timeout（队列空）
  virtual common::Err recv(CanFrame& f, std::uint32_t timeout_ms) = 0;

  /// 计数与诊断
  virtual std::uint32_t tx_count() const = 0;
  virtual std::uint32_t rx_count() const = 0;
  virtual std::uint32_t rx_dropped() const = 0;   ///< 队列满时丢掉的帧
  virtual std::uint32_t error_count() const = 0;  ///< 出错中断次数
  virtual std::uint32_t busoff_count() const = 0; ///< 进入 bus-off 的次数

  /// bus-off 后控制器会自己停摆，必须重新 Start 才能恢复
  virtual bool need_recovery() const = 0;
  virtual common::Err recover() = 0;
};

}  // namespace bsp

#endif  // BSP_CAN_HPP
