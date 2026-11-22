/**
 * @file    bsp_can.cpp
 * @brief   CAN1 板级实现（PA11 = RX, PA12 = TX，500 kbps）
 *
 * 两种方式的分工（任务01 要求 3）：
 *   发送：HAL_CAN_AddTxMessage（阻塞式入邮箱）
 *   接收：FIFO0 挂起中断（HAL_CAN_RxFifo0MsgPendingCallback）
 *
 * 数据流：中断只负责把报文搬进环形缓冲，解析交给任务。
 */
#include "bsp_can.hpp"

#include <cstring>

#include "main.h"
#include "log.hpp"

extern "C" {
extern CAN_HandleTypeDef hcan;
}

namespace bsp {
namespace {

CAN_HandleTypeDef* const kCan = &hcan;

constexpr std::size_t kRxQueueSize = 16;   // 2 的幂，便于取模

CanFrame g_rx_queue[kRxQueueSize];
volatile std::uint32_t g_rx_head = 0;      // 中断写
volatile std::uint32_t g_rx_tail = 0;      // 任务读
volatile std::uint32_t g_tx_cnt = 0;
volatile std::uint32_t g_rx_cnt = 0;
volatile std::uint32_t g_rx_drop = 0;
volatile std::uint32_t g_err_cnt = 0;
volatile bool g_started = false;
volatile bool g_busoff = false;
volatile std::uint32_t g_busoff_cnt = 0;

}  // namespace

class Can1 final : public Can {
 public:
  common::Err start(std::uint32_t accept_std_id = 0xFFFFFFFFu) override
  {
    // 过滤器：32 位掩码模式，筛选器组 0，投递到 FIFO0
    // 若指定了 accept_std_id，则只放行该标准 ID（硬件挡掉其余帧）
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    if (accept_std_id == 0xFFFFFFFFu) {
      filter.FilterIdHigh = 0x0000;
      filter.FilterIdLow = 0x0000;
      filter.FilterMaskIdHigh = 0x0000;   // 掩码全 0 = 谁发都收
      filter.FilterMaskIdLow = 0x0000;
    } else {
      // 标准 ID 在 32 位过滤器里占 bit31..21，HAL 用高 16 位装 bit31..16
      const std::uint32_t stid = (accept_std_id & 0x7FFu) << 21;
      filter.FilterIdHigh = static_cast<std::uint32_t>((stid >> 16) & 0xFFFFu);
      filter.FilterIdLow = static_cast<std::uint32_t>(stid & 0xFFFFu);
      const std::uint32_t mask = 0x7FFu << 21;   // 只比较这 11 位
      filter.FilterMaskIdHigh = static_cast<std::uint32_t>((mask >> 16) & 0xFFFFu);
      filter.FilterMaskIdLow = static_cast<std::uint32_t>(mask & 0xFFFFu);
    }
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(kCan, &filter) != HAL_OK) {
      LOG_E("can: config filter failed");
      return common::Err::BusError;
    }
    const HAL_StatusTypeDef st = HAL_CAN_Start(kCan);
    if (st != HAL_OK) {
      LOG_E("can: start failed st=%d state=%d err=0x%08lX MSR=0x%08lX ESR=0x%08lX",
            (int)st, (int)kCan->State,
            (unsigned long)kCan->ErrorCode,
            (unsigned long)kCan->Instance->MSR,
            (unsigned long)kCan->Instance->ESR);
      return common::Err::BusError;
    }
    if (HAL_CAN_ActivateNotification(kCan, CAN_IT_RX_FIFO0_MSG_PENDING |
                                           CAN_IT_ERROR | CAN_IT_BUSOFF |
                                           CAN_IT_ERROR_PASSIVE |
                                           CAN_IT_ERROR_WARNING) != HAL_OK) {
      LOG_E("can: activate rx notification failed");
      return common::Err::BusError;
    }
    g_started = true;
    LOG_I("can: started (filter id=0x%03lX, FIFO0 irq on)",
          (unsigned long)accept_std_id);
    return common::Err::Ok;
  }

  common::Err send(const CanFrame& f, std::uint32_t timeout_ms) override
  {
    if (!g_started) return common::Err::NotInit;
    if (f.len > 8) return common::Err::InvalidParam;

    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = f.id & 0x7FFu;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = f.len;

    // 三个发送邮箱都满时等，最多 timeout_ms
    const std::uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(kCan) == 0) {
      if ((HAL_GetTick() - t0) > timeout_ms) return common::Err::Timeout;
      HAL_Delay(1);
    }

    std::uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(kCan, &hdr, const_cast<std::uint8_t*>(f.data),
                             &mailbox) != HAL_OK) {
      return common::Err::BusError;
    }
    ++g_tx_cnt;
    return common::Err::Ok;
  }

  common::Err recv(CanFrame& f, std::uint32_t timeout_ms) override
  {
    if (!g_started) return common::Err::NotInit;

    const std::uint32_t t0 = HAL_GetTick();
    while (g_rx_tail == g_rx_head) {
      if ((HAL_GetTick() - t0) > timeout_ms) return common::Err::Timeout;
      HAL_Delay(1);
    }
    f = g_rx_queue[g_rx_tail % kRxQueueSize];
    g_rx_tail = (g_rx_tail + 1) % kRxQueueSize;
    return common::Err::Ok;
  }

  std::uint32_t tx_count() const override { return g_tx_cnt; }
  std::uint32_t rx_count() const override { return g_rx_cnt; }
  std::uint32_t rx_dropped() const override { return g_rx_drop; }
  std::uint32_t error_count() const override { return g_err_cnt; }
  std::uint32_t busoff_count() const override { return g_busoff_cnt; }
  bool need_recovery() const override { return g_busoff; }
  common::Err recover() override { return recover_from_busoff(); }

  /// 中断里调用：只做搬运，不做解析
  void on_rx_pending()
  {
    CAN_RxHeaderTypeDef hdr = {0};
    std::uint8_t data[8] = {0};
    while (HAL_CAN_GetRxFifoFillLevel(kCan, CAN_RX_FIFO0) > 0) {
      if (HAL_CAN_GetRxMessage(kCan, CAN_RX_FIFO0, &hdr, data) != HAL_OK) break;

      const std::uint32_t next = (g_rx_head + 1) % kRxQueueSize;
      if (next == g_rx_tail) {
        ++g_rx_drop;                       // 队列满：丢新帧，并记账
        continue;
      }
      g_rx_queue[g_rx_head].id = hdr.StdId;
      g_rx_queue[g_rx_head].len = hdr.DLC;
      std::memcpy(g_rx_queue[g_rx_head].data, data, 8);
      g_rx_head = next;
      ++g_rx_cnt;
    }
  }

  void on_error()
  {
    ++g_err_cnt;
    // bus-off：发送错误计数器溢出，控制器自己停摆，必须重新 Start 才能恢复
    if ((kCan->ErrorCode & HAL_CAN_ERROR_BOF) != 0u) {
      g_busoff = true;
    }
  }

  /// 任务侧调用：重启控制器（bus-off 恢复）
  common::Err recover_from_busoff()
  {
    LOG_W("can: bus-off, restarting controller");
    (void)HAL_CAN_Stop(kCan);
    if (HAL_CAN_Start(kCan) != HAL_OK) return common::Err::BusError;
    if (HAL_CAN_ActivateNotification(kCan, CAN_IT_RX_FIFO0_MSG_PENDING |
                                              CAN_IT_ERROR | CAN_IT_BUSOFF |
                                              CAN_IT_ERROR_PASSIVE |
                                              CAN_IT_ERROR_WARNING) != HAL_OK) {
      return common::Err::BusError;
    }
    g_busoff = false;
    ++g_busoff_cnt;
    return common::Err::Ok;
  }
};

namespace {
Can1 g_can1;
}  // namespace

Can& can1 = g_can1;

void can1_on_rx_pending() { g_can1.on_rx_pending(); }
void can1_on_error() { g_can1.on_error(); }

}  // namespace bsp

/* -------------------- HAL 回调（覆盖弱符号） -------------------- */

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_h)
{
  if (hcan_h == &hcan) bsp::can1_on_rx_pending();
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan_h)
{
  if (hcan_h == &hcan) bsp::can1_on_error();
}
