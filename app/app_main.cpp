/**
 * @file    app_main.cpp
 * @brief   应用层入口 + 任务划分（任务01 工程要求 2）
 *
 * 任务划分一览（详见 docs/05-任务设计.md）：
 *   task_led       200 ms   LED 心跳          （CubeMX 生成，保留）
 *   task_log       500 ms   栈水线观察        （CubeMX 生成，保留）
 *   Comm           1 ms     CAN 收发          （本文件，优先级较高）
 *   Sensor         5 ms     MPU6050 采样      （本文件，优先级中）
 *   log_task       事件驱动 日志发送          （common/log.cpp，优先级最低）
 *
 * 这个文件里不允许出现 HAL_ 前缀。
 */
#include <cstdint>

#include "bsp_can.hpp"
#include "bsp_i2c.hpp"
#include "can.hpp"
#include "cmsis_os2.h"
#include "i2c.hpp"
#include "log.hpp"
#include "mpu6050.hpp"

namespace {

#ifdef CAN_NODE_B
constexpr std::uint32_t kMyId = 0x456;
constexpr std::uint32_t kPeerId = 0x123;
#else
constexpr std::uint32_t kMyId = 0x123;
constexpr std::uint32_t kPeerId = 0x456;
#endif

#ifdef STACK_TEST_SMALL
constexpr std::uint32_t kCommStack = 200;   // 失败路径测试：故意给不够的栈
#else
constexpr std::uint32_t kCommStack = 1024;
#endif

module::Mpu6050 g_mpu(bsp::i2c1);
osThreadId_t g_comm_id = nullptr;
osThreadId_t g_sensor_id = nullptr;

// ---------------------------------------------------------------- Comm 任务
// 1 ms 周期里做四件事：收帧、bus-off 恢复、按 100 ms 发帧、5 s 打一次统计
void comm_task(void* arg)
{
  (void)arg;
  std::uint32_t seq = 0;
  std::uint32_t last_tx = 0;
  std::uint32_t last_stats = 0;
  std::uint32_t last_log = 0;

  for (;;) {
    const std::uint32_t now = osKernelGetTickCount();

    // 1) 收：把队列里所有帧取出来
    bsp::CanFrame rx{};
    while (bsp::can1.recv(rx, 0) == common::Err::Ok) {
      // 日志限速：每秒最多一条，避免刷屏把日志队列塞满
      if ((now - last_log) >= 1000u) {
        last_log = now;
        const std::uint32_t peer_seq = rx.data[0] | (rx.data[1] << 8) |
                                       (rx.data[2] << 16) | (rx.data[3] << 24);
        LOG_I("rx id=0x%03lX len=%u seq=%lu", static_cast<unsigned long>(rx.id),
              rx.len, static_cast<unsigned long>(peer_seq));
      }
    }

    // 2) bus-off 恢复：先退避 500 ms，避免孤立节点反复霸占总线
    if (bsp::can1.need_recovery()) {
      osDelay(500);
      const common::Err r = bsp::can1.recover();
      LOG_W("can recover -> %s", common::to_str(r));
    }

    // 3) 发：每 100 ms 一帧
    if ((now - last_tx) >= 100u) {
      last_tx = now;
      bsp::CanFrame f{};
      f.id = kMyId;
      f.len = 8;
      f.data[0] = static_cast<std::uint8_t>(seq & 0xFF);
      f.data[1] = static_cast<std::uint8_t>((seq >> 8) & 0xFF);
      f.data[2] = static_cast<std::uint8_t>((seq >> 16) & 0xFF);
      f.data[3] = static_cast<std::uint8_t>((seq >> 24) & 0xFF);
      f.data[4] = 0xA5;
      f.data[5] = 0x5A;
      f.data[6] = static_cast<std::uint8_t>(kMyId & 0xFF);
      f.data[7] = 0x00;
      if (bsp::can1.send(f, 20) == common::Err::Ok) ++seq;
    }

    // 4) 统计：每 5 s 一条
    if ((now - last_stats) >= 5000u) {
      last_stats = now;
      LOG_I("stats: tx=%lu rx=%lu drop=%lu err=%lu busoff=%lu logsent=%lu logdrop=%lu",
            static_cast<unsigned long>(bsp::can1.tx_count()),
            static_cast<unsigned long>(bsp::can1.rx_count()),
            static_cast<unsigned long>(bsp::can1.rx_dropped()),
            static_cast<unsigned long>(bsp::can1.error_count()),
            static_cast<unsigned long>(bsp::can1.busoff_count()),
            static_cast<unsigned long>(common::log_sent()),
            static_cast<unsigned long>(common::log_dropped()));
#ifdef LOG_OVERFLOW_TEST
      // ??????:??? 40 ?,?????? 12,????
      for (int i = 0; i < 40; ++i) {
        LOG_I("burst %d", i);
      }
#endif
      LOG_I("stack free: Comm=%lu Sensor=%lu",
            static_cast<unsigned long>(osThreadGetStackSpace(g_comm_id)),
            static_cast<unsigned long>(osThreadGetStackSpace(g_sensor_id)));
    }

    osDelay(1);
  }
}

// -------------------------------------------------------------- Sensor 任务
void sensor_task(void* arg)
{
  (void)arg;
  osDelay(300);

  common::Err e = g_mpu.init();
  LOG_I("mpu init -> %s", common::to_str(e));
  if (e != common::Err::Ok) {
    LOG_W("mpu init failed, try bus recovery");
    (void)bsp::i2c1.recover_bus();
  }

  std::uint32_t last_log = 0;
  std::uint32_t lost = 0;

  for (;;) {
    module::Mpu6050::Raw raw{};
    const common::Err r = g_mpu.read_raw(raw);
    if (r != common::Err::Ok) {
      ++lost;
      if (lost == 1u) {   // 只报一次，避免刷屏
        LOG_W("mpu read lost (%s), recovery...", common::to_str(r));
        (void)bsp::i2c1.recover_bus();
      }
    } else {
      lost = 0;
      const std::uint32_t now = osKernelGetTickCount();
      if ((now - last_log) >= 1000u) {   // 5 ms 采样，1 Hz 打印
        last_log = now;
        LOG_I("acc[%d,%d,%d] gyro[%d,%d,%d]", raw.ax, raw.ay, raw.az, raw.gx,
              raw.gy, raw.gz);
      }
    }
    osDelay(5);
  }
}

}  // namespace

extern "C" void app_init(void)
{
  common::log_init();     // 先建日志队列 + Log 任务
  LOG_I("app layer ready (C++17, HAL-free)");

  // CAN：只放行对端 ID（硬件过滤，自己发的帧不会回环进队列）
  const common::Err ce = bsp::can1.start(kPeerId);
  LOG_I("can start -> %s (accept id=0x%03lX)", common::to_str(ce),
        static_cast<unsigned long>(kPeerId));

  const osThreadAttr_t comm_attr = {
      .name = "Comm", .stack_size = kCommStack, .priority = osPriorityAboveNormal};
  g_comm_id = osThreadNew(comm_task, nullptr, &comm_attr);
  if (g_comm_id == nullptr) LOG_E("create Comm failed");

  const osThreadAttr_t sensor_attr = {
      .name = "Sensor", .stack_size = 1024, .priority = osPriorityNormal};
  g_sensor_id = osThreadNew(sensor_task, nullptr, &sensor_attr);
  if (g_sensor_id == nullptr) LOG_E("create Sensor failed");
}
