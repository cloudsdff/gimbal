/**
 * @file    app_main.cpp
 * @brief   任务02 阶段一：姿态解算（互补滤波 + 上电自校准 + 零漂抑制）
 *
 * 当前实现范围：
 *   [x] MPU6050 上电自校准（静止 1 秒求陀螺零偏）
 *   [x] 零偏扣除 + 小角速度死区（零漂抑制）
 *   [x] 200 Hz 采样、50 Hz 串口输出三轴角度
 *   [ ] 上位机实时波形（下一步）
 *   [ ] 蓝牙指令控制指向（硬件到货后）
 *   [ ] 云台闭环（云台到货后）
 *
 * 这个文件里不允许出现 HAL_ 前缀。
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "attitude.hpp"
#include "bsp_i2c.hpp"
#include "bsp_uart.hpp"
#include "cmsis_os2.h"
#include "i2c.hpp"
#include "log.hpp"
#include "mpu6050.hpp"

namespace {

// ---- 采样与输出参数 ----
constexpr std::uint32_t kSampleHz = 200;
constexpr std::uint32_t kSamplePeriodMs = 1000 / kSampleHz;   // 5 ms
constexpr float kDt = 1.0f / static_cast<float>(kSampleHz);    // 固定步长
constexpr int kOutputDiv = 4;                                  // 每 4 帧 → 50 Hz
constexpr std::uint16_t kCalSamples = 200;                     // 校准 1 秒

module::Mpu6050 g_mpu(bsp::i2c1);
module::Attitude g_att(0.98f);

/// 角度 → 0.01° 整数（F103 上 %f 很慢，用整数更稳）
inline int cd(float deg) { return static_cast<int>(deg * 100.0f); }

void attitude_task(void* arg)
{
  (void)arg;

  // 1) 传感器初始化
  const common::Err e = g_mpu.init();
  LOG_I("mpu init -> %s", common::to_str(e));
  if (e != common::Err::Ok) {
    LOG_E("mpu6050 unavailable, attitude task halted");
    for (;;) osDelay(1000);
  }

  // 2) 上电自校准
  g_att.begin_calibration(kCalSamples);
  LOG_I("calibrating %u samples (keep still)...", (unsigned)kCalSamples);

  const std::uint32_t t_start = osKernelGetTickCount();
  std::uint32_t last_stats = t_start;
  std::uint32_t frames = 0;          // 已处理的采样帧
  std::uint32_t sent = 0;            // 已发出的角度帧
  std::uint32_t dropped = 0;         // DMA 忙导致的丢弃
  bool cal_logged = false;

  for (;;) {
    const std::uint32_t now = osKernelGetTickCount();

    module::Mpu6050::Raw raw{};
    const common::Err r = g_mpu.read_raw(raw);

    if (r == common::Err::Ok) {
      const common::Err st = g_att.update(raw, kDt);
      ++frames;

      if (!cal_logged && g_att.calibrated()) {
        cal_logged = true;
        const module::GyroBias b = g_att.bias();
        // 整数打印：F103 的 newlib-nano 默认不带浮点 printf
        LOG_I("calib done: gyro bias = [%d, %d, %d] mdps",
              (int)(b.x * 1000.0f), (int)(b.y * 1000.0f), (int)(b.z * 1000.0f));
      }

      if (st == common::Err::Ok && (frames % kOutputDiv) == 0) {
        const module::Euler eu = g_att.euler();
        char line[48];
        const int n = std::snprintf(line, sizeof(line), "A,%d,%d,%d\r\n", cd(eu.yaw),
                                    cd(eu.pitch), cd(eu.roll));
        if (n > 0) {
          // DMA 发送，不阻塞采样节拍；上一帧没发完就丢弃本次
          if (bsp::uart1.write_async(reinterpret_cast<const std::uint8_t*>(line),
                                     static_cast<std::size_t>(n)) == common::Err::Ok) {
            ++sent;
          } else {
            ++dropped;
          }
        }
      }
    } else {
      LOG_W("mpu read failed: %s", common::to_str(r));
      (void)bsp::i2c1.recover_bus();
    }

    // 3) 每秒统计：实际采样率、发送数、丢弃数、当前角度（0.01°）
    if ((now - last_stats) >= 1000u) {
      const std::uint32_t span = now - last_stats;
      last_stats = now;
      const module::Euler eu = g_att.euler();
      LOG_I("att %luHz samp=%lu sent=%lu drop=%lu | yaw=%d pitch=%d roll=%d (cd) alpha_m=%d",
            (unsigned long)(frames * 1000u / (now - t_start ? (now - t_start) : 1u)),
            (unsigned long)frames, (unsigned long)sent, (unsigned long)dropped,
            cd(eu.yaw), cd(eu.pitch), cd(eu.roll), (int)(g_att.alpha() * 1000.0f));
      (void)span;
    }

    // 4) 严格 5 ms 周期
    osDelayUntil(now + kSamplePeriodMs);
  }
}

}  // namespace

extern "C" void app_init(void)
{
  common::log_init();
  LOG_I("task02 stage-1: attitude estimation");

  const osThreadAttr_t attr = {
      .name = "Attitude", .stack_size = 2048, .priority = osPriorityAboveNormal};
  if (osThreadNew(attitude_task, nullptr, &attr) == nullptr) {
    LOG_E("create Attitude task failed");
  }
}
