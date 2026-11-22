/**
 * @file    app_main.cpp
 * @brief   任务02：姿态解算 + 二轴云台串级控制
 *
 * 实现范围：
 *   [x] MPU6050 上电自校准（陀螺零偏 + 加速度计零姿态）
 *   [x] 互补滤波 200 Hz 采样 / 50 Hz 输出
 *   [x] 二轴云台串级控制（角度环 P + 角速度环 PD）
 *   [x] 上电缓启动、输出限幅、角度死区、积分限幅
 *   [x] 串口指令：T,<yaw_cd>,<pitch_cd> 设目标；K,<milli> 调 alpha；C 重校准
 *   [ ] 蓝牙（USART2）—— 硬件到货后接
 *
 * 这个文件里不允许出现 HAL_ 前缀。
 */
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "attitude.hpp"
#include "bsp_i2c.hpp"
#include "pwm.hpp"
#include "bsp_uart.hpp"
#include "cmsis_os2.h"
#include "gimbal.hpp"
#include "i2c.hpp"
#include "log.hpp"
#include "mpu6050.hpp"
#include "pwm.hpp"
#include "servo.hpp"

namespace {

// ---- 采样与输出 ----
constexpr std::uint32_t kSampleHz = 200;
constexpr std::uint32_t kSamplePeriodMs = 1000 / kSampleHz;   // 5 ms
constexpr float kDt = 1.0f / static_cast<float>(kSampleHz);
constexpr int kAngleDiv = 4;      // 每 4 帧 → 50 Hz 角度帧
constexpr int kServoDiv = 20;     // 每 20 帧 → 10 Hz 舵机帧
constexpr std::uint16_t kCalSamples = 200;

module::Mpu6050 g_mpu(bsp::i2c1);
module::Attitude g_att(0.98f);
module::Servo g_servo_yaw(bsp::servo_pwm, 1);
module::Servo g_servo_pitch(bsp::servo_pwm, 2);
module::Gimbal g_gimbal(g_servo_yaw, g_servo_pitch);

/// 指令缓冲区
char g_cmd[64];
std::size_t g_cmd_len = 0;

inline int cd(float deg) { return static_cast<int>(deg * 100.0f); }

/// 解析一行指令（由主循环喂字符）
void handle_command(const char* line)
{
  if (line[0] == 'T' && line[1] == ',') {
    int y = 0, p = 0;
    if (std::sscanf(line + 2, "%d,%d", &y, &p) == 2) {
      g_gimbal.set_target(static_cast<float>(y) / 100.0f, static_cast<float>(p) / 100.0f);
      LOG_I("cmd target: yaw=%d.%02d pitch=%d.%02d", y / 100, std::abs(y % 100), p / 100,
            std::abs(p % 100));
    }
  } else if (line[0] == 'K' && line[1] == ',') {
    const int milli = std::atoi(line + 2);
    if (milli >= 500 && milli <= 999) {
      g_att.set_alpha(static_cast<float>(milli) / 1000.0f);
      LOG_I("cmd alpha -> %d/1000", milli);
    } else {
      LOG_W("cmd alpha out of range: %d", milli);
    }
  } else if (line[0] == 'C') {
    g_att.begin_calibration(kCalSamples);
    LOG_I("cmd recalibrate");
  }
}

void attitude_task(void* arg)
{
  (void)arg;

  // ---- 1) 传感器 ----
  common::Err e = g_mpu.init();
  while (e != common::Err::Ok) {
    LOG_E("mpu init -> %s, retry in 2s (check VCC/GND/SCL=B6/SDA=B7)", common::to_str(e));
    osDelay(2000);
    (void)bsp::i2c1.recover_bus();
    e = g_mpu.init();
  }
  LOG_I("mpu init -> Ok");

  // ---- 2) 舵机与云台 ----
  if (bsp::servo_pwm.init() == common::Err::Ok) {
    g_servo_yaw.set_limits(20.0f, 160.0f);
    g_servo_pitch.set_limits(20.0f, 160.0f);
    const common::Err ge = g_gimbal.init(90.0f);
    LOG_I("gimbal init -> %s", common::to_str(ge));
  } else {
    LOG_E("pwm init failed, gimbal disabled");
  }

  // ---- 3) 自校准 ----
  g_att.begin_calibration(kCalSamples);
  LOG_I("calibrating %u samples (keep still)...", (unsigned)kCalSamples);

  const std::uint32_t t_start = osKernelGetTickCount();
  std::uint32_t last_stats = t_start;
  std::uint32_t frames = 0, sent = 0, dropped = 0;
  bool cal_logged = false;

  for (;;) {
    const std::uint32_t now = osKernelGetTickCount();

    // 3.1 读一帧 IMU
    module::Mpu6050::Raw raw{};
    const common::Err r = g_mpu.read_raw(raw);

    if (r == common::Err::Ok) {
      const common::Err st = g_att.update(raw, kDt);
      ++frames;

      if (!cal_logged && g_att.calibrated()) {
        cal_logged = true;
        const module::GyroBias b = g_att.bias();
        LOG_I("calib done: gyro bias = [%d, %d, %d] mdps", (int)(b.x * 1000.0f),
              (int)(b.y * 1000.0f), (int)(b.z * 1000.0f));
      }

      if (st == common::Err::Ok) {
        // 3.2 角速度（扣零偏，单位 dps）
        const module::GyroBias b = g_att.bias();
        module::Rates rates{};
        rates.yaw = static_cast<float>(raw.gz) / module::Attitude::kGyroLsbPerDps - b.z;
        rates.pitch = static_cast<float>(raw.gy) / module::Attitude::kGyroLsbPerDps - b.y;
        rates.roll = static_cast<float>(raw.gx) / module::Attitude::kGyroLsbPerDps - b.x;

        // 3.3 云台控制（200 Hz）
        (void)g_gimbal.update(g_att.euler(), rates, kDt);

        // 3.4 输出角度帧（50 Hz）
        if ((frames % kAngleDiv) == 0) {
          const module::Euler eu = g_att.euler();
          char line[48];
          const int n = std::snprintf(line, sizeof(line), "A,%d,%d,%d\r\n", cd(eu.yaw),
                                      cd(eu.pitch), cd(eu.roll));
          if (n > 0 && bsp::uart1.write_async(
                           reinterpret_cast<const std::uint8_t*>(line),
                           static_cast<std::size_t>(n)) == common::Err::Ok) {
            ++sent;
          } else {
            ++dropped;
          }
        }

        // 3.5 输出舵机帧（10 Hz，调试用：看控制器输出了多少）
        if ((frames % kServoDiv) == 0) {
          char line[48];
          const int n = std::snprintf(line, sizeof(line), "G,%lu,%lu\r\n",
                                      (unsigned long)g_servo_yaw.pulse_us(),
                                      (unsigned long)g_servo_pitch.pulse_us());
          if (n > 0) {
            (void)bsp::uart1.write_async(reinterpret_cast<const std::uint8_t*>(line),
                                         static_cast<std::size_t>(n));
          }
        }
      }
    } else {
      LOG_W("mpu read failed: %s", common::to_str(r));
      (void)bsp::i2c1.recover_bus();
    }

    // 3.6 串口指令（每周期读 1 字节，非阻塞）
    std::uint8_t ch = 0;
    if (bsp::uart1.read(&ch, 1, 0) == common::Err::Ok) {
      if (ch == '\n' || ch == '\r') {
        if (g_cmd_len > 0) {
          g_cmd[g_cmd_len] = '\0';
          handle_command(g_cmd);
          g_cmd_len = 0;
        }
      } else if (g_cmd_len < sizeof(g_cmd) - 1) {
        g_cmd[g_cmd_len++] = static_cast<char>(ch);
      } else {
        g_cmd_len = 0;   // 溢出丢弃
      }
    }

    // 3.7 每秒统计
    if ((now - last_stats) >= 1000u) {
      last_stats = now;
      const module::Euler eu = g_att.euler();
      LOG_I("att %luHz samp=%lu sent=%lu drop=%lu rx=%lu | yaw=%d pitch=%d roll=%d servo=%lu/%lu",
            (unsigned long)(frames * 1000u / (now - t_start ? (now - t_start) : 1u)),
            (unsigned long)frames, (unsigned long)sent, (unsigned long)dropped,
            (unsigned long)bsp::uart1_rx_count(), cd(eu.yaw), cd(eu.pitch), cd(eu.roll),
            (unsigned long)g_servo_yaw.pulse_us(),
            (unsigned long)g_servo_pitch.pulse_us());
    }

    osDelayUntil(now + kSamplePeriodMs);
  }
}

}  // namespace

extern "C" void app_init(void)
{
  common::log_init();
  LOG_I("task02: attitude + gimbal control");

  const osThreadAttr_t attr = {
      .name = "Gimbal", .stack_size = 2048, .priority = osPriorityAboveNormal};
  if (osThreadNew(attitude_task, nullptr, &attr) == nullptr) {
    LOG_E("create Gimbal task failed");
  }
}
