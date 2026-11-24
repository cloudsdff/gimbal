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
#include "bluetooth.hpp"
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

/// 蓝牙初始化结果（app_init 在调度器之前执行，日志只能延后到任务里打印）
common::Err g_bt_init = common::Err::NotInit;

/// 指令缓冲区（调试串口和蓝牙各一份，互不干扰）
struct CmdBuf {
  char buf[64];
  std::size_t len = 0;
};

CmdBuf g_cmd_uart1;
CmdBuf g_cmd_bt;

/// 往某个指令缓冲里喂一个字节，遇到换行就执行
void feed(CmdBuf& c, std::uint8_t ch);

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
  } else if (line[0] == 'B' && line[1] == ',') {
    // B,<baud>:??????(HC-05 ?? 9600 ??,??? AT ??)
    const int baud = std::atoi(line + 2);
    LOG_I("cmd bt baud -> %d : %s", baud, common::to_str(bsp::uart2_set_baud((std::uint32_t)baud)));
  } else if (line[0] == 'S' && line[1] == ',') {
    // S,<yaw_sign>,<pitch_sign>:????????? S,1,-1 ??
    int sy = 1, sp = 1;
    if (std::sscanf(line + 2, "%d,%d", &sy, &sp) == 2) {
      g_gimbal.set_sign((float)sy, (float)sp);
      LOG_I("cmd axis sign -> %d,%d", sy, sp);
    }
  } else if (line[0] == 'P' && line[1] == ',') {
    // P,<yaw_us>,<pitch_us>:?????????(500-2500),????????
    int uy = 1500, up = 1500;
    if (std::sscanf(line + 2, "%d,%d", &uy, &up) == 2) {
      (void)g_gimbal.stop();          // ????,?????
      (void)bsp::servo_pwm.set_pulse_us(1, (std::uint32_t)uy);
      (void)bsp::servo_pwm.set_pulse_us(2, (std::uint32_t)up);
      LOG_I("cmd openloop: yaw=%dus pitch=%dus", uy, up);
    }
  } else if (line[0] == 'G' && line[1] == ',') {
    // G,<kp_x100>,<kd_x100>,<ki_x1000>:?????
    int kp = 0, kd = 0, ki = 0;
    if (std::sscanf(line + 2, "%d,%d,%d", &kp, &kd, &ki) == 3) {
      module::Gimbal::Gains g = g_gimbal.gains();
      g.kp = (float)kp / 100.0f;
      g.kd = (float)kd / 100.0f;
      g.ki = (float)ki / 1000.0f;
      g_gimbal.set_gains(g);
      LOG_I("cmd gains: kp=%d kd=%d ki=%d (x100/x100/x1000)", kp, kd, ki);
    }
  } else if (line[0] == 'F') {
    // F:??"??????"??(roll ?? �180� ??)
    g_att.set_mount_flip(!g_att.mount_flip());
    LOG_I("cmd mount flip -> %d (now recalibrate with C)", g_att.mount_flip() ? 1 : 0);
  } else if (line[0] == 'D') {
    // D:??(??????),??????/??
    (void)g_gimbal.stop();
    LOG_I("cmd disable: gimbal stopped (send X to resume)");
  } else if (line[0] == 'X') {
    // X:???????(????????)
    g_gimbal.resume();
    LOG_I("cmd resume: gimbal enabled");
  } else if (line[0] == 'C') {
    g_att.begin_calibration(kCalSamples);
    LOG_I("cmd recalibrate");
  }
}

/// 往某个指令缓冲里喂一个字节，遇到换行就执行
void feed(CmdBuf& c, std::uint8_t ch)
{
  if (ch == '\n' || ch == '\r') {
    if (c.len > 0) {
      c.buf[c.len] = '\0';
      handle_command(c.buf);
      c.len = 0;
    }
  } else if (c.len < sizeof(c.buf) - 1) {
    c.buf[c.len++] = static_cast<char>(ch);
  } else {
    c.len = 0;   // 溢出丢弃，避免执行半截指令
  }
}

void attitude_task(void* arg)
{
  (void)arg;

  // 调度器已启动，这里才开始打印（app_init 在调度器之前，
  // osMessageQueuePut 会失败，日志会丢 —— 实测过）
  LOG_I("task02: attitude + gimbal control");
  LOG_I("bluetooth init -> %s", common::to_str(g_bt_init));

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
  std::uint32_t consecutive_fail = 0;   // consecutive read failures (safety stop)
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
      ++consecutive_fail;
      LOG_W("mpu read failed: %s (consecutive=%lu)", common::to_str(r),
            (unsigned long)consecutive_fail);
      (void)bsp::i2c1.recover_bus();

      // 安全策略：传感器连续失效约 0.5 秒（100 帧）就自动停机。
      // 不这么做会形成正反馈：角度算错 → 舵机全力纠正 → 大电流 → 干扰更大 → 更错
      if (consecutive_fail >= 100 && g_gimbal.enabled()) {
        (void)g_gimbal.stop();
        LOG_E("safety stop: imu failed 100x, gimbal disabled (send X to resume)");
      }
    }

    // 3.6 串口指令（调试口 + 蓝牙，每周期各读 1 字节，非阻塞）
    std::uint8_t ch = 0;
    if (bsp::uart1.read(&ch, 1, 0) == common::Err::Ok) {
      feed(g_cmd_uart1, ch);
    }
    if (bsp::uart2.read(&ch, 1, 0) == common::Err::Ok) {
      feed(g_cmd_bt, ch);
    }

    // 3.7 每秒统计
    if ((now - last_stats) >= 1000u) {
      last_stats = now;
      const module::Euler eu = g_att.euler();
      LOG_I("att %luHz samp=%lu sent=%lu drop=%lu rx=%lu bt=%lu | yaw=%d pitch=%d roll=%d servo=%lu/%lu",
            (unsigned long)(frames * 1000u / (now - t_start ? (now - t_start) : 1u)),
            (unsigned long)frames, (unsigned long)sent, (unsigned long)dropped,
            (unsigned long)bsp::uart1_rx_count(), (unsigned long)bsp::uart2_rx_count(),
            cd(eu.yaw), cd(eu.pitch), cd(eu.roll),
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
  bsp::uart1_start_rx();   // 关键：启动串口中断接收，否则指令永远收不到

  // 蓝牙指令通道（HC-05 → USART2 / PA2-PA3）
  // 注意：这里不能打印 —— 调度器还没启动，日志入队会失败（实测丢过日志）
  g_bt_init = bsp::uart2_start();

  const osThreadAttr_t attr = {
      .name = "Gimbal", .stack_size = 2048, .priority = osPriorityAboveNormal};
  if (osThreadNew(attitude_task, nullptr, &attr) == nullptr) {
    LOG_E("create Gimbal task failed");
  }
}
