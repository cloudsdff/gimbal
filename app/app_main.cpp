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
#include "bsp_i2c_soft.hpp"
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
constexpr std::uint32_t kSampleHz = 100;   // 线材差时降频，减轻 I2C 负载
constexpr std::uint32_t kSamplePeriodMs = 1000 / kSampleHz;   // 5 ms
constexpr float kDt = 1.0f / static_cast<float>(kSampleHz);
constexpr int kAngleDiv = 4;      // 每 4 帧 → 50 Hz 角度帧
constexpr int kServoDiv = 20;     // 每 20 帧 → 10 Hz 舵机帧
constexpr std::uint16_t kCalSamples = 200;

// 用软件 I2C：硬件 I2C 在舵机干扰下会卡死在 BUSY（实测 11 次），bit-bang 没有这个状态
module::Mpu6050 g_mpu(bsp::soft_i2c);
module::Attitude g_att(0.98f);
module::Servo g_servo_yaw(bsp::servo_pwm, 1);
module::Servo g_servo_pitch(bsp::servo_pwm, 2);
module::Gimbal g_gimbal(g_servo_yaw, g_servo_pitch);

/// 蓝牙初始化结果（app_init 在调度器之前执行，日志只能延后到任务里打印）
common::Err g_bt_init = common::Err::NotInit;
bool g_quiet = false;   // 静默模式：停止 50 Hz 角度帧，只留 1 Hz 统计（录视频时用）
module::Mpu6050::Raw g_last_raw{};   // 最近一帧原始数据（调试用）

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
    // (comment cleaned)
    const int baud = std::atoi(line + 2);
    LOG_I("cmd bt baud -> %d : %s", baud, common::to_str(bsp::uart2_set_baud((std::uint32_t)baud)));
  } else if (line[0] == 'S' && line[1] == ',') {
    // (comment cleaned)
    int sy = 1, sp = 1;
    if (std::sscanf(line + 2, "%d,%d", &sy, &sp) == 2) {
      g_gimbal.set_sign((float)sy, (float)sp);
      LOG_I("cmd axis sign -> %d,%d", sy, sp);
    }
  } else if (line[0] == 'P' && line[1] == ',') {
    // (comment cleaned)
    int uy = 1500, up = 1500;
    if (std::sscanf(line + 2, "%d,%d", &uy, &up) == 2) {
      (void)g_gimbal.stop();
      (void)bsp::servo_pwm.set_pulse_us(1, (std::uint32_t)uy);
      (void)bsp::servo_pwm.set_pulse_us(2, (std::uint32_t)up);
      LOG_I("cmd openloop: yaw=%dus pitch=%dus", uy, up);
    }
  } else if (line[0] == 'G' && line[1] == ',') {
    // (comment cleaned)
    int kp = 0, kd = 0, ki = 0, ff = 0, ffs = 0;
    const int n = std::sscanf(line + 2, "%d,%d,%d,%d,%d", &kp, &kd, &ki, &ff, &ffs);
    if (n >= 3) {
      module::Gimbal::Gains g = g_gimbal.gains();
      g.kp = (float)kp / 100.0f;
      g.kd = (float)kd / 100.0f;
      g.ki = (float)ki / 1000.0f;
      if (n >= 4) g.ff_pitch = (float)ff / 100.0f;
      if (n >= 5) g.ff_scale = (float)ffs / 100.0f;
      g_gimbal.set_gains(g);
      LOG_I("cmd gains: kp=%d kd=%d ki=%d ff=%d", kp, kd, ki, ff);
    }
  } else if (line[0] == 'L' && line[1] == ',') {
    // (comment cleaned)
    const int v = std::atoi(line + 2);
    const float deg = (float)v / 100.0f;
    g_servo_yaw.set_backlash(deg);
    g_servo_pitch.set_backlash(deg);
    LOG_I("cmd backlash -> %d (0.01deg)", v);
  } else if (line[0] == 'F') {
    // (comment cleaned)
    g_att.set_mount_flip(!g_att.mount_flip());
    LOG_I("cmd mount flip -> %d (now recalibrate with C)", g_att.mount_flip() ? 1 : 0);
  } else if (line[0] == 'D') {
    // (comment cleaned)
    (void)g_gimbal.stop();
    LOG_I("cmd disable: gimbal stopped (send X to resume)");
  } else if (line[0] == 'X') {
    // (comment cleaned)
    g_gimbal.resume();
    LOG_I("cmd resume: gimbal enabled");
  } else if (line[0] == 'Z') {
    // Z：就地置零（把当前姿态当作 0°），比 C 更可预测
    g_att.zero_here();
    LOG_I("cmd zero_here: current pose = 0deg");
  } else if (line[0] == 'A') {
    // A：切换"仅用加速度计"模式（A 再按一次切回互补滤波）
    g_att.set_accel_only(!g_att.accel_only());
    LOG_I("cmd accel_only -> %d (1=no gyro integration)", g_att.accel_only() ? 1 : 0);
  } else if (line[0] == 'M') {
    // M：重新初始化 MPU6050（等价于"软复位传感器"）。
    // 用途：舵机干扰后芯片可能掉回 sleep 或配置被冲掉，
    // 此时地址还能 ACK 但读寄存器全是垃圾 —— 重新写一遍配置就能救回来。
    LOG_I("mpu re-init by command...");
    (void)bsp::soft_i2c.recover_bus();
    const common::Err re = g_mpu.reinit();
    LOG_I("mpu re-init -> %s", common::to_str(re));
  } else if (line[0] == 'I') {
    // I：扫描 I2C 总线，打印所有应答的地址（排查"模块到底在不在总线上"）
    LOG_I("i2c scan start (0x08-0x77)...");
    int found = 0;
    for (std::uint8_t a = 0x08; a <= 0x77; ++a) {
      if (bsp::soft_i2c.probe(a, 5) == common::Err::Ok) {
        LOG_I("  i2c device found at 0x%02X", a);
        ++found;
      }
    }
    if (found == 0) {
      LOG_E("i2c scan: no device on bus (check VCC/GND/SCL/SDA)");
    } else {
      LOG_I("i2c scan done, %d device(s)", found);
    }
  } else if (line[0] == 'Q') {
    // Q：切换静默模式（停止 50 Hz 角度帧，只留每秒统计行）
    g_quiet = !g_quiet;
    LOG_I("cmd quiet -> %d (1=stopped angle frames)", g_quiet ? 1 : 0);
  } else if (line[0] == 'O') {
    // O：打印一帧当前角度（静默模式下按需查看）
    const module::Euler eu = g_att.euler();
    LOG_I("angle: yaw=%d pitch=%d roll=%d (0.01deg)", cd(eu.yaw), cd(eu.pitch),
          cd(eu.roll));
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
  // 软件 I2C 的引脚必须先配好（PB6/PB7 开漏输出），否则 bit-bang 驱动不了总线
  bsp::SoftI2c::init_pins();
  LOG_I("soft i2c pins ready (PB6=SCL PB7=SDA, bit-bang)");
  common::Err e = g_mpu.reinit();
  int retry = 0;
  while (e != common::Err::Ok) {
    // (comment cleaned)
    if (++retry == 2) {
    }
    LOG_E("mpu init -> %s, retry in 2s (check VCC/GND/SCL=B6/SDA=B7)", common::to_str(e));
    osDelay(2000);
    (void)bsp::soft_i2c.recover_bus();
    e = g_mpu.reinit();
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
  std::uint32_t consecutive_fail = 0;
  std::uint32_t last_recover = 0;
  bool cal_logged = false;

  for (;;) {
    const std::uint32_t now = osKernelGetTickCount();

    // 3.1 读一帧 IMU
    module::Mpu6050::Raw raw{};
    const common::Err r = g_mpu.read_raw(raw);

    if (r == common::Err::Ok) {
      g_last_raw = raw;
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
        // 用姿态模块算好的角速度：它已经扣过零偏、也按安装方向取反过。
        // 这里如果自己拿 raw 算，符号会和角度不一致，阻尼变成正反馈（踩过）。
        // 用欧拉角微分做阻尼：符号必然和角度一致（用陀螺会因为翻面补偿而反号）
        const module::GyroBias& cr = g_att.angle_rate();
        module::Rates rates{};
        rates.yaw = cr.z;
        rates.pitch = cr.y;
        rates.roll = cr.x;

        // 3.3 云台控制（200 Hz）
        (void)g_gimbal.update(g_att.euler(), rates, kDt);

        // 3.4 输出角度帧（50 Hz）
        if (!g_quiet && (frames % kAngleDiv) == 0) {
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
      // 日志限速：只在第 1、10、100、1000... 次失败时打印，
      // 否则每秒 200 条会把串口淹没，真正有用的信息反而看不见
      if (consecutive_fail == 1 || consecutive_fail == 10 || consecutive_fail == 100 ||
          consecutive_fail == 1000 || (consecutive_fail % 5000) == 0) {
        LOG_W("mpu read failed: %s (consecutive=%lu)", common::to_str(r),
              (unsigned long)consecutive_fail);
      }
      // 每帧都尝试恢复（恢复本身只要几十微秒），
      // 但只把恢复动作记进日志、且日志限速 500 ms 一次。
      // 之前把"恢复动作"本身限速到 200 ms，结果出错后 200 ms 内的读全白费，
      // 采样率从 100 Hz 掉到 12 Hz（实测）。限速的对象应该是日志，不是恢复。
      (void)bsp::soft_i2c.recover_bus();
      if ((now - last_recover) >= 500u) {
        last_recover = now;
        LOG_W("i2c read failed, recovering (bus reset every frame)");
      }

      // 传感器卡死后光靠总线恢复救不回来：地址还能 ACK，但寄存器读出来是垃圾。
      // 这时必须重新写一遍 PWR_MGMT_1 / 量程寄存器（= reinit），实测能直接救回，
      // 不用断电重启。放在 30 次（约 0.3 秒）时触发，早于下面的安全停机。
      if (consecutive_fail == 20 || consecutive_fail == 120) {
        const common::Err re = g_mpu.reinit();
        LOG_W("mpu reinit after 30 fails -> %s", common::to_str(re));
        if (re == common::Err::Ok) {
          consecutive_fail = 0;
        }
      }

      // 安全策略：传感器连续失效约 0.5 秒（100 帧）就自动停机。
      // 不这么做会形成正反馈：角度算错 → 舵机全力纠正 → 大电流 → 干扰更大 → 更错
      if (consecutive_fail >= 300 && g_gimbal.enabled()) {
        (void)g_gimbal.stop();
        LOG_E("safety stop: imu failed 300x (3s), gimbal disabled (send X to resume)");
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
      LOG_I("att %luHz samp=%lu sent=%lu drop=%lu rx=%lu bt=%lu | yaw=%d pitch=%d roll=%d servo=%lu/%lu raw=%d,%d,%d",
            (unsigned long)(frames * 1000u / (now - t_start ? (now - t_start) : 1u)),
            (unsigned long)frames, (unsigned long)sent, (unsigned long)dropped,
            (unsigned long)bsp::uart1_rx_count(), (unsigned long)bsp::uart2_rx_count(),
            cd(eu.yaw), cd(eu.pitch), cd(eu.roll),
            (unsigned long)g_servo_yaw.pulse_us(),
            (unsigned long)g_servo_pitch.pulse_us(), (int)g_last_raw.ax, (int)g_last_raw.ay,
            (int)g_last_raw.az);
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
