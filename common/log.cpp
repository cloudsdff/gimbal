/**
 * @file    log.cpp
 * @brief   异步日志实现：业务任务入队，Log 任务负责发送
 *
 * 线程安全：
 *   - 生产者只调 vsnprintf（可重入）+ osMessageQueuePut（线程安全），不需要锁
 *   - 消费者只有一个（Log 任务），printf 不会被并发调用
 *   - 队列满时丢弃并计数，绝不阻塞业务任务
 */
#include "log.hpp"

#include <cstdio>
#include <cstring>

#include "bsp_uart.hpp"
#include "cmsis_os2.h"
#include "main.h"      // huart1 / HAL_GetTick()

extern "C" {
extern UART_HandleTypeDef huart1;
}

namespace common {
namespace {

// 单条日志最大长度（含前缀）。
// 96 不够用：加了 bt= 之后统计行到 95 字节，正好顶到边界，
// 一旦再长一点就会被截断 —— 连行尾的 \r\n 一起丢掉，
// 后面的角度帧就接到同一行上，看起来像"串口交织"（实际是截断）。
// 128 留足余量，代价是 12×32 = 384 B RAM。
constexpr std::size_t kSlotSize = 160;
constexpr std::size_t kQueueDepth = 12;   // 12 × 128 = 1536 B RAM

osMessageQueueId_t g_queue = nullptr;
volatile std::uint32_t g_dropped = 0;
volatile std::uint32_t g_sent = 0;

const char* tag_of(LogLevel lv)
{
  static const char* kTag[] = {"D", "I", "W", "E"};
  return kTag[static_cast<int>(lv)];
}

/// 直接写串口（只允许 Log 任务或内核启动前调用）
///
/// 关键：USART1 上还有"角度帧"在走 DMA（app 层以 50 Hz 发送）。
/// 两个写者必须串行，否则日志行会被 A,... 插进中间（实测过两次）。
///
/// 所以这里不自己调 HAL，而是走 bsp::uart1.write()：
/// 那条路径内部会抢 TX 锁、等 DMA 空闲、并置"总线占用"标志，
/// 串行化逻辑只存在一处，不会两边各写一套而漏掉。
void emit(const char* s)
{
  if (s == nullptr) return;
  const std::size_t n = std::strlen(s);
  if (n == 0) return;
  (void)bsp::uart1.write(reinterpret_cast<const std::uint8_t*>(s), n, 100);
}

/// Log 任务：从队列取字符串，慢慢发给串口
void log_task(void* arg)
{
  (void)arg;
  char buf[kSlotSize];
  for (;;) {
    if (osMessageQueueGet(g_queue, buf, nullptr, osWaitForever) == osOK) {
      emit(buf);
      ++g_sent;
    }
  }
}

}  // namespace

void log_init()
{
  if (g_queue != nullptr) return;

  g_queue = osMessageQueueNew(kQueueDepth, kSlotSize, nullptr);

  const osThreadAttr_t attr = {
      .name = "log_task", .stack_size = 512, .priority = osPriorityBelowNormal1};
  (void)osThreadNew(log_task, nullptr, &attr);
}

std::uint32_t log_dropped() { return g_dropped; }
std::uint32_t log_sent() { return g_sent; }

void log_impl(LogLevel lv, const char* fmt, ...)
{
  // 只用一块缓冲：先写前缀，再在剩余空间里格式化正文（省一半栈）
  char line[kSlotSize];
  const int n = std::snprintf(line, sizeof(line), "[%s][%lu] ", tag_of(lv),
                              static_cast<unsigned long>(HAL_GetTick()));
  va_list ap;
  va_start(ap, fmt);
  if (n > 0 && static_cast<std::size_t>(n) < sizeof(line)) {
    std::vsnprintf(line + n, sizeof(line) - static_cast<std::size_t>(n), fmt, ap);
  }
  va_end(ap);

  const std::size_t len = std::strlen(line);
  if (len + 2u < sizeof(line)) {
    line[len] = '\r';
    line[len + 1] = '\n';
    line[len + 2] = '\0';
  }

  // 内核还没跑起来（main 里的初始化日志）：直接打印
  if (g_queue == nullptr || osKernelGetState() != osKernelRunning) {
    emit(line);
    return;
  }

  // 内核运行中：入队，永不阻塞；队列满就丢并计数
  if (osMessageQueuePut(g_queue, line, 0, 0) != osOK) {
    ++g_dropped;
  }
}

}  // namespace common

/* 给 C 代码用的入口（CubeMX 生成的 main.c 是 C） */
extern "C" void common_log_info(const char* fmt, ...)
{
  char body[80];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(body, sizeof(body), fmt, ap);
  va_end(ap);
  ::common::log_impl(::common::LogLevel::Info, "%s", body);
}
// (comment cleaned)
extern "C" void common_log_sync(const char* s)
{
  while (*s != '\0') {
    std::uint8_t ch = static_cast<std::uint8_t>(*s);
    (void)HAL_UART_Transmit(&huart1, &ch, 1, 5);
    ++s;
  }
}