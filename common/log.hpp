/**
 * @file    log.hpp
 * @brief   统一日志接口（异步：队列 + 低优先级任务）
 *
 * 为什么是异步：
 *   实测一行 33 字节在 115200 下要占 2.9 ms（见 docs/02-UART实测记录.md）。
 *   如果业务任务直接 printf，1 ms 周期的控制任务会被打印拖死。
 *
 * 现在的结构：
 *   业务任务 → vsnprintf 格式化到栈上缓冲 → 入队（永不阻塞）→ Log 任务负责慢慢发串口
 *   队列满时丢日志并计数，绝不阻塞业务。
 */
#ifndef COMMON_LOG_HPP
#define COMMON_LOG_HPP

#include <cstdarg>
#include <cstdint>

#include "error_code.hpp"

namespace common {

enum class LogLevel : std::uint8_t { Debug = 0, Info, Warn, Error };

#ifndef LOG_LEVEL
#define LOG_LEVEL 3
#endif

/// 创建日志队列与 Log 任务（在启动调度器之前调用）
void log_init();

/// 实际入口：格式化 + 入队（内核未运行时退化为同步打印）
void log_impl(LogLevel lv, const char* fmt, ...);

/// 被丢弃的日志条数（队列满）
std::uint32_t log_dropped();

/// ???????????
std::uint32_t log_sent();

}  // namespace common

#if LOG_LEVEL >= 3
#define LOG_D(...) ::common::log_impl(::common::LogLevel::Debug, __VA_ARGS__)
#else
#define LOG_D(...) do {} while (0)
#endif

#if LOG_LEVEL >= 2
#define LOG_I(...) ::common::log_impl(::common::LogLevel::Info, __VA_ARGS__)
#else
#define LOG_I(...) do {} while (0)
#endif

#if LOG_LEVEL >= 1
#define LOG_W(...) ::common::log_impl(::common::LogLevel::Warn, __VA_ARGS__)
#else
#define LOG_W(...) do {} while (0)
#endif

#define LOG_E(...) ::common::log_impl(::common::LogLevel::Error, __VA_ARGS__)

#endif  // COMMON_LOG_HPP
