/**
 * @file    error_code.hpp
 * @brief   全局统一错误码 —— 任务01 工程要求 4（统一错误码，禁止静默失败）
 *
 * 规则：
 *   1. 所有对外 API 返回 Err，不返回 void
 *   2. 数据用输出参数（引用/指针）返回
 *   3. 任何失败路径都必须能返回一个非 Ok 的值，并允许被日志记录
 */
#ifndef COMMON_ERROR_CODE_HPP
#define COMMON_ERROR_CODE_HPP

#include <cstdint>

namespace common {

enum class Err : std::uint8_t {
  Ok = 0,        ///< 成功
  Timeout,       ///< 超时（通信无响应）
  Busy,          ///< 忙（上一次操作未完成 / 重复初始化）
  InvalidParam,  ///< 参数非法（空指针、长度 0、范围外）
  NotInit,       ///< 未初始化就调用
  BusError,      ///< 总线错误（I2C NACK、CAN 发送失败等）
  NoAck,         ///< 设备无应答（I2C 地址不通）
  NotFound,      ///< 探测不到设备 / 资源不存在
  Overflow,      ///< 缓冲区不足
  Unknown        ///< 其他
};

inline const char* to_str(Err e)
{
  switch (e) {
    case Err::Ok:           return "Ok";
    case Err::Timeout:      return "Timeout";
    case Err::Busy:         return "Busy";
    case Err::InvalidParam: return "InvalidParam";
    case Err::NotInit:      return "NotInit";
    case Err::BusError:     return "BusError";
    case Err::NoAck:        return "NoAck";
    case Err::NotFound:     return "NotFound";
    case Err::Overflow:     return "Overflow";
    default:                return "Unknown";
  }
}

inline bool ok(Err e) { return e == Err::Ok; }

}  // namespace common

#endif  // COMMON_ERROR_CODE_HPP
