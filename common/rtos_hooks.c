/**
 * @file    rtos_hooks.c
 * @brief   FreeRTOS 应用钩子（失败路径测试用）
 *
 * 打开 configCHECK_FOR_STACK_OVERFLOW = 2 之后，任务栈越界会在上下文切换时被检出，
 * 并调用这里的 vApplicationStackOverflowHook。
 *
 * 与"栈踩坏内存后十几秒才以 HardFault 崩掉"相比，这个钩子能在**溢出发生的那一刻**
 * 报出是哪个任务溢出的——这就是"不允许静默失败"在 RTOS 层面的体现。
 */
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

/* 由 C++ 侧提供 */
extern void common_log_sync(const char* s);

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
  (void)xTask;
  /* 注意：这里运行在中断关闭的上下文，不能用带队列的日志，也不能调 RTOS API。
     必须直接写串口，否则连报错都发不出去 —— 那才是真正的静默失败。 */
  common_log_sync("\r\n!!! STACK OVERFLOW in task [");
  common_log_sync(pcTaskName);
  common_log_sync("] !!!\r\n");
  /* 栈已经坏了，继续跑没有意义：停在这里等待调试器或复位 */
  for (;;) {
  }
}
