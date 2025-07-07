#ifndef _TASKSCHEDULER_H
#define _TASKSCHEDULER_H

#include "hal_data.h"

#define TASK_INTERVAL_1MS     1
#define TASK_INTERVAL_5MS     5
#define TASK_INTERVAL_20MS    20
#define TASK_INTERVAL_100MS   100
#define TASK_INTERVAL_500MS   500
#define TASK_INTERVAL_1S      1000

typedef struct {
    void (*taskFunc)(void);       // 函数指针
    uint32_t interval;            // 调度间隔（单位：ms）
    uint32_t lastExecTime;        // 上次执行时间（单位：ms）
} Task_t;

void TaskScheduler(void);

/* RA6M5系统时钟相关 */
extern volatile uint32_t g_system_tick_ms;
uint32_t GetSystemTick(void);
void SystemClock_Init(void);

#endif //_TASKSCHEDULER_H
