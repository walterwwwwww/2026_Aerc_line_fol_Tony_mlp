#ifndef APP_PIDTEST_H
#define APP_PIDTEST_H

#include "main.h"

/* 测试模式 */
typedef enum {
    PIDTEST_OFF       = 0,   /* 正常巡线 */
    PIDTEST_SPEED     = 1,   /* 速度环测试 */
    PIDTEST_ANGRATE   = 2,   /* 角速度环测试 */
    PIDTEST_OUTER     = 3,   /* 外环 (偏差→角速度) 测试 */
    PIDTEST_MODE_MAX  = 4
} PidTestMode;

/* 导出变量 */
extern PidTestMode pidtest_mode;
extern int         pidtest_running;      /* 1 = 正在发测试信号 */
extern float       pidtest_setpoint;     /* 当前测试目标值 (给 VOFA 看) */

/* 函数 */
void app_pidtest_init(void);
void app_pidtest_tick(void);             /* 1ms 调用一次 */
int  app_pidtest_is_active(void);        /* 返回 1 则当前在测试模式, START() 要避让 */

#endif