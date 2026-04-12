#ifndef APP_CONTROL_H
#define APP_CONTROL_H

/*
 * app_control.h
 * 主控制逻辑头文件 — 包含 START() 状态机和 get_speed() 期望速度计算
 *
 * ⚠️ 核心算法: 本模块逻辑严格复现原始代码, 不得修改
 *   仅将函数名从 START→app_START, get_speed→app_get_speed
 *   内部调用替换为 app_* 系列函数
 */

#include "main.h"

/* ---- 导出变量 (与原始完全一致) ---- */
extern int   run;                    // 运行标志 (0=停车, 1=运行)
extern int   time_cnt;               // 运行时间计数 (ms)  注: 避免与 stdlib time 冲突
extern int   max_time;               // 最大运行时间 (ms)

extern int   chusaidaojuli;          // 出赛道距离累积
extern int   chusaidao;              // 出赛道标志

extern float P_tobeA, D_tobeA;       // 偏差→角速度 PD 参数

/* ---- 函数声明 ---- */
void app_START(int mode);            // 主状态机 (1ms 调用)
void app_get_speed(void);            // 计算期望角速度和期望速度

#endif /* APP_CONTROL_H */
