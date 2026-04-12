#include "app_common.h"

/*
 * app_control.c
 * 主控制逻辑 — START() 状态机 + get_speed() 期望计算
 *
 * ⚠️ 核心算法严格复现, 不做任何改动
 * 仅进行以下替换:
 *   nyy_LED_set()    → app_LED_set()
 *   nyy_motor_get()  → app_motor_get()
 *   nyy_motor_out()  → app_motor_out()
 *   fengshan()       → app_fengshan()
 *   全局变量名 time  → time_cnt (避免与 <time.h> 冲突)
 */

/* ---- 控制变量 (与原始完全一致) ---- */
int run = 0;
int time_cnt = 0;           // 原名 "time", 改名避免冲突
int max_time = 10500;

int chusaidaojuli = 0, chusaidao = 0;

float P_tobeA = 500, D_tobeA = 1500;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主状态机
// 参数说明     mode: 0=正常模式 (当前只用 mode=0)
// 备注信息     每 1ms 由 TIM7 中断调用
//              算法逻辑与原始 START(0) 完全一致
//-------------------------------------------------------------------------------------------------------------------
void app_START(int mode)
{
    /* 按键1 状态 LED 指示 */
    if (anniu_zhuangtai[0][0] == 1) app_LED_set(2, 0);   // 按下时灭
    if (anniu_zhuangtai[0][0] == 0) app_LED_set(2, 1);   // 释放时亮

    if (mode == 0)
    {
        /* ---- 按键1 下降沿: 启动 ---- */
        if (anniu_zhuangtai[0][0] == 0 && anniu_zhuangtai[0][1] == 1 && run == 0)
        {
            run = 1;
            time_cnt = 0;
            chusaidao = 0;
            // app_fengshan(1);  // 暂时不用重新启动风扇 (与原始注释一致)
        }

        if (run == 1)   /* ---- 运行中 ---- */
        {
            time_cnt++;
            path_menmory();

            if (time_cnt < 3000)            /* 延迟发车 */
            {
                jiaosudutobe = 0;
                speed_ = 0;
                angle_ring(1);
                app_motor_get(1);
                app_motor_out(1);
            }
            else if (time_cnt < max_time)   /* 正常循迹 */
            {
                app_get_speed();
                angle_ring(1);
                app_motor_get(1);
                app_motor_out(1);
            }
            else                            /* 定时停车 */
            {
                run = 0;
                time_cnt = 0;
            }
        }
        else /* run == 0 */   /* ---- 停车状态 ---- */
        {
            if (jiaozhun == 1)
            {
                speed_ = 0;
                jiaosudutobe = 0;
                angle_ring(0);
                app_motor_get(0);
                app_motor_out(0);
            }
            else
            {
                app_get_speed();
                speed_ = 0;
                angle_ring(1);
                app_motor_get(0);
                app_motor_out(1);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算期望角速度和期望速度
// 备注信息     算法与原始 get_speed() 完全一致
//              adc_error → PD 控制 → jiaosudutobe (期望角速度)
//              出赛道检测: adc_error 持续为 0 → 停车
//-------------------------------------------------------------------------------------------------------------------
void app_get_speed(void)
{
    if (adc_error == 0)
    {
        if (chusaidaojuli < 100000)
            chusaidaojuli += speedl + speedr;
        if (chusaidaojuli >= 100000)
            chusaidao = 1;
    }
    else
    {
        chusaidaojuli = 0;

        float R = (P_tobeA * adc_error + D_tobeA * (adc_error - adc_error_l));
        if (run == 0)
            jiaosudutobe = R * (speedl + speedr) / 300.0f;
        else
            jiaosudutobe = R * (speed_ + speed_) / 300.0f;

        speed_ = 50;
    }

    if (chusaidao == 1)
    {
        speed_ = 0;
        jiaosudutobe = 0;
        // app_fengshan(0);  // 与原始注释一致
    }
}