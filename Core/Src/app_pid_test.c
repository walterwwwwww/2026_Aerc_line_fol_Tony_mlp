/*
 * app_pid_test.c
 * ─────────────────────────────────────────────────────────────────
 * PID 分环测试模块  —— 配合 VOFA+ 在线调参用
 *
 * 思路:
 *   把整个三环串联架构拆开, 一次只让一个环工作, 信号发生器给被测环
 *   注入阶跃/方波/正弦激励, 用 VOFA+ 实时观察跟踪曲线, 对症调参.
 *
 * ─────────────────────────────────────────────────────────────────
 * 集成方法 (3 步, 不改原有逻辑)
 * ─────────────────────────────────────────────────────────────────
 *   1) 把本文件放到 Core/Src/    (同 app_motor.c 同级)
 *
 *   2) 在 app_control.c 的 app_START() 函数体最开头, 加这两行:
 *
 *          extern int  pid_test_mode;
 *          extern void app_pid_test_tick(void);
 *          if (pid_test_mode != 0) { app_pid_test_tick(); return; }
 *
 *      这两行放在 "if (anniu_zhuangtai[0][0] == 1) ..." 之前.
 *      pid_test_mode = 0 时完全不影响原逻辑.
 *
 *   3) (可选, 推荐) 在 app_vofa_cmd.c 的 param_table[] 末尾追加:
 *
 *          extern int pid_test_mode, pid_test_sig, pid_test_amp;
 *          extern int pid_test_offset, pid_test_period;
 *          extern int pid_test_run, pid_test_basespeed;
 *
 *          { "tmode",   T_INT, &pid_test_mode,      "test 0/1/2/3"  },
 *          { "tsig",    T_INT, &pid_test_sig,       "0const1step2sq3sin"},
 *          { "tamp",    T_INT, &pid_test_amp,       "test amp"      },
 *          { "toff",    T_INT, &pid_test_offset,    "test offset"   },
 *          { "tperiod", T_INT, &pid_test_period,    "test period ms"},
 *          { "trun",    T_INT, &pid_test_run,       "0=stop 1=run"  },
 *          { "tbase",   T_INT, &pid_test_basespeed, "mode2 base spd"},
 *
 *      不加也能用, 但只能用调试器或自己加命令.
 *
 * ─────────────────────────────────────────────────────────────────
 * 测试模式
 * ─────────────────────────────────────────────────────────────────
 *   tmode = 0    关闭测试, 回到原 app_START 正常逻辑
 *   tmode = 1    速度环单独测试 (调 KP_L / KI_L / KD_L)
 *   tmode = 2    角速度环单独测试 (调 P_A / I_A)
 *   tmode = 3    开环 PWM 直接输出 (硬件诊断, 不闭环)
 *
 * ─────────────────────────────────────────────────────────────────
 * 信号发生器 (所有模式公用)
 * ─────────────────────────────────────────────────────────────────
 *   tsig = 0   常量      out = offset + amp
 *   tsig = 1   阶跃      out = offset + amp  (启动瞬间从 0 跳到目标)
 *   tsig = 2   方波      在 (offset+amp) 和 (offset-amp) 之间切换
 *                       每 tperiod ms 翻一次, 周期 = 2*tperiod
 *   tsig = 3   正弦      out = offset + amp * sin(2π*t / (2*tperiod))
 *
 * ─────────────────────────────────────────────────────────────────
 * 典型调参 session
 * ─────────────────────────────────────────────────────────────────
 *
 *   ① 调速度环 KP_L  (车放赛道直道)
 *      让 speed_ 在 10 和 30 之间方波
 *        tmode=1  tsig=2  tamp=10  toff=20  tperiod=1500  trun=1
 *      VOFA 看 CH5 (speed_) vs CH6 (speedl) vs CH7 (speedr)
 *      ─ speedl/r 跟得慢 → KP_L 加 50%
 *      ─ speedl/r 高频抖 → KP_L 减 30%
 *      ─ 长期对不齐  → KI_L +0.3
 *
 *   ② 调角速度环 P_A  (车顶起来! 防止画 8 字)
 *      让 jiaosudutobe 在 ±3 之间方波, 前进保持速度 15
 *        tmode=2  tsig=2  tamp=3  toff=0  tperiod=1000  tbase=15  trun=1
 *      VOFA 看 CH1 (期望) vs CH2 (实测) vs CH4 (duty)
 *      ─ CH2 跟不上 CH1 → P_A 加 30%
 *      ─ CH2 振铃 → P_A 减
 *      ─ CH3 长期非 0 → I_A +0.2
 *      ─ CH4 触顶 ±18000 → duty_limit=25000
 *
 *   ③ 硬件诊断  (车顶起来或放短直道)
 *      固定 PWM = 1500 直行, 看左右轮速度对称性
 *        tmode=3  tsig=0  tamp=0  toff=1500  trun=1
 *      VOFA 看 CH6 vs CH7
 *      ─ 差超过 10% → 电机/编码器问题, 不是 PID 问题
 *
 *   ④ 紧急停车            trun=0
 *   ⑤ 退出测试模式回到正常 tmode=0
 *
 * ─────────────────────────────────────────────────────────────────
 * ⚠️ 安全
 *   - 模式 1 (速度环): 车放赛道直道段, 留 2 米以上缓冲
 *   - 模式 2 (角速度环): 强烈建议把车顶起来 (轮子离地), 否则会自己扭
 *   - 模式 3 (开环): 顶起来或人扶住
 *   - 任何时候按硬件 STOP 键, 或串口发 trun=0 立即停车
 *   - 调完务必 tmode=0 切回正常逻辑, 否则下次按启动键不会跑赛道
 * ─────────────────────────────────────────────────────────────────
 */

#include "app_common.h"
#include <math.h>

/* ====================== 外部全局 (来自其他模块) ====================== */
extern int   speed_;            /* app_motor.c — 目标速度 */
extern int   pwm_, pwml, pwmr;  /* app_motor.c — PWM 中间量 */
extern int   motor_pwm_limit;   /* app_motor.c — PWM 限幅 */
extern float duty;              /* app_gyro.c  — 差速量 */
extern float jiaosudutobe;      /* app_gyro.c  — 期望角速度 */
extern void  angle_ring(int mode);    /* app_gyro.c  */
extern void  app_motor_get(int mode); /* app_motor.c */
extern void  app_motor_out(int i);    /* app_motor.c */

/* ====================== 测试控制变量 (VOFA+ 可改) ===================== */
int pid_test_mode      = 0;     /* 0=off  1=speed  2=gyro  3=open-loop */
int pid_test_sig       = 0;     /* 0=const  1=step  2=square  3=sine */
int pid_test_amp       = 20;    /* 信号幅值 */
int pid_test_offset    = 0;     /* 信号偏置 (DC) */
int pid_test_period    = 2000;  /* 方波/正弦半周期 (ms) */
int pid_test_run       = 0;     /* 1=运行  0=停止 */
int pid_test_basespeed = 15;    /* 模式 2 的前进速度 */

/* ====================== 内部状态 ====================== */
static uint32_t pid_test_t = 0; /* 自启动以来累计 ms */

/* ====================== 信号发生器 ====================== */
/* 返回 t 时刻被测信号的瞬时值. t 单位 ms. */
static float pid_test_signal(uint32_t t)
{
    float A   = (float)pid_test_amp;
    float OFF = (float)pid_test_offset;
    int   T   = pid_test_period > 1 ? pid_test_period : 1;

    switch (pid_test_sig)
    {
        case 0:  /* 常量 */
            return OFF + A;

        case 1:  /* 阶跃 (启动后立即到目标值) */
            return OFF + A;

        case 2:  /* 方波 */
            return ((t / (uint32_t)T) % 2u == 0u) ? (OFF + A) : (OFF - A);

        case 3:  /* 正弦 */
        {
            /* 周期 = 2*T (ms), 角速度 ω = π/T */
            float w = 3.14159265f / (float)T;
            return OFF + A * sinf(w * (float)t);
        }

        default:
            return OFF;
    }
}

/* ====================== 安全停车 ====================== */
static void pid_test_safe_stop(void)
{
    speed_       = 0;
    jiaosudutobe = 0;
    duty         = 0;
    pwm_         = 0;
    pwml         = 0;
    pwmr         = 0;
    app_motor_out(0);
    pid_test_t   = 0;
}

/* ====================== 主测试 tick ======================
 * 由 app_START() 在 pid_test_mode != 0 时转发调用, 1ms 一次.
 */
void app_pid_test_tick(void)
{
    /* 任何时候 trun=0 都立刻停车并清零计时 */
    if (pid_test_run == 0)
    {
        pid_test_safe_stop();
        return;
    }

    pid_test_t++;
    float sig = pid_test_signal(pid_test_t);

    switch (pid_test_mode)
    {
        /* ─────────────────────────────────────────────────────
         * 模式 1: 速度环单独测试
         *   注入: speed_ ← signal
         *   关闭: 角速度环 (duty=0, 不调 angle_ring)
         *         位置环 (不调 app_get_speed)
         *   生效环: 速度环 (KP_L / KI_L / KD_L)
         *   观察:  CH5 speed_  vs  CH6 speedl  vs  CH7 speedr
         *          CH8 pwm_    (速度环输出)
         * ─────────────────────────────────────────────────────
         */
        case 1:
        {
            int target = (int)sig;
            /* 限制在 motor_pwm_limit 对应的合理速度区间内, 避免 pwm_ 溢出 */
            if (target >  120) target =  120;
            if (target < -120) target = -120;

            speed_       = target;
            jiaosudutobe = 0;
            duty         = 0;          /* 不打方向, 走直线 */
            app_motor_get(1);          /* 计算 pwm_ (速度环 PID) */
            duty         = 0;          /* 保险: 重置, 防止外部干扰 */
            app_motor_out(1);
            break;
        }

        /* ─────────────────────────────────────────────────────
         * 模式 2: 角速度环单独测试
         *   注入: jiaosudutobe ← signal
         *   保持: speed_ = pid_test_basespeed (恒定前进, 让差速有意义)
         *   关闭: 位置环 (不调 app_get_speed)
         *   生效环: 角速度环 (P_A / I_A) + 速度环 (差速合成)
         *   观察:  CH1 jiaosudutobe  vs  CH2 jiaosudu
         *          CH3 jiaosuderror  CH4 duty
         *
         *   ⚠️ 必须把车顶起来! 否则 jiaosudutobe 信号会让车扭来扭去.
         * ─────────────────────────────────────────────────────
         */
        case 2:
        {
            int bs = pid_test_basespeed;
            if (bs < 12) bs = 12;       /* 防止落入 duty_max=1000 低速保护 */
            if (bs > 60) bs = 60;       /* 安全上限 */

            speed_       = bs;
            jiaosudutobe = sig;
            angle_ring(1);              /* 内环: jiaosudutobe → duty */
            app_motor_get(1);           /* 速度环 + 差速合成 */
            app_motor_out(1);
            break;
        }

        /* ─────────────────────────────────────────────────────
         * 模式 3: 开环 PWM (无 PID)
         *   直接灌定值到电机, 用于硬件检查:
         *     - 左右轮转速对称性
         *     - 编码器方向
         *     - 死区补偿是否合适
         *   左右两路相同 (直行). 信号正负代表正反转.
         *   观察:  CH6 vs CH7 (对称性) ;  CH4 应该 ≈ 0
         * ─────────────────────────────────────────────────────
         */
        case 3:
        {
            int v = (int)sig;
            if (v >  motor_pwm_limit) v =  motor_pwm_limit;
            if (v < -motor_pwm_limit) v = -motor_pwm_limit;

            speed_       = 0;
            jiaosudutobe = 0;
            duty         = 0;
            pwm_         = v;
            pwml         = v;
            pwmr         = v;
            app_motor_out(1);   /* 直接按 pwml/pwmr 输出 (内部还会加 dead_zone) */
            break;
        }

        /* ─────────────────────────────────────────────────────
         * 未知模式: 安全停车并自动退出
         * ─────────────────────────────────────────────────────
         */
        default:
            pid_test_run = 0;
            pid_test_safe_stop();
            break;
    }
}