#include "app_common.h"

/*
 * app_adc.c
 * 14 路 SensorBoard ADC 模块 + 转弯计数 + 强制转向
 */

/* ---- 校准数据 ---- */
float adc_beijing[SENSOR_COUNT];
float adc_xian[SENSOR_COUNT];

float adc_quan[SENSOR_HALF] = {0.0, 1.0, 1.7, 2.1, 2.4, 2.8, 3.5};

/* ---- 传感器数据 ---- */
float adc_test[SENSOR_COUNT] = {0};
float adc_sum = 0;
float adc_error_average[10] = {0};
float adc_error = 0, adc_error_l = 0;
int   if_bai[SENSOR_COUNT] = {0};
int   jiaozhun = 0;

uint16_t adc_raw[SENSOR_COUNT] = {0};

/* ---- 丢线 ---- */
static int lost_line_cnt = 0;
#define LOST_LINE_TIMEOUT  300
#define LOST_LINE_GLITCH   30

/* ============================ 转弯计数 + 强制转向 ============================ */

/*
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║  使用步骤:                                                       ║
 * ║                                                                   ║
 * ║  第一步: 先把 force_table 留空, 跑一圈赛道                        ║
 * ║         用 VOFA+ 看 turn_count (CH10) 确认每个弯的编号            ║
 * ║                                                                   ║
 * ║  第二步: 确认编号后, 在 force_table 里填入要强制转的弯:            ║
 * ║         { 弯号, 角度 }                                            ║
 * ║         角度正数 = 右转, 负数 = 左转                               ║
 * ║         例: { 3, 45.0f }  表示第3弯强制右转45度                   ║
 * ║                                                                   ║
 * ║  第三步: 如果方向反了, 把角度取反                                  ║
 * ║                                                                   ║
 * ║  可调参数:                                                        ║
 * ║    TURN_ENTRY_MS    — 外侧传感器持续多久算进弯 (防误触)           ║
 * ║    TURN_EXIT_MS     — 外侧传感器消失多久算出弯                    ║
 * ║    FORCE_ERROR_MAG  — 强制转向的力度 (越大转越急)                  ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 */

/* ---- 转弯检测参数 ---- */
#define TURN_ENTRY_MS       30      /* 外侧传感器持续 30ms 才算进弯 (防抖) */
#define TURN_EXIT_MS        100     /* 外侧传感器消失 300ms 才算出弯 */
#define FORCE_ERROR_MAG     8.0f   /* 强制转向时的偏差幅值 (可调, 越大转越急) */

/* ---- 强制转向配置表 ---- */
typedef struct {
    int   turn_number;      /* 第几个弯 (从1开始) */
    float target_angle;     /* 强制转过的角度 (正=右转, 负=左转) */
} TurnForceConfig;

static const TurnForceConfig force_table[] = {
    /* ↓↓↓ 在这里添加需要强制转向的弯道 ↓↓↓ */

    // { 3,  45.0f },     /* 第3弯: 强制右转 45° */
    // { 5, -45.0f },     /* 第5弯: 强制左转 45° */
    // { 8,  90.0f },     /* 第8弯: 强制右转 90° */

    { 5,  135.0f },     /* 第5弯: 强制右转 135° */
    { 6,  45.0f },     /* 第6弯: 强制右转 135° */


    /* ↑↑↑ 添加到这里 ↑↑↑ */
};
#define FORCE_TABLE_SIZE  (sizeof(force_table) / sizeof(force_table[0]))

/* ---- 导出变量 (给 VOFA+ 观测) ---- */
int   turn_count = 0;               /* 当前第几弯 */
int   turn_forced_flag = 0;         /* 1 = 正在强制转向中 */

/* ---- 内部状态 ---- */
static int turn_in_bend = 0;        /* 是否在弯道中 */
static int turn_entry_cnt = 0;      /* 进弯确认计数 */
static int turn_exit_cnt = 0;       /* 出弯确认计数 */

static float forced_start_angle = 0;
static float forced_target_angle = 0;

/* ============================ SPI 底层 ============================ */

static void ir_sensor_read(uint16_t *out)
{
    uint16_t cmd = 2000;
    ADC_CS_LOW();
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&cmd, 1, 10);

    /* 给从机留足够时间: 进 ISR + 快照 adc_buf + 装填 TX DR
     * 72MHz 下 ~3000 次空循环 ≈ 40µs, 对你这个轮询率完全不是瓶颈 */
    for (volatile int d = 0; d < 3000; d++);

    HAL_SPI_Receive(&hspi2, (uint8_t *)out, 14, 10);
    ADC_CS_HIGH();
}

/* ============================ 应用层接口 ============================ */

void app_adc_init(void)
{
    ADC_CS_HIGH();
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_beijing[i] = 60000.0f;
        adc_xian[i]    = 0.0f;
    }
    uint16_t dummy[SENSOR_COUNT];
    ir_sensor_read(dummy);

    /* 重置转弯计数 */
    turn_count = 0;
    turn_forced_flag = 0;
    turn_in_bend = 0;
    turn_entry_cnt = 0;
    turn_exit_cnt = 0;
}

void app_adc_get(void)
{
    adc_error_l = adc_error;
    adc_sum = 0;

    uint16_t raw[SENSOR_COUNT];
    ir_sensor_read(raw);
    memcpy(adc_raw, raw, sizeof(adc_raw));

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_test[i] = (float)raw[i];
        adc_sum += adc_test[i];
    }

    /* ---- 校准控制 ---- */
    if (anniu_zhuangtai[1][0] == 0 && anniu_zhuangtai[1][1] == 1 && jiaozhun == 0)
    {
        jiaozhun = 1;
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            adc_xian[i]    = 0.0f;
            adc_beijing[i] = 60000.0f;
        }
    }
    if (anniu_zhuangtai[2][0] == 0 && anniu_zhuangtai[2][1] == 1 && jiaozhun == 1)
        jiaozhun = 0;

    if (jiaozhun == 1) app_LED_set(3, 1);
    if (jiaozhun == 0) app_LED_set(3, 0);

    /* ---- 校准采集 ---- */
    if (jiaozhun == 1)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            if (adc_test[i] > adc_xian[i])    adc_xian[i]    = adc_test[i];
            if (adc_test[i] < adc_beijing[i])  adc_beijing[i] = adc_test[i];
        }
    }
    /* ==================== 正常巡线模式 ==================== */
    else if (adc_beijing[3] != 0)
    {
        /* 归一化到 0~10 */
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            float range = adc_xian[i] - adc_beijing[i];
            if (range > 1.0f)
                adc_test[i] = (adc_test[i] - adc_beijing[i]) / range * 10.0f;
            else
                adc_test[i] = 0.0f;
        }

        /* 滑动平均缓冲区移位 */
        adc_error = 0;
        for (int i = 0; i < 9; i++)
            adc_error_average[i] = adc_error_average[i + 1];
        adc_error_average[9] = 0;

        /* 二值化 */
        for (int i = 0; i < SENSOR_COUNT; i++)
            if_bai[i] = (adc_test[i] <= 6.0f) ? 1 : 0;

        /* 内侧第3/4/5位置传感器是否触发 (任意一侧) */
        int outer_hit = (if_bai[2] == 0 || if_bai[3] == 0 || if_bai[4] == 0 ||
                         if_bai[9] == 0 || if_bai[10] == 0 || if_bai[11] == 0);

        /* ============================================================
         *  1) 强制转向中 → 用陀螺仪转到目标角度, 跳过传感器扫描
         * ============================================================ */
        int line_found = 0;

        if (turn_forced_flag)
        {
            /* 计算已转过的角度 (处理 ±180° 翻转) */
            float turned = jiaoduz - forced_start_angle;
            if (turned > 180.0f)  turned -= 360.0f;
            if (turned < -180.0f) turned += 360.0f;

            /* 判断是否转够 */
            int done = 0;
            if (forced_target_angle > 0 && turned >= forced_target_angle) done = 1;
            if (forced_target_angle < 0 && turned <= forced_target_angle) done = 1;

            if (done)
            {
                turn_forced_flag = 0;   /* 转够了, 交还给传感器巡线 */
            }
            else
            {
                /* 还没转够: 给一个恒定偏差, 驱动车持续转弯 */
                line_found = 1;
                float dir = (forced_target_angle > 0) ? 1.0f : -1.0f;
                adc_error_average[9] = dir * FORCE_ERROR_MAG * adc_quan[SENSOR_HALF - 1];
            }
        }

        /* ============================================================
         *  2) 正常传感器扫描 (非强制转向时)
         * ============================================================ */
        if (!turn_forced_flag && !line_found)
        {
            /* T型路口: 两侧最外同时为黑 → 强制右转 */
            if (if_bai[0] == 0 && if_bai[SENSOR_COUNT - 1] == 0)
            {
                line_found = 1;
                adc_error_average[9] = 10.0f * adc_quan[SENSOR_HALF - 1];
            }
            else
            {
                for (int i = 0; i < SENSOR_HALF; i++)
                {
                    int j = SENSOR_COUNT - 1 - i;
                    if (if_bai[i] == 0 || if_bai[j] == 0)
                    {
                        line_found = 1;
                        if (if_bai[i] == 0 && if_bai[j] == 0)
                            adc_error_average[9] = (adc_test[j] - adc_test[i]) * adc_quan[SENSOR_HALF - 1 - i];
                        else if (if_bai[i] == 0)
                            adc_error_average[9] = -10.0f * adc_quan[SENSOR_HALF - 1 - i];
                        else
                            adc_error_average[9] = 10.0f * adc_quan[SENSOR_HALF - 1 - i];
                        break;
                    }
                }
            }
        }

        /* ============================================================
         *  3) 转弯检测 + 计数 (每周期都跑, 不管是否强制)
         * ============================================================ */
        if (!turn_in_bend)
        {
            /* ---- 未在弯中: 检测进弯 ---- */
            if (outer_hit)
            {
                turn_entry_cnt++;
                if (turn_entry_cnt >= TURN_ENTRY_MS)
                {
                    /* 确认进弯, 计数 +1 */
                    turn_in_bend = 1;
                    turn_entry_cnt = 0;
                    turn_exit_cnt = 0;
                    turn_count++;

                    /* 查表: 这个弯是否需要强制转向 */
                    for (int t = 0; t < (int)FORCE_TABLE_SIZE; t++)
                    {
                        if (force_table[t].turn_number == turn_count)
                        {
                            turn_forced_flag = 1;
                            forced_start_angle = jiaoduz;
                            forced_target_angle = force_table[t].target_angle;
                            break;
                        }
                    }
                }
            }
            else
            {
                turn_entry_cnt = 0;
            }
        }
        else
        {
            /* ---- 在弯中: 检测出弯 ---- */
            if (!outer_hit && !turn_forced_flag)
            {
                turn_exit_cnt++;
                if (turn_exit_cnt >= TURN_EXIT_MS)
                {
                    turn_in_bend = 0;
                    turn_exit_cnt = 0;
                }
            }
            else
            {
                turn_exit_cnt = 0;
            }
        }

        /* ============================================================
         *  4) 丢线处理
         * ============================================================ */
        if (!line_found)
        {
            lost_line_cnt++;
            if (lost_line_cnt < LOST_LINE_TIMEOUT)
                adc_error_average[9] = adc_error_average[8];
            else
                adc_error_average[9] = 0;
        }
        else
        {
            if (lost_line_cnt >= LOST_LINE_GLITCH)
            {
                duty = 0;
                jiaosuderror = 0;
                jiaosuderrorl = 0;
                jiaosudutobe = 0;
                for (int i = 0; i < 9; i++)
                    adc_error_average[i] = adc_error_average[9];
                chusaidaojuli = 0;
                chusaidao = 0;
            }
            lost_line_cnt = 0;

            adc_error_average[9] /= 14.0f;
        }

        /* 10 次滑动平均 */
        for (int i = 0; i < 10; i++)
        {
            adc_error += adc_error_average[i] / 10.0f;
        }
    }
}