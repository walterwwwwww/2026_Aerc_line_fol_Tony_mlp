#include "app_common.h"

/*
 * app_adc.c
 * 14 路 SensorBoard ADC 模块 + 转弯计数 + 强制转向
 */

/* ---- 校准数据 ---- */
float adc_beijing[SENSOR_COUNT];
float adc_xian[SENSOR_COUNT];

volatile float sensor_weight[SENSOR_COUNT] = SENSOR_WEIGHT_DEFAULTS;

/* ---- 传感器数据 ---- */
float adc_test[SENSOR_COUNT] = {0};
float adc_sum = 0;
float adc_error_average[10] = {0};
float adc_error = 0, adc_error_l = 0;
int   if_bai[SENSOR_COUNT] = {0};
int   jiaozhun = 0;
int   calibrated = 0;

uint16_t adc_raw[SENSOR_COUNT] = {0};

/* Main-loop SPI worker publishes complete frames. The 1ms ISR only reads the
 * active buffer; it never blocks on SPI. Main cannot overwrite that buffer
 * while the ISR is running, since the ISR preempts main. */
static uint16_t adc_snapshot[2][SENSOR_COUNT];
static volatile uint8_t adc_snapshot_active = 0u;

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
int turn_entry_ms = 10;               /* 进弯确认时间 (ms, VOFA 可调) */
#define FORCE_ERROR_MAG     8.0f   /* 强制转向时的偏差幅值 (可调, 越大转越急) */

/* ---- 每弯配置槽 (VOFA 可调) ----
 *   每个槽: 弯号 + 强制角度 + 出弯时间
 *   弯号=0 表示空槽 (跳过)
 *   角度=0 表示直走 (不强制转向, 只使用该弯的出弯时间)
 */
/* Independent runtime switch. Table data survives enable/disable in RAM. */
volatile int turn_table_enabled = 0;
// int   turn_slot_num[20]   = { 1,  2,  4,  5,  7, 11, 12, 15, 16, 17, 18, 19, 20, 21};
// float turn_slot_angle[20] = {40, 40, 40,  1,  1, -40,-40,-60, 60,-60, 60,-60, 60,-60};
// int   turn_slot_exit[20]  = { 5, 200, 5,  5, 20,   2,  2,  5, 5, 5, 5, 5, 5};   /* 多余初始化为 0 */

int   turn_slot_num[20]   = { 7, 8, 11, 14 };
float turn_slot_angle[20] = {-75, -25, 20, 20};
int   turn_slot_exit[20]  = { 100, 100, 600, 10};   /* 多余初始化为 0 */


/* ---- 导出变量 (给 VOFA+ 观测) ---- */
int   turn_count = 0;               /* 当前第几弯 */
int   turn_forced_flag = 0;         /* 1 = 正在强制转向中 */

/* ---- 内部状态 ---- */
static int turn_in_bend = 0;        /* 是否在弯道中 */
static int turn_entry_cnt = 0;      /* 进弯确认计数 */
static int turn_exit_cnt = 0;       /* 出弯确认计数 */

static float forced_start_angle = 0;
static float forced_target_angle = 0;

static int current_exit_ms = 5;        /* 当前弯的出弯确认时间 (ms), 从配置表读取 */

int app_turn_table_set_enabled(int enabled)
{
    /* Main-loop commands must not race the TIM7 turn-state machine. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (enabled && run != 0)
    {
        __set_PRIMASK(primask);
        return 0;
    }
    turn_table_enabled = enabled ? 1 : 0;
    if (!turn_table_enabled)
    {
        /* Give control back to the sensors, even in an active forced turn.
         * Keep bend count and the entire table; do not create a new bend. */
        turn_forced_flag = 0;
        forced_start_angle = 0;
        forced_target_angle = 0;
        current_exit_ms = 5;
    }
    __set_PRIMASK(primask);
    return 1;
}

/* ============================ SPI 底层 (FROZEN PROTOCOL) ============================ */

/*
 * Frozen/proven 16-bit SPI protocol — must match Sensor_board firmware.
 *
 * READ command: 0x00C8 (200u)
 * READ response: 17 x uint16_t words
 *    word 0      = 0xBEEF magic
 *    word 1      = sequence counter from the sensor board
 *    words 2..15 = 14 x raw ADC values
 *    word 16     = checksum
 * checksum = 0x1357 ^ 0x00C8 ^ frame[0] ^ ... ^ frame[15]
 */

#define IR_SENSOR_CMD_READ              200u
#define IR_SENSOR_READ_MAGIC            0xBEEFu
#define IR_SENSOR_READ_CHECK_SEED       0x1357u
#define IR_SENSOR_READ_FRAME_WORDS      (SENSOR_COUNT + 3u)
#define IR_SENSOR_READ_VALUE_OFFSET     2u
#define IR_SENSOR_READ_CHECK_INDEX      (IR_SENSOR_READ_FRAME_WORDS - 1u)

#define IR_SENSOR_SPI_TIMEOUT_MS        3u
#define IR_SENSOR_CMD_GAP_CYCLES        4000u  /* sensor prepare is ~memcpy+checksum, fast */
#define IR_SENSOR_CS_SETUP_CYCLES       2000u
#define IR_SENSOR_CS_GUARD_CYCLES       2000u
#define IR_SENSOR_RECOVER_GUARD_CYCLES  1000u
#define IR_SENSOR_READ_FAIL_LIMIT       3u
#define IR_SENSOR_FILTER_SAMPLE_COUNT   1u    /* 1 sample keeps ISR ~1.1ms; increase if line noise appears */

#define IR_SENSOR_ADC_MAX_RAW           4095u

/* ---- SPI online / recovery state ---- */
static uint8_t  ir_sensor_online = 0u;
static uint8_t  ir_sensor_consecutive_fail = 0u;
static uint32_t ir_sensor_recover_count = 0u;

/* ---- extern: MX_SPI2_Init lives in spi.c ---- */
extern void MX_SPI2_Init(void);

/* ---- NOP-based delay (for CS timing) ---- */
static void ir_sensor_delay_cycles(uint32_t loops)
{
    volatile uint32_t i;
    for (i = 0u; i < loops; i++)
    {
        __NOP();
    }
}

/* ---- value extraction: trimmed mean when >2 samples, pass-through when 1 ---- */
static uint16_t ir_sensor_filter_value(const uint16_t samples[IR_SENSOR_FILTER_SAMPLE_COUNT])
{
#if IR_SENSOR_FILTER_SAMPLE_COUNT == 1
    (void)samples;
    return samples[0];
#else
    uint32_t sum = 0u;
    uint16_t min_val = samples[0u];
    uint16_t max_val = samples[0u];
    uint8_t i;

    for (i = 0u; i < IR_SENSOR_FILTER_SAMPLE_COUNT; i++)
    {
        uint16_t v = samples[i];
        sum += v;
        if (v < min_val) min_val = v;
        if (v > max_val) max_val = v;
    }

    return (uint16_t)((sum - min_val - max_val) /
                      (IR_SENSOR_FILTER_SAMPLE_COUNT - 2u));
#endif
}

/* ---- XOR checksum verifier ---- */
static uint16_t ir_sensor_read_checksum(const uint16_t *frame)
{
    uint8_t i;
    uint16_t check = IR_SENSOR_READ_CHECK_SEED ^ IR_SENSOR_CMD_READ;
    for (i = 0u; i < IR_SENSOR_READ_CHECK_INDEX; i++)
    {
        check ^= frame[i];
    }
    return check;
}

/* ---- abort + reinit SPI2 after a bad transaction ---- */
static void ir_sensor_spi_recover(void)
{
    HAL_SPI_Abort(&hspi2);
    HAL_SPI_DeInit(&hspi2);
    MX_SPI2_Init();

    ADC_CS_HIGH();
    ir_sensor_delay_cycles(IR_SENSOR_RECOVER_GUARD_CYCLES);

    ir_sensor_recover_count++;
    ir_sensor_online = 0u;
}

/* ---- core read: 5 samples, each is a full 17-word protocol transaction ---- */
static uint8_t ir_sensor_read_raw(uint16_t *out)
{
    HAL_StatusTypeDef ret;
    uint16_t command;
    uint16_t rx_frame[IR_SENSOR_READ_FRAME_WORDS];
    uint16_t rx_value[SENSOR_COUNT][IR_SENSOR_FILTER_SAMPLE_COUNT];
    uint8_t i, sample;

    if (out == 0) return 0u;

    command = IR_SENSOR_CMD_READ;
    memset(rx_value, 0, sizeof(rx_value));

    for (sample = 0u; sample < IR_SENSOR_FILTER_SAMPLE_COUNT; sample++)
    {
        memset(rx_frame, 0, sizeof(rx_frame));

        /* ---- CS timing: guard gap between transactions ---- */
        ADC_CS_HIGH();
        ir_sensor_delay_cycles(IR_SENSOR_CS_GUARD_CYCLES);

        /* ---- assert CS, wait setup ---- */
        ADC_CS_LOW();
        ir_sensor_delay_cycles(IR_SENSOR_CS_SETUP_CYCLES);

        /* ---- send READ command (1 x 16-bit word) ---- */
        ret = HAL_SPI_Transmit(&hspi2,
                               (uint8_t *)&command,
                               1u,
                               IR_SENSOR_SPI_TIMEOUT_MS);

        if (ret != HAL_OK)
        {
            ADC_CS_HIGH();
            ir_sensor_consecutive_fail++;
            if (ir_sensor_consecutive_fail >= IR_SENSOR_READ_FAIL_LIMIT)
            {
                ir_sensor_spi_recover();
            }
            return 0u;
        }

        /* ---- inter-frame gap while slave prepares response ---- */
        ir_sensor_delay_cycles(IR_SENSOR_CMD_GAP_CYCLES);

        /* ---- receive 17-word READ frame ---- */
        ret = HAL_SPI_Receive(&hspi2,
                              (uint8_t *)rx_frame,
                              IR_SENSOR_READ_FRAME_WORDS,
                              IR_SENSOR_SPI_TIMEOUT_MS);

        ADC_CS_HIGH();

        if (ret != HAL_OK)
        {
            ir_sensor_consecutive_fail++;
            if (ir_sensor_consecutive_fail >= IR_SENSOR_READ_FAIL_LIMIT)
            {
                ir_sensor_spi_recover();
            }
            return 0u;
        }

        /* ---- validate magic and checksum ---- */
        if ((rx_frame[0u] != IR_SENSOR_READ_MAGIC) ||
            (rx_frame[IR_SENSOR_READ_CHECK_INDEX] !=
             ir_sensor_read_checksum(rx_frame)))
        {
            ir_sensor_consecutive_fail++;
            if (ir_sensor_consecutive_fail >= IR_SENSOR_READ_FAIL_LIMIT)
            {
                ir_sensor_spi_recover();
            }
            return 0u;
        }

        /* ---- extract 14 ADC values from this sample ---- */
        for (i = 0u; i < SENSOR_COUNT; i++)
        {
            rx_value[i][sample] =
                rx_frame[IR_SENSOR_READ_VALUE_OFFSET + i];

            if (rx_value[i][sample] > IR_SENSOR_ADC_MAX_RAW)
            {
                ir_sensor_consecutive_fail++;
                if (ir_sensor_consecutive_fail >= IR_SENSOR_READ_FAIL_LIMIT)
                {
                    ir_sensor_spi_recover();
                }
                return 0u;
            }
        }
    }

    /* ---- all 5 samples OK: trimmed mean per channel ---- */
    for (i = 0u; i < SENSOR_COUNT; i++)
    {
        out[i] = ir_sensor_filter_value(rx_value[i]);
    }

    ir_sensor_online = 1u;
    ir_sensor_consecutive_fail = 0u;

    return 1u;
}

/* ---- public wrapper: populates the 14-value output array ---- */
static void ir_sensor_read(uint16_t *out)
{
    if (!ir_sensor_read_raw(out))
    {
        /*
         * Failed read: zero-fill so callers get a clean empty state.
         * The caller (app_adc_get) checks the data range against
         * calibration to decide validity — zeros will safely pass
         * through as "no line detected".
         */
        memset(out, 0, SENSOR_COUNT * sizeof(uint16_t));
    }
}

/* ============================ 应用层接口 ============================ */

void app_adc_init(void)
{
    ADC_CS_HIGH();
    ir_sensor_delay_cycles(IR_SENSOR_RECOVER_GUARD_CYCLES);

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        adc_beijing[i] = 60000.0f;
        adc_xian[i]    = 0.0f;
    }

    /* First read: establishes sensor board presence (retries internally) */
    uint16_t dummy[SENSOR_COUNT];
    ir_sensor_read(dummy);
    memcpy(adc_raw, dummy, sizeof(adc_raw));
    memcpy(adc_snapshot[0], dummy, sizeof(dummy));
    memcpy(adc_snapshot[1], dummy, sizeof(dummy));
    adc_snapshot_active = 0u;

    /* 重置转弯计数 */
    turn_count = 0;
    turn_forced_flag = 0;
    turn_in_bend = 0;
    turn_entry_cnt = 0;
    turn_exit_cnt = 0;
}

void app_adc_poll(void)
{
    uint8_t next = adc_snapshot_active ^ 1u;
    ir_sensor_read(adc_snapshot[next]);
    __DMB();
    adc_snapshot_active = next;
}

void app_adc_get(void)
{
    adc_error_l = adc_error;
    adc_sum = 0;

    uint16_t raw[SENSOR_COUNT];
    uint8_t active = adc_snapshot_active;
    memcpy(raw, adc_snapshot[active], sizeof(raw));
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
        calibrated = 0;
        for (int i = 0; i < SENSOR_COUNT; i++)
        {
            adc_xian[i]    = 0.0f;
            adc_beijing[i] = 60000.0f;
        }
    }
    if (anniu_zhuangtai[2][0] == 0 && anniu_zhuangtai[2][1] == 1 && jiaozhun == 1)
    {
        jiaozhun = 0;
        calibrated = 1;
    }

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
    else if (calibrated)
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

        /* 外侧传感器: 左0~3, 右10~13 */
        int outer_hit = (if_bai[0] == 0 || if_bai[1] == 0 || if_bai[2] == 0 || if_bai[3] == 0 ||
                         if_bai[10] == 0 || if_bai[11] == 0 || if_bai[12] == 0 || if_bai[13] == 0);

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
                float outer_weight = dir < 0.0f ? sensor_weight[0] : sensor_weight[SENSOR_COUNT - 1];
                adc_error_average[9] = dir * FORCE_ERROR_MAG * outer_weight;
            }
        }

        /* ============================================================
         *  2) 正常传感器扫描 (非强制转向时)
         * ============================================================ */
        if (!turn_forced_flag && !line_found)
        {
            /* T型路口检测已移除 */
            /* if (if_bai[0] == 0 && if_bai[SENSOR_COUNT - 1] == 0) { ... } */
            {
                for (int i = 0; i < SENSOR_HALF; i++)
                {
                    int j = SENSOR_COUNT - 1 - i;
                    if (if_bai[i] == 0 || if_bai[j] == 0)
                    {
                        line_found = 1;
                        adc_error_average[9] = sensor_pair_error(
                            adc_test[i], adc_test[j], sensor_weight[i], sensor_weight[j],
                            if_bai[i] == 0, if_bai[j] == 0);
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
                if (turn_entry_cnt >= turn_entry_ms)
                {
                    /* 确认进弯, 计数 +1 */
                    turn_in_bend = 1;
                    turn_entry_cnt = 0;
                    turn_exit_cnt = 0;
                    turn_count++;

                    /* 查配置槽: 匹配弯号 → 读取角度 + 出弯时间 */
                    current_exit_ms = 5;   /* 默认值 (未匹配到槽位时) */
                    for (int t = 0; turn_table_enabled && t < TURN_LOOKUP_SLOT_COUNT; t++)
                    {
                        if (turn_slot_num[t] == 0) continue;   /* 空槽位, 跳过 */
                        if (turn_slot_num[t] == turn_count)
                        {
                            current_exit_ms = turn_slot_exit[t];

                            /* 角度 != 0 → 强制转向 */
                            if (turn_slot_angle[t] != 0.0f)
                            {
                                turn_forced_flag = 1;
                                forced_start_angle = jiaoduz;
                                forced_target_angle = turn_slot_angle[t];
                            }
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
            /* ---- 在弯中: 检测出弯 (使用当前弯的 exit_ms) ---- */
            if (!outer_hit && !turn_forced_flag)
            {
                turn_exit_cnt++;
                if (turn_exit_cnt >= current_exit_ms)
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
