#include "app_common.h"
#include "app_vofa_cmd.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/*
 * app_vofa_cmd.c
 * VOFA+ 运行时参数调整 — 实现
 *
 * ============================================================================
 *  工作机制
 * ============================================================================
 *   1) USART2 RX DMA 循环接收到环形缓冲区 rx_dma_buf (128 字节)
 *   2) VOFA+ 发一条命令 + '\n' 后 USART2 线空闲 → IDLE 中断触发
 *   3) IDLE 中断里调 rx_collect() 把 DMA 缓冲新数据拷到行缓冲 line_buf
 *   4) 碰到 '\n' 或 '\r' 就标记 cmd_ready = 1
 *   5) 主循环 while(1) 里的 app_vofa_cmd_process() 看到 cmd_ready, 解析命令
 *   6) 解析器在 param_table 里查名字, 直接改对应的全局变量
 *   7) 下一次 TIM7 中断 (最多 1ms 后) 就用新值计算 PID
 *
 * ============================================================================
 *  为什么这样设计
 * ============================================================================
 *   - RX 用 DMA: 完全不占 CPU 资源, 不影响 1ms TIM7 控制环
 *   - 用 IDLE 中断而不是字符中断: IDLE 只在一串字符结束时触发一次,
 *     不会像字符中断那样每收 1 字节就中断一次
 *   - 解析放主循环: strtof + snprintf 太慢 (几十到几百 μs),
 *     放中断里会导致 TIM7 错过周期, 放主循环不影响控制环
 *   - 命令回显用阻塞 TX: 量少 (<100 字节), 每次仅在收到命令瞬间触发
 *
 * ============================================================================
 *  中断优先级分配
 * ============================================================================
 *   TIM7 (1ms 控制环)   = 1     最高, 不能被抢占
 *   SPI1/2 DMA          = 2, 3
 *   USART2 (命令接收)   = 7     较低, 低于控制
 *   DMA1 Stream5 (RX)   = 7     环形模式很少触发
 *   DMA1 Stream6 (TX)   = 7     (在 app_vofa.c 里配置)
 *
 * ============================================================================
 *  协议细节
 * ============================================================================
 *   - 命令必须以 '\n' 或 '\r' 结尾 (任意一个都行)
 *   - 名字大小写不敏感 ("p_a" 和 "P_A" 等价)
 *   - 数值: float 用 "15.5" 或 "1.5e1", int 用 "15"
 *   - 单行最大 63 字节 (LINE_BUF_SIZE - 1)
 *   - 未识别的名字返回 "[ERR] unknown '...'"
 *   - 名字能识别但格式错误时, strtof/strtol 会返回 0, 参数会被置为 0
 */

/* ---- DMA RX 句柄 ---- */
static DMA_HandleTypeDef hdma_usart2_rx;

/* ---- 接收缓冲 ---- */
#define RX_DMA_BUF_SIZE   128   /* DMA 环形接收缓冲 */
#define LINE_BUF_SIZE     64    /* 单条命令最大长度 */

static uint8_t  rx_dma_buf[RX_DMA_BUF_SIZE];   /* DMA 直接写入这里 */
static uint16_t rx_dma_tail = 0;               /* 上次已处理的位置 */

static char     line_buf[LINE_BUF_SIZE];       /* 组装中的命令行 */
static uint16_t line_len = 0;
static volatile uint8_t cmd_ready = 0;          /* 有完整命令待解析 */

/* 用于回显 */
static char tx_echo_buf[96];


/* ============================ 工具函数 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送一段字符串到 USART2 (用于命令回显)
// 参数说明     s: 以 '\0' 结尾的字符串
// 备注信息     阻塞发送, 只在收到命令后触发, 不影响正常控制
//              发送前会等 VOFA TX DMA 空闲, 避免和波形发送冲突
//              不用 DMA 是因为量太少 (<100字节), 阻塞更简单
//-------------------------------------------------------------------------------------------------------------------
static void vofa_uart_echo(const char *s)
{
    /* 等上一次 DMA 发送完 (VOFA 发波形用的是 DMA) */
    uint32_t timeout = 2000;
    while ((huart2.gState != HAL_UART_STATE_READY) && --timeout) { }
    if (timeout == 0) return;

    HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串比较 (忽略大小写)
// 返回         0 相等, <0 a<b, >0 a>b
//-------------------------------------------------------------------------------------------------------------------
static int stricmp_local(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = (char)tolower((unsigned char)*a);
        char cb = (char)tolower((unsigned char)*b);
        if (ca != cb) return ca - cb;
        a++; b++;
    }
    return (int)(unsigned char)*a - (int)(unsigned char)*b;
}


/* ============================ 参数表 ============================ */

/*
 * 参数表结构:
 *   添加新参数只需在 param_table[] 里加一行,
 *   同时在上面 extern 声明对应的全局变量
 */

typedef enum { T_FLOAT, T_INT } ParamType;

typedef struct {
    const char *name;      /* 命令里用的名字 (大小写不敏感) */
    ParamType   type;      /* T_FLOAT 或 T_INT */
    volatile void *addr;   /* Parameters may also be read by the control ISR. */
    const char *desc;      /* 简短描述 (用于 ? 命令回显) */
} ParamEntry;

/* ---- 外部全局变量声明 ---- */
/*
 * 这些全局变量分别定义在 app_motor.c / app_gyro.c / app_control.c
 * 我们在这里通过地址暴露给命令行
 */

/* app_gyro.c */
extern float P_A, I_A;
extern int   gyro_duty_limit;

/* app_motor.c */
extern float KP_L, KI_L, KD_L;
extern float KP_R, KI_R, KD_R;
extern int   motor_pid_limit;
extern int   motor_pwm_limit;
extern int   motor_dead_zone;
extern int   speed_;

/* app_control.c */
extern float P_tobeA, D_tobeA;
extern int   base_speed;
extern int   run;
extern int   max_time;

/* app_adc.c (转弯配置) */
extern int   turn_entry_ms;
extern int   turn_slot_num[20];
extern float turn_slot_angle[20];
extern int   turn_slot_exit[20];

/* ---- 参数映射表 ---- */
static const ParamEntry param_table[] = {
    /* ========= 外环 (方向环) PD ========= */
    { "P_tobeA",    T_FLOAT, &P_tobeA,          "outer P"     },
    { "D_tobeA",    T_FLOAT, &D_tobeA,          "outer D"     },

    /* Independent magnitudes for software sensor indices 0..13. */
    { "weight1",  T_FLOAT, &sensor_weight[0],  "sensor 1 weight" },
    { "weight2",  T_FLOAT, &sensor_weight[1],  "sensor 2 weight" },
    { "weight3",  T_FLOAT, &sensor_weight[2],  "sensor 3 weight" },
    { "weight4",  T_FLOAT, &sensor_weight[3],  "sensor 4 weight" },
    { "weight5",  T_FLOAT, &sensor_weight[4],  "sensor 5 weight" },
    { "weight6",  T_FLOAT, &sensor_weight[5],  "sensor 6 weight" },
    { "weight7",  T_FLOAT, &sensor_weight[6],  "sensor 7 weight" },
    { "weight8",  T_FLOAT, &sensor_weight[7],  "sensor 8 weight" },
    { "weight9",  T_FLOAT, &sensor_weight[8],  "sensor 9 weight" },
    { "weight10", T_FLOAT, &sensor_weight[9],  "sensor 10 weight" },
    { "weight11", T_FLOAT, &sensor_weight[10], "sensor 11 weight" },
    { "weight12", T_FLOAT, &sensor_weight[11], "sensor 12 weight" },
    { "weight13", T_FLOAT, &sensor_weight[12], "sensor 13 weight" },
    { "weight14", T_FLOAT, &sensor_weight[13], "sensor 14 weight" },

    /* ========= 中环 (角速度环) PI ========= */
    { "P_A",        T_FLOAT, &P_A,              "gyro P"      },
    { "I_A",        T_FLOAT, &I_A,              "gyro I"      },

    /* ========= 内环 (速度环) PID ========= */
    { "KP_L",       T_FLOAT, &KP_L,             "speed KP"    },
    { "KI_L",       T_FLOAT, &KI_L,             "speed KI"    },
    { "KD_L",       T_FLOAT, &KD_L,             "speed KD"    },
    { "KP_R",       T_FLOAT, &KP_R,             "(unused)"    },
    { "KI_R",       T_FLOAT, &KI_R,             "(unused)"    },
    { "KD_R",       T_FLOAT, &KD_R,             "(unused)"    },

    /* ========= 速度设定 ========= */
    { "base_speed", T_INT,   &base_speed,       "cruise spd"  },
    { "speed",      T_INT,   &speed_,           "current spd" },

    /* ========= 限幅参数 ========= */
    { "pid_limit",  T_INT,   &motor_pid_limit,  "PID step lim"},
    { "pwm_limit",  T_INT,   &motor_pwm_limit,  "PWM limit"   },
    { "dead_zone",  T_INT,   &motor_dead_zone,  "dead zone"   },
    { "duty_limit", T_INT,   &gyro_duty_limit,  "duty lim"    },

    /* ========= 运行控制 ========= */
    { "run",        T_INT,   &run,              "0=stop 1=go" },
    { "max_time",   T_INT,   &max_time,         "auto stop ms"},

    /* ========= 转弯配置 ========= */
    { "entry_ms",   T_INT,   &turn_entry_ms,    "turn entry ms"},
    { "turn1",      T_INT,   &turn_slot_num[0], "slot1 turn#"},
    { "turn2",      T_INT,   &turn_slot_num[1], "slot2 turn#"},
    { "turn3",      T_INT,   &turn_slot_num[2], "slot3 turn#"},
    { "turn4",      T_INT,   &turn_slot_num[3], "slot4 turn#"},
    { "turn5",      T_INT,   &turn_slot_num[4], "slot5 turn#"},
    { "turn6",      T_INT,   &turn_slot_num[5], "slot6 turn#"},
    { "turn7",      T_INT,   &turn_slot_num[6], "slot7 turn#"},
    { "turn8",      T_INT,   &turn_slot_num[7], "slot8 turn#"},
    { "angle1",     T_FLOAT, &turn_slot_angle[0],"slot1 angle"},
    { "angle2",     T_FLOAT, &turn_slot_angle[1],"slot2 angle"},
    { "angle3",     T_FLOAT, &turn_slot_angle[2],"slot3 angle"},
    { "angle4",     T_FLOAT, &turn_slot_angle[3],"slot4 angle"},
    { "angle5",     T_FLOAT, &turn_slot_angle[4],"slot5 angle"},
    { "angle6",     T_FLOAT, &turn_slot_angle[5],"slot6 angle"},
    { "angle7",     T_FLOAT, &turn_slot_angle[6],"slot7 angle"},
    { "angle8",     T_FLOAT, &turn_slot_angle[7],"slot8 angle"},
    { "exit1",      T_INT,   &turn_slot_exit[0], "slot1 exit ms"},
    { "exit2",      T_INT,   &turn_slot_exit[1], "slot2 exit ms"},
    { "exit3",      T_INT,   &turn_slot_exit[2], "slot3 exit ms"},
    { "exit4",      T_INT,   &turn_slot_exit[3], "slot4 exit ms"},
    { "exit5",      T_INT,   &turn_slot_exit[4], "slot5 exit ms"},
    { "exit6",      T_INT,   &turn_slot_exit[5], "slot6 exit ms"},
    { "exit7",      T_INT,   &turn_slot_exit[6], "slot7 exit ms"},
    { "exit8",      T_INT,   &turn_slot_exit[7], "slot8 exit ms"},
    { "turn9",      T_INT,   &turn_slot_num[8], "slot9 turn#"},
    { "turn10",     T_INT,   &turn_slot_num[9], "slot10 turn#"},
    { "turn11",     T_INT,   &turn_slot_num[10],"slot11 turn#"},
    { "turn12",     T_INT,   &turn_slot_num[11],"slot12 turn#"},
    { "turn13",     T_INT,   &turn_slot_num[12],"slot13 turn#"},
    { "turn14",     T_INT,   &turn_slot_num[13],"slot14 turn#"},
    { "turn15",     T_INT,   &turn_slot_num[14],"slot15 turn#"},
    { "turn16",     T_INT,   &turn_slot_num[15],"slot16 turn#"},
    { "turn17",     T_INT,   &turn_slot_num[16],"slot17 turn#"},
    { "turn18",     T_INT,   &turn_slot_num[17],"slot18 turn#"},
    { "turn19",     T_INT,   &turn_slot_num[18],"slot19 turn#"},
    { "turn20",     T_INT,   &turn_slot_num[19],"slot20 turn#"},
    { "angle9",     T_FLOAT, &turn_slot_angle[8],"slot9 angle"},
    { "angle10",    T_FLOAT, &turn_slot_angle[9],"slot10 angle"},
    { "angle11",    T_FLOAT, &turn_slot_angle[10],"slot11 angle"},
    { "angle12",    T_FLOAT, &turn_slot_angle[11],"slot12 angle"},
    { "angle13",    T_FLOAT, &turn_slot_angle[12],"slot13 angle"},
    { "angle14",    T_FLOAT, &turn_slot_angle[13],"slot14 angle"},
    { "angle15",    T_FLOAT, &turn_slot_angle[14],"slot15 angle"},
    { "angle16",    T_FLOAT, &turn_slot_angle[15],"slot16 angle"},
    { "angle17",    T_FLOAT, &turn_slot_angle[16],"slot17 angle"},
    { "angle18",    T_FLOAT, &turn_slot_angle[17],"slot18 angle"},
    { "angle19",    T_FLOAT, &turn_slot_angle[18],"slot19 angle"},
    { "angle20",    T_FLOAT, &turn_slot_angle[19],"slot20 angle"},
    { "exit9",      T_INT,   &turn_slot_exit[8], "slot9 exit ms"},
    { "exit10",     T_INT,   &turn_slot_exit[9], "slot10 exit ms"},
    { "exit11",     T_INT,   &turn_slot_exit[10],"slot11 exit ms"},
    { "exit12",     T_INT,   &turn_slot_exit[11],"slot12 exit ms"},
    { "exit13",     T_INT,   &turn_slot_exit[12],"slot13 exit ms"},
    { "exit14",     T_INT,   &turn_slot_exit[13],"slot14 exit ms"},
    { "exit15",     T_INT,   &turn_slot_exit[14],"slot15 exit ms"},
    { "exit16",     T_INT,   &turn_slot_exit[15],"slot16 exit ms"},
    { "exit17",     T_INT,   &turn_slot_exit[16],"slot17 exit ms"},
    { "exit18",     T_INT,   &turn_slot_exit[17],"slot18 exit ms"},
    { "exit19",     T_INT,   &turn_slot_exit[18],"slot19 exit ms"},
    { "exit20",     T_INT,   &turn_slot_exit[19],"slot20 exit ms"},
};
#define PARAM_TABLE_SIZE (sizeof(param_table) / sizeof(param_table[0]))


/* ============================ 命令处理 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     打印所有参数 (响应 "?" 命令)
// 备注信息     遍历 param_table, 逐行发送 "NAME = VALUE  // desc\r\n"
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_cmd_dump_params(void)
{
    vofa_uart_echo("\r\n===== Params =====\r\n");
    for (int i = 0; i < (int)PARAM_TABLE_SIZE; i++)
    {
        const ParamEntry *p = &param_table[i];
        if (p->type == T_FLOAT)
        {
            snprintf(tx_echo_buf, sizeof(tx_echo_buf),
                     "%-11s = %-10.3f  // %s\r\n",
                     p->name, *(volatile float *)p->addr, p->desc);
        }
        else
        {
            snprintf(tx_echo_buf, sizeof(tx_echo_buf),
                     "%-11s = %-10d  // %s\r\n",
                     p->name, *(volatile int *)p->addr, p->desc);
        }
        vofa_uart_echo(tx_echo_buf);
    }
    vofa_uart_echo("==================\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     紧急停车 (响应 "!" 命令)
// 备注信息     run=0 + 清零所有积分和 PWM 输出, 防止积分残留
//-------------------------------------------------------------------------------------------------------------------
static void emergency_stop(void)
{
    run           = 0;
    speed_        = 0;
    jiaosudutobe  = 0;
    jiaosuderror  = 0;
    jiaosuderrorl = 0;
    duty          = 0;
    pwm_          = 0;
    pwml          = 0;
    pwmr          = 0;
    L_err[0] = L_err[1] = L_err[2] = 0;
    L_sum = 0;
    vofa_uart_echo("\r\n[STOP] emergency stop\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析一行命令
// 参数说明     line: 命令字符串 (可能含前后空白), 解析过程会原地修改
// 备注信息     支持三种格式:
//                 NAME=VALUE    设置参数
//                 ?             查询所有参数
//                 !             紧急停车
//-------------------------------------------------------------------------------------------------------------------
static void parse_line(char *line)
{
    /* 去掉前后空白/换行 */
    while (*line == ' ' || *line == '\t' || *line == '\r') line++;
    int len = (int)strlen(line);
    while (len > 0 && (line[len-1] == ' ' || line[len-1] == '\r' ||
                       line[len-1] == '\t' || line[len-1] == '\n'))
    {
        line[--len] = 0;
    }
    if (len == 0) return;

    /* ---- 特殊命令: ? 和 ! ---- */
    if (line[0] == '?' && line[1] == 0)
    {
        app_vofa_cmd_dump_params();
        return;
    }
    if (line[0] == '!' && line[1] == 0)
    {
        emergency_stop();
        return;
    }

    /* ---- 一般命令: NAME=VALUE ---- */
    char *eq = strchr(line, '=');
    if (!eq)
    {
        vofa_uart_echo("\r\n[ERR] expect NAME=VALUE or ? or !\r\n");
        return;
    }
    *eq = 0;
    char *name  = line;
    char *value = eq + 1;

    /* 查表 */
    for (int i = 0; i < (int)PARAM_TABLE_SIZE; i++)
    {
        const ParamEntry *p = &param_table[i];
        if (stricmp_local(name, p->name) == 0)
        {
            if (p->type == T_FLOAT)
            {
                char *end = NULL;
                float v = strtof(value, &end);
                if (strncmp(p->name, "weight", 6) == 0)
                {
                    while (end && isspace((unsigned char)*end)) end++;
                    if (end == value || !end || *end || !isfinite(v) ||
                        v < 0.0f || v > SENSOR_WEIGHT_MAX)
                    {
                        vofa_uart_echo("\r\n[ERR] sensor weight must be a finite number in 0..100\r\n");
                        return;
                    }
                }
                *(volatile float *)p->addr = v;
                snprintf(tx_echo_buf, sizeof(tx_echo_buf),
                         "\r\n[OK] %s = %.3f\r\n", p->name, v);
            }
            else
            {
                int v = (int)strtol(value, NULL, 10);
                *(volatile int *)p->addr = v;
                snprintf(tx_echo_buf, sizeof(tx_echo_buf),
                         "\r\n[OK] %s = %d\r\n", p->name, v);
            }
            vofa_uart_echo(tx_echo_buf);
            return;
        }
    }

    /* 没找到 */
    snprintf(tx_echo_buf, sizeof(tx_echo_buf),
             "\r\n[ERR] unknown '%s' (type ? to list)\r\n", name);
    vofa_uart_echo(tx_echo_buf);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     主循环处理函数 — 在 main() 的 while(1) 里定期调用
// 备注信息     ⚠️ 绝对不能放在 TIM7 中断里! strtof / snprintf 太慢
//              典型耗时 50~100μs, 完全能满足 1ms TIM7 周期之外
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_cmd_process(void)
{
    if (!cmd_ready) return;

    /* 备份一份本地再解析, 避免解析过程中新命令到达覆盖 */
    char local[LINE_BUF_SIZE];
    __disable_irq();
    strncpy(local, line_buf, LINE_BUF_SIZE);
    local[LINE_BUF_SIZE - 1] = 0;
    line_len  = 0;
    cmd_ready = 0;
    __enable_irq();

    parse_line(local);
}


/* ============================ DMA + IDLE ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从 DMA 环形缓冲中收取新数据, 按 '\n' 切分组装到 line_buf
// 备注信息     在 USART2 IDLE 中断里调用
//              DMA head 通过 NDTR 计算: head = BUF_SIZE - NDTR
//              tail 是我们自己维护的 "上次已处理位置"
//-------------------------------------------------------------------------------------------------------------------
static void rx_collect(void)
{
    /* DMA 写入位置 = 缓冲大小 - 剩余待写入字节数 */
    uint16_t head = RX_DMA_BUF_SIZE - (uint16_t)__HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    while (rx_dma_tail != head)
    {
        char ch = (char)rx_dma_buf[rx_dma_tail];
        rx_dma_tail = (rx_dma_tail + 1) % RX_DMA_BUF_SIZE;

        /* 遇到行结束符 */
        if (ch == '\n' || ch == '\r')
        {
            if (line_len > 0)
            {
                line_buf[line_len] = 0;
                cmd_ready = 1;
                /* 如果已有命令未处理, 后续字符在下次 ready 清零后接着来 */
                return;
            }
            /* 空行忽略 */
            continue;
        }

        /* 普通字符 */
        if (line_len < LINE_BUF_SIZE - 1)
        {
            line_buf[line_len++] = ch;
        }
        else
        {
            /* 单行过长, 丢弃重来 */
            line_len = 0;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     USART2 IRQ 里的 IDLE 检测 (由 stm32f4xx_it.c 调用)
// 备注信息     CubeMX 生成的 USART2_IRQHandler 只处理 RXNE/TXE/TC 等标志,
//              但不处理 IDLE. 我们在它之前手动检测 IDLE 并清除标志.
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_cmd_on_uart_irq(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        rx_collect();
    }
}


/* ============================ 初始化 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 USART2 RX DMA + IDLE 中断
// 备注信息     在 main() 的 USER CODE BEGIN 2 中调用, 必须在 app_vofa_init() 之后
//              (app_vofa_init 初始化了 TX DMA, 我们在这里追加 RX DMA)
//
//              DMA 资源:
//                USART2_TX → DMA1 Stream6 Channel4  (app_vofa.c)
//                USART2_RX → DMA1 Stream5 Channel4  (本文件)
//-------------------------------------------------------------------------------------------------------------------
void app_vofa_cmd_init(void)
{
    line_len    = 0;
    cmd_ready   = 0;
    rx_dma_tail = 0;

    /* ---- 配置 USART2 RX DMA: DMA1_Stream5_Channel4 ---- */
    hdma_usart2_rx.Instance                 = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode                = DMA_CIRCULAR;   /* 环形模式: 永不停 */
    hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_usart2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_Init(&hdma_usart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    /* ---- 使能 DMA1_Stream5 中断 ---- */
    /* 环形模式下正常不会触发 TC, 但 HAL 内部处理 Half/Complete 需要 */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* ---- 启动 DMA 接收 (环形, 永不停止) ---- */
    HAL_UART_Receive_DMA(&huart2, rx_dma_buf, RX_DMA_BUF_SIZE);

    /* ---- 使能 IDLE 中断 (USART2_IRQn 已由 CubeMX 使能) ---- */
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    /* 欢迎信息 */
    HAL_Delay(5);
    vofa_uart_echo("\r\n[VOFA CMD] ready. type '?' to list params.\r\n");
}


/* ============================ DMA RX 中断转发 ============================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     DMA1 Stream5 中断处理 (USART2 RX DMA)
// 备注信息     CubeMX 不自动生成这个 IRQHandler (因为默认没用 RX DMA),
//              HAL 库弱符号机制保证不会冲突.
//              环形模式下很少触发, 仅在缓冲区写满一圈时进
//-------------------------------------------------------------------------------------------------------------------
void DMA1_Stream5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
