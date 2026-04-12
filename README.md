# LineFollower STM32 移植工程

## 1. 工程概述

本工程将原始 LineFollower 项目 (CYT2BL3 / 逐飞科技平台) 移植到 **STM32F407VGT6** 平台,
搭配 **14 路 SensorBoard** 红外传感器阵列。

### 核心原则
- 算法与控制逻辑 **严格保持原样** (PID、状态机、偏差计算、角速度环、路径记忆)
- 仅修改硬件抽象层 (HAL 驱动调用)
- 传感器通道数从 16 → 14 的索引调整属于硬件适配, 不算算法修改

---

## 2. 硬件映射总表

| 功能 | 原始 (CYT2BL3) | STM32F407 | 引脚 |
|------|----------------|-----------|------|
| 左编码器脉冲 | TC_CH19 P08_0 | TIM2_CH1 (TI1ED) | PA0 |
| 左编码器方向 | TC_CH19 P08_1 | GPIO Input | PA1 |
| 右编码器脉冲 | TC_CH07 P02_0 | TIM4_CH1 (TI1ED) | PD12 |
| 右编码器方向 | TC_CH07 P02_1 | GPIO Input | PD13 |
| 左电机正转 | P18_1 PWM | TIM1_CH1 | PA8 |
| 左电机反转 | P18_3 PWM | TIM1_CH2 | PA9 |
| 右电机正转 | P18_7 PWM | TIM1_CH3 | PA10 |
| 右电机反转 | P18_6 PWM | TIM1_CH4 | PA11 |
| 蜂鸣器 | — | TIM3_CH3 | PB0 |
| 风扇 | P06_5 PWM | TIM3_CH4 | PB1 |
| IMU SPI | SPI1 P06_x | SPI1 | PB3/PB4/PB5 |
| IMU CS | SPI1_CS0 | GPIO | PD7 |
| 传感器 SPI | (同SPI1) | SPI2 | PB13/PB14/PB15 |
| 传感器 CS1 | — | GPIO | PD8 |
| 传感器 CS2 | — | GPIO | PD9 |
| 按键1 (启动) | anniu1 P13_0 | BTN1 (上拉) | PD3 |
| 按键2 (校准入) | anniu2 P13_1 | BTN2 (上拉) | PD2 |
| 按键3 (校准出) | anniu3 P13_2 | BTN3 (上拉) | PD1 |
| LED1~LED4 | P12_1/P12_0/P11_2/P11_1 | GPIO | PC12/PC11/PC10/PA15 |
| 调试串口 | UART_0 | USART2 | PD5/PD6 |
| 超声波串口 | — | USART3 | PB10/PB11 |
| 1ms定时器 | PIT_CH1 1000us | TIM7 | 内部 |

---

## 3. 文件结构

```
stm32_proj/
├── stm32_proj.ioc              ← CubeMX 配置文件
├── Core/
│   ├── Inc/
│   │   ├── app_common.h        ← 公共头文件 (替代 zf_common_headfile.h)
│   │   ├── app_encoder.h       ← 编码器
│   │   ├── app_adc.h           ← 14路传感器
│   │   ├── app_motor.h         ← 电机驱动
│   │   ├── app_gyro.h          ← 陀螺仪
│   │   ├── app_button.h        ← 按键/LED
│   │   └── app_control.h       ← 主控制逻辑
│   └── Src/
│       ├── app_encoder.c       ← 编码器实现
│       ├── app_adc.c           ← 14路传感器实现
│       ├── app_motor.c         ← 电机驱动实现
│       ├── app_gyro.c          ← 陀螺仪实现
│       ├── app_button.c        ← 按键/LED 实现
│       ├── app_control.c       ← 控制逻辑 (START + get_speed)
│       ├── main_user_code.c    ← main.c 用户代码片段 (参考用)
│       └── stm32f4xx_it_user.c ← TIM7 中断回调 (参考用)
├── README.md
└── VERIFICATION_CHECKLIST.md
```

### 原始 → 移植文件对照

| 原始文件 | 移植文件 | 说明 |
|---------|---------|------|
| nyy_encoder.c/h | app_encoder.c/h | 逐飞API→HAL TIM外部时钟 |
| nyy_adc.c/h | app_adc.c/h | SPI1 16ch→SPI2 14ch |
| nyy_motor.c/h | app_motor.c/h | 逐飞PWM→HAL TIM1 |
| nyy_gyro.c/h | app_gyro.c/h | 逐飞IMU963RA→SPI1通用 |
| nyy_anniu.c/h | app_button.c/h + app_control.c | 4按键→3按键; 控制逻辑拆分 |
| cm4_isr.c | stm32f4xx_it_user.c | PIT→TIM7回调 |
| main_cm4.c | main_user_code.c | 初始化序列适配 |

---

## 4. 集成步骤

### 步骤 1: CubeMX 生成工程
1. 用 STM32CubeMX 打开 `stm32_proj.ioc`
2. 选择工具链 (CMake / Keil / STM32CubeIDE)
3. 点击 GENERATE CODE

### 步骤 2: 添加源文件
1. 将 `Core/Inc/app_*.h` 复制到生成工程的 `Core/Inc/`
2. 将 `Core/Src/app_*.c` (app_encoder/adc/motor/gyro/button/control) 复制到 `Core/Src/`
3. 在 IDE 中将所有 app_*.c 添加到编译目标

### 步骤 3: 修改 main.c
按照 `main_user_code.c` 中的标记:
1. USER CODE BEGIN Includes → `#include "app_common.h"`
2. USER CODE BEGIN 2 → 初始化序列
3. USER CODE BEGIN 3 → 调试打印
4. USER CODE BEGIN 4 → printf 重定向 + TIM7 回调函数

### 步骤 4: 编译下载调试

---

## 5. 按键复用方案

| 按键 | 功能 | 检测方式 | 对应原始 |
|------|------|---------|---------|
| BTN1 (PD3) | 启动/停止 | 释放瞬间触发 | anniu1 |
| BTN2 (PD2) | 进入校准 | 释放瞬间触发 | anniu2 |
| BTN3 (PD1) | 退出校准 | 释放瞬间触发 | anniu3 |

原始第 4 个按键 (anniu4) 在代码中未使用, 3 个按键完全覆盖所有功能。

按键电平适配: STM32 上拉 (按下=LOW) 在 `app_get_button()` 中取反,
使 `anniu_zhuangtai[][]` 行为与原始 (下拉, 按下=HIGH) 完全一致。

---

## 6. 传感器适配 (16路→14路)

### 布局变化
```
原始 16 路: [0]..[7] | [8]..[15]  配对: (i, 15-i), i=0..7
移植 14 路: [0]..[6] | [7]..[13]  配对: (i, 13-i), i=0..6
```

### 权重
```
原始 8 权重: {0.1, 0.4, 1.2, 1.9, 2.2, 2.5, 3.0, 3.5}
移植 7 权重: {0.3, 1.0, 1.7, 2.1, 2.4, 2.8, 3.5}
```

### 偏差归一化
- 原始: `/= 16`  →  移植: `/= 14`
- 保证 adc_error 数值范围基本一致

### SPI 协议
当前使用通用 16 位 SPI 读取, 如实际 ADC 芯片 (AD7689/ADS7953 等)
需要特定命令字, 修改 `app_adc.c` 中的 `sensor_spi_read16()`。

---

## 7. 陀螺仪芯片适配

当前默认 ICM-20602。如使用其他芯片, 修改:
- `app_gyro.c`: `IMU_REG_*` 寄存器地址, `imu_init_sequence()` 初始化
- `app_gyro.h`: `GYRO_SCALE_FACTOR` (必须保证 oncez 单位为 deg/s)

| 芯片 | GYRO_SCALE_FACTOR (±2000°/s) |
|------|------------------------------|
| IMU963RA (原始) | 7.05 |
| ICM-20602 | 16.4 |
| MPU-6500 | 16.4 |

---

## 8. 已知注意事项

1. **编码器采样率**: 本版本 10ms 平均 vs 原始 1ms 直接输出。
   如需恢复: `app_encoder.c` 中 `enc_cnt >= 10` 改为 `>= 1`, 去掉 `/10`。

2. **电机/编码器方向**: 如实际方向不对, 参见各 .c 文件中的注释调整。

3. **风扇阻塞**: `app_fengshan(1)` 约 24 秒, 不需要可注释。

4. **SPI 传感器协议**: 需根据实际 ADC 芯片修改读取函数。
