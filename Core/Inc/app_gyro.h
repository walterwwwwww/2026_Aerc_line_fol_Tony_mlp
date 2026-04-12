#ifndef APP_GYRO_H
#define APP_GYRO_H

/*
 * app_gyro.h
 * 陀螺仪 (IMU) 模块头文件 — ICM-20948 版本
 *
 * 硬件映射:
 *   SPI1 (PB3/PB4/PB5) Master, CS = PD7 (软件控制)
 *
 * 与原始代码差异:
 *   原始: imu963ra_init() / imu963ra_get_gyro() / imu963ra_gyro_z (逐飞库)
 *   现在: 通过 SPI1 直接读写 ICM-20948 寄存器
 *
 * ICM-20948 vs ICM-20602 关键差异:
 *   - 寄存器分 4 个 Bank (0~3), 通过 REG_BANK_SEL (0x7F) 切换
 *   - WHO_AM_I = 0x00, 期望值 = 0xEA
 *   - 陀螺仪数据在 Bank 0: GYRO_ZOUT_H = 0x37
 *   - 陀螺仪配置在 Bank 2: GYRO_CONFIG_1 = 0x01
 *
 * 陀螺仪缩放因子:
 *   原始代码: oncez = (float)(imu963ra_gyro_z) / 7.05
 *   IMU963RA 的灵敏度约 7.05 LSB/(°/s) 在 ±2000°/s 量程
 *   ICM-20948 ±2000°/s: 灵敏度 = 16.4 LSB/(°/s)
 *   → 必须改为 16.4 才能保证 oncez 单位 (°/s) 正确
 */

#include "main.h"

/* ---- IMU 缩放因子 (ICM-20948 ±2000°/s) ---- */
#define GYRO_SCALE_FACTOR   16.4f

/* ---- 导出变量 (与原始代码完全一致) ---- */
extern float jiaoduz, jiaodux, jiaoduy;         // 积分角度 Z/X/Y (°)
extern float jiaoduz_l, jiaodux_l, jiaoduy_l;   // 上一次角度
extern float wenpiao;                            // 零漂
extern float oncez;                              // 单次角速度读数 (°/s, 含零漂)
extern float jiaosudu;                           // 去零漂角速度 (°/s)

/* 路径记忆 */
extern float length, once_L;
extern float now_x, now_y;
extern int   num_coord;
extern int   coord[200][2];
extern float pi;

/* 角速度环 */
extern float jiaosuderror, jiaosuderrorl;
extern float jiaosudutobe, duty;
extern float P_A, I_A;

/* ---- 函数声明 ---- */
void  app_gyro_init(void);              // IMU 初始化 + 零漂校准
void  app_gyro_get(void);               // 读取角速度 + 积分 (1ms 调用)
void  path_menmory(void);               // 路径记录 (算法不变)
void  angle_ring(int mode);             // 角速度环 (算法不变)

#endif /* APP_GYRO_H */