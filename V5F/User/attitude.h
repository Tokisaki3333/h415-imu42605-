#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include <stdint.h>

/* ==================== 姿态解算 + 零偏管理（DMA 中断经 attitude_update 驱动） ==================== */

/* 复位姿态为初始单位四元数 */
void attitude_reset(void);

/* 每帧姿态解算（DMA 中断调用）：
 * gx/gy/gz —— 陀螺原始 LSB；dt_us —— 本帧实测间隔（0=无效用标称）；ts_us —— DRDY 时间戳 */
void attitude_update(int16_t gx, int16_t gy, int16_t gz, uint32_t dt_us, uint64_t ts_us);

/* ==================== 在线零偏跟踪状态（OLED/上层显示） ==================== */
extern volatile float    bias_gx, bias_gy, bias_gz;   /* 当前零偏 LSB */
extern volatile uint8_t  g_v5f_motion;                /* 1=运动 0=静止 */
extern volatile uint32_t s_still_cnt;                /* 最远的一次被采用(用于当前零偏)的静止段距今秒，运动时也持续更新 */
extern volatile uint32_t s_run_frames;               /* 当前连续静止帧数（诊断：确认期 4000=0.5s） */
extern float s_eg_lp_x, s_eg_lp_y, s_eg_lp_z;        /* |eg| 低通值（诊断：静止时应远小于 13.1） */

#endif /* __ATTITUDE_H */
