#ifndef __INS_H
#define __INS_H

#include <stdint.h>

/* ==================== 惯性预处理器（EKF 前置，传感器级） ====================
 * 原始加速度 → 去零偏 → 旋转到世界 → 去重力 → 线加速度 ins_a（EKF 过程输入）；
 * 静止判定 ins_still（EKF 的 ZUPT 量测门控）。
 * 位置/速度/零偏估计由后续 6 状态 EKF 承担（本模块不做越俎代庖）。 */

void ins_init(void);

/* DMA 中断调用：ax/ay/az 机体加速度原始 LSB；q0..q3 姿态四元数（attitude 提供）；
 * dt_us 本帧间隔；ts_us DRDY 时间戳（暂未用） */
void ins_update(int16_t ax, int16_t ay, int16_t az,
                float q0, float q1, float q2, float q3,
                uint32_t dt_us, uint64_t ts_us);

extern volatile float ins_a[3];     /* 线加速度 m/s²（世界系，EKF 过程输入） */
extern volatile uint8_t ins_still;  /* 1=加速度静止（EKF ZUPT 门控） */
extern volatile float ins_amag;     /* 判据值 |线加速度| 低通 m/s²（诊断） */

#endif /* __INS_H */
