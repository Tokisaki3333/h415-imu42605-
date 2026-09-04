#include "ins.h"
#include "attitude.h"      /* g_v5f_obs.q（姿态四元数） */
#include <math.h>

/* ==================== 标定常数 ====================
 * 假设初始角度向上（世界系 +Z 朝上），重力方向向下 (0,0,-g)，严格 = 9.7985；
 * 零偏 LSB 写死（传感器静态零偏，EKF 后续可再估计）。 */
#define ACC_G_LSB      8192.0f
#define ACC_BIAS_X     78.8f
#define ACC_BIAS_Y     81.4f
#define ACC_BIAS_Z     1.5f
#define ACC_G_MS2      9.7985f

/* 静止判定阈值（EKF ZUPT 量测门控）：|线加速度| 低通 < 此值判静止 */
#define ACC_STILL_A      0.4f

volatile float ins_a[3] = {0, 0, 0};
volatile uint8_t ins_still = 0;
volatile float ins_amag = 0;
static float s_amag_lp = 0;

void ins_init(void)
{
    ins_a[0] = ins_a[1] = ins_a[2] = 0;
    ins_still = 0;
    ins_amag = 0;
    s_amag_lp = 0;
}

void ins_update(int16_t ax, int16_t ay, int16_t az,
                float q0, float q1, float q2, float q3,
                uint32_t dt_us, uint64_t ts_us)
{
    (void)dt_us; (void)ts_us;

    /* 1. 原始数据 - 零偏 → 比力（m/s²） */
    float a_bx = ((float)ax - ACC_BIAS_X) / ACC_G_LSB * ACC_G_MS2;
    float a_by = ((float)ay - ACC_BIAS_Y) / ACC_G_LSB * ACC_G_MS2;
    float a_bz = ((float)az - ACC_BIAS_Z) / ACC_G_LSB * ACC_G_MS2;

    /* 2. 旋转到世界（非共轭，初始 Z 向上） */
    float ax_w = (1.0f - 2.0f*(q2*q2 + q3*q3)) * a_bx
               + 2.0f*(q1*q2 - q0*q3) * a_by
               + 2.0f*(q1*q3 + q0*q2) * a_bz;
    float ay_w = 2.0f*(q1*q2 + q0*q3) * a_bx
               + (1.0f - 2.0f*(q1*q1 + q3*q3)) * a_by
               + 2.0f*(q2*q3 - q0*q1) * a_bz;
    float az_w = 2.0f*(q1*q3 - q0*q2) * a_bx
               + 2.0f*(q2*q3 + q0*q1) * a_by
               + (1.0f - 2.0f*(q1*q1 + q2*q2)) * a_bz;

    /* 3. 减去重力分量（(0,0,-g)，初始 Z 向上，g 严格 9.7985）→ 线加速度（EKF 过程输入） */
    ins_a[0] = ax_w;
    ins_a[1] = ay_w;
    ins_a[2] = az_w - ACC_G_MS2;

    /* 4. 静止判定（EKF ZUPT 门控）：|线加速度| 低通 < 阈值 */
    {
        float amag = sqrtf(ins_a[0]*ins_a[0] + ins_a[1]*ins_a[1] + ins_a[2]*ins_a[2]);
        s_amag_lp += (amag - s_amag_lp) * 0.00390625f;   /* 32ms 低通 */
        ins_amag = s_amag_lp;
        ins_still = (s_amag_lp < ACC_STILL_A) ? 1 : 0;
    }
}
