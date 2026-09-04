#include "attitude.h"
#include "observer.h"      /* g_v5f_obs（四元数存储） */
#include <math.h>

/* ==================== 传感器参数 ==================== */
// #define GYRO_SENS_RAD  ((1.0f / 65.536f) * (3.14159265358979f / 180.0f))
#define GYRO_SENS_RAD  ((1.0f / 16.4f) * (3.14159265358979f / 180.0f))
/* 三轴统一增益校正：假设 IMU 每读取多算 +0.15%（scale≈1.0015），角速率读数放大；
 * 这里反向乘 1/(1+0.0015) 抵消，使积分接近真实转速（转台实测后按需调整）。 */
/* 已弃用：三轴统一 +0.15% 曾假设方向（后被逐轴 1440° 标定取代，符号见下）
// #define GYRO_SCALE_CORR  (1.0f / 1.0015f) */
/* 逐轴 1440° 标准旋转标定增益校正（乘在角速率）。实测为“少算”→用放大(1/(1−e)):
 * X: 1440° 偏 →11.1°  → 1/(1−11.1/1440) ≈ 1.00777
 * Y: 1440° 偏 → 3.9°  → 1/(1− 3.9/1440) ≈ 1.00272
 * Z: 待该轴 1440° 标定后填入(GYRO_SCALE_Z) */
#define GYRO_SCALE_X  (1.0f / (1.0f - (11.1f / 1440.0f)))   /* X:1440°少算 11.1°→放大(压缩版劣化,已回) */
#define GYRO_SCALE_Y  (1.0f / (1.0f + ( 3.9f / 1440.0f)))   /* Y:1440°试 3.9° 压缩(已对上方向) */
#define GYRO_SCALE_Z  (1.0f / (1.0f + ((1.529f / 1000.0f) + (3.6f / 1440.0f)) * 0.5f))  /* Z:取中(上一版全局压到位于从+3.6翻到-3.3，砍半取零位) */
/* 静止判定阈值：0.12 °/s（也写“0.12°”可行）。
 * 位移判据用 |eg| 低通（LSB 域）< STILL_ERR_LSB2，故先声明 °/s 阈值再换算单位。 */
#define STILL_DPS        1.0f        /* 静止判定阈值，°/s */
#define GYRO_LSB_PER_DPS 16.4f        /* LSB/(°/s)，与 GYRO_SENS_RAD 同源 */
#define STILL_ERR_LSB2   (STILL_DPS * GYRO_LSB_PER_DPS)   /* = 0.12×16.4 ≈ 1.968 LSB */
#define SAMPLE_DT      (1.0f / 8000.0f)   /* 采样帧兜底 dt：手册理想值 8kHz＝125us（DRDY 实测差分主路径优先；此值仅兜底） */
#define GYRO_DT_SAT_TICK  100000u   /* 10Ns 计数饱和上限 = 1ms（1e-3 s / 1e-8 = 1e5 计数） */   /* dt 实测饱和上限：8kHz 周期 125us，8 倍余量防中断丢失 */

/* ==================== 在线零偏跟踪（静止段累积） ====================
 * 判定 0.12°/s（|eg| 低通值，32ms 时间常数）；0.5s 确认；段末前溯弃 0.5s；
 * 每段最长 10s 强制切断；bias 用最近最多 20s 静止均值；至少 1s 静止才修正。 */
#define STILL_CONFIRM_FRAMES 4000u           /* 连续静止 0.5s（@8kHz）确认后开始累积段 */
#define STILL_TAIL_FRAMES    4000u           /* 段结束（运动）前溯弃用 0.5s */
#define STILL_TARGET_FRAMES  160000u         /* bias 用最近最多 20s 静止样本 */
#define STILL_MIN_FRAMES     8000u           /* 至少 1s 静止数据才允许修正 bias */
#define STILL_SEG_MAX_FRAMES 80000u          /* 每段最长 10s：强制切断（弃尾 0.5s，新段不弃头），最旧数据 ≤30s */
#define STILL_MAX_SEGS       64u

typedef struct {
    uint64_t start_us;                  /* 段开始时刻（确认完成后，DRDY ts） */
    uint64_t end_us;                    /* 段结束时刻（DRDY ts） */
    uint32_t cnt;                       /* 段内有效帧数（已弃尾） */
    int64_t  sum_gx, sum_gy, sum_gz;    /* 段内 LSB 和 */
} still_seg_t;

static still_seg_t s_segs[STILL_MAX_SEGS];
static uint8_t  s_seg_cnt = 0;          /* 段数 */
volatile uint32_t s_run_frames = 0;     /* 当前连续静止帧数（含确认期） */
static uint8_t  s_seg_active = 0;       /* 当前段已激活 */
static uint32_t s_cur_cnt = 0;          /* 当前段帧数 */
static uint64_t s_cur_start_us = 0;     /* 当前段开始时刻 */
static int64_t  s_cur_sum_gx = 0, s_cur_sum_gy = 0, s_cur_sum_gz = 0;
static int16_t  s_tail[STILL_TAIL_FRAMES][3];   /* 尾部 0.5s 环形（段末精确弃用），24KB */
static uint32_t s_tail_idx = 0;
static int64_t  s_tail_sgx = 0, s_tail_sgy = 0, s_tail_sgz = 0;  /* 环内最近 TAIL_FRAMES 样本三轴和；写样本时 O(1) 更新 */

static uint64_t s_oldest_us = 0;       /* 最近 20s 静止窗口内最老样本时刻（显示用） */
float s_eg_lp_x = 0, s_eg_lp_y = 0, s_eg_lp_z = 0;  /* |eg| 低通（32ms），静止判定用 */
volatile uint32_t s_still_cnt = 0;      /* 最远被采用(算当前零偏)的静止段距今秒；恒推进，表示当前零偏时效 */

volatile float    bias_gx = 2.4936f, bias_gy = 1.6591f, bias_gz = 11.4812f;   /* 当前零偏默认（校准值） */
volatile uint8_t  g_v5f_motion = 1;    /* 1=运动 0=静止（中断更新，供 OLED/上层） */

/* ==================== 段封存：前溯弃用 0.5s（尾部环形精确扣除）后入段，清空当前段 ====================
 * 每段 ≤10s，前溯必在段内（不跨块）；强制切断（10s）时也复用。 */
static void still_seal_segment(uint64_t end_us)
{
    if (s_cur_cnt >= STILL_TAIL_FRAMES) {
        /* 前溯弃用尾 0.5s：直接扣除 O(1) 维护的环内最近 4000 样本三轴和（原 4000 循环已废） */
        s_cur_cnt -= STILL_TAIL_FRAMES;
        s_cur_sum_gx -= s_tail_sgx;
        s_cur_sum_gy -= s_tail_sgy;
        s_cur_sum_gz -= s_tail_sgz;
        if (s_cur_cnt > 0 && s_seg_cnt < STILL_MAX_SEGS) {
            still_seg_t *p = &s_segs[s_seg_cnt++];
            p->start_us = s_cur_start_us;
            p->end_us = end_us;
            p->cnt = s_cur_cnt;
            p->sum_gx = s_cur_sum_gx;
            p->sum_gy = s_cur_sum_gy;
            p->sum_gz = s_cur_sum_gz;
        }
    }
    s_cur_cnt = 0;
    s_cur_sum_gx = s_cur_sum_gy = s_cur_sum_gz = 0;
}

/* ==================== 每瞬间 bias = 最近最多 20s 合法静止样本均值 ====================
 * 无时间窗口限制（旧数据保留，只用最近 20s 量）；同步裁剪更旧段并更新显示。 */
static void still_update_bias(uint64_t now_us)
{
    int64_t sgx = s_cur_sum_gx, sgy = s_cur_sum_gy, sgz = s_cur_sum_gz;
    uint32_t c = s_cur_cnt, i = s_seg_cnt;
    uint64_t oldest = (s_cur_cnt > 0) ? s_cur_start_us : 0;
    while (c < STILL_TARGET_FRAMES && i > 0) {
        const still_seg_t *p = &s_segs[--i];
        sgx += p->sum_gx; sgy += p->sum_gy; sgz += p->sum_gz;
        c += p->cnt;
        oldest = p->start_us;   /* 最老计入段 = 窗口边界 */
    }
    if (c > 0) {
        s_oldest_us = oldest;
        s_still_cnt = (uint32_t)((now_us - s_oldest_us) / 100000000ULL);   /* 最老数据距今秒（独立于修正） */
    }
    if (c >= STILL_MIN_FRAMES) {   /* 至少 1s 静止数据才修正，避免瞬态/短窗噪声拉偏 */
        bias_gx = (float)sgx / (float)c;
        bias_gy = (float)sgy / (float)c;
        bias_gz = (float)sgz / (float)c;
    }
    if (i > 0) {   /* 裁剪：窗口边界外的更旧段移除 */
        uint32_t j;
        for (j = 0; j + i < s_seg_cnt; j++) s_segs[j] = s_segs[j + i];
        s_seg_cnt = (uint8_t)(s_seg_cnt - i);
    }
}

/* ==================== 姿态复位 ==================== */
void attitude_reset(void)
{
    g_v5f_obs.q0 = 1.0f;
    g_v5f_obs.q1 = 0.0f;
    g_v5f_obs.q2 = 0.0f;
    g_v5f_obs.q3 = 0.0f;
}

/* ==================== 每帧姿态解算 + 在线零偏跟踪 ==================== */
void attitude_update(int16_t gx, int16_t gy, int16_t gz, uint32_t dt_us, uint64_t ts_us)
{
    /* 去偏角速率（单精度，LSB 域）→ rad/s 供积分 */
    float egx = (float)gx - bias_gx;
    float egy = (float)gy - bias_gy;
    float egz = (float)gz - bias_gz;
    float wx = egx * GYRO_SENS_RAD * GYRO_SCALE_X;   /* X:1440标定(少算11.1°)→放大补偿 */
    float wy = egy * GYRO_SENS_RAD * GYRO_SCALE_Y;   /* Y:1440标定(少算 3.9°)→放大补偿 */
    float wz = egz * GYRO_SENS_RAD * GYRO_SCALE_Z;   /* Z:实测每1000°多走1.529°→压缩补偿 */

    float dt = SAMPLE_DT;
    if (dt_us > 0 && dt_us <= GYRO_DT_SAT_TICK) dt = (float)dt_us * 1e-8f;   /* tick=10Ns 转秒 */   /* 实测 dt，饱和保护 */
    float q0 = g_v5f_obs.q0, q1 = g_v5f_obs.q1, q2 = g_v5f_obs.q2, q3 = g_v5f_obs.q3;

    /* ---- 高精度积分：单帧精确指数步（旋转矢量 → 跨帧单位旋量） ----
     * 对恒定角速度单帧精确；避免一阶 Euler 在连续大转动/三轴异相强非交换激励下的方向漂移。
     * 与本模块 ODE q̇=½q⊗ω̂ 一致：增量旋量 r 右乘 q（q ← q⊗r）。 */
    {
        float wsq   = wx*wx + wy*wy + wz*wz;
        float theta = sqrtf(wsq) * dt;          /* 本帧总转角 rad */
        float c, s;
        if (theta >= 1e-3f) {                   /* 常规/大角：精确 cos/sin 旋量 */
            float half = theta * 0.5f;
            c = cosf(half);
            s = sinf(half) / theta;             /* 使 rx=ωx·dt·s = (ωx/|ω|)·sin(θ/2) */
        } else {                                /* 极小角展开：c≈1-θ²/8、s≈1/2，等价一阶 Euler */
            c = 1.0f - wsq*dt*dt*0.125f;
            s = 0.5f;
        }
        float rx = wx*dt*s, ry = wy*dt*s, rz = wz*dt*s;
        /* q ← q⊗r, r=(c,rx,ry,rz) 单位旋量；右乘与本模块 ODE q̇=½q⊗ω̂ 一致
         * （此前误用左乘 r⊗q：单轴可交换时不显，多轴复合时交叉项符号反 → 巨大错乱） */
        float n0 = q0*c    - q1*rx - q2*ry - q3*rz;
        float n1 = q0*rx   + q1*c  + q2*rz - q3*ry;
        float n2 = q0*ry   - q1*rz + q2*c  + q3*rx;
        float n3 = q0*rz   + q1*ry - q2*rx + q3*c;
        q0 = n0; q1 = n1; q2 = n2; q3 = n3;
    }

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 1e-8f) {
        float inv_norm = 1.0f / norm;
        q0 *= inv_norm; q1 *= inv_norm; q2 *= inv_norm; q3 *= inv_norm;
    }

    g_v5f_obs.q0 = q0;
    g_v5f_obs.q1 = q1;
    g_v5f_obs.q2 = q2;
    g_v5f_obs.q3 = q3;

    /* ---- 在线零偏跟踪（静止段累积） ----
     * 判定 |eg| 低通值（32ms）< 0.2°/s；连续静止 0.5s 确认后开始累积段（原始 LSB 求和）；
     * 段结束（运动）前溯弃用 0.5s；每段 10s 强制切断；bias = 最近最多 20s 静止均值。 */
    s_eg_lp_x += ((egx < 0 ? -egx : egx) - s_eg_lp_x) * 0.00390625f;   /* 1/256，τ≈32ms */
    s_eg_lp_y += ((egy < 0 ? -egy : egy) - s_eg_lp_y) * 0.00390625f;
    s_eg_lp_z += ((egz < 0 ? -egz : egz) - s_eg_lp_z) * 0.00390625f;
    if (s_eg_lp_x < STILL_ERR_LSB2 && s_eg_lp_y < STILL_ERR_LSB2 && s_eg_lp_z < STILL_ERR_LSB2) {
        g_v5f_motion = 0;   /* 静止 */
        s_run_frames++;
        /* 环内 3 轴和 O(1) 维护：先扣将被覆盖槽旧值，再写新值并加和 */
        s_tail_sgx -= s_tail[s_tail_idx][0];
        s_tail_sgy -= s_tail[s_tail_idx][1];
        s_tail_sgz -= s_tail[s_tail_idx][2];
        s_tail[s_tail_idx][0] = gx;   /* 尾部环形（段末弃用 0.5s） */
        s_tail[s_tail_idx][1] = gy;
        s_tail[s_tail_idx][2] = gz;
        s_tail_sgx += gx;
        s_tail_sgy += gy;
        s_tail_sgz += gz;
        s_tail_idx = (s_tail_idx + 1) % STILL_TAIL_FRAMES;
        if (s_run_frames >= STILL_CONFIRM_FRAMES) {
            if (!s_seg_active) {
                s_seg_active = 1;
                s_cur_cnt = 0;
                s_cur_start_us = ts_us;
                s_cur_sum_gx = s_cur_sum_gy = s_cur_sum_gz = 0;
            }
            s_cur_cnt++;
            s_cur_sum_gx += gx;
            s_cur_sum_gy += gy;
            s_cur_sum_gz += gz;
            if (s_cur_cnt >= STILL_SEG_MAX_FRAMES) {
                /* 10s 强制切断：弃尾 0.5s 入段，新段从当前帧起（不弃头） */
                still_seal_segment(ts_us);
                s_cur_start_us = ts_us;
            }
            still_update_bias(ts_us);   /* 每帧：bias = 最近最多 20s 静止均值 */
        }
    } else {
        g_v5f_motion = 1;   /* 运动 */
        if (s_seg_active) {
            s_seg_active = 0;
            /* 段封存：前溯弃 0.5s（必在段内，不跨块）+ 入列表 */
            still_seal_segment(ts_us);
        }
        s_run_frames = 0;
        still_update_bias(ts_us);   /* 运动分支也刷新：s_still_cnt=(now-oldest)/1s 持续推进，恒有效 */
    }
}
