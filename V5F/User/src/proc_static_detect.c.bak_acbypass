#include "v5f_proc.h"

/* =====================================================================
 * 处理函数 2：动静判定（后验，吃前一项的结果）
 *
 *   输入        h->imu.gyro_dps[]  —— 处理函数 1 输出的零偏校正后角速度
 *   统计量      s = max_axis | W 帧滑窗均值 |，W = V5F_DET_W = 128 帧（16 ms）
 *   判运动      s > 阈值            连续 V5F_DET_DEB_ON  帧（1 ms）
 *   判静止      s < V5F_DET_THR_OFF 连续 V5F_DET_DEB_OFF 帧（100 ms）  ← 滞回 + 防抖
 *
 *   门控：本函数写 gate（g_v5f_proc_gate），供**下一帧**的零偏校正读取 ——
 *   本帧的判定用本帧的校正输出、作用于下一帧的牵引（后验、依赖前项结果，
 *   后续融合其它传感器时同样的位置、同样的方向）。
 *   回退粒度固定写入 V5F_PROC_ROLLBACK_GRANULES = 24 格（480 ms）：实测本机
 *   典型转动（峰值 100 dps）的检测延迟 0.46 s，24 格正好覆盖；钳位到"本段静止
 *   已累计的粒度数"的逻辑在处理函数 1 里。
 *
 *   阈值选型（151 s 静止记录实测）：sigma_raw = 0.1224 dps，
 *   sigma_s = sigma_raw/sqrt(W) = 0.01082 dps；
 *   常态 10 sigma = 0.108 dps；上电前 12 s（V5F_PROC_BOOT_FRAMES）用 20 sigma
 *   = 0.216 dps —— 标定初值与实际器件零点有偏差，这段留给牵引把 bias 拉正，
 *   阈值：查表（s_thr_tab），索引 = 累积静止证据 >> V5F_DET_THR_SHIFT。
 *   证据不足（过去静止段太少 -> bias 还没被牵引够）时阈值抬到最高 2x 基础阈值；
 *   证据够了落回基础阈值 10 sigma = 0.108 dps。判据与生成脚本见 v5f_proc.h 与
 *   build_thr_table.py；运行期只做移位/查表/比较，不做 exp/sqrt 现算。
 *   回判静止阈值 = 0.6x 当前阈值（滞回 + 防抖）。
 *   滑窗未满（前 16 ms）时不判、锁定静止 —— 检测器只吃前一项的有效结果。
 * ===================================================================== */

static float    s_win[3][V5F_DET_W];      /* 三轴滑窗环形缓冲 */
static float    s_sum[3];                 /* 窗口内三轴累加和 */
static uint16_t s_head;                   /* 环形写指针 */
static uint16_t s_fill;                   /* 已填帧数（满 W 后窗有效） */
static uint16_t s_on_cnt;                 /* 连续超阈计数 */
static uint16_t s_off_cnt;                /* 连续低阈计数 */
static uint8_t  s_state = 1u;             /* 当前状态：1 = 静止 */

/* ---- 阈值查找表（离线算好，运行期只移位取档 + 查表） ----
 * thr(T) = clamp(10*sqrt(sigma_s^2 + e0^2*exp(-2T/tau)), base, 3*base)
 *   sigma_s = 0.1224/sqrt(128) = 0.01082 dps（实测），base = 10*sigma_s = 0.108 dps
 *   上限 3*base = 30*sigma_s = 0.3246 dps；设计 e0 = 0.083 dps 使该上限正好覆盖
 *   启动前 12 s（前 5 档 0~12.8 s），此后按判据衰减，约 50 s 落到 base
 * 由 build_thr_table.py 生成；改判据请重跑该脚本，不要把 exp/sqrt 搬进固件。 */
static const float s_thr_tab[V5F_DET_THR_NBUCK] = {
    0.3246f, 0.3246f, 0.3246f, 0.3246f, 0.3246f, 0.3054f, 0.2549f, 0.2156f,
    0.1854f, 0.1628f, 0.1462f, 0.1342f, 0.1258f, 0.1200f, 0.1160f, 0.1134f,
    0.1116f, 0.1104f, 0.1096f, 0.1091f,
};

static float jf_abs(float x) { return (x < 0.0f) ? -x : x; }

uint8_t v5f_proc_static_detect(volatile v5f_hold_t *h, volatile v5f_proc_gate_t *gate)
{
    float    level, inv, thr_on, thr_off;
    uint16_t idx;
    uint8_t  i;

    /* ---- 滑窗更新（三轴，O(1)） ---- */
    for (i = 0u; i < 3u; i++) {
        float x = h->imu.gyro_dps[i];
        s_sum[i] += x - s_win[i][s_head];
        s_win[i][s_head] = x;
    }
    s_head = (uint16_t)((s_head + 1u) % V5F_DET_W);
    if (s_fill < V5F_DET_W) s_fill++;

    inv   = (s_fill > 0u) ? (1.0f / (float)s_fill) : 0.0f;
    level = 0.0f;
    for (i = 0u; i < 3u; i++) {
        float a = jf_abs(s_sum[i] * inv);
        if (a > level) level = a;
    }
    h->stat.level_dps = level;
    h->stat.changed   = 0u;

    /* ---- 初值窗 / 滑窗未满：不判，锁定静止 ---- */
    if (h->imu.corr_valid == 0u || s_fill < V5F_DET_W) {
        s_state   = 1u;
        s_on_cnt  = 0u;
        s_off_cnt = 0u;
        h->stat.valid = 0u;
    } else {
        h->stat.valid = 1u;
        /* 查表：证据（累积静止粒度数）越少阈值越高，最高 2x；证据够了回到 base。
         * 证据由处理函数 1 维护（回退时扣减），所以回退后阈值会自动回升。 */
        idx = (uint16_t)(h->imu.bias_evidence_gran >> V5F_DET_THR_SHIFT);
        if (idx >= V5F_DET_THR_NBUCK) idx = V5F_DET_THR_NBUCK - 1u;
        thr_on = s_thr_tab[idx];
        if (thr_on < V5F_DET_THR_BASE) thr_on = V5F_DET_THR_BASE;
        thr_off = thr_on * V5F_DET_THR_OFF_RATIO;

        if (s_state != 0u) {                          /* 静止中：找运动 */
            if (level > thr_on) {
                if (++s_on_cnt >= V5F_DET_DEB_ON) {
                    s_state = 0u;
                    s_on_cnt = 0u;
                    h->stat.changed = 1u;
                }
            } else {
                s_on_cnt = 0u;
            }
        } else {                                      /* 运动中：找静止 */
            if (level < thr_off) {
                if (++s_off_cnt >= V5F_DET_DEB_OFF) {
                    s_state = 1u;
                    s_off_cnt = 0u;
                    h->stat.changed = 1u;
                }
            } else {
                s_off_cnt = 0u;
            }
        }
    }

    h->stat.is_static       = s_state;
    gate->is_static         = s_state;
    gate->rollback_granules = V5F_PROC_ROLLBACK_GRANULES;
    return V5F_PROC_OK;
}
