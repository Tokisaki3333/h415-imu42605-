#include "v5f_proc.h"
#include <math.h>   /* sqrtf：-Ofast 下编译成一条 fsqrt.s，不引入 libm 调用 */

/* =====================================================================
 * 处理函数 3：姿态四元数（陀螺积分）
 *
 *   输入   h->imu.gyro_dps[] —— 处理函数 1 的补偿输出
 *           （零偏牵引值 + 逐轴标度系数已载入，此处不再做任何定标/零偏处理）
 *   递推   q ← q × dq，右乘即"机体轴增量"（陀螺输出的就是机体角速度）
 *          dq = { 1, 0.5·wx·dt, 0.5·wy·dt, 0.5·wz·dt }      半角一阶近似
 *          ★ 这里的 w 必须是 rad/s：陀螺给的是 °/s，先乘 DEG2RAD 再进四元数。
 *            漏这一步不会报错，只会把姿态角整体放大 180/pi = 57.3 倍。
 *   近似误差  精确式 dq = { cos(θ/2), sin(θ/2)·(ω/|ω|) }，一阶式的相对角误差 ≈ θ^2/24；
 *          本机 dt = 124.55 us，满量程 |ω| = 2000 dps = 34.9 rad/s
 *          → θ ≤ 4.35e-3 rad → 相对误差 ≤ 8e-7（整圈 360° 累积 < 0.0004°，可忽略）。
 *          单帧 dt 越大误差按 θ^2 增长，故本函数依赖"每帧都被调用"这一前提。
 *   归一化  每帧一次：s = 1/sqrt(|q|^2)，q *= s。单精度下不算归一化，模长每帧
 *          漂移约 1e-7，几秒内就积累到不可忽略；V5F 有单精度 FPU，sqrt+div
 *          各一条指令，代价可忽略，故直接精确归一化，不做"仅在 |q|≈1 成立"的
 *          牛顿近似（那会在异常大 dt 下给出错误的负增益）。
 *   时间基准  dt 取自 h->imu.fresh.drdy_tick（10 ns 计数），与调用次数无关；
 *          上游时间戳回退帧（corr_valid = 0）本函数不更新姿态。
 *   使能门    零偏收敛（h->imu.bias_ok，累计静止 ≥ 3τ_boot = 6 s）之前不积分，
 *          输出恒为单位四元数、valid = 0；收敛后一直积分（bias_ok 是锁存）。
 *          理由：初值残差 e0 造成的启动角度暂态是 e0·τ_work 量级，与其带着它
 *          出门，不如等零偏收敛后再把参考系定在那一刻。
 *   全部状态为文件静态，仅被 DMA1_Channel2_IRQHandler 主任务区单线程调用，无需加锁。
 * ===================================================================== */

#define ATT_DT_SCALE   1e-8f           /* 10 ns 计数 → s */
#define ATT_DEG2RAD    0.0174532925f   /* ° → rad（π/180） */

static float    s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };   /* 上电姿态：单位四元数 */
static uint64_t s_last_tick;
static uint8_t  s_started;

uint8_t v5f_proc_attitude(volatile v5f_hold_t *h)
{
    uint64_t tick, dt;
    float    half, dw, dx, dy, dz, w, x, y, z, nw, nx, ny, nz, s;
    uint8_t  i;

    for (i = 0u; i < 4u; i++) h->att.q[i] = s_q[i];   /* 无论本帧是否更新都发布当前姿态 */
    h->att.valid = 0u;

    if (h->imu.corr_valid == 0u) return V5F_PROC_OK;  /* 上游本帧无效：保持上一帧姿态 */

    /* 零偏收敛前不出姿态：bias_ok = 累计静止 ≥ 3τ_boot（6 s）后的一次性锁存
     * （在处理函数 1 里置位，回退不会清）。置位前四元数冻结在单位四元数、
     * valid = 0；置位那一帧才建立 dt 基准并开始积分。
     * => 姿态参考 = 零偏收敛那一刻的机体姿态；上电初值残差 e0 造成的启动角度
     *    暂态（e0·τ_work 量级，本机实测 0.31°）整段被丢掉，不进入输出。 */
    if (h->imu.bias_ok == 0u) return V5F_PROC_OK;

    tick = h->imu.fresh.drdy_tick;
    if (s_started == 0u) {                            /* 首帧：只建立 dt 基准，姿态不变 */
        s_started   = 1u;
        s_last_tick = tick;
        h->att.valid = 1u;
        return V5F_PROC_OK;
    }

    dt          = tick - s_last_tick;
    s_last_tick = tick;

    /* 本帧机体增量（半角）：θ/2 = 0.5 · ω[rad/s] · dt[s]，gyro_dps 是 °/s 故先转 rad */
    half = 0.5f * (float)dt * ATT_DT_SCALE * ATT_DEG2RAD;
    dw   = 1.0f;
    dx   = h->imu.gyro_dps[0] * half;
    dy   = h->imu.gyro_dps[1] * half;
    dz   = h->imu.gyro_dps[2] * half;

    /* q ← q × dq（Hamilton 积，q = {w,x,y,z}） */
    w = s_q[0]; x = s_q[1]; y = s_q[2]; z = s_q[3];
    nw = w * dw - x * dx - y * dy - z * dz;
    nx = w * dx + x * dw + y * dz - z * dy;
    ny = w * dy - x * dz + y * dw + z * dx;
    nz = w * dz + x * dy - y * dx + z * dw;

    /* 归一化：s = 1/|q| */
    s = 1.0f / sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
    s_q[0] = nw * s;
    s_q[1] = nx * s;
    s_q[2] = ny * s;
    s_q[3] = nz * s;

    for (i = 0u; i < 4u; i++) h->att.q[i] = s_q[i];
    h->att.valid = 1u;
    return V5F_PROC_OK;
}
