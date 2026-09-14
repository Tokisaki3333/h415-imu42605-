# -*- coding: utf-8 -*-
"""VER=17 -> 18：M7 的门用错了标志，另外补两条运动中真正有害的误差。

实测（VER=15/17，16 s 剧烈摇晃 |a_lin| 峰 7.7 g、|w| 峰 2229 dps）：
    剧烈段内 mag 门只开 0.3%（tilt 7.2%）—— 偏航基准在最需要它的时候被关掉了。
根因：门用的是 mag.trust，而 trust = mag.ok **且** |a_lin| < 0.05 g。
    那个倾斜条件是给**旧链**加的：旧链的 Bw/psi_true 是用旧链姿态算的，
    倾角不可信时它们也不可信。EKF 的 M7 根本不用那些 —— 它用 mag.f（机体标定
    后的磁场，纯传感器量）+ 自己的 q_hat。加速度不会污染磁场测量。
=> 门改成 mag.ok（只查 |y| 模长一致性 = 外部磁干扰），这才是外生的物理量。

运动中 M7 真正有害的两条，正确做法是进 R，不是关门：
 1) 倾角误差耦合：h 由 R(q_hat)*f 算，倾角误差 dh 使方位角偏 tan(I)*dh
    （真倾角 53.74 度 -> 1.364 倍）。滤波器自己知道 sigma_tilt（P[6][6]+P[7][7]）。
    ★ 注意与设计 4.2 的"H 只挂偏航项"不矛盾：H 只挂偏航 => 更新不会污染倾角估计；
      但倾角的**误差**会进新息。两件事，都要管。
 2) 样本陈旧：IST 约 187 Hz（5.3 ms），而 2229 dps = 11.9 度/ms -> 一个样本最多陈旧
    63 度。这条**可补**：按自采样以来的机体转动量把 f 转到当前机体系
    （v(t) = Exp(-dth) v(t_s)），用 mag.fresh.drdy_tick 那个边的 gyro 积分。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_RECT_DPS_PER_G   0.85f'
assert u.count(a) == 1
u = u.replace(a, """#define V5F_EKF_DIP_TAN          1.364f  /* tan(真倾角 I)：I = 53.74 度（WMM）。
                                          * 倾角误差 dh 会让水平方位角偏 tan(I)*dh，
                                          * 所以 M7 的 R 要加上 (tan(I)*sigma_tilt)^2。
                                          * ★ 用 WMM 的真倾角，不用实测那个 63~64 度
                                          *   （实测 dip 异常是磁力计自身的问题，
                                          *    导航系里的地磁方向仍由真实地球磁场决定） */
#define V5F_EKF_RECT_DPS_PER_G   0.85f""", 1)
assert u.count('#define V5F_FW_VER        17u') == 1
u = u.replace('#define V5F_FW_VER        17u', '#define V5F_FW_VER        18u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1j')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: DIP_TAN + VER 18')

t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


# --- 1) 矢量按四元数旋转的工具 + 磁样本龄累加器 ---
sub("""static void H_zero(uint8_t m)""",
    """/* o = q (x) v (x) q*：把一个**矢量**按单位四元数旋转（与 q_mul 不是一回事）。
 * 用于把磁力计采样时刻的机体系矢量转到当前机体系。 */
static void q_rot_vec(const float *q, const float *v, float *o)
{
    float u[3], tt[3], c[3];
    u[0] = q[1]; u[1] = q[2]; u[2] = q[3];
    tt[0] = 2.0f*(u[1]*v[2] - u[2]*v[1]);
    tt[1] = 2.0f*(u[2]*v[0] - u[0]*v[2]);
    tt[2] = 2.0f*(u[0]*v[1] - u[1]*v[0]);
    c[0] = u[1]*tt[2] - u[2]*tt[1];
    c[1] = u[2]*tt[0] - u[0]*tt[2];
    c[2] = u[0]*tt[1] - u[1]*tt[0];
    o[0] = v[0] + q[0]*tt[0] + c[0];
    o[1] = v[1] + q[0]*tt[1] + c[1];
    o[2] = v[2] + q[0]*tt[2] + c[2];
}

static void H_zero(uint8_t m)""", 'rotvec')

sub("static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */",
    """static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */
static float    s_mag_dth[3];         /* 自磁力计采样以来的**机体**转动量 rad（补样本陈旧） */
static uint32_t s_ist_last;""", 'magacc')

# --- 2) M7：先补样本陈旧，再用自适应 R ---
sub("""    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;
    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);                    /* B = R(q) f -> 导航系地磁方向 */
    hh = atan2f(Bn[0], Bn[1]);             /* 地磁水平分量方位（东/北），应恒等于 D */
    H_zero(1u);
    s_H[0][8] = -1.0f;                     /* dh/d(dth_z) = -1（弧度/弧度） */
    R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;
    r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);""",
    """    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;
    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    /* 补样本陈旧：IST 约 187 Hz，2229 dps 下一个样本最多陈旧 63 度。
     * v(t) = Exp(-dth) v(t_s)，dth = 自采样以来的**机体**转动量。 */
    {
        float am = sqrtf(s_mag_dth[0]*s_mag_dth[0] + s_mag_dth[1]*s_mag_dth[1]
                       + s_mag_dth[2]*s_mag_dth[2]);
        if (am > 1e-4f) {
            float half = 0.5f * am;
            float s = sinf(half) / am;
            float dq[4];
            dq[0] = cosf(half);
            dq[1] = -s * s_mag_dth[0];
            dq[2] = -s * s_mag_dth[1];
            dq[3] = -s * s_mag_dth[2];
            {
                float tmp[3];
                q_rot_vec(dq, mf, tmp);
                mf[0] = tmp[0]; mf[1] = tmp[1]; mf[2] = tmp[2];
            }
        }
    }
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);                    /* B = R(q) f -> 导航系地磁方向 */
    hh = atan2f(Bn[0], Bn[1]);             /* 地磁水平分量方位（东/北），应恒等于 D */
    H_zero(1u);
    s_H[0][8] = -1.0f;                     /* dh/d(dth_z) = -1（弧度/弧度） */
    /* 自适应 R：观测值里除了磁力计自身噪声，还混进了**当前倾角误差**的耦合
     * （方位角偏 tan(I)*dtheta_h，见 v5f_tune.h 的 V5F_EKF_DIP_TAN）。
     * 用已传播的 P 自己算 —— 这是自适应 R，不是门；门仍然只用外生量。 */
    R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD
         + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN * (s_Pn[6][6] + s_Pn[7][7]);
    r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);""", 'm7')

# --- 3) 磁样本边沿清零 + 每帧累加 ---
sub("""    raw_w_rads(h, w);
    raw_f_mps2(h, f);
    for (i = 0u; i < 3u; i++) {
        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
        s_dvb[i] += f[i] * dt;
    }
    s_dt_e += dt;""",
    """    raw_w_rads(h, w);
    raw_f_mps2(h, f);
    /* 磁力计新样本边沿：把"自采样以来的转动量"清零（补样本陈旧用） */
    {
        uint32_t ic = g_shm ? g_shm->ist.hdr.cnt : 0u;
        if (ic != s_ist_last) {
            s_ist_last = ic;
            s_mag_dth[0] = 0.0f; s_mag_dth[1] = 0.0f; s_mag_dth[2] = 0.0f;
        }
    }
    for (i = 0u; i < 3u; i++) {
        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
        s_dvb[i] += f[i] * dt;
        s_mag_dth[i] += w[i] * dt;      /* 用未扣 bg 的量测即可：bg 只有 0.1 dps 量级 */
    }
    s_dt_e += dt;""", 'magdth')

# --- 4) 门改用 mag.ok ---
sub("""    gate->ekf_mag_yaw = (uint8_t)((h->mag.trust != 0u) && (V5F_EKF_YAW_OBS_EN != 0u));""",
    """    /* ★ 门用 mag.ok（只查 |y| 模长一致性 = 外部磁干扰），**不用** mag.trust。
     *   trust 里那个 |a_lin| < 0.05 g 是给旧链加的：旧链的 Bw/psi_true 由旧链姿态算出，
     *   倾角不可信时它们不可信。EKF 的 M7 用的是 mag.f（机体磁场，纯传感器量）
     *   配自己的 q_hat，加速度不污染磁场测量 —— 那个条件对 M7 无效，
     *   而且恰好在陀螺被整流误差污染时把偏航基准关掉（实测剧烈段只开 0.3%）。
     *   运动带来的两条真实误差走 R：倾角耦合 + 样本陈旧（在 ekf_m7_mag 里处理）。 */
    gate->ekf_mag_yaw = (uint8_t)((h->mag.ok != 0u) && (V5F_EKF_YAW_OBS_EN != 0u));""", 'gate')

sub("""            s_alin_g = 0.0f; s_sat = 0u;""",
    """            s_alin_g = 0.0f; s_sat = 0u;
            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;""", 'clr')

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1j')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: M7 门改 mag.ok + 样本陈旧补偿 + 自适应 R')
print()
print('fw_tag 期望 = %d' % ((18 << 16) | (112 << 8) | 1 | 2 | 4))
