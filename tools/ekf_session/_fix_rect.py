# -*- coding: utf-8 -*-
"""VER=16 -> 17：剧烈段暴露出两个"诚实性"缺口。

实测（VER=15，16 s 剧烈摇晃，|a_lin| 峰 7.7 g，整流剂量 45.6 g*s）：
    真实偏航误差：段前 +0.150 度 -> 刚结束 +15.694 度 -> 稳定后 **+0.071 度**（磁环拉回来了）
    旧链同一段：  +177.112 -> -175.998 -> -174.714（**永久留下 +7.2 度**，没有环）
    EKF 倾角误差：0.704 -> 0.639 -> 0.720 度（全程 <=0.72）
    旧链倾角误差：0.242 -> 4.715 -> 1.168 度
=> 恢复能力是我们的强项，但暴露两个缺口：

1) **sigma_yaw 在撒谎**：真实误差到了 15.7 度，而 sigma_yaw 最大只有 0.441 度（差 35 倍）。
   原因：EKF 只把陀螺噪声建模成白噪声 sigma_g=0.1224 dps，**没有**建模加速度整流误差。
   项目自己的结论：g 敏感度标量模型不成立（补偿反而更差），所以**不该去补**；
   但**必须把它算进协方差** —— 不知道就说不知道，否则下游会信一个错了 15 度的航向。
   做法：Q_tt += (k_rect * |a_lin|)^2 * dt，k_rect 取实测上界 0.85 dps/g。
   副作用是好的：P[8][8] 涨起来后磁环的 K 变大，剧烈运动后的再收敛更快。

2) **加计削顶没被承认**：accel |LSB| 触到 32767（±16 g 满量程）有 14 帧。
   削顶帧的比力是错的，M6 不该用它；同时要在上报里留下证据（新增 gate_bits bit11）。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

# ---------- 常量 ----------
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_Q_FLOOR          1.0e-12f'
assert u.count(a) == 1
u = u.replace(a, """/* 加速度整流（离心项 a_c = w^2*r）在陀螺上的误差上界 [dps/g]。
 * 【项目实测】大幅摇晃 20.8 s、积分 a_c dt = 20.8 g*s -> 末态姿态反了 17.77 度；
 *   小幅 2.9 g*s -> 2.38 度。反推 0.85 / 0.82 dps/g。
 *   ★ 不拿它去**补偿**：tools/calib/g_sens_axis.py 扫出来的标量模型对三条记录
 *     全部是负效果（8.649 -> 8.997 度），所以 V5F_GYRO_GSENS_* 是 0。
 *   但它是**真实存在的不确定度**，必须进 Q，否则 sigma_yaw 会在剧烈运动里撒谎
 *   （实测真实偏航误差 15.7 度时 sigma_yaw 只报 0.441 度，差 35 倍）。 */
#define V5F_EKF_RECT_DPS_PER_G   0.85f
#define V5F_EKF_Q_FLOOR          1.0e-12f""", 1)
a = '#define V5F_EKF_GB_ORIGIN        0x0400u   /* ENU 原点已建立（位置列才有绝对意义） */'
assert u.count(a) == 1
u = u.replace(a, a + '\n#define V5F_EKF_GB_SAT           0x0800u   /* 本周期有加计削顶帧（比力不可信） */', 1)
assert u.count('#define V5F_FW_VER        16u') == 1
u = u.replace('#define V5F_FW_VER        16u', '#define V5F_FW_VER        17u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1i')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: RECT_DPS_PER_G + GB_SAT + VER 17')

# ---------- proc_ekf.c ----------
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


sub("static float    s_a_nav[3];",
    "static float    s_a_nav[3];\n"
    "static float    s_alin_g;             /* 本周期 |a_lin|（机体系比力去掉零偏后的模长，g）：\n"
    "                                       * 整流不确定度按它缩放，进 Q_tt */\n"
    "static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */", 'vars')

# 建 F 时记录 |a_lin|
sub("""    {
        float fb[3], fnv[3];
        for (i = 0u; i < 3u; i++) fb[i] = s_dvb[i]/s_dt_e - s_x[IX_BA + i];
        rot_bn(R, fb, fnv);""",
    """    {
        float fb[3], fnv[3];
        for (i = 0u; i < 3u; i++) fb[i] = s_dvb[i]/s_dt_e - s_x[IX_BA + i];
        /* |a_lin|（g）：整流误差 ~ k_rect*|a_lin|，作为姿态过程噪声的来源 */
        s_alin_g = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]) / V5F_EKF_G_MPS2;
        rot_bn(R, fb, fnv);""", 'alin')

# Q_tt 加整流项
sub("""    sa2 = V5F_EKF_SIG_A_MPS2 * V5F_EKF_SIG_A_MPS2;
    sg2 = V5F_EKF_SIG_G_RADS  * V5F_EKF_SIG_G_RADS;""",
    """    sa2 = V5F_EKF_SIG_A_MPS2 * V5F_EKF_SIG_A_MPS2;
    /* 姿态过程噪声 = 白噪声 + **加速度整流**。
     * 后者是运动相关的系统项（a_c = w^2*r 恒 >= 0，整流不抵消、单向累积），
     * 只靠 sigma_g 建模会让 sigma_yaw 在剧烈运动里严重偏小（实测差 35 倍）。
     * 不补偿它（标量模型不成立），但必须承认它。 */
    {
        float sr = V5F_EKF_RECT_DPS_PER_G * s_alin_g * DEG2RAD;
        float sq = V5F_EKF_SIG_G_RADS * V5F_EKF_SIG_G_RADS + sr * sr;
        sg2 = sq;
    }""", 'qtt')

# 削顶：step8 判、stage16 粘住、上报
sub("""    gate->ekf_tilt = (uint8_t)(h->imu.acc_valid
                    && (fabsf(am2 - 1.0f) < V5F_EKF_TILT_AMAG_TOL)
                    && ((h->stat.level_dps < V5F_EKF_TILT_LEV_DPS) || h->stat.ac_bypass));""",
    """    /* 加计削顶：|LSB| 到满量程 -> 该帧比力是错的，M6 不许用，且要留下证据 */
    {
        uint8_t sat = 0u;
        uint32_t k;
        for (k = 0u; k < 3u; k++) {
            int32_t v = (int32_t)h->imu.accel_lsb[k];
            if (v >= 32000 || v <= -32000) sat = 1u;
        }
        if (sat) s_sat = 1u;
        gate->ekf_tilt = (uint8_t)(!sat && h->imu.acc_valid
                        && (fabsf(am2 - 1.0f) < V5F_EKF_TILT_AMAG_TOL)
                        && ((h->stat.level_dps < V5F_EKF_TILT_LEV_DPS) || h->stat.ac_bypass));
    }""", 'sat')

sub("""            s_gate_bits = 0u;                  /* 本周期观测记账从这里开始 */""",
    """            s_gate_bits = 0u;                  /* 本周期观测记账从这里开始 */
            if (s_sat) { s_gate_bits |= V5F_EKF_GB_SAT; s_sat = 0u; }""", 'satbit')

sub("""            s_pos_wait = 0.0f; s_baro_wait = 0.0f;""",
    """            s_pos_wait = 0.0f; s_baro_wait = 0.0f;
            s_alin_g = 0.0f; s_sat = 0u;""", 'satclr')

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1i')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: Q_tt 加整流项 + 加计削顶检测/门/上报位')
print()
print('fw_tag 期望 = %d' % ((17 << 16) | (112 << 8) | 1 | 2 | 4))
