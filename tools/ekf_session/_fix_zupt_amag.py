# -*- coding: utf-8 -*-
"""VER=19 -> 20：
 1) ZUPT 门去掉 a_lin，改用**外生**的 ||a_离线校正| - 1|。
    a_lin 是旧链加速度牵引环的在线残差，而那个环自己的判据就是 |a_lin|
    （ON=40mg/OFF=100mg）→ 实测卡在极限环里（静置段 |a_lin| p50 0.0487、
    max 0.1084 g），ZUPT 门因此只有 52.7% 开。这违反"门的判据只依赖本帧原始
    传感器值 + 离线标定常量"。
    换成 | |a| - 1 |：静止时比力模长恒为 1 g，**与姿态无关**，只用原始 LSB +
    离线常量，实测静止残差 2.2 mg（比运动小两个数量级），不需要迟滞。
 2) 上报加 ekf.sigma_tilt_deg（列 107，nis 后移一位）—— 剧烈运动里
    r_yaw = 真实偏航误差 + tan(I)*倾角误差，而倾角误差无法从别处推出来，
    没有 sigma_tilt 就无法验证 M7 那个自适应 R 是否诚实。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
H = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\SPI_rx.h'
S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'

# ---------------- v5f_tune.h ----------------
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_ZUPT_ALIN2       (0.05f * 0.05f)   /* |a_lin| < 0.05 g（a_lin 单位就是 g）*/'
assert u.count(a) == 1
u = u.replace(a, """/* ★ 不要再用 a_lin 做 ZUPT 判据。a_lin 是旧链加速度牵引环的**在线残差**，而那个环
 * 自己的判据就是 |a_lin|（V5F_ACG_ALIN_ON_MG=40 / OFF_MG=100）—— 实测它卡在自己的
 * 极限环里：静置段 |a_lin| p50 0.0487、max 0.1084 g，正好压在 40~100 mg 之间，
 * ZUPT 门因此只开 52.7%。用它就是把另一个环的病理接手过来。
 * 正确的外生量是 | |a| - 1 |：静止时比力模长恒为 1 g（**与姿态无关**），
 * 只用原始 LSB + 离线常量即可算出，实测静止残差 2.2 mg（运动大两个数量级）。 */
#define V5F_EKF_ZUPT_AMAG_TOL    0.03f   /* | |a_离线| - 1 | 上限（g）。3% 已比静止残差
                                          * （2.2 mg）宽 13 倍，且真实运动轻松超过它 */
#define V5F_EKF_ZUPT_ALIN2       (0.05f * 0.05f)   /* 已废弃：只留常量以便对照旧记录 */""", 1)
assert u.count('#define V5F_FW_VER        19u') == 1
u = u.replace('#define V5F_FW_VER        19u', '#define V5F_FW_VER        20u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1l')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: ZUPT 判据换外生量 + VER 20')

# ---------------- SPI_rx.h：sigma_tilt ----------------
t = open(H, 'rb').read().decode('gbk')
a = "    float            sigma_pos_h;     /* 水平位置 1sigma，m */"
assert t.count(a) == 1
t = t.replace(a, """    float            sigma_tilt_deg;  /* 倾角(水平两轴合成) 1sigma，度。
                                       * 剧烈运动里 r_yaw = 真实偏航误差 + tan(I)*倾角误差，
                                       * 倾角误差只能从这里读 —— 没有它就无法验证
                                       * M7 的自适应 R 是否诚实。 */
    float            sigma_pos_h;     /* 水平位置 1sigma，m */""", 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(H, H + '.bak_s1l')
open(H, 'wb').write(t.encode('gbk'))
print('SPI_rx.h: sigma_tilt_deg 字段')

# ---------------- proc_ekf.c ----------------
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


sub("static float    s_sig[3];             /* 偏航(度) / 水平位置(m) / 水平速度(m/s) */",
    "static float    s_sig[4];             /* 偏航(度) / 水平位置(m) / 水平速度(m/s) / 倾角(度) */",
    'sig')
sub("""    s_sig[2] = sqrtf(s_Pn[3][3] + s_Pn[4][4]);
}""",
    """    s_sig[2] = sqrtf(s_Pn[3][3] + s_Pn[4][4]);
    s_sig[3] = sqrtf(s_Pn[6][6] + s_Pn[7][7]) * RAD2DEG;   /* 倾角（水平两轴合成） */
}""", 'sig2')
sub("    h->ekf.sigma_vel_h   = s_sig[2];",
    "    h->ekf.sigma_vel_h   = s_sig[2];\n    h->ekf.sigma_tilt_deg = s_sig[3];", 'pub')
sub("""            s_sig[2] = V5F_EKF_P0_VEL_MPS;""",
    """            s_sig[2] = V5F_EKF_P0_VEL_MPS;
            s_sig[3] = V5F_EKF_P0_TILT_RAD * RAD2DEG;""", 'sig0')

# ZUPT 判据换外生量
sub("""    gate->ekf_zupt = (uint8_t)(h->stat.is_static
                    && (h->stat.e_ac < V5F_EKF_ZUPT_EAC_MAX)
                    && ((h->imu.a_lin[0]*h->imu.a_lin[0]
                       + h->imu.a_lin[1]*h->imu.a_lin[1]
                       + h->imu.a_lin[2]*h->imu.a_lin[2]) < V5F_EKF_ZUPT_ALIN2)
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));""",
    """    /* | |a_离线校正| - 1 |：**外生**（原始 LSB + 离线标定常量），且与姿态无关 ——
     * 静止时比力模长恒为 1 g。原来用 a_lin，那是旧链牵引环的在线残差，而那个环自己
     * 的判据就是 |a_lin|，实测卡在极限环里（静置 |a_lin| p50 0.0487 g），
     * ZUPT 门因此只开 52.7%。 */
    {
        static const float b0[3] = { V5F_ACCEL_BIAS_LSB_X, V5F_ACCEL_BIAS_LSB_Y,
                                     V5F_ACCEL_BIAS_LSB_Z };
        float s2 = 0.0f;
        uint32_t k;
        for (k = 0u; k < 3u; k++) {
            float v = ((float)h->imu.accel_lsb[k] - b0[k]) / g_v5f_accel_lsb_per_g[k];
            s2 += v * v;
        }
        amag_ok = (uint8_t)(h->imu.acc_valid
                            && (fabsf(sqrtf(s2) - 1.0f) < V5F_EKF_ZUPT_AMAG_TOL));
    }
    gate->ekf_zupt = (uint8_t)(h->stat.is_static
                    && (h->stat.e_ac < V5F_EKF_ZUPT_EAC_MAX)
                    && amag_ok
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));""", 'zupt')
sub("    uint8_t  gq_ok, gn_ok, snr_ok, vd_ok;",
    "    uint8_t  gq_ok, gn_ok, snr_ok, vd_ok, amag_ok;", 'decl')

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1l')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: ZUPT 外生判据 + sigma_tilt')

# ---------------- SPI_rx.c：加一列 ----------------
t = open(S, 'rb').read().decode('gbk')
a = '#define JF_CH_NUM     112u'
assert t.count(a) == 1
t = t.replace(a, '#define JF_CH_NUM     113u', 1)
a = '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 454 = 帧头2+帧长2+载荷448+帧尾2 */'
assert t.count(a) == 1
t = t.replace(a, '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 458 = 帧头2+帧长2+载荷452+帧尾2 */', 1)
a = "    ch[c++] = g_v5f_hold.ekf.sigma_vel_h;\n"
assert t.count(a) == 1
t = t.replace(a, a + "    ch[c++] = g_v5f_hold.ekf.sigma_tilt_deg;   /* 倾角 1sigma（度）*/\n", 1)
a = '     * 108       sigma_vel_h     水平速度 1sigma m/s'
assert t.count(a) == 1
t = t.replace(a, """     * 107       sigma_tilt_deg  倾角(水平两轴合成) 1sigma（度）—— 剧烈运动里
     *                          r_yaw = 真实偏航误差 + tan(I)*倾角误差，倾角误差只能从
     *                          这里读；也是验证 M7 自适应 R 是否诚实的唯一途径
     * 108       sigma_vel_h     水平速度 1sigma m/s""", 1)
for old, new in [('     * 108..112  ekf.nis[5]', '     * 109..113  ekf.nis[5]'),
                 ('     * ★ 0~82 列号一个都不动，新量只追加在尾部。 */',
                  '     * ★ 0~106 列号一个都不动，107 起为新增/顺移（共 113 列）。 */')]:
    assert t.count(old) == 1, old
    t = t.replace(old, new, 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(S, S + '.bak_s1l')
open(S, 'wb').write(t.encode('gbk'))
print('SPI_rx.c: 113 列 + sigma_tilt 上报')

# ---------------- 工具表 ----------------
import re
CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
c = c.replace("assert n == 112, '运行时列数变了(%d) —— 同步改 JF_CH_NUM 与本表' % n",
              "assert n == 113, '运行时列数变了(%d) —— 同步改 JF_CH_NUM 与本表' % n")
c = c.replace("'sigma_pos_h': 105, 'sigma_vel_h': 106, 'nis': 107}",
              "'sigma_pos_h': 105, 'sigma_vel_h': 107, 'sigma_tilt': 106, 'nis': 108}")
open(CP, 'w', encoding='utf-8', newline='\n').write(c)

JL = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()
a = "CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112}"
assert j.count(a) == 1
j = j.replace(a, """# ---- VER=20：113 列（在 112 列的基础上插了 ekf_sigma_tilt_deg，nis 后移一位）----
CH_113 = dict(CH_112)
CH_113.update({k: (v + 1 if v >= 107 else v) for k, v in list(CH_113.items())})
CH_113.update({'ekf_sigma_tilt_deg': 107, 'ekf_nis': 108})

CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113}""", 1)
j = j.replace("CH = CH_112", "CH = CH_113", 1)
j = j.replace("""    if nch == 112:
        return CH_112""", """    if nch == 112:
        return CH_112
    if nch == 113:
        return CH_113""", 1)
j = j.replace('未知帧长 %d 通道；已知 78/80/112', '未知帧长 %d 通道；已知 78/80/112/113', 1)
open(JL, 'w', encoding='utf-8', newline='\n').write(j)
print('count_cols.py / jf_load.py: 113 列表已同步')
print()
print('fw_tag 期望 = %d' % ((20 << 16) | (113 << 8) | 1 | 2 | 4))
