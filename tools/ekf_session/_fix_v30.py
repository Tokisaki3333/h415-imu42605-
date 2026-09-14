# -*- coding: utf-8 -*-
"""VER=29 -> 30：① M7 的 H 补上倾角两列、R 去掉人为膨胀 ② ekf_update 加注入掩码。

M7 正确雅可比（已推导）：h = atan2(Bx,By)，B = R(q)f，dth 为导航系姿态误差
    dh/ddx = Bx*Bz/Bh^2,  dh/ddy = By*Bz/Bh^2,  dh/ddz = -1
两项系数就是 tan(I_measured) = 2.08（实测倾角 64.26 度）。原来只写了 -1，
于是倾角误差被整块记到偏航上 -> bg_z 被推到钳位、偏航转飞（VER=27/29 两次）。
写进 H 之后，R 里的 (2.08*sigma_tilt)^2 必须去掉：耦合不再是噪声，否则重复计数。
掩码：让每道观测只能注入它有物理资格动的状态（其余 K 行清零，dx 与 P 同时受限）。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
T = R + r'\V5F\User\inc\v5f_tune.h'
P = R + r'\V5F\User\src\proc_ekf.c'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)
    print('  ok:', nm)


# ---- ① ekf_update 签名 + K 行掩码 ----
sub("""static uint8_t ekf_update(const float *R, uint8_t m, const float *r,
                          float nis_max, float *nis_out, uint8_t *rej)""",
    """static uint8_t ekf_update(const float *R, uint8_t m, const float *r,
                          float nis_max, float *nis_out, uint8_t *rej,
                          uint16_t inj_mask)""", 'sig')
sub("""    for (i = 0u; i < EKF_N; i++) {
        s = 0.0f;
        for (k = 0u; k < m; k++) s += s_K[i][k] * r[k];
        s_dx[i] = s;
    }""",
    """    /* ★ 注入掩码：每道观测只能动它有物理资格动的状态。
     * 把 K 里不该动的行清零，dx 与 P 的修正就同时受限 —— 这叫"观测在子空间里作用"。
     * 为什么必须做：K = P H^T S^-1 会通过协方差交叉项把观测注入到 H 里根本没有的
     * 状态上。实测 VER=29：气压观测一恢复，就经 P[8][2] 往偏航注入，bg_z 顶到
     * ±10 dps 钳位、偏航以 10 dps 转飞（-156 度 vs 旧链 +4 度）。 */
    for (i = 0u; i < EKF_N; i++) {
        if (!(inj_mask & (uint16_t)(1u << i))) {
            for (j = 0u; j < m; j++) s_K[i][j] = 0.0f;
        }
    }
    /* dx = K r */
    for (i = 0u; i < EKF_N; i++) {
        s = 0.0f;
        for (k = 0u; k < m; k++) s += s_K[i][k] * r[k];
        s_dx[i] = s;
    }""", 'mask')

# ---- ② 七个调用点加掩码 ----
for old, new, nm in [
    ("ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0]);",
     "ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0], 0x01C0u);", 'M6'),
    ("ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[1], &s_rej[1]);",
     "ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[1], &s_rej[1], 0x0038u);", 'M5'),
    ("ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);",
     "ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2], 0x0004u);", 'M3'),
    ("ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3]);",
     "ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3], 0x0180u);", 'M7'),
    ("ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[0], &s_rej[4]);",
     "ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[0], &s_rej[4], 0x0003u);", 'M1'),
    ("ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL, &s_rej[4]);",
     "ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL, &s_rej[4], 0x8004u);", 'M2'),
    ("ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[1], &s_rej[1]);",
     "ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[1], &s_rej[1], 0x0018u);", 'M4'),
]:
    sub(old, new, nm)

# ---- ③ M7 的 H 补倾角两列 + R 去掉膨胀 ----
sub("""    H_zero(1u);
    s_H[0][8] = -1.0f;                     /* dh/d(dth_z) = -1（弧度/弧度） */
    /* 自适应 R：观测值里除了磁力计自身噪声，还混进了**当前倾角误差**的耦合
     * （方位角偏 tan(I)*dtheta_h，见 v5f_tune.h 的 V5F_EKF_DIP_TAN）。
     * 用已传播的 P 自己算 —— 这是自适应 R，不是门；门仍然只用外生量。 */
    R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD
         + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN * (s_Pn[6][6] + s_Pn[7][7]);""",
    """    /* ★ 完整的雅可比：h = atan2(Bx, By)，B = R(q)f，dth 为导航系姿态误差
     *   dh/ddx = Bx*Bz/Bh^2,  dh/ddy = By*Bz/Bh^2,  dh/ddz = -1
     * 前两项的系数就是 tan(I_measured) = 2.08（实测倾角 64.26 度）。
     * 原来只写了 -1，于是倾角误差被**整块记到偏航头上** -> bg_z 顶到钳位、偏航转飞
     * （VER=27/29 两次实测 -156 度 vs 旧链 +4 度）。补上之后：
     *   · 磁观测同时约束倾角（运动中多一个姿态参考，正好补重力观测关闭的空档）
     *   · R 里那个 (2.08*sigma_tilt)^2 必须**去掉** —— 耦合已经在 H 里，
     *     留在 R 里就是重复计数，等于把信息白白扔掉。 */
    {
        float bh2 = Bn[0]*Bn[0] + Bn[1]*Bn[1];
        H_zero(1u);
        if (bh2 > 1e-6f) {
            s_H[0][6] = Bn[0]*Bn[2]/bh2;
            s_H[0][7] = Bn[1]*Bn[2]/bh2;
        }
        s_H[0][8] = -1.0f;
        R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;
    }""", 'm7')

assert t.count('/*') == t.count('*/')
assert t.count('{') == t.count('}')
sw(P, t, 'gbk', 's1x')
print('proc_ekf.c: M7 雅可比补全 + 注入掩码 + 7 个调用点')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        29u') == 1
u = u.replace('#define V5F_FW_VER        29u', '#define V5F_FW_VER        30u', 1)
sw(T, u, 'gbk', 's1x')

p2 = open(P, 'rb').read().decode('gbk')
s2 = open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', u).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
print()
for k, v in [('VER=30', ver == 30),
             ('M7 第6列', 's_H[0][6] = Bn[0]*Bn[2]/bh2' in p2),
             ('M7 第7列', 's_H[0][7] = Bn[1]*Bn[2]/bh2' in p2),
             ('R 不再膨胀', 'V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN * (s_Pn' not in p2),
             ('掩码在', 'inj_mask & (uint16_t)(1u << i)' in p2),
             ('7 处调用带掩码', p2.count('0x01C0u') + p2.count('0x0038u') + p2.count('0x0004u')
              + p2.count('0x0180u') + p2.count('0x0003u') + p2.count('0x8004u')
              + p2.count('0x0018u') == 7),
             ('花括号平衡', p2.count('{') == p2.count('}')),
             ('注释配平', p2.count('/*') == p2.count('*/'))]:
    print('  %-16s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
