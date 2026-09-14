# -*- coding: utf-8 -*-
"""VER=21 -> 22：
 A) 耦合系数用**实测倾角**：tan(64.26 度)=2.08，不是 WMM 的 tan(53.74 度)=1.364。
    实测（静止段 19599 帧）：EKF 姿态量出的磁倾角 64.261±1.956 度，
    而 WMM 真值 53.74 度 —— 误差 10.5 度，且随姿态变 4.1 度（两静止姿内部
    std 仅 0.16~0.18 度 -> 确定性软铁/正交性缺陷，不是噪声）。
    方位角对倾角误差的灵敏度是 tan(I_measured)，所以自适应 R 原来小了 1.5 倍。
    ★ 这条同时说明**三维磁矢量不能用来约束倾角**：会把 10.5 度的姿相关误差
      灌进倾角，而当前倾角误差只有 0.14 度（差 75 倍）。要走路 1，先重做磁标定。

 B) 水平速度的**死区**（路径 2）。泄漏是常值水平偏置 g·dtheta_h，与真实常值
    加速度在观测上不可分；但代价不对称：泄漏造成的是**无界增长**的速度误差，
    而被吃掉的真实小加速度是**瞬态**的（悬停时水平加速度本就近 0）。
    死区按滤波器自己的倾角不确定度**正比例缩放**：T = K*g*sigma_tilt_h，
    所以"倾角越不可信 -> 速度通道越不敏感"，自洽。
    ★ 死区只改**估计**，不改协方差：Q_vv 里那一项（VER=21 加的）继续让
      sigma_v 诚实增长，下游仍然知道速度不能信。
"""
import re
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_DIP_TAN          1.364f'
assert u.count(a) == 1
u = u.replace(a, """#define V5F_EKF_DIP_TAN          2.08f   /* = tan(I_measured) = tan(64.26 度)。
                                          * ★ 用**实测**倾角，不用 WMM 的 53.74 度：
                                          *   方位角 h 对倾角误差的灵敏度就是 tan(I_measured)
                                          *   （实测静止段 EKF 姿态量出的 I = 64.26 +- 1.96 度，
                                          *   WMM 真值 53.74 度 -> 磁标定有 10.5 度软铁误差，
                                          *   且随姿态变 4.1 度，内部 std 仅 0.16 度）。
                                          *   原来写 1.364 是把自适应 R 给小了 1.5 倍。 */""", 1)
a = '#define V5F_EKF_ZUPT_AMAG_TOL    0.03f'
assert u.count(a) == 1
u = u.replace(a, """/* 水平速度死区系数：T = K * g * sigma_tilt_h（每次 EKF 步都按当前倾角不确定度重算）。
 * 目的是挡掉"倾角误差把重力漏进水平加速度"造成的常值偏置（实测 sigma_tilt 0.63 度
 * -> 0.108 m/s^2；不挡的话速度会以这个速率无界漂）。代价是被吃掉的真实小加速度也
 * 在这一量级 —— 但它是**瞬态**的，而泄漏造成的是**无界增长**，两者不对称。
 * 设 0 即关闭。 */
#define V5F_EKF_VDEAD_K          1.0f

#define V5F_EKF_ZUPT_AMAG_TOL    0.03f""", 1)
assert u.count('#define V5F_FW_VER        21u') == 1
u = u.replace('#define V5F_FW_VER        21u', '#define V5F_FW_VER        22u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1o')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: DIP_TAN=2.08 + VDEAD_K + VER 22')

t = open(P, 'rb').read().decode('gbk')
a = """    for (i = 0u; i < 3u; i++) s_x[IX_V + i] += dv[i];
    for (i = 0u; i < 3u; i++) s_a_nav[i] = an[i];"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, """    for (i = 0u; i < 3u; i++) s_x[IX_V + i] += dv[i];
    for (i = 0u; i < 3u; i++) s_a_nav[i] = an[i];""", 1)

# 死区：加在 dv 用于位置/速度推进**之前**的水平两轴
a2 = """    for (i = 0u; i < 3u; i++) an[i] = dv[i] / s_dt_e;
    for (i = 0u; i < 3u; i++) {
        s_x[IX_P + i] += s_x[IX_V + i] * s_dt_e + 0.5f * an[i] * s_dt_e * s_dt_e;
    }
    for (i = 0u; i < 3u; i++) s_x[IX_V + i] += dv[i];"""
assert t.count(a2) == 1, t.count(a2)
b2 = """    /* ---- 水平速度死区（挡"重力经倾角误差漏进水平加速度"的常值偏置）----
     * 这个偏置 g*dtheta_h 与真实常值加速度在观测上不可分，所以只能按量级取舍：
     * 泄漏造成**无界增长**的速度误差，而被吃掉的真实小加速度是**瞬态**的
     * （悬停时水平加速度本就近 0）。死区随滤波器自己的倾角不确定度正比例缩放，
     * 于是"倾角越不可信 -> 速度通道越不敏感"，自洽。
     * ★ 只改估计，不改协方差：Q_vv 里那一项继续让 sigma_v 诚实增长。 */
    if (V5F_EKF_VDEAD_K > 0.0f) {
        float td = V5F_EKF_VDEAD_K * V5F_EKF_G_MPS2
                 * sqrtf(s_Pn[6][6] + s_Pn[7][7]) * s_dt_e;   /* 本周期对应的 dv 门限 */
        for (i = 0u; i < 2u; i++) {
            if (dv[i] < td && dv[i] > -td) dv[i] = 0.0f;
        }
    }
    for (i = 0u; i < 3u; i++) an[i] = dv[i] / s_dt_e;
    for (i = 0u; i < 3u; i++) {
        s_x[IX_P + i] += s_x[IX_V + i] * s_dt_e + 0.5f * an[i] * s_dt_e * s_dt_e;
    }
    for (i = 0u; i < 3u; i++) s_x[IX_V + i] += dv[i];"""
t = t.replace(a2, b2, 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1o')
open(P, 'wb').write(t.encode('gbk'))

chk = open(P, 'rb').read().decode('gbk')
for k, v in [('死区已加', 'td = V5F_EKF_VDEAD_K * V5F_EKF_G_MPS2' in chk),
             ('死区在 an 计算之前', chk.index('td = V5F_EKF_VDEAD_K')
              < chk.index('an[i] = dv[i] / s_dt_e')),
             ('注释配平', chk.count('/*') == chk.count('*/'))]:
    print('  %-16s %s' % (k, v))
    assert v, k
import sys
ut = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', ut).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c',
                         'rb').read().decode('gbk')).group(1))
print()
print('VER=%d NCH=%d fw_tag = %d' % (ver, nch, (ver << 16) | (nch << 8) | 1 | 2 | 4))
