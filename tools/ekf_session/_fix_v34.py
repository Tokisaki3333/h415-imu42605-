# -*- coding: utf-8 -*-
"""VER=33 -> 34：路径 B —— 速度泄漏积分器，泄漏率由 sigma_tilt 自适应。

为什么必须是"泄漏"而不是"死区"：
  死区按幅值砍，会把真实低频运动一起砍掉（实测一段 |a_lin|=0.129、门限 0.038 的
  真实运动被吃掉 86%），而且门限正比于 sigma_tilt，越不确定越激进。
  泄漏积分器 v *= (1 - lambda*dt) 不砍任何单帧信息，只把误差**限成有界**：
  常值泄漏 b 的稳态速度误差 = b/lambda = b*tau。
  取 lambda = K*sigma_tilt（tau = 1/(K*sigma_tilt)）：
    sigma_tilt = 0.2 度 -> tau = 57 s   （姿态可信，几乎不干预，慢速平移精度保住）
    sigma_tilt = 5 度   -> tau = 2.3 s  （姿态不可信，把速度限在 g*0.087*2.3 = 1.7 m/s）
  只作用于**水平两轴**：垂直方向有气压/ZUPT 约束，不需要。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_VDEAD_K          0.0f'
assert u.count(a) == 1
u = u.replace(a, a + """
/* ---- 路径 B：速度泄漏积分器（替代死区）----
 * lambda = K * sigma_tilt，即 tau = 1/(K*sigma_tilt)，每次 EKF 步对**水平两轴**做
 *   v *= (1 - lambda*dt_e)
 * 为什么不用死区：死区按幅值砍，实测把一段真实运动吃掉 86%，且门限正比于
 *   sigma_tilt -> 越不确定越激进。泄漏不砍任何单帧信息，只把常值泄漏造成的
 *   无界漂移限成有界（稳态误差 = b*tau）。
 * K=5 时：sigma_tilt 0.2 度 -> tau 57 s（几乎不干预）；5 度 -> tau 2.3 s（限住）。
 * 设 K=0 即关闭。 */
#define V5F_EKF_VLEAK_K          5.0f
#define V5F_EKF_VLEAK_TAU_MAX_S  10000.0f   /* tau 上限（lambda 下限），防姿态极好时不泄漏 */""", 1)
assert u.count('#define V5F_FW_VER        33u') == 1
u = u.replace('#define V5F_FW_VER        33u', '#define V5F_FW_VER        34u', 1)
assert u.count('/*') == u.count('*/')
sw(T, u, 'gbk', 's2c')
print('v5f_tune.h: VLEAK_K / VLEAK_TAU_MAX_S + VER 34')

t = open(P, 'rb').read().decode('gbk')
old = """    for (i = 0u; i < 3u; i++) an[i] = dv[i] / s_dt_e;"""
assert t.count(old) == 1, t.count(old)
new = """    /* ---- 路径 B：速度泄漏（只作用于水平两轴）----
     * lambda = K*sigma_tilt，即 tau = 1/(K*sigma_tilt)。姿态可信时 tau 很大
     * （几乎不干预，保住慢速平移精度）；姿态不可信时 tau 变小，把"重力经倾角
     * 误差漏进加速度"造成的**无界**漂移限成有界（稳态误差 = b*tau）。
     * 与死区的本质区别：这里不砍任何单帧信息，只给速度一个与不确定度匹配的漏。 */
    if (V5F_EKF_VLEAK_K > 0.0f) {
        float stg = sqrtf(s_Pn[6][6] + s_Pn[7][7]);      /* 倾角 1sigma，rad */
        float lam = V5F_EKF_VLEAK_K * stg;
        float lam_min = 1.0f / V5F_EKF_VLEAK_TAU_MAX_S;
        if (lam < lam_min) lam = lam_min;
        for (i = 0u; i < 2u; i++) {
            dv[i] -= s_x[IX_V + i] * lam * s_dt_e;
        }
    }
    for (i = 0u; i < 3u; i++) an[i] = dv[i] / s_dt_e;"""
t = t.replace(old, new, 1)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2c')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=34', ver == 34), ('泄漏在', 'V5F_EKF_VLEAK_K * stg' in c),
             ('只水平两轴', 'for (i = 0u; i < 2u; i++) {\n            dv[i] -= s_x[IX_V + i]' in c),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
