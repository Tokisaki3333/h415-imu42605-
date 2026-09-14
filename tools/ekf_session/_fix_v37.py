# -*- coding: utf-8 -*-
"""VER=36 -> 37：姿态观测（重力 M6）也加**单周期修正上限**。

病：实测 ekf_q 全程抖（静止也抖），而旧链 att_q 稳。
    我先前"运动时整流项抬高 P -> K 变大"的解释不成立 —— 静止也抖，与运动无关。
    真因是**带宽**：
      旧链 Mahony tau=5 s        -> 闭环带宽 0.03 Hz -> 噪声压 ~76 倍，稳
      我的 EKF R=(0.5 度)^2，每 2.87 ms 满增益更新 -> 带宽 ~100 Hz -> 噪声原样进姿态
    根子：把"单样本观测噪声"直接当 R，然后以 349 Hz 全增益更新。而加计/磁的
    方向误差（DC 偏置、逐样本噪声、安装残差）**不是白噪声、是相关的** ——
    拿它全带宽喂姿态就是把噪声灌进去。这与气压那个错同源（相关观测必须先限带宽）。
修：本周期姿态修正角度上限 V5F_EKF_TILT_STEP_MAX_DEG = 0.02 度
    -> 等效 tau = dt/(0.02*deg2rad) = 8.2 s（与旧链 Mahony 的 5 s 同量级）。
    小角度下用"线性回缩"代替 slerp，误差 < 1e-4。
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
a = '#define V5F_EKF_YAW_STEP_MAX_DEG 0.05f'
assert u.count(a) == 1
u = u.replace(a, a + """
/* 重力/倾斜观测（M6）的**单周期姿态修正上限**（度）。
 * 实测 ekf_q 全程抖（静止也抖）而旧链稳，根因是带宽而非增益大小：
 *   R=(0.5 度)^2 配 349 Hz 满增益更新 -> 闭环带宽约 100 Hz，
 *   加计方向噪声（0.127 度）几乎原样进姿态；旧链 Mahony tau=5 s -> 0.03 Hz。
 * 0.02 度/周期 -> 等效 tau = 2.87ms/(0.02*0.01745) = 8.2 s，与旧链同量级。
 * 加计/磁的方向误差是**相关**的（DC 偏置、安装残差），不是白噪声，
 * 必须先限带宽再进姿态。 */
#define V5F_EKF_TILT_STEP_MAX_DEG 0.02f""", 1)
assert u.count('#define V5F_FW_VER        36u') == 1
u = u.replace('#define V5F_FW_VER        36u', '#define V5F_FW_VER        37u', 1)
assert u.count('/*') == u.count('*/')
sw(T, u, 'gbk', 's2g')
print('v5f_tune.h: TILT_STEP_MAX_DEG + VER 37')

t = open(P, 'rb').read().decode('gbk')
a = """    st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0], 0x7FC0u);"""
assert t.count(a) == 1, t.count(a)
b = """    {
        /* ★ 单周期姿态修正上限（= 重力环的 tau 下限，约 8 s）。
         *   没有它，R=(0.5 度)^2 配 349 Hz 满增益更新，闭环带宽约 100 Hz，
         *   加计的方向噪声（0.127 度）几乎原样进姿态 —— 实测 ekf_q 全程抖，
         *   而旧链 Mahony（tau=5 s，带宽 0.03 Hz）稳。加计/磁的方向误差是
         *   **相关**的（DC 偏置、安装残差），不是白噪声，必须先限带宽。 */
        float q_before[4];
        uint32_t n;
        for (n = 0u; n < 4u; n++) q_before[n] = s_x[IX_Q + n];
        st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0], 0x7FC0u);
        {
            float qc[4], dq[4], ang, cap = V5F_EKF_TILT_STEP_MAX_DEG * DEG2RAD;
            qc[0] = q_before[0]; qc[1] = -q_before[1];
            qc[2] = -q_before[2]; qc[3] = -q_before[3];
            q_mul(&s_x[IX_Q], qc, dq);        /* 相对旋转，小角度 */
            ang = 2.0f * sqrtf(dq[1]*dq[1] + dq[2]*dq[2] + dq[3]*dq[3]);
            if (ang > cap && ang > 1e-9f) {
                float sc = cap / ang;          /* 线性回缩（小角度等价于 slerp） */
                float qn[4];
                for (n = 0u; n < 4u; n++) {
                    qn[n] = q_before[n] + (s_x[IX_Q + n] - q_before[n]) * sc;
                }
                for (n = 0u; n < 4u; n++) s_x[IX_Q + n] = qn[n];
                q_norm(&s_x[IX_Q]);
            }
        }
    }"""
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2g')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=37', ver == 37), ('倾角限幅在', 'V5F_EKF_TILT_STEP_MAX_DEG * DEG2RAD' in c),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
