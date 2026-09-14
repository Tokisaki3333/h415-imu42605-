# -*- coding: utf-8 -*-
"""补做 VER=36：只改 proc_ekf.c（v5f_tune.h 上一轮已写好）。"""
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


t = open(P, 'rb').read().decode('gbk')

if 'static float q_yaw' not in t:
    a = 'static void H_zero(uint8_t m)'
    assert t.count(a) == 1
    t = t.replace(a, """/* 四元数的导航系偏航角（与上报里的定义一致） */
static float q_yaw(const float *q)
{
    return atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
}

static void H_zero(uint8_t m)""", 1)
    print('  q_yaw 工具已加')

a = ("    r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);\n"
     "    st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3], 0x0100u);\n"
     "    if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;")
assert t.count(a) == 1, t.count(a)
b = """    r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);
    {
        /* ★ 增益限幅：本周期对偏航的修正不得超过 V5F_EKF_YAW_STEP_MAX_DEG。
         *   没有它，运动中被整流项抬高的 P[8][8] 会让 K 冲到 0.9，磁观测变成
         *   "每周期硬拉"，偏航就逐样本跟随 IST8310 的 0.5~0.7 度噪声 —— 实测正是
         *   静止稳、缓慢抬板边自行震动，而旧链（纯积分 + 低增益倾斜修正）完全稳定。
         *   限幅后它变成一个有 tau 下限（约 3.3 s）的**牵引**：噪声被压平，
         *   而真实陀螺零偏漂移（0.01~0.1 dps）远在限幅速率之内，照拉无误。 */
        float y0 = q_yaw(&s_x[IX_Q]);
        float y1, dlt, lim = V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD;
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3], 0x0100u);
        y1 = q_yaw(&s_x[IX_Q]);
        dlt = wrap_pi(y1 - y0);
        if (dlt > lim || dlt < -lim) {
            float corr = dlt - ((dlt > 0.0f) ? lim : -lim);
            float hq[4], qt[4];
            uint32_t n2;
            hq[0] = cosf(-0.5f * corr); hq[1] = 0.0f; hq[2] = 0.0f;
            hq[3] = sinf(-0.5f * corr);
            q_mul(hq, &s_x[IX_Q], qt);
            for (n2 = 0u; n2 < 4u; n2++) s_x[IX_Q + n2] = qt[n2];
            q_norm(&s_x[IX_Q]);
        }
    }
    if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;"""
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2f')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=36', ver == 36), ('限幅在', 'V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD' in c),
             ('q_yaw 在', 'static float q_yaw' in c),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
