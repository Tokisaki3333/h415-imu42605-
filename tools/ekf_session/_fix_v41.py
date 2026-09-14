# -*- coding: utf-8 -*-
"""VER=40 -> 41：磁偏航的**死点门**。

问题（用户指出，我漏了）：h = atan2(Bx,By) 的分母是水平分量 Bh，我那两个倾角列
H[0][6]=Bx*Bz/Bh^2、H[0][7]=By*Bz/Bh^2 也带 Bh^2。姿态正确时真实地磁在导航系的
倾角是 53.74 度 -> Bh/|B| = 0.59 恒成立，永远不会接近死点；但**姿态误差大到约
50 度以上时**，估计出来的 B 会接近竖直 -> Bh -> 0 -> atan2 输出任意值，
H 的系数放大到无穷。而"姿态误差 50 度"恰恰发生在多轴运动中（那时 tilt 门是关的，
倾角靠陀螺积分）—— 与"只有三轴同时转才极大误差"完全对上。
我原来的护栏 if (bh2 > 1e-6f) 形同虚设：Bh=0.001 时条件数已放大 1000 倍。

修：死点门 —— |B 的水平分量| / |B| 必须大于 V5F_EKF_MAG_BH_MIN（0.30），
否则整帧丢弃并记 bit9。0.59 的正常值有近 2 倍余量。
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
a = '#define V5F_EKF_MAG_K_MAX        0.002f'
assert u.count(a) == 1
u = u.replace(a, a + """
/* 磁偏航的**死点门**：|B 的水平分量| / |B| 必须大于它，否则该样本的方位角无定义。
 * 真实地磁在导航系的倾角是 53.74 度 -> Bh/|B| = 0.59 恒成立，所以正常样本有约
 * 2 倍余量；只有当**姿态误差大到约 50 度以上**时（多轴运动中 tilt 门关闭、
 * 倾角靠陀螺积分），估计出来的 B 才会接近竖直 -> Bh -> 0 -> atan2 输出任意值、
 * H 的倾角列放大到无穷 —— 实测单周期跳到 169 度就是这么来的。
 * ★ 原来那个 if (bh2 > 1e-6f) 形同虚设：Bh = 0.001 时条件数已经放大 1000 倍。 */
#define V5F_EKF_MAG_BH_MIN       0.30f""", 1)
assert u.count('#define V5F_FW_VER        40u') == 1
u = u.replace('#define V5F_FW_VER        40u', '#define V5F_FW_VER        41u', 1)
assert u.count('/*') == u.count('*/')
sw(T, u, 'gbk', 's2l')
print('v5f_tune.h: MAG_BH_MIN + VER 41')

t = open(P, 'rb').read().decode('gbk')
a = """    {
        float bh2 = Bn[0]*Bn[0] + Bn[1]*Bn[1];
        H_zero(1u);
        if (bh2 > 1e-6f) {
            s_H[0][6] = Bn[0]*Bn[2]/bh2;
            s_H[0][7] = Bn[1]*Bn[2]/bh2;
        }"""
assert t.count(a) == 1, t.count(a)
b = """    {
        float bh2 = Bn[0]*Bn[0] + Bn[1]*Bn[1];
        /* ★ 死点门：水平分量太小 -> 方位角无定义（atan2 数值爆炸、倾角列放大到无穷）。
         *   正常值 Bh/|B| = cos(53.74 度) = 0.59，余量约 2 倍。见 v5f_tune.h 的注释。 */
        if (bh2 < V5F_EKF_MAG_BH_MIN * V5F_EKF_MAG_BH_MIN) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
            return;
        }
        H_zero(1u);
        {
            s_H[0][6] = Bn[0]*Bn[2]/bh2;
            s_H[0][7] = Bn[1]*Bn[2]/bh2;
        }"""
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2l')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=41', ver == 41),
             ('死点门在', 'bh2 < V5F_EKF_MAG_BH_MIN * V5F_EKF_MAG_BH_MIN' in c),
             ('旧护栏已去', 'bh2 > 1e-6f' not in c),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
