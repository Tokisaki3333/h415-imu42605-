# -*- coding: utf-8 -*-
"""VER=34 -> 35：M7 掩码改为**只动偏航**（0x0100），但保留 H 的倾角两列。

为什么这是对的、而不是退回到 VER=30 的错误：
  · VER=30 的错误是 H 里**没有**倾角列 -> 新息把倾斜误差整块当成偏航误差 ->
    bg_z 顶钳位、偏航转飞。
  · VER=34 的错误是把倾角列**放开修正** -> 这套磁力计的 dip 标定实测差 10.5 度
    （64.26 vs WMM 53.74），让它去修倾角等于往倾角灌 10 度系统误差，
    再经 tan(I)=2.08 的杠杆进方位角 -> 偏航偏 151 度（bg 却是好的 0.005 dps，
    正说明不是零偏在驱动）。
  · 正确：H 保留倾角列（新息能**正确预测**倾斜的贡献，不再误记），
    但 K 的倾角行清零（磁观测**不修正**倾角）。这叫带约束的部分更新：
    不做最优，但绝不用一个标定不准的量去污染姿态。
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


t = open(P, 'rb').read().decode('gbk')
a = '&s_nis[4], &s_rej[3], 0x71C0u);'
assert t.count(a) == 1, t.count(a)
t = t.replace(a, '&s_nis[4], &s_rej[3], 0x0100u);', 1)
old = '        s_H[0][8] = -1.0f;'
assert t.count(old) == 1
t = t.replace(old, """        /* 倾角列保留在 H 里（新息要能预测倾斜的贡献），但掩码只放行第 8 项：
         * 磁观测**不修正倾角** —— 这套磁力计的 dip 标定实测差 10.5 度，
         * 用它修倾角会把 10 度系统误差灌进姿态（VER=34 实测偏航偏 151 度，
         * 而 bg 只有 0.005 dps，说明不是零偏在驱动）。 */
        s_H[0][8] = -1.0f;""", 1)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2d')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        34u') == 1
u = u.replace('#define V5F_FW_VER        34u', '#define V5F_FW_VER        35u', 1)
sw(T, u, 'gbk', 's2d')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=35', ver == 35), ('M7 掩码只偏航', '&s_rej[3], 0x0100u);' in c),
             ('H 倾角列仍在', 's_H[0][6] = Bn[0]*Bn[2]/bh2' in c and 's_H[0][7] = Bn[1]*Bn[2]/bh2' in c),
             ('0x71C0 无残留', '0x71C0u' not in c),
             ('花括号平衡', c.count('{') == c.count('}'))]:
    print('  %-16s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
