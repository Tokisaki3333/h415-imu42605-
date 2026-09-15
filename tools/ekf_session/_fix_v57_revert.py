# -*- coding: utf-8 -*-
"""VER=56 -> 57：回退慢环符号翻转（数据证明它更糟）。

VER=55: 开机 mag_r=1.16 度, mag_r p50 27.7 max 69.8
VER=56: 开机 mag_r=41.71 度, mag_r p50 21.9 max 167.2, mag_dqz 最大 167,
        静止段 dyaw 在 ±180 附近乱跳 -> 翻转是错的。
=> 去掉 r 取反，回到 VER=55 的环路行为。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'
def dump(p, s, e, tag):
    d = s.encode(e); shutil.copy2(p, p + '.bak_' + tag)
    open(p, 'wb').write(d)

t = open(P, 'rb').read().decode('gbk')
a = "            r[0] = -r[0]; r[1] = -r[1];\n"
assert t.count(a) == 1, '取反行锚点 %d' % t.count(a)
t = t.replace(a, "            /* \u2605VER=57 \u5df2\u56de\u9000 VER=56 \u7684\u53d6\u53cd\uff1a\u5b9e\u6d4b\u7ffb\u8f6c\u540e\n"
                  "             * mag_r p90 63.5 max 167\u3001\u9759\u6b62\u6bb5 dyaw \u5728 \u00b1180 \u9644\u8fd1\u4e71\u8df3\uff0c\u6bd4 VER=55 \u66f4\u5dee\u3002 */\n", 1)
dump(P, t, 'gbk', 'v57')
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        56u') == 1
u = u.replace('#define V5F_FW_VER        56u', '#define V5F_FW_VER        57u', 1)
dump(T, u, 'gbk', 'v57')
c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0+10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c','rb').read().decode('gbk')).group(1))
CK = [('VER==57', ver == 57),
      ('取反已去掉', 'r[0] = -r[0]' not in m7),
      ('snap 仍在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('大误差触发在', 'fabsf(r[0]) + fabsf(r[1]) > 0.5f' in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
