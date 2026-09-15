# -*- coding: utf-8 -*-
"""VER=52 -> 53：按用户确认，翻转启动一步式的符号。

用户明确结论：**初始对齐方向就是反的**（一步式符号错），
之后旋转中被正常慢环牵到真北 —— 表现为"转了 180 度"。
=> 慢环（标准 Kalman）符号正确；错的是我那条自推几何式。

改：dpsi = (r[0]*b0y - r[1]*b0x)/n2   ->   dpsi = (r[1]*b0x - r[0]*b0y)/n2
保留 VER=52 的自校验（转完重算新息，变大就还原并反向）作为兜底：
即使这次我又搞错，它自己会反回来，且 mag_dqz(132) 报的是**实际施加**的角度。
"""
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
a = "            if (n2 > 1e-6f) dpsi = (r[0] * b0y - r[1] * b0x) / n2;"
assert t.count(a) == 1, 'dpsi 锚点 %d 次' % t.count(a)
b = ("            /* \u2605VER=53 \u7b26\u53f7\u7ffb\u6b63\uff1a\u7528\u6237\u786e\u8ba4\u521d\u59cb\u4e00\u6b65\u5f0f\u65b9\u5411\u53cd\u4e86\uff0c\n"
     "             * \u800c\u6162\u73af\uff08\u6807\u51c6 Kalman\uff09\u65b9\u5411\u6b63\u786e\uff08\u65cb\u8f6c\u4e2d\u80fd\u7275\u5230\u771f\u5317\uff09\u3002\n"
     "             * \u8868\u73b0\u4e3a\uff1a\u521d\u59cb\u843d\u5230\u53cd\u5411\uff0c\u4e4b\u540e\u88ab\u7275\u56de\u771f\u5317 => \u770b\u8d77\u6765\u50cf\"\u8f6c\u4e86 180 \u5ea6\"\u3002*/\n"
     "            if (n2 > 1e-6f) dpsi = (r[1] * b0x - r[0] * b0y) / n2;")
t = t.replace(a, b, 1)
dump(P, t, 'gbk', 'v53')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        52u') == 1
u = u.replace('#define V5F_FW_VER        52u', '#define V5F_FW_VER        53u', 1)
dump(T, u, 'gbk', 'v53')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==53', ver == 53),
      ('符号已翻正', 'dpsi = (r[1] * b0x - r[0] * b0y) / n2;' in m7),
      ('旧符号已无', '(r[0] * b0y - r[1] * b0x) / n2' not in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('反向重做仍在', 'h2 = -0.5f * dpsi;' in m7),
      ('投影仍在', 'r[0] -= kk * b0x;' in m7),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
