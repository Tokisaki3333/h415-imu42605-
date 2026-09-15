# -*- coding: utf-8 -*-
"""VER=53 -> 54：修掉 180 度鞍点。

根因：我做投影时用的是**小角度线性化**（绕偏航只让 v 沿垂直于自身的方向动）。
误差接近 180 度时 v ≈ −v0，新息 r = v − v0 = −2v0 **完全平行于 v0**，
被投影全部丢光；而线性化偏航修正量正比于 sin(Δ)，**在 Δ=180 度处恰好为零**
-> 180 度成了鞍点：停住不动，等噪声推开才慢慢转回（正是"z 转 180 后过会转回来"）。
一步式用的也是线性化 cross/n2，cross=0 -> 在 180 度处什么也不做。

三处改动：
 A. 删掉投影（它丢了 dot 里的符号信息）
 B. 一步式改用**精确角** dpsi = atan2(cross, dot)（含 ±180 度，一次到位）
 C. 自校验的误差度量也用完整新息（不再投影），否则它判不出好坏
 D. 死点门限 0.30 -> 0.12：本份数据 mag_bh 全程 0.29~0.32，**正好卡在 0.30**，
    地磁被反复误关（mag_used 时通时断）
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
n = 0

# A) 删投影
a = t.index("        /* \u2605VER=51-A \u6295\u5f71")
b = t.index("}\n", t.index("r[1] -= kk * b0y;", a)) + 2
blk = t[a:b]
assert 'r[0] -= kk * b0x;' in blk and len(blk) < 900
t = t[:a] + t[b:]
n += 1; print('  ok A-删投影(%d 字节)' % len(blk))

# B) 一步式改精确角
a = "            if (n2 > 1e-6f) dpsi = (r[1] * b0x - r[0] * b0y) / n2;"
assert t.count(a) == 1, 'dpsi 锚点 %d' % t.count(a)
b = ("""            /* \u2605VER=54 \u7cbe\u786e\u89d2\uff1a\u7ebf\u6027\u5316 cross/n2 \u5728 180 \u5ea6\u5904\u4e3a 0\uff08\u978d\u70b9\uff09\uff0c
             * \u5fc5\u987b\u7528 atan2(cross, dot) \u624d\u80fd\u4e00\u6b65\u5230\u4f4d\u3002
             *   \u53e0\u4ee3\u89d2 delta = atan2(b0y,b0x) - atan2(v_y,v_x)\uff0cv = v0 + r
             *   => delta = atan2(cross, dot)
             *      cross = b0x*r1 - b0y*r0
             *      dot   = b0x*r0 + b0y*r1 + n2        (n2 = |v0|^2) */
            if (n2 > 1e-6f) {
                float crs = b0x * r[1] - b0y * r[0];
                float dt2 = b0x * r[0] + b0y * r[1] + n2;
                dpsi = atan2f(crs, dt2);
            }""")
t = t.replace(a, b, 1)
n += 1; print('  ok B-一步式精确角')

# C) 自校验度量去掉投影
a = """                float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y, kk2;
                kk2 = (n2 > 1e-6f) ? (rx2 * b0x + ry2 * b0y) / n2 : 0.0f;
                rx2 -= kk2 * b0x; ry2 -= kk2 * b0y;
                e1 = sqrtf(rx2 * rx2 + ry2 * ry2);"""
assert t.count(a) == 1, '自校验锚点 %d' % t.count(a)
b = """                float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y;
                /* \u2605VER=54 \u5ea6\u91cf\u7528**\u5b8c\u6574\u65b0\u606f**\uff08\u4e0d\u6295\u5f71\uff09\uff0c
                 * \u5426\u5219 180 \u5ea6\u90a3\u79cd"\u5b8c\u5168\u5e73\u884c"\u7684\u8bef\u5dee\u88ab\u6295\u6389\u5c31\u5224\u4e0d\u51fa\u597d\u574f\u3002 */
                e1 = sqrtf(rx2 * rx2 + ry2 * ry2);"""
t = t.replace(a, b, 1)
n += 1; print('  ok C-自校验度量去投影')

dump(P, t, 'gbk', 'v54')

# D) 死点门限
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'(#define\s+V5F_EKF_MAG_BH_MIN\s+)([0-9.]+)f', u)
assert m and m.group(2) == '0.30', 'BH_MIN = %s' % (m and m.group(2))
u = u[:m.start()] + m.group(1) + '0.12f' + u[m.end():]
assert u.count('#define V5F_FW_VER        53u') == 1
u = u.replace('#define V5F_FW_VER        53u', '#define V5F_FW_VER        54u', 1)
dump(T, u, 'gbk', 'v54')
n += 1; print('  ok D-BH_MIN 0.30->0.12, VER=54')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
bmin = re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t2).group(1)
CK = [('VER==54', ver == 54), ('编辑数==4', n == 4),
      ('投影已删', 'r[0] -= kk * b0x;' not in m7),
      ('精确角在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('cross/dot 在', 'b0x * r[1] - b0y * r[0]' in m7 and '+ n2;' in m7),
      ('线性化已无', '(r[1] * b0x - r[0] * b0y) / n2' not in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('自校验度量去投影', 'kk2' not in m7),
      ('BH_MIN=0.12', bmin.startswith('0.12')),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)   BH_MIN=%s' % ((ver << 16) | (nch << 8) | 7, ver, nch, bmin))
