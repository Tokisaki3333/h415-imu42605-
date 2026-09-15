# -*- coding: utf-8 -*-
"""VER=53 -> 54（修正版）：先回退，再用精确锚点重打。"""
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

# 1) 回退到补丁前
shutil.copy2(P + '.bak_v54', P)
shutil.copy2(T + '.bak_v54', T)
c0 = open(P, 'rb').read().decode('gbk')
t0 = open(T, 'rb').read().decode('gbk')
print('回退: proc 投影在=%s  { } 平衡=%s  tune VER=%s BH_MIN=%s'
      % ('r[0] -= kk * b0x;' in c0, c0.count('{') == c0.count('}'),
         re.search(r'#define V5F_FW_VER\s+(\d+)u', t0).group(1),
         re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t0).group(1)))
assert 'r[0] -= kk * b0x;' in c0 and c0.count('{') == c0.count('}')


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = c0
n = 0

# A) 删投影：从 VER=51-A 注释起到**外层块**的 '}'（用 \n + 8 空格 + } 精确定位，
#    避免匹配到 if 自己的 12 空格收尾）
a = t.index("        /* \u2605VER=51-A \u6295\u5f71")
p = t.index("r[1] -= kk * b0y;", a)
q = t.index("\n        }\n", p) + len("\n        }\n")
blk = t[a:q]
assert 'r[0] -= kk * b0x;' in blk and blk.rstrip().endswith('}')
assert blk.count('{') == blk.count('}'), '待删块自身括号不平: %d/%d' % (blk.count('{'), blk.count('}'))
t = t[:a] + t[q:]
n += 1
print('  ok A-删投影(%d 字节, 块内括号 %d/%d 平衡)' % (len(blk), blk.count('{'), blk.count('}')))

# B) 一步式精确角
a = "            if (n2 > 1e-6f) dpsi = (r[1] * b0x - r[0] * b0y) / n2;"
assert t.count(a) == 1
b = ("""            /* \u2605VER=54 \u7cbe\u786e\u89d2\uff1a\u7ebf\u6027\u5316 cross/n2 \u5728 180 \u5ea6\u5904\u4e3a 0\uff08\u978d\u70b9\uff09\uff0c
             * \u5fc5\u987b\u7528 atan2(cross, dot) \u624d\u80fd\u4e00\u6b65\u5230\u4f4d\uff1a
             *   delta = atan2(b0y,b0x) - atan2(v_y,v_x)\uff0cv = v0 + r
             *   cross = b0x*r1 - b0y*r0\uff1bdot = b0x*r0 + b0y*r1 + n2 */
            if (n2 > 1e-6f) {
                float crs = b0x * r[1] - b0y * r[0];
                float dt2 = b0x * r[0] + b0y * r[1] + n2;
                dpsi = atan2f(crs, dt2);
            }""")
t = t.replace(a, b, 1)
n += 1; print('  ok B-一步式精确角')

# C) 自校验度量去投影
a = """                float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y, kk2;
                kk2 = (n2 > 1e-6f) ? (rx2 * b0x + ry2 * b0y) / n2 : 0.0f;
                rx2 -= kk2 * b0x; ry2 -= kk2 * b0y;
                e1 = sqrtf(rx2 * rx2 + ry2 * ry2);"""
assert t.count(a) == 1
b = """                float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y;
                /* \u2605VER=54 \u5ea6\u91cf\u7528**\u5b8c\u6574\u65b0\u606f**\uff0c\u5426\u5219 180 \u5ea6\u7684\u5e73\u884c\u8bef\u5dee\u88ab\u6295\u6389\u5c31\u5224\u4e0d\u51fa\u597d\u574f */
                e1 = sqrtf(rx2 * rx2 + ry2 * ry2);"""
t = t.replace(a, b, 1)
n += 1; print('  ok C-自校验度量去投影')

assert t.count('{') == t.count('}'), '改写后括号不平 %d/%d' % (t.count('{'), t.count('}'))
dump(P, t, 'gbk', 'v54b')

# D) 死点门限 + VER
u = t0
m = re.search(r'(#define\s+V5F_EKF_MAG_BH_MIN\s+)([0-9.]+)f', u)
assert m and m.group(2) == '0.30'
u = u[:m.start()] + m.group(1) + '0.12f' + u[m.end():]
assert u.count('#define V5F_FW_VER        53u') == 1
u = u.replace('#define V5F_FW_VER        53u', '#define V5F_FW_VER        54u', 1)
dump(T, u, 'gbk', 'v54b')
n += 1; print('  ok D-BH_MIN 0.30->0.12, VER=54')

# 复核
c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
bmin = re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t2).group(1)
CK = [('VER==54', ver == 54), ('编辑数==4', n == 4),
      ('投影已删', 'r[0] -= kk * b0x;' not in m7 and 'kk' not in m7),
      ('精确角在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('线性化已无', '(r[1] * b0x - r[0] * b0y) / n2' not in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('BH_MIN=0.12', bmin.startswith('0.12')),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('死点仍在', 's_mag_bh < V5F_EKF_MAG_BH_MIN' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)   BH_MIN=%s' % ((ver << 16) | (nch << 8) | 7, ver, nch, bmin))
