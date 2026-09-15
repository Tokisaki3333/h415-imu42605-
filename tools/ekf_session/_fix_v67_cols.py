# -*- coding: utf-8 -*-
"""VER=66 -> 67：追加 5 列诊断（用户要求）。

  133 mag_vx  134 mag_vy      M7 修正用的**投影向量** v = (Bn[0],Bn[1])
  135 mag_v0x 136 mag_v0y     模型水平矢量 v0 = (b0x,b0y)
  137 mag_yawpre               **修正前** EKF 偏航（度，由 s_x[IX_Q] 现算）
  另：128/129 mag_rx/ry 已是新息 r；42~44 mag_f 已是机体系原始磁场。
用途：90 度量级的大宗漂移若来自某个量被弄反，v 与 v0 的对照、以及 yawpre
      与 mag_yaw(修后) 的差会直接暴露是哪一步。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
H = U + r'\inc\SPI_rx.h'
S = U + r'\src\SPI_rx.c'
T = U + r'\inc\v5f_tune.h'
NEW = [('mag_vx', 'float', 's_mag_vx'), ('mag_vy', 'float', 's_mag_vy'),
       ('mag_v0x', 'float', 's_mag_v0x'), ('mag_v0y', 'float', 's_mag_v0y'),
       ('mag_yawpre', 'float', 's_mag_yawpre')]
n = 0
t = open(P, 'rb').read().decode('gbk')

# 1) 静态量
a = "static float    s_mag_rx, s_mag_ry;"
assert t.count(a) == 1
t = t.replace(a, a + "\n"
              "static float    s_mag_vx, s_mag_vy;      /* \u2605VER=67 \u6295\u5f71\u5411\u91cf v */\n"
              "static float    s_mag_v0x, s_mag_v0y;    /* \u2605VER=67 \u6a21\u578b v0 */\n"
              "static float    s_mag_yawpre;            /* \u2605VER=67 \u4fee\u6b63\u524d EKF \u504f\u822a(\u5ea6) */", 1)
n += 1; print('  ok 1-静态量')

# 2) M7 内采集（插在 r[1] 赋值之后）
a = "        r[1] = Bn[1] - b0y;\n"
assert t.count(a) == 1, 'r[1] 锚点 %d' % t.count(a)
b = a + ("        s_mag_vx = Bn[0]; s_mag_vy = Bn[1];\n"
         "        s_mag_v0x = b0x; s_mag_v0y = b0y;\n"
         "        {\n"
         "            float qw = s_x[IX_Q], qx = s_x[IX_Q+1], qy = s_x[IX_Q+2], qz = s_x[IX_Q+3];\n"
         "            s_mag_yawpre = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz)) * RAD2DEG;\n"
         "        }\n")
t = t.replace(a, b, 1)
n += 1; print('  ok 2-M7 内采集 5 个量')

# 3) publish
m = re.search(r'^([ \t]*)h->ekf\.mag_ry[^\n]*$', t, re.M)
assert m, 'publish mag_ry 行未找到'
t = t[:m.end()] + ''.join('\n%s h->ekf.%s = %s;' % (m.group(1), f, src) for f, _, src in NEW) + t[m.end():]
n += 1; print('  ok 3-publish +5')
assert t.count('{') == t.count('}'), '括号不平'
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v67'); open(P, 'wb').write(t.encode('gbk'))

# 4) 结构体
h = open(H, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)float[ \t]+mag_ry[^\n]*$', h, re.M)
assert m, 'struct mag_ry 未找到'
h = h[:m.end()] + ''.join('\n%s %-8s %s;   /* VER=67 */' % (m.group(1), ty, f) for f, ty, _ in NEW) + h[m.end():]
shutil.copy2(H, H + '.bak_v67'); open(H, 'wb').write(h.encode('gbk'))
n += 1; print('  ok 4-struct +5')

# 5) 通道 + 列数
s = open(S, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)ch\[c\+\+\][ \t]*=[ \t]*g_v5f_hold\.ekf\.mag_ry[^\n]*$', s, re.M)
assert m, 'mag_ry 通道行未找到'
s = s[:m.end()] + ''.join('\n%s ch[c++] = g_v5f_hold.ekf.%s;   /* VER=67 */' % (m.group(1), f) for f, _, _ in NEW) + s[m.end():]
mm = re.search(r'#define[ \t]+JF_CH_NUM[ \t]+(\d+)u', s)
assert mm and mm.group(1) == '139', 'JF_CH_NUM=%s' % (mm and mm.group(1))
s = s[:mm.start(1)] + '144' + s[mm.end(1):]
shutil.copy2(S, S + '.bak_v67'); open(S, 'wb').write(s.encode('gbk'))
n += 1; print('  ok 5-通道 +5, JF_CH_NUM=144')

# 6) VER
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        66u') == 1
u = u.replace('#define V5F_FW_VER        66u', '#define V5F_FW_VER        67u', 1)
shutil.copy2(T, T + '.bak_v67'); open(T, 'wb').write(u.encode('gbk'))
n += 1; print('  ok 6-VER=67')

# 校验
c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
h2 = open(H, 'rb').read().decode('gbk'); s2 = open(S, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
CK = [('VER==67', ver == 67), ('编辑数==6', n == 6),
      ('v 采集在', 's_mag_vx = Bn[0];' in m7),
      ('yawpre 在', 's_mag_yawpre = atan2f(' in m7),
      ('采集在 update 之前', m7.index('s_mag_vx = Bn[0];') < m7.index('st = ekf_update')),
      ('publish +5', all('h->ekf.%s =' % f in c for f, _, _ in NEW)),
      ('struct +5', all(re.search(r'\b%s\b' % f, h2) for f, _, _ in NEW)),
      ('channel +5', all('g_v5f_hold.ekf.%s' % f in s2 for f, _, _ in NEW)),
      ('取反仍在', 'mf[0] = -mf[0];' in m7),
      ('死区仍在', 'if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {' in m7),
      ('时间窗 snap 仍在', 's_boot_t < V5F_EKF_MAG_ANCHOR_TS' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('JF_CH_NUM==144', nch == 144),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
print('新列 133 mag_vx 134 mag_vy 135 mag_v0x 136 mag_v0y 137 mag_yawpre')
