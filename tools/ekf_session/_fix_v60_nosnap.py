# -*- coding: utf-8 -*-
"""VER=59 -> 60：去掉 snap 的宽触发（闪现的源头）。

证据与推理：
  实测环路残差常在 20~160 度，而 snap 触发条件是"|r0|+|r1|>0.5"约等于误差>60度
  -> 每个 EKF 周期都在打 ±180 的整角修正，与慢环来回对打 = 用户看到的"闪现"。
  "假点"（180 度鞍点）只在**启动/重对齐那一刻**决定，之后不需要反复打。

改：条件从  if (!s_mag_anchor || |r|大)  退回  if (!s_mag_anchor)
    但**保留精确角** atan2(cross,dot)（启动那一次就是靠它破假点，这是你要的那部分）。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
a = "        if (!s_mag_anchor || (fabsf(r[0]) + fabsf(r[1]) > 0.5f)) {"
assert t.count(a) == 1, 'snap 条件锚点 %d 次' % t.count(a)
b = ("        /* \u2605VER=60 \u53bb\u6389\u5bbd\u89e6\u53d1\uff1a\u5b9e\u6d4b\u73af\u8def\u6b8b\u5dee\u5e38\u5728 20~160 \u5ea6\uff0c\n"
     "         * \u5bbd\u89e6\u53d1\u4f1a\u8ba9\u5b83\u6bcf\u4e2a\u5468\u671f\u90fd\u6253 \u00b1180 \u6574\u89d2\u4fee\u6b63\uff0c\u4e0e\u6162\u73af\u5bf9\u6253 = \u95ea\u73b0\u3002\n"
     "         * \u5047\u70b9\u53ea\u5728\u542f\u52a8/\u91cd\u5bf9\u9f50\u90a3\u4e00\u523b\u51b3\u5b9a\uff0c\u4e00\u6b21\u5c31\u591f\uff1b\u7cbe\u786e\u89d2\u4fdd\u7559\u3002 */\n"
     "        if (!s_mag_anchor) {")
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}'), '括号不平'
# 关键：多行调用完整性
m = re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S)
assert m, 'ekf_update 调用被破坏'
shutil.copy2(P, P + '.bak_v60'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        59u') == 1
u = u.replace('#define V5F_FW_VER        59u', '#define V5F_FW_VER        60u', 1)
shutil.copy2(T, T + '.bak_v60'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==60', ver == 60),
      ('宽触发已去', 'fabsf(r[0]) + fabsf(r[1]) > 0.5f' not in m7),
      ('snap 仅启动', 'if (!s_mag_anchor) {' in m7),
      ('精确角保留', 'dpsi = atan2f(crs, dt2);' in m7),
      ('自校验保留', 'if (e1 > e0) {' in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('无 rp/取反/v0投影', 'rp[' not in m7 and 'r[0] = -r[0]' not in m7 and 'kk2' not in m7),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
