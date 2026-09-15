# -*- coding: utf-8 -*-
"""VER=60 -> 61：窄触发治假点（既不闪现、也不漏启动）。

数据（VER=60, fw_tag 3967751）：开机 mag_r = 118.95 度（= 偏航差 180 的特征，
|r|/|v| ~ 2.1 rad），而 mag_dqz = 0.003 —— 一步式没执行。
原因：snap 挂在 s_mag_anchor 上，而对齐块复位它与 M7 的先后次序让"启动那一次"被吃掉；
VER=60 又把宽触发去掉了 => 两边都没兜住。

改：窄触发 —— 只当**接近 180 度**时才响。
  |r| = |v - v0|；mag_r = |r|/|v| * RAD2DEG
  正常残差 20~60 度 -> |r0|+|r1| <= 0.5   （不碰）
  假点     ~180 度   -> |r0|+|r1| >= 0.8   （必中）
  取 0.8：既不会像 VER=55 那样每周期都打（那才是"闪现"），也不会漏掉启动假点。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
a = "        if (!s_mag_anchor) {"
assert t.count(a) == 1, 'snap 条件锚点 %d 次' % t.count(a)
b = ("        /* \u2605VER=61 \u7a84\u89e6\u53d1\uff1a\u53ea\u5f53**\u63a5\u8fd1 180 \u5ea6**\u5f02\u5e38\u65f6\u624d\u4e00\u6b65\u6574\u89d2\u3002\n"
     "         * \u5e38\u89c4\u6b8b\u5dee 20~60 \u5ea6 -> |r| <= 0.5\uff08\u4e0d\u78b0\uff0c\u4e0d\u4f1a\u50cf VER=55 \u90a3\u6837\u6bcf\u5468\u671f\u5bf9\u6253 = \u95ea\u73b0\uff09\uff1b\n"
     "         * \u5047\u70b9 ~180 \u5ea6 -> |r| >= 0.8\uff08\u5fc5\u4e2d\uff09\u3002\u4e0d\u518d\u4f9d\u8d56 s_mag_anchor\u3002 */\n"
     "        if (!s_mag_anchor || (fabsf(r[0]) + fabsf(r[1]) > 0.8f)) {")
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}')
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v61'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        60u') == 1
u = u.replace('#define V5F_FW_VER        60u', '#define V5F_FW_VER        61u', 1)
shutil.copy2(T, T + '.bak_v61'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==61', ver == 61),
      ('窄触发在', 'fabsf(r[0]) + fabsf(r[1]) > 0.8f' in m7),
      ('精确角在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('自校验在', 'if (e1 > e0) {' in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('无 rp/取反/v0投影', 'rp[' not in m7 and 'r[0] = -r[0]' not in m7 and 'kk2' not in m7),
      ('死点 0.12', re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t2).group(1).startswith('0.12')),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
