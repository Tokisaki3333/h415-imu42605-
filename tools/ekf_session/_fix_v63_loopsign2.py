# -*- coding: utf-8 -*-
"""VER=62 -> 63：干净地翻转慢环符号。

VER=62 数据（fw_tag 4098823）：
  boot idx0~54  mag_r = 0.00 度   mag_dqz ±0.017   <- 开机对齐完美
  3 秒窗末      mag_r = 124.49 度                  <- 窗内被推走
  全程 mag_dqz max 2.449、>20 度 0 帧               <- snap 没参与
  最终稳定在 mag_r 124~142，150 度新息门在那儿挡着
=> 开机是对的，环路把偏航**推离**真北，推到接近 150 度门才停 -> 这就是"假点"。
   环路符号是反的（VER=56 那次测试被当时并存的宽触发 snap 搅浑了，不能算数）。
   现在 snap 只在开机窗内，干扰消失，可以干净地测这一条。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
a = "            st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4], &s_rej[3],"
assert t.count(a) == 1, 'update 锚点 %d' % t.count(a)
b = ("            /* \u2605VER=63 \u6162\u73af\u7b26\u53f7\u7ffb\u8f6c\uff08\u5e72\u51c0\u6d4b\u8bd5\uff09\uff1a\n"
     "             * VER=62 \u5f00\u673a mag_r=0.00\uff08\u5bf9\u9f50\u5b8c\u7f8e\uff09\uff0c\u968f\u540e\u88ab\u63a8\u5230 124 \u5ea6\u5e76\u505c\u4f4f\n"
     "             * \uff08\u63a8\u5230 150 \u5ea6\u65b0\u606f\u95e8\u624d\u505c\uff09-> \u73af\u8def\u628a\u504f\u822a\u63a8**\u79bb**\u771f\u5317\u3002\n"
     "             * \u51e0\u4f55\u91cf r \u4e0d\u52a8\uff08snap\u3001mag_rx/ry \u8bca\u65ad\u4ecd\u7528\u5b83\uff09\u3002 */\n"
     "            r[0] = -r[0]; r[1] = -r[1];\n"
     + a)
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}')
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v63'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        62u') == 1
u = u.replace('#define V5F_FW_VER        62u', '#define V5F_FW_VER        63u', 1)
shutil.copy2(T, T + '.bak_v63'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==63', ver == 63),
      ('取反在 update 前', m7.index('r[0] = -r[0]; r[1] = -r[1];') < m7.index('st = ekf_update')),
      ('诊断在前', m7.index('s_mag_rx = r[0];') < m7.index('r[0] = -r[0];')),
      ('snap 仍用几何 r', 'dpsi = atan2f(crs, dt2);' in m7),
      ('时间窗触发在', 'if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {' in m7),
      ('自校验在', 'if (e1 > e0) {' in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('无 rp/v0投影', 'rp[' not in m7 and 'kk2' not in m7),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
