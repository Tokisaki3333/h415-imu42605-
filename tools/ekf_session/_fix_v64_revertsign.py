# -*- coding: utf-8 -*-
"""VER=63 -> 64：回退符号取反（PC 仿真判定原版符号正确）。

PC 仿真（用旧链姿态当真值，喂真实录制的陀螺/地磁，复现 M7 数学）：
  sgn=+1  起始0 -> 末 -3.05 度，p50 3.10   -> 收敛
  sgn=-1  起始0 -> 末 177.09 度，p50 166.21 -> 发散
  从 +30/+60/+90/+120/+150/+179 出发，sgn=+1 全部往 0 走；sgn=-1 全部往外跑。
=> 原始符号正确；"假点"只是 sin(Delta) 在 180 度附近太小导致收敛极慢，
   必须靠开机窗内那一次精确角整角把它一步带出来（VER=62 已做）。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

shutil.copy2(P + '.bak_v63', P)          # 回到 VER=62 内容（原版符号）
c = open(P, 'rb').read().decode('gbk')
print('回退后: 取反残留=%s  { }平衡=%s  update完整=%s' %
      ('r[0] = -r[0]' in c, c.count('{') == c.count('}'),
       bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', c, re.S))))
assert 'r[0] = -r[0]' not in c and c.count('{') == c.count('}')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        63u') == 1
u = u.replace('#define V5F_FW_VER        63u', '#define V5F_FW_VER        64u', 1)
shutil.copy2(T, T + '.bak_v64'); open(T, 'wb').write(u.encode('gbk'))
shutil.copy2(P, P + '.bak_v64'); open(P, 'wb').write(c.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==64', ver == 64),
      ('无符号取反', 'r[0] = -r[0]' not in m7),
      ('时间窗 snap 在', 'if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {' in m7),
      ('精确角在', 'dpsi = atan2f(crs, dt2);' in m7),
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
