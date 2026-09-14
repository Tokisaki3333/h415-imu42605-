# -*- coding: utf-8 -*-
"""10 段 20 cm 平动的验收：① 每段水平位移是否 ≈0.20 m（已知真值！）
② 段间静止时速度/位置是否被 ZUPT 压住 ③ sigma 是否仍然诚实增长
④ 末段抬起晃动时的门控。"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
gb = a[:, 103].astype(np.int32)
print('fw_tag %.0f  帧数 %d  时长 %.2f s' % (a[0, 76], N, t[-1]))

p = a[:, 83:86]
v = a[:, 86:89]
aln = np.linalg.norm(a[:, 10:13], axis=1)
w3 = np.linalg.norm(a[:, 26:29], axis=1)
eac = a[:, 81]

k = max(int(0.2/dt.mean()), 1)
sm_w = np.convolve(w3, np.ones(k)/k, mode='same')
sm_a = np.convolve(aln, np.ones(k)/k, mode='same')
mv = (sm_a > 0.05) | (sm_w > 2.0) | (eac > 0.2)
idx = np.where(np.diff(mv.astype(np.int8)) != 0)[0]
segs = []
for i in range(len(idx)-1):
    s0, s1 = idx[i], idx[i+1]
    if s1 - s0 < int(0.15/dt.mean()):
        continue
    segs.append((s0, s1, bool(mv[min(s0+1, N-1)])))
if segs and segs[0][0] > 0:
    segs.insert(0, (0, segs[0][0], False))
if segs and segs[-1][1] < N:
    segs.append((segs[-1][1], N, bool(mv[N-1])))

print()
print('  # 类型  t起    t止   时长  |a|峰 |w|峰 | 水平位移  |Δp|   段末|v|  sig_v   zupt  mag')
print('  ' + '-'*96)
tot = 0.0
nmove = 0
for n, (s0, s1, ismv) in enumerate(segs):
    dp = p[s1-1, :2] - p[s0, :2]
    d = float(np.linalg.norm(dp))
    if ismv and 0.05 < d < 1.0:
        nmove += 1
        tot += d
    print('  %2d %-4s %6.2f %6.2f %5.2f %6.2f %6.0f | %7.4f %7.4f %8.4f %7.4f %6.4f %4.0f%% %4.0f%%'
          % (n, '动' if ismv else '静', t[s0], t[s1-1], t[s1-1]-t[s0],
             aln[s0:s1].max(), w3[s0:s1].max(),
             dp[0], dp[1], d, np.linalg.norm(v[s1-1]), a[s1-1, 106],
             100*np.mean((gb[s0:s1] & 0x10) != 0), 100*np.mean((gb[s0:s1] & 0x40) != 0)))
print()
print('  识别为"位移在 5~100 cm"的段 %d 个，位移合计 %.4f m' % (nmove, tot))
print('  若 10 段 × 20 cm，真值应为 2.000 m')

# 静止段的漂移
print()
print('=== 静止段：速度/位置是否被压住 ===')
for s0, s1, ismv in segs:
    if ismv or (s1-s0) < int(0.3/dt.mean()):
        continue
    print('  t=%6.2f~%6.2f (%4.1f s)  |v| 均值 %.4f max %.4f m/s   |Δp| %.5f m   '
          'sig_v %.4f  sig_p_h %.3f'
          % (t[s0], t[s1-1], t[s1-1]-t[s0], np.linalg.norm(v[s0:s1], axis=1).mean(),
             np.linalg.norm(v[s0:s1], axis=1).max(),
             np.linalg.norm(p[s1-1, :2]-p[s0, :2]), a[s1-1, 106], a[s1-1, 105]))

print()
print('=== sigma 是否诚实增长（不能只压速度不长 sigma）===')
print('  sigma_vel_h 首 %.4f 末 %.4f  max %.4f' % (a[0, 106], a[-1, 106], a[:, 106].max()))
print('  sigma_pos_h 首 %.4f 末 %.4f  max %.4f' % (a[0, 105], a[-1, 105], a[:, 105].max()))
print('  p_h 末值 (%.4f, %.4f)   |p_h| 末 %.4f m' % (p[-1, 0], p[-1, 1], np.linalg.norm(p[-1, :2])))
print('  p_h 轨迹范围 x %.3f~%.3f  y %.3f~%.3f' % (
    p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()))

print()
print('=== 末段（抬起晃动）门控 ===')
if len(segs) and segs[-1][2]:
    s0, s1, _ = segs[-1]
    print('  t=%.2f~%.2f (%.1f s) |a|峰 %.2f g  |w|峰 %.0f dps' % (
        t[s0], t[s1-1], t[s1-1]-t[s0], aln[s0:s1].max(), w3[s0:s1].max()))
    for nm, b in [('zupt', 0x10), ('tilt', 0x20), ('mag', 0x40), ('baro', 0x04), ('chi2', 0x200)]:
        print('    %-5s %5.1f%%' % (nm, 100*np.mean((gb[s0:s1] & b) != 0)))
