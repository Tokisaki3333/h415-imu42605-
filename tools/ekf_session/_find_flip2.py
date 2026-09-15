# -*- coding: utf-8 -*-
"""VER=53 数据：用旧链作基准定位 EKF 翻转；列数用 fw_tag 自洽性自动判定。"""
import glob
import os
import time

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
raw = np.fromfile(P, dtype='<f4')
print('%s %s  %d 个 float (%.1f MB)'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         raw.size, raw.size*4/1e6))
best = None
for nch in (139, 133, 128, 122, 117, 115, 113, 112):
    if raw.size < nch*3:
        continue
    f0, f1, f2 = raw[76], raw[nch+76], raw[2*nch+76]
    ok = (f0 == f1 == f2) and 1e6 < f0 < 4e6
    rem = raw.size % nch
    print('  试 %3d 列: fw_tag=%10.0f,%10.0f,%10.0f  自洽=%s  余 %d' % (nch, f0, f1, f2, ok, rem))
    if ok and best is None:
        best = nch
assert best, '无法判定列数'
NCH = best
b = raw[:raw.size // NCH * NCH].reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)
print('\n=> 列数 %d  帧 %d  时长 %.2f s  fw_tag %d' % (NCH, N, t[-1], b[0, 76]))


def quat(q):
    n = np.sqrt((q*q).sum(axis=1))
    n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    return q / n[:, None]


def yaw(q):
    q = quat(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


ye, yl = yaw(b[:, 89:93]), yaw(b[:, 0:4])
g = np.linalg.norm(b[:, 26:29], axis=1)
d = wrap(ye - yl)
dd = np.abs(wrap(np.diff(d)))
print('EKF-旧链 偏置 dyaw: p10 %+.1f  p50 %+.1f  p90 %+.1f 度' % tuple(np.percentile(d, [10, 50, 90])))
print('|gyro| p50 %.1f max %.1f dps' % (np.median(g), g.max()))

J = np.flatnonzero(dd > 90.0)
print('\n|dyaw 单帧变化| > 90 度的帧: %d 个' % len(J))
step, spike = [], []
for k in J:
    later = d[min(N-1, k+1600)]
    if abs(wrap(d[k+1] - d[k])) > 90.0 and abs(wrap(later - d[k])) > 90.0:
        step.append(int(k))
    else:
        spike.append(int(k))
print('  阶跃(真被转过去) %d 个    单帧尖峰(撕读假象) %d 个' % (len(step), len(spike)))
print('\n --- 阶跃事件（dyaw 前后 + 当时地磁量）---')
for k in step[:12]:
    print('   t=%7.3f  dyaw %+8.2f -> %+8.2f   mag_dqz %+8.3f  mag_r %7.2f  bh %.3f  used %d  |w| %7.2f  gate %d'
          % (t[k], d[k], d[k+1], b[k+1, 132], b[k+1, 119], b[k+1, 118],
             int(b[k+1, 120]), g[k+1], int(b[k+1, 103])))
print('\n --- 尖峰事件（前 6）---')
for k in spike[:6]:
    print('   t=%7.3f  dyaw %+8.2f -> %+8.2f -> %+8.2f   |w| %7.2f' %
          (t[k], d[k], d[k+1], d[min(N-1, k+2)], g[k+1]))

big = np.abs(b[J+1, 132]) > 20.0
print('\n翻转帧里 |mag_dqz|>20 度 占 %.0f%%   |mag_dqz| 全程 >20 度帧数 %d  >45 度 %d'
      % (100*big.mean() if len(J) else 0, (np.abs(b[:, 132]) > 20).sum(), (np.abs(b[:, 132]) > 45).sum()))

nh = min(N, 1200)
print('\n=== 启动首段（%d 帧）===   idx  t  ekf_yaw  leg_yaw  dyaw  mag_dqz  mag_r  used' % nh)
for k in range(0, nh, 120):
    print('   %5d %6.3f %+9.2f %+9.2f %+8.2f %+8.3f %6.2f  %d'
          % (k, t[k], ye[k], yl[k], d[k], b[k, 132], b[k, 119], int(b[k, 120])))
print('首段 dyaw 首 %+.3f -> 末 %+.3f  (变化 %+.3f 度)'
      % (d[0], d[nh-1], wrap(d[nh-1] - d[0])))
print('全程: 旧链 yaw 首末 %+.2f 度   EKF yaw 首末 %+.2f 度   dyaw 首 %+.2f 末 %+.2f'
      % (wrap(yl[-1]-yl[0]), wrap(ye[-1]-ye[0]), d[0], d[-1]))
