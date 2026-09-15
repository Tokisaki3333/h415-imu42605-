# -*- coding: utf-8 -*-
"""用旧链 att.q 作基准，定位 EKF 的 180 度翻转时刻，并区分"阶跃"与"单帧尖峰"。"""
import glob
import os
import time

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
raw = np.fromfile(P, dtype='<f4')
NCH = next(n for n in (139, 133, 128, 122) if raw.size % n == 0)
b = raw[:raw.size // NCH * NCH].reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)
print('%s %s  列 %d  帧 %d  时长 %.2f s  fw_tag %d'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         NCH, N, t[-1], b[0, 76]))
assert NCH >= 139


def quat(q):
    n = np.sqrt((q*q).sum(axis=1))
    n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    return q / n[:, None]


def yaw(q):
    q = quat(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def rp(q):
    q = quat(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return (np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))),
            np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1))))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


ye, yl = yaw(b[:, 89:93]), yaw(b[:, 0:4])
re_, pe = rp(b[:, 89:93])
rl_, pl = rp(b[:, 0:4])
g = np.linalg.norm(b[:, 26:29], axis=1)
d = wrap(ye - yl)
dd = np.abs(wrap(np.diff(d)))
print('EKF-旧链 偏置 dyaw: p10 %+8.2f  p50 %+8.2f  p90 %+8.2f 度' %
      tuple(np.percentile(d, [10, 50, 90])))

J = np.flatnonzero(dd > 90.0)
print('\n|dyaw 单帧变化| > 90 度的帧: %d 个' % len(J))
if len(J):
    # 分类：阶跃（1 秒后仍在新值） vs 尖峰（下一帧就回）
    step, spike = [], []
    for k in J:
        after = d[min(N-1, k+1)]
        later = d[min(N-1, k+1600)]      # ~0.2 s 后
        if abs(wrap(after - d[k])) > 90.0 and abs(wrap(later - d[k])) > 90.0:
            step.append(k)
        else:
            spike.append(k)
    print('  阶跃(真被转过去) %d 个   单帧尖峰(撕读假象) %d 个' % (len(step), len(spike)))
    print('\n --- 阶跃事件 ---')
    for k in step[:12]:
        print('   idx %6d t=%7.3f  dyaw %+8.2f -> %+8.2f   mag_dqz %+8.3f  mag_r %7.2f  mag_bh %.3f'
              '  used %d  |gyro| %7.2f  gate %d' %
              (k, t[k], d[k], d[min(N-1, k+1)], b[k+1, 132], b[k+1, 119], b[k+1, 118],
               int(b[k+1, 120]), g[k+1], int(b[k+1, 103])))
    print('\n --- 尖峰事件（前 8）---')
    for k in spike[:8]:
        print('   idx %6d t=%7.3f  dyaw %+8.2f -> %+8.2f -> %+8.2f  |gyro| %7.2f'
              % (k, t[k], d[k], d[k+1], d[min(N-1, k+2)], g[k+1]))

    # 翻转时刻是否伴随大 mag_dqz（= 一步式）
    big = np.abs(b[J+1, 132]) > 20.0
    print('\n翻转帧里 |mag_dqz| > 20 度 的占 %.0f%%  -> %s'
          % (100*big.mean(), '是一步式干的' if big.mean() > 0.5 else '不是一步式，是慢环/撕读'))
    print('|mag_dqz| 全程 > 20 度的帧数: %d；> 45 度: %d'
          % ((np.abs(b[:, 132]) > 20).sum(), (np.abs(b[:, 132]) > 45).sum()))

print('\n=== 启动首段（看一步式方向）===')
print(' idx   t(s)   ekf_yaw   leg_yaw    dyaw    mag_dqz  mag_r  mag_bh used gate')
for k in range(0, min(N, 400, N), 40):
    print(' %5d %6.3f %+9.3f %+9.3f %+8.3f %+9.3f %6.2f  %5.3f  %3d  %4d'
          % (k, t[k], ye[k], yl[k], d[k], b[k, 132], b[k, 119], b[k, 118],
             int(b[k, 120]), int(b[k, 103])))
print('\n首 400 帧 dyaw: 首 %.3f  末 %.3f  变化 %+.3f 度'
      % (d[0], d[min(399, N-1)], wrap(d[min(399, N-1)] - d[0])))
print('全程 dyaw 首 %.2f 末 %.2f   （旧链 yaw 首末 %+.2f / EKF %+.2f 度）'
      % (d[0], d[-1], wrap(yl[-1]-yl[0]), wrap(ye[-1]-ye[0])))
