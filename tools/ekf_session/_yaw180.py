# -*- coding: utf-8 -*-
"""偶现 180 度偏航跳变：是不是 VER=51 的"一次性对齐"被重复触发/被垃圾样本触发。"""
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


def quat(q):
    n = np.sqrt((q*q).sum(axis=1))
    n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    return q / n[:, None]


def yaw(q):
    q = quat(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


ye = yaw(b[:, 89:93])
g = np.linalg.norm(b[:, 26:29], axis=1)
d = np.abs(wrap(np.diff(ye)))
J = np.flatnonzero(d > 90.0)
print('\n单帧 |dyaw| > 90 度的帧: %d 个' % len(J))
if len(J) == 0:
    J = np.flatnonzero(d > 30.0)
    print('  （放宽到 >30 度: %d 个）' % len(J))

if len(J):
    print('\n idx     t(s)    dyaw   |gyro|  mag_dqx mag_dqy mag_dqz | mag_bh  mag_r mag_used | tilt_dqz | gate  prop_ok')
    for k in J[:20]:
        print(' %6d %7.3f %8.2f %7.2f   %7.3f %7.3f %7.3f | %6.3f %7.2f %5d    | %7.4f | %5d %5d'
              % (k, t[k], wrap(ye[k+1]-ye[k]), g[k+1],
                 b[k+1, 130], b[k+1, 131], b[k+1, 132],
                 b[k+1, 118], b[k+1, 119], int(b[k+1, 120]),
                 b[k+1, 135], int(b[k+1, 103]), int(b[k+1, 122])))
    print('\n 相邻帧对照（取第一个跳变）:')
    k0 = J[0]
    for k in range(max(0, k0-2), min(N, k0+4)):
        print('   idx %6d t=%7.3f  dyaw=%8.2f  mag_dqz=%8.3f  mag_bh=%.3f  mag_r=%7.2f  used=%d  dqx=%.3f dqy=%.3f  |gyro|=%.2f  gate=%d'
              % (k, t[k], wrap(ye[k]-ye[k-1]) if k > 0 else 0.0,
                 b[k, 132], b[k, 118], b[k, 119], int(b[k, 120]),
                 b[k, 130], b[k, 131], g[k], int(b[k, 103])))
    print('\n 跳变帧的 mag_used 分布: used=1 占 %.0f%%' % (100*np.nanmean(b[J+1, 120])))
    big = np.abs(b[J+1, 132]) > 45.0
    print(' 其中 mag_dqz(一次性对齐量) > 45 度 的占 %.0f%%  -> %s'
          % (100*big.mean(), '确认是一次性对齐干的' if big.mean() > 0.5 else '不是一次性对齐，是慢环'))

print('\n=== 启动首帧附近（看一次性对齐是否只在开头发生一次）===')
print(' idx   t(s)   mag_dqz   mag_dqx  mag_dqy  mag_used  mag_bh   mag_r   gate  prop_ok')
for k in range(0, min(N, 12)):
    print(' %5d %6.3f %9.3f %8.3f %8.3f %8d  %6.3f %7.2f  %5d  %5d'
          % (k, t[k], b[k, 132], b[k, 130], b[k, 131], int(b[k, 120]),
             b[k, 118], b[k, 119], int(b[k, 103]), int(b[k, 122])))
nz = np.flatnonzero(np.abs(b[:, 132]) > 1.0)
print('\n |mag_dqz| > 1 度的帧共 %d 个，前 15 个时刻: %s'
      % (len(nz), np.array2string(t[nz[:15]], precision=2)))
print(' |mag_dqz| > 45 度的帧共 %d 个，时刻: %s'
      % ((np.abs(b[:, 132]) > 45).sum(),
         np.array2string(t[np.abs(b[:, 132]) > 45][:15], precision=2)))
