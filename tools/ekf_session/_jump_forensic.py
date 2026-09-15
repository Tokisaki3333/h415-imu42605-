# -*- coding: utf-8 -*-
"""跳变帧取证：姿态单帧大跳时，dt / 陀螺 / bg / dq / 门位分别是什么。"""
import glob
import os

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
raw = np.fromfile(P, dtype='<f4')
NCH = next(n for n in (133, 128, 122) if raw.size % n == 0)
b = raw[:raw.size // NCH * NCH].reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
dt_us = b[:, 25]
t = np.cumsum(dt_us * 1e-6)
print('fw_tag %d  列 %d  帧 %d  时长 %.2f s' % (b[0, 76], NCH, N, t[-1]))


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z)
    n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def qang(q):
    w = np.clip(np.abs(q[:, 0]), -1, 1)
    return np.degrees(2*np.arccos(w))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


ye = yaw(b[:, 89:93])
d = np.abs(wrap(np.diff(ye)))
g = np.linalg.norm(b[:, 26:29], axis=1)
bg = np.linalg.norm(b[:, 99:102], axis=1)
print('\n全局：dt_us p50 %.2f  p90 %.2f  p99 %.2f  max %.1f  (>200us 占 %.3f%%)'
      % (np.median(dt_us), np.percentile(dt_us, 90), np.percentile(dt_us, 99),
         dt_us.max(), 100*(dt_us > 200).mean()))
print('      |gyro| p50 %.3f max %.1f dps     |bg| p50 %.4f max %.1f dps'
      % (np.median(g), g.max(), np.median(bg), bg.max()))
print('      |gyro_bias_dps| 列34-36: max %.1f   加速度 |a_lin| max %.2f'
      % (np.abs(b[:, 34:37]).max(), np.abs(b[:, 10:13]).max()))

J = np.flatnonzero(d > 10.0)
print('\n单帧 |dyaw| > 10 度的帧: %d 个 (%.3f%%)' % (len(J), 100*len(J)/N))
if len(J):
    print('  这些帧: |dyaw| p50 %.2f  max %.2f 度' % (np.median(d[J]), d[J].max()))
    print('  它们的 dt_us: p50 %.2f  max %.1f   (>200us 占 %.1f%%)'
          % (np.median(dt_us[J+1]), dt_us[J+1].max(), 100*(dt_us[J+1] > 200).mean()))
    print('  它们的 |gyro|: p50 %.2f  max %.1f dps' % (np.median(g[J+1]), g[J+1].max()))
    print('  它们的 |bg|  : p50 %.4f  max %.4f dps' % (np.median(bg[J+1]), bg[J+1].max()))
    print('  它们的 mag_used 占 %.1f%%   |dq| max %.3f 度'
          % (100*np.nanmean(b[J+1, 120]), np.abs(b[J+1, 130:133]).max()))
    gbj = b[J+1, 103].astype(int)
    print('  它们的门位: ' + '  '.join('%s %.0f%%' % (n, 100*((gbj & v) != 0).mean())
          for v, n in [(0x20, 'tilt'), (0x40, 'mag'), (0x200, 'chi2'), (0x10, 'zupt')]))
    print('\n  逐帧明细（前 12 个）：idx  t(s)  dt_us  |gyro|  |bg|  dyaw  mag_dq')
    for k in J[:12]:
        print('    %7d %7.3f %7.2f %7.2f %7.3f %7.2f   %s'
              % (k, t[k], dt_us[k+1], g[k+1], bg[k+1], wrap(ye[k+1]-ye[k]),
                 np.round(b[k+1, 130:133], 4)))
    print('\n  相邻帧 dt_us 分布（跳变帧及其前后）:')
    for off in (-2, -1, 0, 1, 2):
        v = dt_us[np.clip(J+off, 0, N-1)]
        print('    J%+d: p50 %7.2f  p90 %7.2f  max %10.1f' % (off, np.median(v), np.percentile(v, 90), v.max()))
