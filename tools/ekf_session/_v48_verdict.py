# -*- coding: utf-8 -*-
"""VER=48 判决（133 列）：门放开后地磁是否终于开始牵引。"""
import glob
import os
import time

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
raw = np.fromfile(P, dtype='<f4')
NCH = 133 if raw.size % 133 == 0 else 128 if raw.size % 128 == 0 else 122
b = raw[:raw.size // NCH * NCH].reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
t = np.cumsum(b[:, 25] * 1e-6)
print('%s  %s  列 %d  帧 %d  时长 %.2f s  fw_tag %d'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         NCH, N, t[-1], b[0, 76]))
if NCH < 133:
    raise SystemExit('不是 133 列数据')

g = np.linalg.norm(b[:, 26:29], axis=1)
mu = b[:, 120]
print('\n【1】启动')
for k, c in (('prop_ok', 122), ('f_ok', 123)):
    print('  %-8s =1 占比 %.1f%%' % (k, 100 * (b[:, c] == 1).mean()))
print('  prop_row 最大值 %d（应 17）   stage 覆盖 %d 个值（应含 0..22）'
      % (int(b[:, 124].max()), len(np.unique(b[:, 125]))))
print('  mag_used=1 占比 %.1f%%   mag gate 位 %.1f%%' % (100 * mu.mean(), 100 * ((b[:, 103].astype(int) & 0x40) != 0).mean()))

print('\n【2】修正落在哪个轴')
mdq = b[:, 130:133]
nz = np.abs(mdq).max(axis=1) > 0
print('  mag_dq* 非零占比 %.1f%%' % (100 * nz.mean()))
if nz.any():
    a = np.abs(mdq[nz])
    m = a.mean(axis=0)
    print('  三轴平均 |dq| : x %.4f  y %.4f  z %.4f 度   占比 %.1f/%.1f/%.1f %%'
          % (m[0], m[1], m[2], 100*m[0]/m.sum(), 100*m[1]/m.sum(), 100*m[2]/m.sum()))

print('\n【3】是否真的拉回来了')
for nm, c in (('mag_r(119)', 119), ('p_yy(121)', 121), ('sigma_yaw(104)', 104),
              ('mag_bh(118)', 118)):
    v = b[:, c]
    print('  %-14s 首 %.4g   p10 %.4g  p50 %.4g  p90 %.4g  末 %.4g'
          % (nm, v[0], np.percentile(v, 10), np.median(v), np.percentile(v, 90), v[-1]))

print('\n【4】死点一致性')
bh = b[:, C_] if False else b[:, 118]
dead = bh < 0.30
print('  mag_bh<0.30 占比 %.2f%%  这些帧 mag_used=0 且 dq 全 0: %s'
      % (100*dead.mean(),
         bool(np.all(mu[dead] == 0) and np.all(np.abs(mdq[dead]).max() == 0)) if dead.any() else 'n/a'))

print('\n【5】分段（|gyro| 中位 + 总转角自证）')
def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z); w, x, y, z = w/n, x/n, y/n, z/n
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
ye = yaw(b[:, 89:93])
def wrap(d):
    return (d + 180.0) % 360.0 - 180.0
for k in range(4):
    s, e = k*N//4, (k+1)*N//4
    tot = np.abs(wrap(np.diff(ye[s:e]))).sum()
    print('  %5.1f-%5.1fs |gyro|p50 %7.1f  EKF总转角 %9.1f  mag_used %4.0f%%  |mag_r|p50 %6.2f  p_yy p50 %.4g'
          % (t[s], t[e-1], np.median(g[s:e]), tot, 100*mu[s:e].mean(),
             np.median(np.abs(b[s:e, 119])), np.median(b[s:e, 121])))
gb = b[:, 103].astype(int)
print('  gate bits: ' + '  '.join('%s %.1f%%' % (n, 100*((gb & v) != 0).mean())
      for v, n in [(0x20, 'tilt'), (0x40, 'mag'), (0x80, 'aligned'), (0x200, 'chi2')]))
