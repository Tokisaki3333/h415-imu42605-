# -*- coding: utf-8 -*-
"""VER=15 收尾：|a_nav| 的尖峰到底在哪、停顿段（真静止）的偏航精度、以及"合加速度"标准单位读数。"""
import numpy as np
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
an = np.linalg.norm(a[:, 93:96], axis=1)
qn = np.linalg.norm(a[:, 89:93], axis=1)
print('|a_nav| 分位: ' + '  '.join('p%d %.3f' % (q, np.percentile(an, q))
                                  for q in (50, 90, 99, 99.9, 100)))
top = np.argsort(an)[-8:][::-1]
print('最大的 8 帧：')
for i in sorted(top):
    print('   idx %6d  t=%6.3f s  |a_nav| %7.3f  |ekf_q| %.4f' % (i, t[i], an[i], qn[i]))
print('|ekf_q|<0.9 的帧: %s (共 %d)' % (np.where(qn < 0.9)[0][:12], int((qn < 0.9).sum())))
m = qn > 0.99
print('只在 |ekf_q|>0.99 的帧上统计 |a_nav|: p50 %.4f p99 %.4f max %.4f'
      % (np.median(an[m]), np.percentile(an[m], 99), an[m].max()))

# 停顿段 = |w| 平滑 < 2 dps 的连续 1 s 以上
w3 = np.linalg.norm(a[:, 26:29], axis=1)
k = int(1.0/dt.mean())
sm = np.convolve(w3, np.ones(k)/k, mode='same')
still = sm < 2.0
idx = np.where(np.diff(still.astype(np.int8)) != 0)[0]
segs = [(idx[i], idx[i+1]) for i in range(len(idx)-1)
        if still[min(idx[i]+1, N-1)] and idx[i+1]-idx[i] > k]
D = -7.53


def rot(q, v):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z);   R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y);   R[:, 2, 1] = 2*(y*z+w*x);  R[:, 2, 2] = 1-2*(x*x+y*y)
    return np.einsum('nij,nj->ni', R, v)


B = rot(a[:, 89:93], a[:, 42:45])
r = (D - np.degrees(np.arctan2(B[:, 0], B[:, 1])) + 180) % 360 - 180
tilt = a[:, 0:4]
print()
print('=== 停顿段（0.1 s 平滑 |w| < 2 dps，且持续 > 1 s）===')
print('  起(s)  止(s)   磁残差r均值   r std   |a_nav|均值  sigma_yaw均值')
al = []
for s0, s1 in segs:
    if s1-s0 < k:
        continue
    rr = r[s0:s1]
    al.append(rr)
    print('  %5.2f  %5.2f    %+8.3f   %6.3f     %7.4f      %7.4f'
          % (t[s0], t[s1], rr.mean(), rr.std(), an[s0:s1].mean(), a[s0:s1, 104].mean()))
if al:
    allr = np.concatenate(al)
    print('  合计 %d 段 / %d 帧: 残差均值 %+.3f 度, std %.3f 度, |.| p90 %.3f 度'
          % (len(al), len(allr), allr.mean(), allr.std(), np.percentile(np.abs(allr), 90)))
print()
print('合加速度（标准单位，EKF 导航系，已去重力）：')
print('  静止段 |a_nav| 均值 %.4f m/s^2   全程 p99 %.4f' % (
    np.mean([an[s0:s1].mean() for s0, s1 in segs]) if segs else float('nan'),
    np.percentile(an, 99)))
