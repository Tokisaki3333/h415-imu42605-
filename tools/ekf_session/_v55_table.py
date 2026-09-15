# -*- coding: utf-8 -*-
"""VER=55 判据：在线看 EKF 是否磁正确；旧链只作离线定位尺。"""
import glob, os, time
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4')
nch = next(n for n in (139, 133, 128, 122)
           if r.size >= n*3 and r[76] == r[n+76] == r[2*n+76] and 1e6 < r[76] < 4e6)
b = r[:r.size//nch*nch].reshape(-1, nch).astype(np.float64)
N = len(b); dt = b[:, 25]*1e-6; t = np.cumsum(dt)
print('%s %s  列%d 帧%d 时长%.2fs  fw_tag %d'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         nch, N, t[-1], b[0, 76]))


def yaw(q):
    n = np.sqrt((q*q).sum(axis=1)); n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    q = q/n[:, None]; w, x, yy, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z+x*yy), 1-2*(yy*yy+z*z)))


def wrap(d): return (d+180.0) % 360.0-180.0


ye, yl = yaw(b[:, 89:93]), yaw(b[:, 0:4])
d = wrap(ye-yl)
g = np.linalg.norm(b[:, 26:29], axis=1)
mr, mbh, mu, mdq, pyy = b[:, 119], b[:, 118], b[:, 120], b[:, 132], b[:, 121]
print('|gyro| p50 %.1f max %.1f dps' % (np.median(g), g.max()))
print('mag_r   p50 %7.2f p90 %7.2f max %7.2f' % (np.median(abs(mr)), np.percentile(abs(mr), 90), abs(mr).max()))
print('mag_dqz |.| p50 %.3f p90 %.3f max %.3f 度 ; >20度 %d 帧 ; >60度 %d 帧'
      % (np.median(abs(mdq)), np.percentile(abs(mdq), 90), abs(mdq).max(),
         (abs(mdq) > 20).sum(), (abs(mdq) > 60).sum()))
J = np.flatnonzero(abs(mdq) > 20)
print('大 mag_dqz 时刻:', np.array2string(t[J[:12]], precision=2))
for k in J[:6]:
    print('   t=%7.3f mag_dqz %+8.2f  mag_r前 %7.2f 后 %7.2f   dyaw %+7.2f -> %+7.2f  used %d'
          % (t[k], mdq[k], mr[k-1] if k else 0, mr[k+1] if k+1 < N else 0,
             d[k-1] if k else 0, d[k+1] if k+1 < N else 0, int(mu[k])))
print('mag_used=1 占 %.1f%%   p_yy 首 %.4g 末 %.4g' % (100*mu.mean(), pyy[0], pyy[-1]))

gs = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(g, 50, mode='edge'), 101), axis=1)
still = gs < 5.0
idx = np.flatnonzero(np.diff(still.astype(np.int8)) != 0)
segs, cur = [], (idx[0] if still[0] else idx[1])
for i in idx:
    if still[i] != still[cur]:
        if still[cur]:
            segs.append((cur, i))
        cur = i
if still[cur]:
    segs.append((cur, N-1))
big = [(s, e) for s, e in segs if e-s > 800]
print('\n静止段定位表（旧链=真值，仅用于量 EKF 偏差）')
print('  区间          时长   dyaw首->末        mag_r p50  used   |dq|p50 |dq|max  p_yy末')
for s, e in big[:14]:
    print('  %5.1f-%5.1fs %5.1fs  %+7.2f -> %+7.2f   %7.2f   %4.0f%%  %6.3f %7.3f  %.4g'
          % (t[s], t[e-1], t[e-1]-t[s], d[s], d[e-1],
             np.median(abs(mr[s:e])), 100*mu[s:e].mean(),
             np.median(abs(mdq[s:e])), abs(mdq[s:e]).max(), pyy[e-1]))
print('\n启动首 40 帧: idx t mag_dqz mag_r mag_bh used')
for k in range(0, min(N, 40), 4):
    print('   %4d %6.3f %+8.3f %7.2f %6.3f %3d' % (k, t[k], mdq[k], mr[k], mbh[k], int(mu[k])))
print('首 1s 内 |mag_dqz| max %.2f 度（应出现一次性大角度）' % abs(mdq[:int(1.0/np.median(dt))]).max())
print('旧链 yaw 首末 %+.2f   EKF yaw 首末 %+.2f' % (wrap(yl[-1]-yl[0]), wrap(ye[-1]-ye[0])))
