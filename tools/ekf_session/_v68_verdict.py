# -*- coding: utf-8 -*-
"""VER=68 判决。144 列的正确索引（VER=67 把 5 列插在 mag_ry 之后，后面全部后移 5）：
 118 mag_bh  119 mag_r  120 mag_used  121 p_yy
 128 mag_rx  129 mag_ry
 130 mag_vx  131 mag_vy  132 mag_v0x  133 mag_v0y  134 mag_yawpre
 135 mag_dqx 136 mag_dqy 137 mag_dqz
 138 tilt_dqx 139 tilt_dqy 140 tilt_dqz 141 tilt_prx 142 tilt_pry 143 tilt_prz
判据核心：EKF 偏航 vs 旧链真值（不是 mag_r —— mag_r 小不代表偏航对）。
"""
import glob, os, time
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4'); n = 144
b = r[:r.size//n*n].reshape(-1, n).astype(np.float64); N = len(b)
t = np.cumsum(b[:, 25]*1e-6)
print('%s %s 列%d 帧%d 时长%.2fs fw_tag %d'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         n, N, t[-1], b[0, 76]))


def qn(q):
    nn = np.sqrt((q*q).sum(axis=1)); nn = np.where((~np.isfinite(nn)) | (nn < 1e-9), 1, nn)
    return q/nn[:, None]
def yaw(q):
    q = qn(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
def rp(q):
    q = qn(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return (np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1))))
def wr(d): return (d+180.0) % 360.0-180.0

ye, yl = yaw(b[:, 89:93]), yaw(b[:, 0:4])
d = wr(ye-yl)
g = np.linalg.norm(b[:, 26:29], axis=1)
mr, mbh, mu, pyy = b[:, 119], b[:, 118], b[:, 120], b[:, 121]
mdq, tdq = b[:, 137], b[:, 140]
yp = b[:, 134]
print('|gyro| p50 %.1f max %.1f' % (np.median(g), g.max()))
print('dyaw(EKF-旧链) p10 %+8.2f p50 %+8.2f p90 %+8.2f | 首 %+8.2f 末 %+8.2f'
      % (*np.percentile(d, [10, 50, 90]), d[0], d[-1]))
print('dyaw 标准差 %.2f   跨度 %.2f' % (d.std(), d.max()-d.min()))
print('mag_r p50 %.2f p90 %.2f max %.2f | mag_dqz |.| p50 %.3f p90 %.3f max %.3f | tilt_dqz p50 %.4f max %.3f'
      % (np.median(abs(mr)), np.percentile(abs(mr), 90), abs(mr).max(),
         np.median(abs(mdq)), np.percentile(abs(mdq), 90), abs(mdq).max(),
         np.median(abs(tdq)), abs(tdq).max()))
print('mag_used=1 %.1f%%  p_yy 首 %.4g 末 %.4g' % (100*mu.mean(), pyy[0], pyy[-1]))
print('旧链yaw首末 %+.2f   EKF %+.2f' % (wr(yl[-1]-yl[0]), wr(ye[-1]-ye[0])))

# 单帧大跳
for thr in (30, 90):
    k = np.flatnonzero(np.abs(wr(np.diff(ye))) > thr)
    print('单帧 |dEKF_yaw| > %d 度: %d 帧' % (thr, len(k)), np.array2string(t[k[:8]], precision=2) if len(k) else '')
k = np.flatnonzero(np.abs(wr(np.diff(d))) > 20)
print('dyaw 单帧变化 > 20 度: %d 帧' % len(k), np.array2string(t[k[:8]], precision=2) if len(k) else '')

gs = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(g, 50, mode='edge'), 101), axis=1)
st = gs < 5.0
i0 = np.flatnonzero(np.diff(st.astype(np.int8)) != 0)
segs, cur = [], (i0[0] if st[0] else i0[1])
for i in i0:
    if st[i] != st[cur]:
        if st[cur]: segs.append((cur, i))
        cur = i
if st[cur]: segs.append((cur, N-1))
big = [x for x in segs if x[1]-x[0] > 800]
print('\n静止段(>0.1s) %d 个' % len(big))
print(' 区间           时长   dyaw首->末          mag_r p50 |dq|p50 used  p_yy末')
for s, e in big[:12]:
    print(' %5.1f-%5.1fs %5.2fs  %+8.2f -> %+8.2f   %7.2f  %6.3f %4.0f%%  %.4g'
          % (t[s], t[e-1], t[e-1]-t[s], d[s], d[e-1], np.median(abs(mr[s:e])),
             np.median(abs(mdq[s:e])), 100*mu[s:e].mean(), pyy[e-1]))
print('\n idx     t   |gyro|  dyaw   mag_r mag_dqz tilt_dqz  p_yy')
for i in range(0, N, max(1, N//24)):
    print('%6d %6.2f %7.1f %+8.2f %7.2f %+7.3f %+8.4f %7.4g'
          % (i, t[i], g[i], d[i], mr[i], mdq[i], tdq[i], pyy[i]))
