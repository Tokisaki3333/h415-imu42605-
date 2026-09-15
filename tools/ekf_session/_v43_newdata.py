# -*- coding: utf-8 -*-
"""VER=48 新数据：大宗运动 -> 放回原位静止(3.15~20.31s, 17.16s) -> 再动。

0) 旧链漂移速度范围（段内线性拟合 + 始末差）
1) EKF 崩溃式跳变（静止段内瞬时 yaw 速率 vs 旧链）
2) 运动结束刚进入静止时，地磁是否大量牵引、幅度是否接近一个旧链漂移
"""
import glob
import os

import numpy as np

cands = [c for c in glob.glob(r'R:\*.bin') + glob.glob(r'C:\Users\33\Documents\v2\**\*.bin', recursive=True)
         if os.path.getsize(c) > 100000]
P = max(cands, key=os.path.getmtime)
b = np.fromfile(P, dtype='<f4').reshape(-1, 122).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)
print('文件 %s   fw_tag %d   帧 %d   时长 %.2f s' % (P, b[0, 76], N, t[-1]))


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


yl, ye = yaw(b[:, 0:4]), yaw(b[:, 89:93])
g = np.linalg.norm(b[:, 26:29], axis=1)
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
    segs.append((cur, N - 1))
big = [(s, e) for s, e in segs if e - s > 2400]
print('静止段(>0.3s): %s' % ['%.2f-%.2fs(%.2fs)' % (t[s], t[e-1], t[e-1]-t[s]) for s, e in big])

# ---------------- 0) 旧链漂移速度范围 ----------------
print('\n【0】旧链漂移速度')
rates_l, rates_e = [], []
for s, e in big:
    tt = t[s:e] - t[s]
    kl = np.degrees(np.polyfit(tt, np.unwrap(np.radians(yl[s:e])), 1)[0])
    ke = np.degrees(np.polyfit(tt, np.unwrap(np.radians(ye[s:e])), 1)[0])
    rates_l.append(kl); rates_e.append(ke)
    print('  段 %6.2f-%6.2fs(%5.2fs): 旧链 %+7.4f 度/秒    EKF %+7.4f 度/秒   '
          '段内 Δ 旧链 %+7.3f / EKF %+7.3f 度'
          % (t[s], t[e-1], t[e-1]-t[s], kl, ke, wrap(yl[e-1]-yl[s]), wrap(ye[e-1]-ye[s])))
s0, e0 = big[0]
print('  最长静止段始末差: 旧链 %+7.3f 度 / EKF %+7.3f 度   (间隔 %.2fs)'
      % (wrap(yl[e0]-yl[s0]), wrap(ye[e0]-ye[s0]), t[e0]-t[s0]))
leg_hi = max(abs(r) for r in rates_l) if rates_l else 0.0
leg_lo = min(abs(r) for r in rates_l) if rates_l else 0.0
print('  => 旧链漂移速度范围 [%.4f, %.4f] 度/秒；取上限 %.4f' % (leg_lo, leg_hi, leg_hi))

# ---------------- 1) EKF 崩溃式跳变 ----------------
stat = np.zeros(N, bool)
for s, e in big:
    stat[s:e] = True
dye, dyl = wrap(np.diff(ye)), wrap(np.diff(yl))
r_e, r_l = np.abs(dye)/dt[:-1], np.abs(dyl)/dt[:-1]
m = stat[:-1] & stat[1:]
print('\n【1】静止段内瞬时 |yaw 速率| (度/秒)')
for nm, r in (('EKF ', r_e), ('旧链', r_l)):
    print('  %s: p50 %7.3f  p90 %7.3f  p99 %7.3f  max %9.3f' %
          (nm, np.median(r[m]), np.percentile(r[m], 90), np.percentile(r[m], 99), r[m].max()))
thr = max(3.0*leg_hi, 2.0)
jump = m & (r_e > thr)
print('  阈值 %.3f 度/秒 (3x 旧链上限, 且>=2)' % thr)
print('  EKF 超阈值帧 %d/%d = %.3f%%   其中旧链同刻也超阈值 %d 帧'
      % (jump.sum(), m.sum(), 100.0*jump.sum()/max(m.sum(), 1), int((jump & (r_l > thr)).sum())))
if jump.any():
    j = np.flatnonzero(jump)
    k = j[np.argmax(r_e[j])]
    print('  最大: t=%.2fs  EKF %.2f 度/秒  旧链同刻 %.2f 度/秒  单帧Δ %.3f度  |gyro| %.1f dps'
          % (t[k], r_e[k], r_l[k], dye[k], g[k]))
    print('  逐帧速率 TOP8: t / EKF / 旧链')
    for k in j[np.argsort(r_e[j])[-8:]][::-1]:
        print('     %6.2fs  %9.3f  %9.3f' % (t[k], r_e[k], r_l[k]))
# EKF-旧链 偏置本身的跳变（更直接的"崩溃式跳变"判据）
d = wrap(ye - yl)
dd = np.abs(wrap(np.diff(d)))
print('  |EKF-旧链| 偏置: p50 %.2f度  p90 %.2f度  max %.2f度' %
      (np.median(np.abs(d)), np.percentile(np.abs(d), 90), np.abs(d).max()))
print('  偏置单帧变化: p50 %.4f  p99 %.3f  max %.3f 度  (max 出现在 t=%.2fs, 该帧|gyro|=%.1f)'
      % (np.median(dd), np.percentile(dd, 99), dd.max(), t[int(np.argmax(dd))], g[int(np.argmax(dd))]))

# ---------------- 2) 运动->静止 的地磁牵引 ----------------
print('\n【2】运动 -> 静止 切换处的地磁牵引')
mr, gat, used = b[:, 119], b[:, 117], b[:, 120]
W1, W2 = 800, 2400   # 前 1s / 后 3s
for k, (s, e) in enumerate(big):
    if s - W1 < 0:
        continue
    a, c = s - W1, min(N - 1, s + W2)
    dy_e = wrap(ye[c] - ye[a]); dy_l = wrap(yl[c] - yl[a])
    print('  进入静止 @%6.2fs: 前1s|mag_r|p50 %7.2f -> 后3s %7.2f 度' %
          (t[s], np.median(np.abs(mr[a:s])), np.median(np.abs(mr[s:c]))))
    print('     该 4s 窗内 Δyaw: EKF %+8.3f 度   旧链 %+8.3f 度   差(地磁牵引) %+8.3f 度'
          % (dy_e, dy_l, wrap(dy_e - dy_l)))
    print('     后段 mag_gate %.0f%%  mag_used %.0f%%  sigma_yaw p50 %.2f  p_yy p50 %.4g'
          % (100*gat[s:c].mean(), 100*used[s:c].mean(), np.median(b[s:c, 104]), np.median(b[s:c, 121])))
print('\n  参照"一个旧链漂移"：%.4f 度/秒 x 4s = %.3f 度' % (leg_hi, leg_hi*4))
print('  全程: |mag_r| p50 %.2f p90 %.2f max %.2f 度  bh p50 %.4f  p_yy p50 %.4g  mag_used %.0f%%'
      % (np.median(np.abs(mr)), np.percentile(np.abs(mr), 90), np.abs(mr).max(),
         np.median(b[:, 118]), np.median(b[:, 121]), 100*used.mean()))
gb = b[:, 103].astype(int)
for v, nm in [(0x20, 'tilt'), (0x40, 'mag'), (0x80, 'aligned'), (0x200, 'chi2')]:
    print('     bit %-8s %5.1f%%' % (nm, 100*((gb & v) != 0).mean()))
