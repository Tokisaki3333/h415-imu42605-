# -*- coding: utf-8 -*-
"""VER=50（139 列）新数据：慢激励 + 多次回原位。
问：静止时还在转吗？转的时候是 M6(tilt_dq*) 还是地磁(mag_dq*) 在驱动？
     tilt_pr* 是否异常？多次回原位的重复性如何？"""
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
if NCH < 139:
    raise SystemExit('不是 139 列')


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


g = np.linalg.norm(b[:, 26:29], axis=1)
ye, yl = yaw(b[:, 89:93]), yaw(b[:, 0:4])
re_, pe = rp(b[:, 89:93])
dq_mag = b[:, 130:133]
dq_til = b[:, 133:136]
pr = b[:, 136:139]
print('陀螺 |w| max %.1f dps（用户称激励更慢）' % g.max())

gs = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(g, 50, mode='edge'), 101), axis=1)
still = gs < 5.0
print('静止帧 %.1f%%   运动帧 |w| p50 %.1f max %.1f dps'
      % (100*still.mean(), np.median(g[~still]) if (~still).any() else 0, g[~still].max()))

print('\n【1】全姿态角速率（度/秒），分静止/运动')
for lab, m in (('静止', still), ('运动', ~still)):
    if m.sum() < 10:
        continue
    rows = []
    for nm, v in (('yaw/EKF', ye), ('roll/EKF', re_), ('pitch/EKF', pe), ('yaw/旧链', yl)):
        r = np.abs(wrap(np.diff(v)))[m[:-1]] / dt[:-1][m[:-1]]
        rows.append('%s p50 %7.3f p99 %8.2f max %10.1f' % (nm, np.median(r), np.percentile(r, 99), r.max()))
    print('  %s: %s' % (lab, rows[0]))
    for x in rows[1:]:
        print('        %s' % x)

print('\n【2】静止段里"无陀螺激励却在转"的帧（|gyro|<5 但 |EKF 姿态单帧变化| > 0.05 度）')
ang = np.abs(wrap(np.diff(ye))) + np.abs(wrap(np.diff(re_))) + np.abs(wrap(np.diff(pe)))
bad = still[:-1] & still[1:] & (ang > 0.05)
print('  这类帧 %d / %d = %.3f%%' % (bad.sum(), still.sum(), 100*bad.sum()/max(still.sum(), 1)))
if bad.any():
    idx = np.flatnonzero(bad)
    print('  这些帧: |gyro| p50 %.3f   姿态单帧变化 p50 %.4f max %.3f 度' %
          (np.median(g[idx]), np.median(ang[idx]), ang[idx].max()))
    print('  mag_used=1 占 %.1f%%   |mag_dq| 均值 %s   |tilt_dq| 均值 %s'
          % (100*np.nanmean(b[idx+1, 120]),
             np.array2string(np.abs(dq_mag[idx+1]).mean(axis=0), precision=4),
             np.array2string(np.abs(dq_til[idx+1]).mean(axis=0), precision=4)))
    print('  |mag_dq| p90 %s   |tilt_dq| p90 %s'
          % (np.array2string(np.percentile(np.abs(dq_mag[idx+1]), 90, axis=0), precision=4),
             np.array2string(np.percentile(np.abs(dq_til[idx+1]), 90, axis=0), precision=4)))
    print('  tilt_pr 均值 %s  （静止且倾角对时应≈0,0,1）'
          % np.array2string(pr[idx+1].mean(axis=0), precision=4))
    print('  tilt_pr 单帧变化 p50 %s'
          % np.array2string(np.median(np.abs(np.diff(pr, axis=0))[idx], axis=0), precision=5))
    print('  时刻(前12): %s' % np.array2string(t[idx[:12]], precision=2))
    print('  gate_bits 取值: %s' % np.unique(b[idx+1, 103].astype(int))[:10])

print('\n【3】全程修正量对比（每次更新注入的角度，度）')
for nm, a in (('mag_dq', dq_mag), ('tilt_dq', dq_til)):
    nz = np.abs(a).max(axis=1) > 0
    if nz.any():
        av = np.abs(a[nz])
        print('  %-8s 非零 %.1f%%  |dq| 均值 x %.4f y %.4f z %.4f  max %.3f'
              % (nm, 100*nz.mean(), *av.mean(axis=0), av.max()))

print('\n【4】tilt_pr（预测机体系重力方向）—— 静止时应稳、且第三分量≈±1')
for lab, m in (('静止', still), ('运动', ~still)):
    if m.sum() < 10:
        continue
    print('  %s: pr p50 %s   pr 单帧变化 p50 %s' %
          (lab, np.array2string(np.median(pr[m], axis=0), precision=4),
           np.array2string(np.median(np.abs(np.diff(pr, axis=0))[m[:-1]], axis=0), precision=5)))

print('\n【5】多次回原位的重复性（静止段之间的姿态差，度）')
idx = np.flatnonzero(np.diff(still.astype(np.int8)) != 0)
segs, cur = [], (idx[0] if still[0] else idx[1])
for i in idx:
    if still[i] != still[cur]:
        if still[cur]:
            segs.append((cur, i))
        cur = i
if still[cur]:
    segs.append((cur, N-1))
big = [(s, e) for s, e in segs if e - s > 800]
print('  静止段 %d 个: %s' % (len(big), ['%.1f-%.1fs' % (t[s], t[e-1]) for s, e in big]))
ref = None
for s, e in big:
    m = slice(s, min(e, s+800))
    ye_m, re_m, pe_m = np.median(ye[m]), np.median(re_[m]), np.median(pe[m])
    if ref is None:
        ref = (ye_m, re_m, pe_m)
        print('    基准 %.1fs: yaw %+.3f roll %+.3f pitch %+.3f' % (t[s], ye_m, re_m, pe_m))
    else:
        print('    回位 %.1fs: dyaw %+8.3f  droll %+8.3f  dpitch %+8.3f 度  （相对基准）'
              % (t[s], wrap(ye_m-ref[0]), re_m-ref[1], pe_m-ref[2]))
print('  漂移参考：旧链 yaw 首末差 %+.2f 度   EKF yaw 首末差 %+.2f 度'
      % (wrap(yl[-1]-yl[0]), wrap(ye[-1]-ye[0])))
