# -*- coding: utf-8 -*-
"""只回答一件事：静止时到底是谁在把姿态转起来。
对静止帧逐一比对：陀螺积分本应 ~0，若 EKF 在转，就看同一帧 mag_dq* 是否非零
（是地磁在驱动），否则看是不是其它观测/传播。"""
import glob
import os
import time

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
raw = np.fromfile(P, dtype='<f4')
NCH = next(n for n in (133, 128, 122) if raw.size % n == 0)
b = raw[:raw.size // NCH * NCH].reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
t = np.cumsum(b[:, 25] * 1e-6)
print('%s  %s  列 %d  帧 %d  时长 %.2f s  fw_tag %d'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         NCH, N, t[-1], b[0, 76]))


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z)
    bad = ~np.isfinite(n) | (n < 1e-6)
    n = np.where(bad, 1.0, n)
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def rp(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z)
    n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    w, x, y, z = w/n, x/n, y/n, z/n
    return (np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1))))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


g = np.linalg.norm(b[:, 26:29], axis=1)
ye = yaw(b[:, 89:93]); yl = yaw(b[:, 0:4])
re_, pe = rp(b[:, 89:93])
gs = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(g, 50, mode='edge'), 101), axis=1)
still = gs < 5.0
print('静止帧占比 %.1f%%' % (100*still.mean()))

# 用 EKF 自己报的 dq 统计（列 130..132；旧版本没有）
has_dq = NCH >= 133 and np.abs(b[:, 130:133]).max() > 0
mu = b[:, 120] if NCH >= 133 else np.full(N, np.nan)
mr = b[:, 119] if NCH >= 133 else np.full(N, np.nan)

print('\n静止帧内（%d 帧）：' % still.sum())
for nm, v in (('EKF yaw', ye), ('旧链 yaw', yl), ('EKF roll', re_), ('EKF pitch', pe)):
    r = np.abs(wrap(np.diff(v)))[still[:-1]] / np.diff(t)[still[:-1]]
    tot = np.abs(wrap(np.diff(v)))[still[:-1]].sum()
    print('  %-9s 总变化 %10.1f 度   速率 p50 %8.3f  p90 %8.2f  max %10.1f 度/秒'
          % (nm, tot, np.median(r), np.percentile(r, 90), r.max()))
print('  陀螺 |w| p50 %.3f dps  -> 纯积分本应 ~0' % np.median(g[still]))

if has_dq:
    dq = b[still, 130:133]
    nz = np.abs(dq).max(axis=1) > 0
    print('\n静止帧内地磁修正：')
    print('  mag_used=1 占 %.1f%%   dq 非零占 %.1f%%' % (100*np.nanmean(mu[still]), 100*nz.mean()))
    if nz.any():
        a = np.abs(dq[nz]); m = a.mean(axis=0)
        print('  |dq| 均值 x %.4f y %.4f z %.4f 度   占比 %.1f/%.1f/%.1f %%'
              % (m[0], m[1], m[2], 100*m[0]/m.sum(), 100*m[1]/m.sum(), 100*m[2]/m.sum()))
        print('  |dq| p50 x %.4f y %.4f z %.4f  max|dq| %.3f 度'
              % tuple(np.median(a, axis=0).tolist() + [a.max()]))
        # 关键：dq 非零的帧里 yaw 转了多少；dq=0 的帧里转了多少
        for lab, msk in (('dq!=0', nz), ('dq==0', ~nz)):
            idx = np.flatnonzero(still)[:-1]
            sel = msk[:len(idx)]
            if sel.sum() > 10:
                rr = np.abs(wrap(np.diff(ye)))[idx[sel]]
                print('    %s 帧: %d  单帧|dyaw| p50 %.4f p90 %.4f max %.3f 度'
                      % (lab, sel.sum(), np.median(rr), np.percentile(rr, 90), rr.max()))
    print('  mag_r 静止 p50 %.2f 度   mag_bh p50 %.4f' % (np.nanmedian(np.abs(mr[still])), np.median(b[still, 118])))

print('\n  其它门（静止帧）：')
gb = b[:, 103].astype(int)
for v, nm in [(0x20, 'tilt'), (0x40, 'mag'), (0x80, 'aligned'), (0x200, 'chi2'), (0x10, 'zupt')]:
    print('    %-8s %5.1f%%' % (nm, 100*((gb[still] & v) != 0).mean()))
print('  sigma_yaw 静止 p50 %.2f 度   p_yy p50 %.4g' %
      (np.median(b[still, 104]) if NCH > 104 else np.nan, np.median(b[still, 121])))
print('  四元数非有限帧占比 %.4f%%' % (100*(~np.isfinite(b[:, 89:93]).all(axis=1)).mean()))
