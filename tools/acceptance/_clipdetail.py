# -*- coding: utf-8 -*-
"""逐段列出削顶细节: 哪根轴、哪个符号、峰值、当时的 |acc| 与 |ω|; 并判断正负号是否对称(净丢失角是否抵消)"""
import numpy as np, glob, os, sys
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P = sorted(glob.glob(r'R:\raw*.bin'), key=os.path.getmtime, reverse=True)[0]
raw = np.fromfile(P, dtype='<f4').reshape(-1, 162); N = raw.shape[0]
c = lambda i: raw[:, i].astype(np.float64)
dt = c(25).copy(); bd = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bd] = 124.58
t = np.cumsum(dt)*1e-6
tag = c(76); ok = (tag == np.median(tag))
L = raw[:, 18:21].astype(np.float64); D = raw[:, 26:29].astype(np.float64)
gy = np.linalg.norm(D, axis=1); an = np.linalg.norm(raw[:, 32:35].astype(np.float64), axis=1)
al = np.abs(L)
clip = (al.max(axis=1) >= 32700) & ok
print('%s VER=%d 覆盖 %.2f s; 削顶 %d 帧 (%.3f%%)' %
      (os.path.basename(P), int(np.median(tag)) >> 16, t[-1], clip.sum(), 100*clip.mean()))
print('\n非削顶帧 |LSB| 分位: p50 %.0f p99 %.0f p99.9 %.0f max %.0f' %
      tuple(np.percentile(al[~clip & ok].max(axis=1), [50, 99, 99.9, 100])))
idx = np.where(clip)[0]
sg = np.split(idx, np.where(np.diff(idx) > 40)[0]+1)
print('\n段  时刻(s)   帧数  时长ms  轴(峰值LSB,符号)                 |ω|dps  |acc|g  前后 |acc|')
tot = np.zeros(3)
for k, s in enumerate(sg):
    ax = int(np.argmax(np.abs(L[s]).max(axis=0)))
    pk = L[s][:, ax]
    sgn = '+' if np.mean(pk) > 0 else '-'
    near = (t >= t[s[0]]-0.1) & (t <= t[s[-1]]+0.1) & ok
    before = (t >= t[s[0]]-0.3) & (t < t[s[0]]) & ok
    tot[ax] += pk.sum()
    print('#%-2d %7.2f %6d %7.0f  axis%d %+7.0f (%s)  %8.0f %7.2f  %.2f' %
          (k+1, t[s[0]], len(s), len(s)*np.median(dt)*1e-3, ax+1, np.mean(pk), sgn,
           np.median(gy[near]), np.median(an[near]), np.median(an[before]) if before.sum() else 0))
print('\n各轴削顶帧的 LSB 代数和: [%+.0f %+.0f %+.0f] (代数和≈0 表示正负对称, 净丢失角抵消)' % tuple(tot))
print('削顶帧 |acc| 中位 %.2f g (非削顶运动帧 %.2f g)' %
      (np.median(an[clip]), np.median(an[ok & ~clip & (t > 5) & (t < 22)])))
