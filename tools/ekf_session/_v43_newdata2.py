# -*- coding: utf-8 -*-
"""稳健版：先剔除单帧毛刺，再算漂移；并定位 gate->ekf_mag_yaw 从哪来。"""
import glob, os, re
import numpy as np

cands = [c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000]
P = max(cands, key=os.path.getmtime)
b = np.fromfile(P, dtype='<f4').reshape(-1, 122).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def rollpitch(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return (np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))),
            np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1))))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


yl, ye = yaw(b[:, 0:4]), yaw(b[:, 89:93])
rl, pl = rollpitch(b[:, 0:4])
re_, pe = rollpitch(b[:, 89:93])
g = np.linalg.norm(b[:, 26:29], axis=1)

# ---- 毛刺识别：任一条链单帧 |Δyaw| > 1 度 or |gyro| > 50 dps ----
dyl, dye = np.abs(wrap(np.diff(yl))), np.abs(wrap(np.diff(ye)))
glitch = np.zeros(N, bool)
glitch[:-1] |= (dyl > 1.0) | (dye > 1.0) | (g[:-1] > 50.0) | (g[1:] > 50.0)
print('毛刺帧 %d / %d = %.2f%%   旧链|Δyaw|>1 占 %.2f%%   EKF|Δyaw|>1 占 %.2f%%   |gyro|>50 占 %.2f%%'
      % (glitch.sum(), N, 100*glitch.mean(), 100*(dyl > 1).mean(), 100*(dye > 1).mean(), 100*(g > 50).mean()))
run = np.flatnonzero(glitch)
if len(run):
    brk = np.flatnonzero(np.diff(run) > 1)
    lens = np.diff(np.r_[-1, brk, len(run)-1])
    print('  毛刺连续段 %d 个，段长 p50 %d max %d 帧；平均间隔 %.3f s'
          % (len(lens), int(np.median(lens)), int(lens.max()), t[-1]/max(len(lens), 1)))
    print('  毛刺帧里 |gyro|p50 %.1f  非毛刺帧 |gyro|p50 %.2f dps' % (np.median(g[glitch]), np.median(g[~glitch])))
    # 毛刺是否三者也一起
    both = (dyl > 1)[:-1] & (dye > 1)[:-1]
    print('  旧链与 EKF 同帧跳的比例 %.1f%%' % (100*both.mean()/max((dyl > 1).mean(), 1e-9)))
    print('  |gyro|>50 的帧里 旧链也跳 占 %.1f%%' % (100*((dyl > 1)[:-1] | (dyl > 1)[1:])[g[:-1] > 50].mean()))

# ---- 静止段（用已剔毛刺的平滑 gyro）----
gc = g.copy(); gc[glitch] = np.nan
gs = np.nanmedian(np.lib.stride_tricks.sliding_window_view(np.pad(gc, 50, mode='edge'), 101), axis=1)
gs = np.where(np.isnan(gs), 0.0, gs)
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
big = [(s, e) for s, e in segs if e - s > 2400]
print('\n静止段: %s' % ['%.2f-%.2fs(%.2fs)' % (t[s], t[e-1], t[e-1]-t[s]) for s, e in big])

print('\n【0】旧链漂移速度（剔除毛刺帧后，用"可靠帧"算）')
ok = ~glitch
for s, e in big:
    m = np.zeros(N, bool); m[s:e] = True; m &= ok
    k = np.flatnonzero(m)
    if len(k) < 100:
        continue
    net_l = wrap(yl[k[-1]] - yl[k[0]]); net_e = wrap(ye[k[-1]] - ye[k[0]])
    span = t[k[-1]] - t[k[0]]
    print('  段 %6.2f-%6.2fs 可靠帧 %d/%d  净变化 旧链 %+7.3f / EKF %+7.3f 度  跨度 %.2fs'
          % (t[s], t[e-1], len(k), e-s, net_l, net_e, span))
    print('     -> 漂移速度 旧链 %+7.4f 度/秒   EKF %+7.4f 度/秒'
          % (net_l/span, net_e/span))
    rl_ = np.abs(wrap(np.diff(yl)))[k[:-1]] / dt[k[:-1]]
    re_ = np.abs(wrap(np.diff(ye)))[k[:-1]] / dt[k[:-1]]
    print('     可靠帧内瞬时速率: 旧链 p50 %.3f p90 %.3f max %.2f | EKF p50 %.3f p90 %.3f max %.2f 度/秒'
          % (np.median(rl_), np.percentile(rl_, 90), rl_.max(),
             np.median(re_), np.percentile(re_, 90), re_.max()))

print('\n【1】崩溃式跳变（只看可靠帧）')
allok = ok[:-1] & ok[1:]
for nm, y, inst in (('旧链', yl, rl), ('EKF ', ye, re_)):
    pass
re_all = np.abs(wrap(np.diff(ye)))/dt[:-1]
rl_all = np.abs(wrap(np.diff(yl)))/dt[:-1]
leg = re_all[allok]
print('  可靠帧: EKF 瞬时速率 p50 %.3f p99 %.2f max %.2f 度/秒' %
      (np.median(leg), np.percentile(leg, 99), leg.max()))
print('  可靠帧: 旧链 p50 %.3f p99 %.2f max %.2f 度/秒' %
      (np.median(rl_all[allok]), np.percentile(rl_all[allok], 99), rl_all[allok].max()))
dd = np.abs(wrap(np.diff(wrap(ye - yl))))
print('  EKF-旧链 偏置: p50 %.2f p90 %.2f max %.2f 度' %
      (np.median(np.abs(wrap(ye-yl))), np.percentile(np.abs(wrap(ye-yl)), 90), np.abs(wrap(ye-yl)).max()))
print('  偏置单帧变化(可靠帧): p50 %.4f p99 %.3f max %.3f 度' %
      (np.median(dd[allok]), np.percentile(dd[allok], 99), dd[allok].max()))

print('\n【2】地磁牵引')
for nm, c in (('mag_r', 119), ('mag_bh', 118), ('mag_used', 120), ('mag_gate', 117), ('p_yy', 121)):
    v = b[:, c]
    print('  %-9s min %.4g  p50 %.4g  max %.4g   非零占比 %.2f%%' %
          (nm, v.min(), np.median(v), v.max(), 100*(v != 0).mean()))

print('\n【3】源码：gate->ekf_mag_yaw / s_mag_gate 的赋值处')
src = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c', 'rb').read().decode('gbk').split('\n')
for i, s in enumerate(src):
    if re.search(r'ekf_mag_yaw|s_mag_gate', s):
        print('  %4d %s' % (i+1, s[:112]))
