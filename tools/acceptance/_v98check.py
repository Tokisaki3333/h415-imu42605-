# -*- coding: utf-8 -*-
"""VER=98 新录制核验: 两个旋转段 + 一个平动段 + 大静置。
   逐列看: R=3.5*sqrt(mag_rs) / 磁权限(仅 mag_used 跳变帧求和) / 新息 / 执行率 / 倾角误差 / EKF vs att.q"""
import numpy as np, os, sys, math, glob
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P = sorted(glob.glob(r'R:\raw*.bin'), key=os.path.getmtime, reverse=True)[0]
sz = os.path.getsize(P); NCH = 162 if (sz//4) % 162 == 0 else 159
raw = np.fromfile(P, dtype='<f4').reshape(-1, NCH); N = raw.shape[0]
c = lambda i: raw[:, i].astype(np.float64)
dt = c(25).copy(); bd = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bd] = 124.58
t = np.cumsum(dt)*1e-6
tag = c(76); ok = (tag == np.median(tag))
w = raw[:, 26:29].astype(np.float64); gy = np.linalg.norm(w, axis=1)
acc = raw[:, 32:35].astype(np.float64); an = np.linalg.norm(acc, axis=1); av = acc/an[:, None]
f = raw[:, 42:45].astype(np.float64); fn = np.linalg.norm(f, axis=1)
dip = np.degrees(np.arcsin(np.clip((f*av).sum(1)/np.maximum(fn, 1e-9), -1, 1)))
rs = c(149); rdeg = c(119); used = c(120); dqz = c(137); n45 = c(45)
clip = (np.abs(raw[:, 18:21]).max(axis=1) >= 32700)
def yaw(q):
    q = q/np.linalg.norm(q, axis=1, keepdims=True); a, b, d, e = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(a*e+b*d), 1-2*(d*d+e*e)))
yA = yaw(raw[:, 0:4].astype(np.float64)); yE = yaw(raw[:, 89:93].astype(np.float64))
sg = c(107)   # sigma_tilt_deg
print('%s VER=%d CH=%d 帧 %d %.2f s 坏帧 %.2f%% 削顶 %d' %
      (os.path.basename(P), int(np.median(tag)) >> 16, (int(np.median(tag)) >> 8) & 0xFF, N, t[-1],
       100*(~ok).mean(), clip.sum()))
# 跳变帧(去 ZOH)
new = np.zeros(N, bool); new[1:] = (np.abs(np.diff(dqz)) > 1e-12) | (np.abs(np.diff(rs)) > 1e-12)
tr = new & ok
R0 = 3.5 if (int(np.median(tag)) >> 16) >= 98 else 20.0
R = R0*np.sqrt(np.maximum(rs, 0))
print('\n  t0  |w|dps |acc|g   n    dipA   sigma_t  R(度)  新息  执行%  权限°/s  att.q_yaw EKF_yaw')
for t0 in np.arange(0, t[-1]-0.5, 2.0):
    m = ok & (t >= t0) & (t < t0+2.0)
    if m.sum() < 200: continue
    u = m & tr & (used > 0.5)
    auth = np.abs(dqz[u]).sum()/2.0
    print('%5.1f %7.1f %6.3f %6.3f %7.2f %7.2f %7.1f %6.2f %6.1f %8.1f %9.2f %8.2f' %
          (t0, np.median(gy[m]), np.median(an[m]), np.median(n45[m]), np.median(dip[m]),
           np.median(sg[m]), np.median(R[m]), np.median(rdeg[m]), 100*np.mean(used[m] > 0.5), auth,
           np.median(yA[m]), np.median(yE[m])))
# 分类统计
stat = ok & (gy < 3.0) & (np.abs(an-1.0) < 0.015)
rot = ok & (gy > 30.0)
trans = ok & (gy < 30.0) & (np.abs(an-1.0) > 0.03) & ~stat
print('\n%-8s %8s %8s %8s %9s %9s %9s %9s' % ('类别', '时长s', '|w|中位', '|acc|中位', 'sigma_t', 'R中位°', '新息中位', '权限°/s'))
for nm, m in (('静置', stat), ('旋转', rot), ('平动', trans)):
    if m.sum() < 500: print('%-8s 帧数不足 (%d)' % (nm, m.sum())); continue
    u = m & tr & (used > 0.5)
    span = m.sum()*np.median(dt)*1e-6
    print('%-8s %8.2f %8.1f %8.3f %9.2f %9.2f %9.2f %9.1f' %
          (nm, span, np.median(gy[m]), np.median(an[m]), np.median(sg[m]), np.median(R[m]),
           np.median(rdeg[m]), np.abs(dqz[u]).sum()/max(span, 1e-9)))
# 静置台阶 + EKF vs att.q
idx = np.where(stat)[0]
segs = [s for s in np.split(idx, np.where(np.diff(idx) > 800)[0]+1) if len(s) >= 6400]
print('\n静置台阶:')
prev = None; errs = []
for k, s in enumerate(segs):
    i = s[len(s)//2]
    u = np.abs(an[s]-1.0) < 0.005
    d = '' if prev is None else 'Δatt.q %+7.2f  ΔEKF %+7.2f  (差 %+6.2f)' % (
        ((np.median(yA[s])-prev[0]+180) % 360)-180, ((np.median(yE[s])-prev[1]+180) % 360)-180,
        (((np.median(yE[s])-prev[1]+180) % 360)-180) - (((np.median(yA[s])-prev[0]+180) % 360)-180))
    print('  #%d %6.2f~%6.2f s (%4.1fs) n %.4f dipA %+6.2f  att.q %8.2f EKF %8.2f  %s' %
          (k+1, t[s[0]], t[s[-1]], t[s[-1]]-t[s[0]], np.median(n45[s]), np.median(dip[s]),
           np.median(yA[s]), np.median(yE[s]), d))
    if prev is not None:
        errs.append(d.split('差 ')[1][:6])
    prev = (np.median(yA[s]), np.median(yE[s]))
# 静置段倾角误差
up = np.stack([2*(raw[:, 1]*raw[:, 3]-raw[:, 0]*raw[:, 2]), 2*(raw[:, 2]*raw[:, 3]+raw[:, 0]*raw[:, 1]),
               1-2*(raw[:, 1]**2+raw[:, 2]**2)], 1).astype(np.float64)
upA = np.stack([2*(raw[:, 1]*raw[:, 3]-raw[:, 0]*raw[:, 2]), 2*(raw[:, 2]*raw[:, 3]+raw[:, 0]*raw[:, 1]),
                1-2*(raw[:, 1]**2+raw[:, 2]**2)], 1).astype(np.float64)
print('\n静置段 |att.q_up − accel| 中位: %.3f°  (VER=97 基线 0.15~0.17°)' %
      np.median(np.degrees(np.arccos(np.clip((upA*av).sum(1), -1, 1)))[stat]))
