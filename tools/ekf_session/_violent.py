# -*- coding: utf-8 -*-
"""剧烈运动段：同一段数据上把 **EKF 与旧链**并排比。

两条真值口径，都不依赖"我知道板子转到哪"：
  A) 偏航：r = wrap(D - h)，h = atan2((R(q)·mag_f)_x, ·_y)。
     导航系已用磁力计对齐到真 ENU 时，真实水平地磁方位恒等于 D，
     所以 r 就是**该姿态的偏航误差**（两套算法各自差一个固定常数，
     取"运动后 − 运动前"就把常数消掉了）。
  B) 倾角：res_g = f_hat 与 R(q)^T·(0,0,1) 的夹角。
     静止时比力方向就是"上"，所以这就是姿态的倾角误差，与磁无关。
另外给出整流剂量 int |a_lin| dt（g·s）—— 项目里 20.8 g·s 对应末态反 17.77 度。
"""
import numpy as np

a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
D = -7.53
print('帧数 %d  时长 %.2f s  fw_tag %.0f' % (N, t[-1], a[0, 76]))


def q2R(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z);   R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y);   R[:, 2, 1] = 2*(y*z+w*x);  R[:, 2, 2] = 1-2*(x*x+y*y)
    return R


def bn(R, v):
    return np.einsum('nij,nj->ni', R, v)


def nb(R, v):
    return np.einsum('nji,nj->ni', R, v)


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


qe, ql = a[:, 89:93], a[:, 0:4]
f = a[:, 42:45]
al = a[:, 10:13]                       # 旧链 a_lin（g）
an = np.linalg.norm(a[:, 93:96], axis=1)

# 姿态误差指标（各算一份）
out = {}
for nm, q in (('EKF', qe), ('旧链', ql)):
    R = q2R(q)
    Bq = bn(R, f)
    r = wrap(D - np.degrees(np.arctan2(Bq[:, 0], Bq[:, 1])))
    fh = f / np.maximum(np.linalg.norm(f, axis=1, keepdims=True), 1e-9)
    up_b = nb(R, np.tile(np.array([0.0, 0.0, 1.0]), (len(R), 1)))
    resg = np.degrees(np.arccos(np.clip(np.sum(fh*up_b, axis=1), -1, 1)))
    out[nm] = (r, resg)

# ---- 找剧烈段：|a_lin| 的 0.1 s 平滑最大值 ----
k = max(int(0.1/dt.mean()), 1)
al_m = np.convolve(np.linalg.norm(al, axis=1), np.ones(k)/k, mode='same')
w3 = np.linalg.norm(a[:, 26:29], axis=1)
print('|a_lin| 平滑 p50 %.3f  p90 %.3f  p99 %.3f  max %.3f g'
      % tuple(np.percentile(al_m, [50, 90, 99, 100])))
print('|w|     p50 %.2f  p90 %.2f  p99 %.2f  max %.2f dps'
      % tuple(np.percentile(w3, [50, 90, 99, 100])))
iv = al_m > 0.10                        # 剧烈判据：平滑 |a_lin| > 100 mg
idx = np.where(np.diff(iv.astype(np.int8)) != 0)[0]
segs = [(idx[i], idx[i+1]) for i in range(len(idx)-1)
        if iv[min(idx[i]+1, N-1)] and idx[i+1]-idx[i] > k]
print('剧烈段（%d 个，判据 0.1 s 平滑 |a_lin| > 100 mg）：' % len(segs))
for s0, s1 in segs:
    dose = np.sum(np.linalg.norm(al[s0:s1], axis=1) * dt[s0:s1])
    print('   t=%6.2f~%6.2f s (%.2f s)  |a_lin|峰 %.3f g  |w|峰 %6.1f dps  整流剂量 %.2f g*s'
          % (t[s0], t[s1], t[s1]-t[s0], al_m[s0:s1].max(), w3[s0:s1].max(), dose))

if segs:
    s0 = segs[0][0]
    s1 = segs[-1][1]
    print()
    print('=== 剧烈段前/后的姿态误差（同一段数据，两套算法并排）===')
    pre = slice(max(0, s0 - int(2.0/dt.mean())), s0)
    post = slice(s1, min(N, s1 + int(2.0/dt.mean())))
    far = slice(min(N-1, s1 + int(5.0/dt.mean())), N)
    print('  %-6s %14s %14s %14s %12s' % ('', '运动前2s', '运动后2s', 'Δ(yaw)', 'res_g前后'))
    for nm in ('EKF', '旧链'):
        r, resg = out[nm]
        r0, r1 = np.median(r[pre]), np.median(r[post])
        print('  %-6s  yaw残差 %+7.3f  %+7.3f 度    Δ %+7.3f 度    %5.3f -> %5.3f 度'
              % (nm, r0, r1, wrap(r1 - r0), np.median(resg[pre]), np.median(resg[post])))
    if len(range(*far.indices(N))) > 10:
        print('  （运动后 5 s 起）')
        for nm in ('EKF', '旧链'):
            r, resg = out[nm]
            print('  %-6s  yaw残差 %+7.3f 度   res_g %5.3f 度' % (
                nm, np.median(r[far]), np.median(resg[far])))

print()
print('=== 全程 r 与 res_g 分位 ===')
for nm in ('EKF', '旧链'):
    r, resg = out[nm]
    print('  %-6s |r| p50 %7.3f p90 %7.3f max %8.3f 度   res_g p50 %6.3f p90 %6.3f max %7.3f 度'
          % (nm, np.median(np.abs(r)), np.percentile(np.abs(r), 90), np.abs(r).max(),
             np.median(resg), np.percentile(resg, 90), resg.max()))
print()
print('  sigma_yaw p50 %.3f max %.3f 度   |a_nav| p50 %.4f max %.2f m/s^2'
      % (np.median(a[:, 104]), a[:, 104].max(), np.median(an), an.max()))
print('  bg 末 %s dps   ba 末 %s m/s^2'
      % (np.round(a[-1, 99:102], 4), np.round(a[-1, 96:99], 6)))
gb = a[:, 103].astype(np.int32)
print('  门开启率: zupt %.1f%%  tilt %.1f%%  mag %.1f%%  baro %.1f%%  chi2剔 %.1f%%'
      % (100*np.mean((gb & 0x10) != 0), 100*np.mean((gb & 0x20) != 0),
         100*np.mean((gb & 0x40) != 0), 100*np.mean((gb & 0x04) != 0),
         100*np.mean((gb & 0x200) != 0)))
if segs:
    m = np.zeros(N, bool)
    for s0, s1 in segs:
        m[s0:s1] = True
    print('  剧烈段内: tilt %.1f%%  mag %.1f%%  chi2剔 %.1f%%'
          % (100*np.mean((gb[m] & 0x20) != 0), 100*np.mean((gb[m] & 0x40) != 0),
             100*np.mean((gb[m] & 0x200) != 0)))
