# -*- coding: utf-8 -*-
"""剧烈段（修正版）：res_g 用 accel_g，并检查陀螺/加计是否削顶。"""
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


def bn(R, v):  return np.einsum('nij,nj->ni', R, v)
def nb(R, v):  return np.einsum('nji,nj->ni', R, v)
def wrap(d):   return (d + 180.0) % 360.0 - 180.0


qe, ql = a[:, 89:93], a[:, 0:4]
magf = a[:, 42:45]          # 磁：算偏航用
accg = a[:, 32:35]          # 比力(g)：算倾角用
gyro_lsb = a[:, 18:21]
acc_lsb = a[:, 21:24]

print()
print('=== 饱和检查（criterion: |LSB| >= 32000，满量程 ±32768）===')
for nm, arr in (('gyro', gyro_lsb), ('accel', acc_lsb)):
    m = np.abs(arr).max(axis=1)
    print('  %-5s |LSB|max %6.0f   削顶帧 %d (%.3f%%)' % (nm, m.max(), int((m >= 32000).sum()),
                                                          100.0*(m >= 32000).mean()))

out = {}
for nm, q in (('EKF', qe), ('旧链', ql)):
    R = q2R(q)
    Bq = bn(R, magf)
    r = wrap(D - np.degrees(np.arctan2(Bq[:, 0], Bq[:, 1])))
    fh = accg / np.maximum(np.linalg.norm(accg, axis=1, keepdims=True), 1e-9)
    up_b = nb(R, np.tile(np.array([0.0, 0.0, 1.0]), (N, 1)))
    resg = np.degrees(np.arccos(np.clip(np.sum(fh*up_b, axis=1), -1, 1)))
    out[nm] = (r, resg)
print('  （静止时 res_g 就是姿态的倾角误差；r 是偏航误差差一个固定常数）')

k = max(int(0.1/dt.mean()), 1)
al = a[:, 10:13]
al_m = np.convolve(np.linalg.norm(al, axis=1), np.ones(k)/k, mode='same')
w3 = np.linalg.norm(a[:, 26:29], axis=1)
iv = al_m > 0.10
idx = np.where(np.diff(iv.astype(np.int8)) != 0)[0]
segs = [(idx[i], idx[i+1]) for i in range(len(idx)-1)
        if iv[min(idx[i]+1, N-1)] and idx[i+1]-idx[i] > k]
big = max(segs, key=lambda s: s[1]-s[0]) if segs else (0, N-1)
s0, s1 = big
dose = np.sum(np.linalg.norm(al[s0:s1], axis=1) * dt[s0:s1])
print()
print('=== 最大剧烈段 t=%.2f~%.2f s (%.2f s)  |a_lin|峰 %.2f g  |w|峰 %.0f dps  整流剂量 %.1f g*s'
      % (t[s0], t[s1], t[s1]-t[s0], al_m[s0:s1].max(), w3[s0:s1].max(), dose))
print('    （项目记录：20.8 g*s 对应旧链末态反 17.77 度）')
pre = slice(max(0, s0-int(2.0/dt.mean())), s0)
post = slice(s1, min(N, s1+int(2.0/dt.mean())))
far = slice(min(N-1, s1+int(5.0/dt.mean())), N)
print()
print('  %-5s %-22s %-22s %-18s' % ('', '静止(段前2s)', '刚结束(后2s)', '稳定后(5s起)'))
for nm in ('EKF', '旧链'):
    r, resg = out[nm]
    print('  %-5s  偏航误差 %+8.3f -> %+8.3f -> %+8.3f 度'
          % (nm, np.median(r[pre]), np.median(r[post]), np.median(r[far])))
    print('  %-5s  倾角误差 %8.3f -> %8.3f -> %8.3f 度'
          % ('', np.median(resg[pre]), np.median(resg[post]), np.median(resg[far])))

print()
print('=== 全程 ===')
for nm in ('EKF', '旧链'):
    r, resg = out[nm]
    print('  %-5s |偏航误差| p50 %7.3f  p90 %7.3f  max %8.3f 度   倾角误差 p50 %6.3f max %7.3f 度'
          % (nm, np.median(np.abs(r)), np.percentile(np.abs(r), 90), np.abs(r).max(),
             np.median(resg), resg.max()))
print()
print('  sigma_yaw p50 %.3f  **max %.3f** 度   <-- 与上面真实误差对照' % (
    np.median(a[:, 104]), a[:, 104].max()))
print('  |a_nav| p50 %.4f max %.2f m/s^2' % (
    np.median(np.linalg.norm(a[:, 93:96], axis=1)), np.linalg.norm(a[:, 93:96], axis=1).max()))
print('  bg 末 %s dps   ba 末 %s m/s^2' % (np.round(a[-1, 99:102], 4), np.round(a[-1, 96:99], 6)))
gb = a[:, 103].astype(np.int32)
m = np.zeros(N, bool); m[s0:s1] = True
print('  剧烈段内门开启率: tilt %.1f%%  mag %.1f%%  zupt %.1f%%  chi2剔 %.1f%%'
      % (100*np.mean((gb[m] & 0x20) != 0), 100*np.mean((gb[m] & 0x40) != 0),
         100*np.mean((gb[m] & 0x10) != 0), 100*np.mean((gb[m] & 0x200) != 0)))
print('  剧烈段外门开启率: tilt %.1f%%  mag %.1f%%'
      % (100*np.mean((gb[~m] & 0x20) != 0), 100*np.mean((gb[~m] & 0x40) != 0)))
