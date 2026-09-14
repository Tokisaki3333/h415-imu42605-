# -*- coding: utf-8 -*-
"""XY（横滚/俯仰）为什么飘：把日志里所有相关的量同时摆出来。

通道（122 帧，CH_112）：att_q=0, gyro_dps=26, accel_g=32, a_lin=10,
                        ekf_q=89, ekf_ba=96, ekf_bg=99, ekf_gate_bits=103,
                        ekf_sigma_tilt_deg=107(CH_113 起), mag_f=42
输出：
  1) 静止段里 roll/pitch 的漂移速率（度/秒）—— 飘不飘、飘多快
  2) 同一段里 ba / bg 是否在长（有没有把倾斜误差吃进加计零偏）
  3) 重力校正(M6)门位 tilt(0x20) 的开启率，按运动强度分
  4) 加计实测倾角 vs EKF 倾角 vs 旧链倾角 —— 谁在飘
"""
import numpy as np

b = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 122).astype(np.float64)
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)


def rp(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    roll = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
    return roll, pitch


roll_e, pit_e = rp(b[:, 89:93])
roll_l, pit_l = rp(b[:, 0:4])
a = b[:, 32:35]                      # accel_g
nrm = np.linalg.norm(a, axis=1)
roll_a = np.degrees(np.arctan2(-a[:, 1], -a[:, 2]))
pit_a = np.degrees(np.arctan2(a[:, 0], np.sqrt(a[:, 1]**2 + a[:, 2]**2)))
ba = b[:, 96:99]
bg = b[:, 99:102]
gb = b[:, 103].astype(int)
g = np.linalg.norm(b[:, 26:29], axis=1)
gs = np.median(np.lib.stride_tricks.sliding_window_view(np.pad(g, 50, mode='edge'), 101), axis=1)

print('时长 %.2f s' % t[-1])
print('\n【1】静止段的 XY 漂移速率（每段线性拟合 度/秒）')
still = gs < 5.0
idx = np.flatnonzero(np.diff(still.astype(np.int8)) != 0)
segs = []
s0 = idx[0] if still[0] else idx[1]
cur = s0
for i in idx:
    if still[i] != still[cur]:
        if still[cur]:
            segs.append((cur, i))
        cur = i
if still[cur]:
    segs.append((cur, len(b)-1))
print('  静态段数 %d' % len(segs))
for s, e in segs[:8]:
    if e - s < 200:
        continue
    tt = t[s:e] - t[s]
    k_e = np.polyfit(tt, roll_e[s:e], 1)[0], np.polyfit(tt, pit_e[s:e], 1)[0]
    k_l = np.polyfit(tt, roll_l[s:e], 1)[0], np.polyfit(tt, pit_l[s:e], 1)[0]
    k_a = np.polyfit(tt, roll_a[s:e], 1)[0], np.polyfit(tt, pit_a[s:e], 1)[0]
    print('  %5.1f-%5.1fs (%4.1fs)  EKF %+6.3f/%+6.3f  旧链 %+6.3f/%+6.3f  加计 %+6.3f/%+6.3f 度/秒'
          % (t[s], t[e-1], t[e-1]-t[s], k_e[0], k_e[1], k_l[0], k_l[1], k_a[0], k_a[1]))

print('\n【2】静止段 ba / bg 是否在长（首尾差，m/s^2 与 dps）')
for s, e in segs[:8]:
    if e - s < 200:
        continue
    print('  %5.1f-%5.1fs  ba 差 %+7.4f %+7.4f %+7.4f   bg 差 %+7.4f %+7.4f %+7.4f'
          % (t[s], t[e-1], *(ba[e-1] - ba[s]), *(bg[e-1] - bg[s])))

print('\n【3】重力校正门 tilt(0x20) 开启率 vs 运动强度')
for lo, hi, nm in [(0, 5, '静止'), (5, 50, '微动'), (50, 200, '慢转'), (200, 1e9, '快转')]:
    m = (g >= lo) & (g < hi)
    if m.sum() > 100:
        print('  %-4s %5.1f%% 帧  tilt门 %5.1f%%  zupt %5.1f%%  sigma_tilt p50 %6.2f  |a|-1 p50 %+.4f'
              % (nm, 100.0*m.mean(), 100.0*((gb[m] & 0x20) != 0).mean(),
                 100.0*((gb[m] & 0x10) != 0).mean(),
                 np.median(b[m, 107]) if b.shape[1] > 107 else -1, np.median(nrm[m] - 1.0)))

print('\n【4】静止段三方倾角对比（p50，度）')
m = gs < 5.0
print('  roll : EKF %+7.3f  旧链 %+7.3f  加计 %+7.3f' % (np.median(roll_e[m]), np.median(roll_l[m]), np.median(roll_a[m])))
print('  pitch: EKF %+7.3f  旧链 %+7.3f  加计 %+7.3f' % (np.median(pit_e[m]), np.median(pit_l[m]), np.median(pit_a[m])))
d_e = np.hypot(roll_e - roll_a, pit_e - pit_a)
d_l = np.hypot(roll_l - roll_a, pit_l - pit_a)
print('  |EKF 倾角 - 加计倾角| p50 %6.3f 度   p90 %6.3f' % (np.median(d_e[m]), np.percentile(d_e[m], 90)))
print('  |旧链倾角 - 加计倾角| p50 %6.3f 度   p90 %6.3f' % (np.median(d_l[m]), np.percentile(d_l[m], 90)))
