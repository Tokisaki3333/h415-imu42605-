# -*- coding: utf-8 -*-
"""找 ekf_q 的"闪现"（单帧大跳变）并把当时开着的门列出来 —— 一次定位是哪道观测在跳。
判据：
  d 角 = 相邻帧四元数之间的旋转角（度）。ekf_q 只在 EKF 周期边界更新，所以正常时
  每个周期的跳变应当与 legacy（每帧陀螺积分）同量级；远大于它的帧就是"闪现"。
"""
import glob
import os
import re
import sys

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))

cands = sorted(glob.glob(r'R:\*.bin'), key=os.path.getmtime)[-3:]
print('R 盘上最近的文件:')
for f in cands:
    print('   %-28s %10d B  %s' % (os.path.basename(f), os.path.getsize(f),
                                   __import__('time').ctime(os.path.getmtime(f))))
P = cands[-1]
NIS0 = NCH - 5
a = np.fromfile(P, dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
print('\n分析 %s   fw_tag %.0f  NCH=%d  %d 帧 %.2f s'
      % (os.path.basename(P), a[0, 76], NCH, N, t[-1]))


def dang(q):
    """相邻帧四元数之间的旋转角（度）"""
    d = np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1)
    return np.degrees(2.0 * np.arccos(d))


dE = dang(a[:, 89:93])      # EKF
dL = dang(a[:, 0:4])        # 旧链
gb = a[:, 103].astype(np.int32)
print('每帧姿态变化角（度）:')
print('  EKF   p50 %.4f  p99 %.4f  p99.9 %.4f  max %.4f'
      % (np.median(dE), np.percentile(dE, 99), np.percentile(dE, 99.9), dE.max()))
print('  旧链  p50 %.4f  p99 %.4f  p99.9 %.4f  max %.4f'
      % (np.median(dL), np.percentile(dL, 99), np.percentile(dL, 99.9), dL.max()))
thr = max(np.percentile(dL, 99.99) * 5.0, 0.05)
big = np.where(dE > thr)[0]
print('  EKF 超过 %.4f 度的帧: %d 个 (%.4f%%)' % (thr, len(big), 100.0*len(big)/N))
BIT = [(0x01, 'gpsP'), (0x02, 'gpsA'), (0x04, 'baro'), (0x08, 'gpsV'), (0x10, 'zupt'),
       (0x20, 'tilt'), (0x40, 'mag'), (0x100, 'step'), (0x200, 'chi2'), (0x800, 'sat'),
       (0x1000, 'forced')]
print('\n最严重的 15 次跳变：')
print('   帧号     t      EKF跳角  旧链跳角   gate_bits  开着的门')
for i in sorted(big, key=lambda k: -dE[k])[:15]:
    j = i + 1
    gates = ' '.join(n for m, n in BIT if gb[j] & m)
    print('  %7d %7.3f  %8.4f %8.4f   0x%04X   %s' % (i, t[j], dE[i], dL[i], gb[j], gates))
print('\n各门开启率: ' + '  '.join('%s %.1f%%' % (n, 100*np.mean((gb & m) != 0))
                                   for m, n in BIT[:9]))
print('|ekf_q| min %.6f max %.6f   sigma_yaw p50 %.3f max %.3f'
      % (np.linalg.norm(a[:, 89:93], axis=1).min(), np.linalg.norm(a[:, 89:93], axis=1).max(),
         np.median(a[:, 104]), a[:, 104].max()))
