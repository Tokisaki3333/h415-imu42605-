# -*- coding: utf-8 -*-
"""决定性测量：磁力计的**倾角 dip** 准不准。
方法：静止段用 EKF 姿态把机体磁场转到导航系，B_n = R(q)·mag_f，
      倾角 I = asin(-B_n[2])。导航系已用磁力计对齐（所以方位角 D 被强制等于 -7.53），
      但**倾角完全不被强制** —— 它由"姿态倾角的准确度 + 磁力计 z 轴标定"共同决定。
      静止段姿态倾角误差只有 0.14~0.17 度，所以量出来的 I 基本就是磁力计的 dip 误差。
对照 WMM 本地：I = 53.74 度，D = -7.53 度。"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
print('fw_tag %.0f  时长 %.1f s' % (a[0, 76], t[-1]))


def q2R(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z);   R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y);   R[:, 2, 1] = 2*(y*z+w*x);  R[:, 2, 2] = 1-2*(x*x+y*y)
    return R


def bn(R, v): return np.einsum('nij,nj->ni', R, v)


magf = a[:, 42:45]
# 静止判据：陀螺平滑 <2 dps 且 |a_lin|<0.05g 且 e_ac<0.2
k = max(int(0.2/dt.mean()), 1)
w3 = np.linalg.norm(a[:, 26:29], axis=1)
aln = np.linalg.norm(a[:, 10:13], axis=1)
still = ((np.convolve(w3, np.ones(k)/k, mode='same') < 2.0)
         & (np.convolve(aln, np.ones(k)/k, mode='same') < 0.05)
         & (a[:, 81] < 0.2))
magq = a[:, 50] > 0.5          # mag.trust（模长 + 倾斜都可信）

print()
print('=== 用 EKF 姿态量磁倾角（静止且 mag.trust 的帧）===')
for nm, q in (('EKF', a[:, 89:93]), ('旧链', a[:, 0:4])):
    R = q2R(q)
    B = bn(R, magf)
    B = B / np.maximum(np.linalg.norm(B, axis=1, keepdims=True), 1e-9)
    I = np.degrees(np.arcsin(np.clip(-B[:, 2], -1, 1)))
    D = np.degrees(np.arctan2(B[:, 0], B[:, 1]))
    m = still & magq
    if m.sum() < 100:
        print('  %s: 可用帧只有 %d，跳过' % (nm, m.sum()))
        continue
    print('  %-4s 静止帧 %6d   I(倾角) 均值 %7.3f  std %5.3f 度   D(方位) 均值 %7.3f std %5.3f'
          % (nm, m.sum(), I[m].mean(), I[m].std(), D[m].mean(), D[m].std()))
print('  WMM 本地真值: I = +53.74 度,  D = -7.53 度')

print()
print('=== 逐静置段（看是否随姿态变化 —— 若随姿态变就是软铁/正交性误差）===')
idx = np.where(np.diff(still.astype(np.int8)) != 0)[0]
segs = [(idx[i], idx[i+1]) for i in range(len(idx)-1)
        if still[min(idx[i]+1, N-1)] and idx[i+1]-idx[i] > int(0.4/dt.mean())]
R = q2R(a[:, 89:93])
B = bn(R, magf)
B = B / np.maximum(np.linalg.norm(B, axis=1, keepdims=True), 1e-9)
I = np.degrees(np.arcsin(np.clip(-B[:, 2], -1, 1)))
if not segs:
    print('  没有 >0.4 s 的静置段')
for s0, s1 in segs:
    m = np.zeros(N, bool); m[s0:s1] = True
    m &= magq
    if m.sum() < 50:
        continue
    # 同时给出该段的姿态（俯仰/横滚）看是否相关
    qq = a[s0:s1, 89:93]
    print('  t=%6.2f~%6.2f  I 均值 %7.3f  std %5.3f 度   (mag_trust %.0f%%)'
          % (t[s0], t[s1-1], I[s0:s1][magq[s0:s1]].mean() if magq[s0:s1].any() else float('nan'),
             I[s0:s1][magq[s0:s1]].std() if magq[s0:s1].any() else float('nan'),
             100*magq[s0:s1].mean()))
