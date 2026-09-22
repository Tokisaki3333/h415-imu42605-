# -*- coding: utf-8 -*-
r"""倾角静差漂移溯源：EKF 的 ba 状态 vs 运行时 accel_bias_g（两套零偏是否互相打架）。

M6 用 raw_f_mps2(=编译期常数零偏) - s_x[IX_BA]；而 col32 的 accel_g 用运行时跟踪的
s_accel_bias_lsb。两者是**独立估计器**，若互相漂开，倾角静差就会随时间增长。
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260923_060100.bin'
fr = load_frames(path)
a = fr[0] if isinstance(fr, tuple) else fr
n = len(a)
dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
t = np.cumsum(dt)
ba = np.asarray(a[:, C['ekf_ba0']:C['ekf_ba0'] + 3], float)
bb = np.asarray(a[:, C['accel_bias_g0']:C['accel_bias_g0'] + 3], float)
up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
an = np.linalg.norm(acc, axis=1)
res = np.degrees(np.linalg.norm(acc / np.maximum(an, 1e-6)[:, None] - up, axis=1))
d = ba - bb              # 两套零偏之差（g）；对倾角的影响 ≈ |d_perp| (rad)
print('t(min)  ekf.ba (g)                    accel_bias_g（运行时）           差 (g)'
      '                    倾角静差 deg')
for s in range(0, int(t[-1]) + 1, 60):
    k = min(np.searchsorted(t, s), n - 1)
    dd = ba[k] - bb[k]
    print('%5d   (%+.4f,%+.4f,%+.4f)   (%+.4f,%+.4f,%+.4f)   (%+.4f,%+.4f,%+.4f)  %.2f'
          % (s // 60, ba[k, 0], ba[k, 1], ba[k, 2], bb[k, 0], bb[k, 1], bb[k, 2],
             dd[0], dd[1], dd[2], res[k]))
k = n - 1
print()
print('|ba - 运行时零偏| 首/末 = %.4f / %.4f g  -> 表观倾角 %.2f / %.2f deg'
      % (np.linalg.norm(ba[0] - bb[0]), np.linalg.norm(ba[k] - bb[k]),
         np.degrees(np.linalg.norm(ba[0] - bb[0])), np.degrees(np.linalg.norm(ba[k] - bb[k]))))
print('倾角静差 首/末 = %.2f / %.2f deg（增长 %.2f deg / %.0f min）'
      % (res[0], res[k], res[k] - res[0], t[k] / 60))
