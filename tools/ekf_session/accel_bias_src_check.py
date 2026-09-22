# -*- coding: utf-8 -*-
r"""决定性检验：静止时 EKF 的 up 与"哪一版加速度计"一致？

accel_g(col32..34)  = (accel_lsb - s_accel_bias_lsb)/lsb_per_g          [运行时跟踪的零偏]
raw_f_mps2 (M6 用)  = (accel_lsb - V5F_ACCEL_BIAS_LSB_*)/lsb_per_g      [编译期标定常数]

两者之差 = (s_accel_bias_lsb - V5F_ACCEL_BIAS_LSB_*)/lsb_per_g。
若静止残差向量 r = accel_g - up_pred 与该差向量（垂直于重力的分量）方向一致，
则说明 EKF 倾角是"跟着编译期零偏走"，而 col32 那一版才是标定值 -> 3.25 deg 静差来源。
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260922_211657.bin'
fr = load_frames(path)
a = fr[0] if isinstance(fr, tuple) else fr
n = len(a)
acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
lsb = np.asarray(a[:, C['accel_lsb0']:C['accel_lsb0'] + 3], float)
bg = np.asarray(a[:, C['accel_bias_g0']:C['accel_bias_g0'] + 3], float)
an = np.linalg.norm(acc, axis=1)
accn = acc / an[:, None]
up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
# 编译期常数（v5f_proc.h）
B0 = np.array([-10.10, -15.42, 43.72])
LSBG = np.array([2028.48, 2040.78, 2016.07])       # 见 v5f_proc.h（轴序按注释）
# 反推 accel_g 用的 lsb_per_g：由 accel_g 与 (lsb - s_bias_lsb) 的关系不可得，
# 这里直接用运行时零偏 g 值做差（两者都换算到 g）
d = bg[0] - (B0 / LSBG)                             # 两版零偏之差（g），= accel_g - accel_compile
r = accn - up
print('录像 %s  帧 %d' % (os.path.basename(path), n))
print('运行时 accel_bias_g = (%+.5f,%+.5f,%+.5f) g' % tuple(bg[0]))
print('编译期 B0/lsb_per_g = (%+.5f,%+.5f,%+.5f) g' % tuple(B0 / LSBG))
print('两者之差 d          = (%+.5f,%+.5f,%+.5f) g   |d| = %.5f g = %.2f deg'
      % (d[0], d[1], d[2], float(np.linalg.norm(d)), float(np.degrees(np.linalg.norm(d)))))
print()
rm = np.median(r, axis=0)
dperp = d - np.dot(d, accn.mean(axis=0)) * accn.mean(axis=0)
print('残差 r 中位 = (%+.5f,%+.5f,%+.5f)  |r| = %.2f deg' % (rm[0], rm[1], rm[2],
      float(np.degrees(np.linalg.norm(rm)))))
print('d 垂直重力分量 = (%+.5f,%+.5f,%+.5f)  |d_perp| = %.2f deg'
      % (dperp[0], dperp[1], dperp[2], float(np.degrees(np.linalg.norm(dperp)))))
cos = float(np.dot(rm, dperp) / max(np.linalg.norm(rm) * np.linalg.norm(dperp), 1e-12))
print('cos(r, d_perp) = %+.3f   （1 = 完全同向 -> 静差就是这两版零偏之差）' % cos)
print('r - d_perp = (%+.5f,%+.5f,%+.5f) g' % tuple((rm - dperp)))
