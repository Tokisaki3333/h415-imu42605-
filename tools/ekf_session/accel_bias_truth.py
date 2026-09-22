# -*- coding: utf-8 -*-
r"""判定哪一版加速度计零偏是"真值"：静止时应满足 |a| = 1.0000 g、且三轴零偏使重力方向自洽。

    A 版（运行时跟踪，frame 的 accel_g）: s_accel_bias_lsb = accel_bias_g * lsb_per_g
    B 版（编译期标定，M6 用）           : V5F_ACCEL_BIAS_LSB_X/Y/Z = -10.10 / -15.42 / 43.72
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.join('tools', 'calib'))
from cols_162 import load_frames, CH_162 as C

LSBG = np.array([2028.48, 2040.78, 2016.07])
B0 = np.array([-10.10, -15.42, 43.72])

for f in sys.argv[1:] or [r'R:\imu_20260922_211657.bin']:
    fr = load_frames(f)
    a = fr[0] if isinstance(fr, tuple) else fr
    lsb = np.asarray(a[:, C['accel_lsb0']:C['accel_lsb0'] + 3], float)
    bg = np.asarray(a[:, C['accel_bias_g0']:C['accel_bias_g0'] + 3], float)
    A = bg[0] * LSBG
    ga = (lsb - A) / LSBG          # 运行时零偏版（= 发布 accel_g 的算法）
    gb = (lsb - B0) / LSBG         # 编译期常数版（= M6 raw_f_mps2 的算法）
    na = np.linalg.norm(ga, axis=1)
    nb = np.linalg.norm(gb, axis=1)
    print('%s  帧 %d' % (os.path.basename(f), len(a)))
    print('  A 运行时零偏 LSB (%+.2f,%+.2f,%+.2f)  |a| 均值 %.5f g  (偏差 %+.1f mg)'
          % (A[0], A[1], A[2], na.mean(), (na.mean() - 1) * 1000))
    print('  B 编译期零偏 LSB (%+.2f,%+.2f,%+.2f)  |a| 均值 %.5f g  (偏差 %+.1f mg)'
          % (B0[0], B0[1], B0[2], nb.mean(), (nb.mean() - 1) * 1000))
