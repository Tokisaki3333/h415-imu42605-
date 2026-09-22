# -*- coding: utf-8 -*-
r"""跨录像比对：静止段的原始陀螺均值（零偏）与 ekf.bg（滤波器估计），判断零偏是否稳定。"""
import sys, os, glob, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
files = sys.argv[1:] or sorted(glob.glob(r'R:\imu_2026*.bin'))
print('%-26s %-4s %-8s %-30s %-30s' % ('录像', 'VER', '静止s', '静止原始陀螺均值 dps',
                                       'ekf.bg 均值 dps'))
for f in files:
    try:
        fr = load_frames(f)
        a = fr[0] if isinstance(fr, tuple) else fr
        ver = int(np.median(a[:, C['fw_tag']])) >> 16
        dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
        dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
        g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
        w = np.linalg.norm(g, axis=1)
        acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
        an = np.linalg.norm(acc, axis=1)
        bg = np.asarray(a[:, C['ekf_bg0']:C['ekf_bg0'] + 3], float)
        boot = np.asarray(a[:, C['gyro_bias_dps0']:C['gyro_bias_dps0'] + 3], float)
        calm = (w < 5.0) & (np.abs(an - 1.0) < 0.03)
        if calm.sum() < 500:
            print('%-26s %-4d %-8.1f （静止样本不足）' % (os.path.basename(f), ver, dt[calm].sum()))
            continue
        mg = g[calm].mean(axis=0)
        sb = bg[calm].mean(axis=0)
        print('%-26s %-4d %-8.1f (%+7.4f,%+7.4f,%+7.4f)  (%+7.4f,%+7.4f,%+7.4f)'
              % (os.path.basename(f), ver, dt[calm].sum(), mg[0], mg[1], mg[2],
                 sb[0], sb[1], sb[2]))
        print('%-26s      %-8s 上电 3s 平均 = (%+7.4f,%+7.4f,%+7.4f) dps'
              % ('', '', boot[0, 0], boot[0, 1], boot[0, 2]))
    except Exception as ex:
        print('!! %s: %s' % (os.path.basename(f), ex))
