# -*- coding: utf-8 -*-
r"""静止窗的陀螺质量：零偏（随时间的误差）、ARW（噪声）、以及被牵引掉的量。

用法: python tools/ekf_session/quality_static.py [R:\imu_xxx.bin]
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260922_205255.bin'
fr = load_frames(path)
a = fr[0] if isinstance(fr, tuple) else fr
n = len(a)
dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
t = np.cumsum(dt)
g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
w = np.linalg.norm(g, axis=1)
acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
an = np.linalg.norm(acc, axis=1)
bias = np.asarray(a[:, C['gyro_bias_dps0']:C['gyro_bias_dps0'] + 3], float)
calm = (w < 5.0) & (np.abs(an - 1.0) < 0.03)

wins = []
i = 0
while i < n:
    if calm[i]:
        j = i
        while j < n and calm[j]:
            j += 1
        if j - i >= 167:
            wins.append((i, j))
        i = j
    else:
        i += 1

print('录像 %s  静止窗 %d 个' % (os.path.basename(path), len(wins)))
print('%-22s %-8s %-30s %-30s' % ('窗(时间s)', '时长s', '原始陀螺均值 dps', 'bias 状态 dps'))
for (i, j) in wins:
    mg = np.mean(g[i:j], axis=0)
    mb = np.mean(bias[i:j], axis=0)
    print('[%6.2f,%6.2f) %-8.2f (%+7.4f,%+7.4f,%+7.4f) (%+7.4f,%+7.4f,%+7.4f)'
          % (t[i], t[j - 1], dt[i:j].sum(), mg[0], mg[1], mg[2], mb[0], mb[1], mb[2]))
print()

# 长时间静止段的 ARW：用 Allan 式短时方差（1 s 平均）
if wins:
    i, j = max(wins, key=lambda s: s[1] - s[0])
    seg = g[i:j]
    dtm = np.median(dt[i:j])
    n1 = int(round(1.0 / dtm))
    m = (len(seg) // n1) * n1
    blk = seg[:m].reshape(-1, n1)
    avg = blk.mean(axis=1)
    if avg.ndim == 1:
        avg = avg.reshape(-1, 3)
    sd = np.std(avg, axis=0)                       # dps，1 s 平均后的标准差
    arw = sd * 60.0                                # deg/rt-h（ARW ≈ σ(1s)·60）
    print('最长静止窗 %.1f s：1 s 平均后 sigma = (%.4f, %.4f, %.4f) dps'
          % (dt[i:j].sum(), sd[0], sd[1], sd[2]))
    print('  -> ARW ≈ (%.2f, %.2f, %.2f) deg/rt-h   （设计口径 0.58 deg/rt-h；'
          '注意含振动/量化' % (arw[0], arw[1], arw[2]))
    print('  -> 静止零偏(均值) 折算 deg/h = (%+.0f, %+.0f, %+.0f)'
          % tuple(np.mean(seg, axis=0) * 3600.0))
    print('  -> bias 状态末值 deg/h = (%+.0f, %+.0f, %+.0f)'
          % tuple(np.mean(bias[i:j], axis=0) * 3600.0))
