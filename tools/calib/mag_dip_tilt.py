# -*- coding: utf-8 -*-
"""对指定 A/C 源，量【姿态无关】的 body dip = angle(m_cal, g_hat)-90，按倾角分箱。
用法: python _who7.py <tune.h> <cap1.bin> [cap2.bin ...]
"""
import os
import sys

import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C          # noqa: E402
import mag360_cal as M        # noqa: E402

c = C.CH_162
tune = sys.argv[1]
for path in sys.argv[2:]:
    A, Cc = M.read_current_AC(tune)
    fr, _ = C.load_frames(path)
    g = np.asarray(fr[:, c['accel_g0']:c['accel_g0'] + 3], float)
    gn = np.linalg.norm(g, axis=1)
    gy = np.linalg.norm(np.radians(np.asarray(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3], float)), axis=1)
    raw = np.asarray(fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3], float)
    m = raw @ A.T + Cc
    mn = np.linalg.norm(m, axis=1)
    ok = (np.abs(gn - 1.0) < 0.03) & (gy < np.radians(60.0))
    u = m[ok] / mn[ok, None]
    gg = g[ok] / gn[ok, None]
    dip = np.degrees(np.arccos(np.clip(np.sum(u * gg, axis=1), -1, 1))) - 90.0
    tilt = np.degrees(np.arccos(np.clip(np.abs(gg[:, 2]), -1, 1)))
    p = np.percentile(dip, [5, 50, 95])
    row = []
    for lo, hi in ((0, 10), (10, 20), (20, 35), (35, 55), (55, 90)):
        s = (tilt >= lo) & (tilt < hi)
        row.append('%d-%d:%s(n%d)' % (lo, hi, ('%+.1f' % np.median(dip[s])) if s.sum() >= 20 else '--', s.sum()))
    print('%-22s %-14s n=%5d  p5/p50/p95 %+6.2f/%+6.2f/%+6.2f | %s'
          % (os.path.basename(tune), os.path.basename(path), ok.sum(), p[0], p[1], p[2], ' '.join(row)))
