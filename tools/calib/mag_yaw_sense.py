# -*- coding: utf-8 -*-
"""方位一致性：绕竖直转时，机体系磁场方位角必须反向转。判 corr(d az/dt, -wz) 的符号与斜率。"""
import os
import sys
import traceback

import numpy as np

HERE = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib'
sys.path.insert(0, HERE)
import cols_162 as C
import mag360_cal as M

c = C.CH_162
REPO = r'C:\Users\33\Documents\v2\h415-imu42605-'
CAND = [('现用VER=112', os.path.join(REPO, 'V5F', 'User', 'inc', 'v5f_tune.h')),
        ('VER=110', os.path.join(REPO, 'bak_src', 'V5F', 'User', 'inc', 'v5f_tune.h.bak_v111')),
        ('VER=102台面', os.path.join(REPO, 'bak_src', 'V5F', 'User', 'inc', 'v5f_tune.h.bak_v109'))]

for log in ('imu_20260921_044249.bin', 'imu_20260921_044537.bin', 'imu_20260921_044559.bin'):
    try:
        fr, _ = C.load_frames('R:' + chr(92) + log)
    except Exception:
        traceback.print_exc(); continue
    raw = np.asarray(fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3], dtype=float)
    gy = np.radians(np.asarray(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3], dtype=float))
    dt = float(np.median(np.asarray(fr[:, c['dt_us']], dtype=float))) * 1e-6
    S = 50
    gz = np.array([gy[i:i + S, 2].mean() for i in range(len(fr) - S)])
    sel = np.flatnonzero(np.abs(gz) > np.radians(20.0))
    print('\n%s VER=%s  S=%d窗 取出 %d 个' % (log, C.frame_report(fr).get('ver'), S, len(sel)))
    for tag, path in CAND:
        try:
            A, Cv = M.read_current_AC(path)
            m = raw.dot(np.asarray(A).T) + np.asarray(Cv)
            m = m / np.maximum(np.linalg.norm(m, axis=1, keepdims=True), 1e-12)
            az = np.unwrap(np.arctan2(m[:, 1], m[:, 0]))
            daz = (az[sel + S] - az[sel]) / (S * dt)
            y = -gz[sel]
            k = float(np.dot(daz, y) / max(np.dot(y, y), 1e-12))
            cr = float(np.dot(daz, y) / max(np.linalg.norm(daz) * np.linalg.norm(y), 1e-12))
            print('   %-12s corr %+6.3f  斜率 %+6.3f   (应 ≈ +1.0)' % (tag, cr, k))
        except Exception as ex:
            print('   %-12s !! %s' % (tag, ex))
