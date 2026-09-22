# -*- coding: utf-8 -*-
r"""检查每个"运动->静止"段的平移过程是否干净（205255 #2 是不是孤立野值）。

判据：tg 之后若门**反复开合**、且残差在几秒内大幅来回（42 -> 13 -> 61 -> 58 -> 35 -> 9 deg），
说明加速度计虽然模长 ≈1 g、但**方向仍在被搬动主导** —— 那 r0 就不是"积攒的倾角误差"。
干净段应当：门持续打开 >= 0.5 s，且 |a| 在 ±1.5% 内，残差单调衰减。

用法: python tools/ekf_session/segment_quality_check.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def run(path, min_closed=0.5):
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
    w = np.linalg.norm(g, axis=1)
    acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
    an = np.linalg.norm(acc, axis=1)
    up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    op = gt > 0.5
    res = np.degrees(np.linalg.norm(acc / np.maximum(an, 1e-6)[:, None] - up, axis=1))
    print('== %s' % os.path.basename(path))
    print('   %-8s %-6s %-7s %-9s %-9s %-10s %-9s %-9s %-9s'
          % ('tg', 'T s', '|w|max', '门切换数', '最长连续开', '|a|范围', 'res@tg', 'res最小',
             'res最大'))
    i = 0
    rows = []
    while i < n:
        if not op[i] and i > 0:
            j = i
            while j < n and not op[j]:
                j += 1
            if dt[i:j].sum() >= min_closed and j < n:
                tg = j
                p1 = min(n, tg + int(3.0 / max(np.median(dt), 1e-4)))
                seg = op[tg:p1]
                sw = int(np.sum(seg[1:] != seg[:-1]))
                # 最长连续开
                best = cur = 0; b0 = b1 = 0
                for q, v in enumerate(seg):
                    if v:
                        cur += 1
                        if cur > best:
                            best = cur; b1 = q + 1; b0 = b1 - cur
                    else:
                        cur = 0
                sa = an[tg + b0:tg + b1]
                s_amin, s_amax = (float(sa.min()), float(sa.max())) if len(sa) else (9, 9)
                amin, amax = float(np.min(an[tg:p1])), float(np.max(an[tg:p1]))
                r = res[tg:p1]
                clean = (best >= 150) and (s_amax - s_amin) < 0.05
                rows.append((tg, dt[i:tg].sum(), w[i:tg].max(), sw, best, amin, amax,
                             res[tg], float(np.min(r)), float(np.max(r)), clean))
            i = j if j > i else i + 1
        else:
            i += 1
    for r in rows:
        print('   %-8d %-6.1f %-7.0f %-9d %-9d %-10s %-9.2f %-9.2f %-9.2f  %s'
              % (r[0], r[1], r[2], r[3], r[4], '%.3f~%.3f' % (r[5], r[6]), r[7], r[8], r[9],
                 'CLEAN' if r[10] else '脏(搬动中)'))
    return rows


if __name__ == '__main__':
    for f in (sys.argv[1:] or [r'R:\imu_20260922_205255.bin', r'R:\imu_20260922_210629.bin']):
        try:
            run(f)
        except Exception as ex:
            print('!! %s: %s' % (os.path.basename(f), ex))
        print()
