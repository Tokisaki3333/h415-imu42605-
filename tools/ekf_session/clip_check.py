# -*- coding: utf-8 -*-
r"""门关段里陀螺**削顶**（|gyro_lsb| >= 32700 = 2000 dps 满量程）的占比 vs 首次修正前残差。"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi
CLIP = 32700


def scan(path, min_closed=0.5):
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
    gl = np.asarray(a[:, C['gyro_lsb0']:C['gyro_lsb0'] + 3], float)
    w = np.linalg.norm(g, axis=1)
    acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
    an = np.linalg.norm(acc, axis=1)
    up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    op = gt > 0.5
    out = []
    i = 0
    while i < n:
        if not op[i] and i > 0:
            j = i
            while j < n and not op[j]:
                j += 1
            if dt[i:j].sum() >= min_closed and j < n:
                tg = j
                clipped = np.any(np.abs(gl[i:tg]) >= CLIP, axis=1)
                e = (acc[tg] / max(an[tg], 1e-6) - up[tg]) * RAD2DEG
                th = np.array([np.sum(g[i:tg, ax] * dt[i:tg]) for ax in range(3)])
                out.append(dict(tg=tg, T=dt[i:tg].sum(), th=th, e=e,
                                clip=100.0 * float(clipped.mean()),
                                wmax=w[i:tg].max(),
                                over2000=100.0 * float((w[i:tg] > 2000.0).mean()),
                                amax=float(an[i:tg].max())))
            i = j
        else:
            i += 1
    print('== %-24s VER=%3d' % (os.path.basename(path), ver))
    print('   %-9s %-6s %-7s %-8s %-12s %-26s %-9s %-9s'
          % ('tg', 'T s', '|w|max', '|a|max', '削顶占比%', '净转角 x/y/z', 'e_x', 'e_y'))
    for s in out:
        print('   %-9d %-6.1f %-7.0f %-8.2f %-12.1f (%7.0f,%7.0f,%7.0f) %-9.2f %-9.2f'
              % (s['tg'], s['T'], s['wmax'], s['amax'], s['clip'], s['th'][0], s['th'][1],
                 s['th'][2], s['e'][0], s['e'][1]))
    return out


if __name__ == '__main__':
    files = sys.argv[1:] or [r'R:\imu_20260922_205255.bin', r'R:\imu_20260922_201451.bin']
    for f in files:
        try:
            scan(f)
        except Exception as ex:
            print('!! %s: %s' % (os.path.basename(f), ex))
        print()
