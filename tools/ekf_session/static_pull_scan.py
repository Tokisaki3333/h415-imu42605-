# -*- coding: utf-8 -*-
r"""扫描所有录像：找"运动 -> 静止"切换，量静止后偏航残差（= mag_yawpre - ekf_yaw）的衰减。

用法: python tools/ekf_session/static_pull_scan.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
DT_MAG = 0.00526
DEG2RAD = math.pi / 180.0


def yaw_of(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def scan(path):
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    w = np.linalg.norm(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], axis=1)
    yaw_e = yaw_of(a[:, C['ekf_q0']:C['ekf_q0'] + 4])
    yaw_m = np.asarray(a[:, C['ekf_mag_yawpre']], dtype=float) * DEG2RAD
    p88 = np.asarray(a[:, C['ekf_p_yy']], dtype=float)
    r_deg = np.asarray(a[:, C['ekf_mag_r_deg']], dtype=float)
    gate = np.asarray(a[:, C['ekf_mag_gate']], dtype=float)
    used = np.asarray(a[:, C['ekf_mag_used']], dtype=float)
    res = np.degrees(wrap_pi(yaw_m - yaw_e))

    stat = w < 5.0
    segs = []
    i = 0
    while i < len(stat):
        if stat[i]:
            j = i
            while j < len(stat) and stat[j]:
                j += 1
            if j - i >= 667:            # >= 2 s
                segs.append((i, j))
            i = j
        else:
            i += 1
    print('== %-28s VER=%3d %6.1f s  |w|p50 %6.1f max %6.1f  p_yy p50 %.3g  R p50 %.1f'
          % (os.path.basename(path), ver, len(w) / 335.0, np.percentile(w, 50), w.max(),
             np.median(p88), np.median(r_deg)))
    for (i, j) in segs:
        d = j - i
        r = res[i:j]
        # 残差趋势：线性拟合斜率（deg/s），以及首尾中位差
        t = np.arange(d) / 335.0
        sl = np.polyfit(t, r, 1)[0] if d > 10 else float('nan')
        print('   静止[%5d,%5d) %5.1f s | res 首 %7.2f 尾 %7.2f deg  斜率 %+6.3f deg/s '
              '| p_yy %.3g  R %.1f  gate %3.0f%% used %3.0f%%'
              % (i, j, d / 335.0, np.median(r[:167]), np.median(r[-167:]), sl,
                 np.median(p88[i:j]), np.median(r_deg[i:j]),
                 100.0 * gate[i:j].mean(), 100.0 * used[i:j].mean()))


if __name__ == '__main__':
    args = sys.argv[1:]
    files = args if args else sorted(glob.glob(r'R:\imu_*.bin'))
    for f in files:
        try:
            scan(f)
        except Exception as e:
            print('!! %s: %s' % (os.path.basename(f), e))
