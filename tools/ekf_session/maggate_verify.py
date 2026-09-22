# -*- coding: utf-8 -*-
r"""地磁失效门控的定型与验证（速率版，抗"来回摆"干扰）：

判据（每帧）：mag 航向速率 vs 陀螺偏航速率，在窗口 T 上取平均
    rate_err = |Δpsi_mag - Δpsi_gyro| / T        [dps]
    rate_err > THR  ->  置"干扰"标志，并保持 HOLD 秒（跨过摆动的过零点）
用法:
  python tools/ekf_session/maggate_verify.py --dist R:\imu_20260922_195513.bin R:\imu_20260922_195551.bin
  python tools/ekf_session/maggate_verify.py --clean R:\imu_20260922_121205.bin R:\imu_20260922_125937.bin
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calib'))
import cols_162 as C
CH = C.CH_162


def unwr(a):
    return np.degrees(np.unwrap(np.radians(a)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def detector(a, T, THR, HOLD):
    N = len(a)
    tk = a[:, CH['tick_tk']].astype(np.float64)
    d = np.diff(tk); d[d < 0] += 16777216.0
    t = np.concatenate([[0.0], np.cumsum(d)]) * 1e-6
    fps = (N - 1) / t[-1]
    k = max(1, int(round(T * fps)))
    gz = a[:, CH['gyro_dps0'] + 2]
    psit = unwr(a[:, CH['psi_true_deg']])
    gi = np.concatenate([[0.0], np.cumsum((gz[1:] + gz[:-1]) * 0.5 * d * 1e-6)])
    e = np.full(N, np.nan)
    dm = wrap(psit[k:] - psit[:-k])
    dg = gi[k:] - gi[:-k]
    e[k:] = np.abs(wrap(dm - dg)) / T          # dps
    trig = np.zeros(N, bool)
    hold_n = int(round(HOLD * fps))
    h = 0
    for i in range(N):
        if not np.isnan(e[i]) and e[i] > THR:
            h = hold_n
        if h > 0:
            trig[i] = True; h -= 1
    return t, e, trig, fps


def main():
    mode = sys.argv[1]
    print('### %s' % mode)
    for path in sys.argv[2:]:
        a, _ = C.load_frames(path)
        best = None
        for T, THR, HOLD in ((0.25, 20.0, 2.0), (0.25, 10.0, 2.0), (0.5, 10.0, 2.0),
                             (0.25, 30.0, 1.0), (0.25, 20.0, 5.0)):
            t, e, trig, fps = detector(a, T, THR, HOLD)
            duty = 100 * trig.mean()
            print('  %-26s T=%.2fs THR=%4.1fdps HOLD=%.1fs -> 门关占空比 %5.1f%%'
                  % (os.path.basename(path), T, THR, HOLD, duty))
        t, e, trig, fps = detector(a, 0.25, 20.0, 2.0)
        good = ~np.isnan(e)
        print('     rate_err dps: p50 %.2f p90 %.2f p99 %.2f max %.1f  (干净~0.2)'
              % (np.percentile(e[good], 50), np.percentile(e[good], 90),
                 np.percentile(e[good], 99), np.nanmax(e)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
