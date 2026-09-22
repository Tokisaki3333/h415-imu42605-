# -*- coding: utf-8 -*-
r"""地磁失效门控的阈值定型：用新录像把"陀螺一致性门"的门限/窗口/回差算出来。

判据: 窗口 T 内 |Δpsi_mag - Δpsi_gyro| > THR  ->  地磁失效(外部干扰)
  - 干净地板: 只在"磁场自己没动"的窗口里量(|Δpsi_mag| < 1 deg) => 那里测到的就是噪声/陀螺误差地板
  - 触发占空比 / 最长连续触发时长(定回差保持时间)
用法: python tools/ekf_session/maggate_thr.py R:\imu_20260922_195513.bin [...]
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


def main():
    for path in sys.argv[1:]:
        a, _ = C.load_frames(path)
        N = len(a)
        tk = a[:, CH['tick_tk']].astype(np.float64)
        d = np.diff(tk); d[d < 0] += 16777216.0
        t = np.concatenate([[0.0], np.cumsum(d)]) * 1e-6
        fps = (N - 1) / t[-1]
        gz = a[:, CH['gyro_dps0'] + 2]
        psit = unwr(a[:, CH['psi_true_deg']])
        gi = np.concatenate([[0.0], np.cumsum((gz[1:] + gz[:-1]) * 0.5 * d * 1e-6)])
        print('=' * 100)
        print('%s  %.1f s  %.1f Hz  VER=%d' % (os.path.basename(path), t[-1], fps,
                                               int(np.median(a[:, CH['fw_tag']])) >> 16))
        for T in (0.5, 1.0, 2.0):
            k = max(1, int(round(T * fps)))
            dm = wrap(psit[k:] - psit[:-k])
            dg = gi[k:] - gi[:-k]
            e = np.abs(wrap(dm - dg))
            quiet = np.abs(dm) < 1.0                     # 磁场自己没动的窗口 = 干净地板
            print('  T=%.1fs (%3d 帧)  地板(quiet %4.1f%%): p50 %5.2f p90 %5.2f p99 %5.2f max %6.2f'
                  % (T, k, 100 * quiet.mean(), np.percentile(e[quiet], 50),
                     np.percentile(e[quiet], 90), np.percentile(e[quiet], 99),
                     e[quiet].max() if quiet.any() else -1))
            trig = e > 5.0
            # 最长连续触发（帧）
            if trig.any():
                idx = np.where(np.diff(np.concatenate([[0], trig.view(np.int8), [0]])) != 0)[0]
                runs = (idx.reshape(-1, 2)[:, 1] - idx.reshape(-1, 2)[:, 0]) * (k / fps)
                runs = runs[idx.reshape(-1, 2)[:, 1] > idx.reshape(-1, 2)[:, 0]]
                print('         THR=5deg 触发占空比 %5.1f%%  最长连续触发 %.2f s  段数 %d'
                      % (100 * trig.mean(), runs.max() if len(runs) else 0.0, len(runs)))
            for thr in (3.0, 5.0, 8.0, 10.0, 15.0):
                print('         THR=%4.1fdeg -> 触发 %5.1f%%   (地板 p99 %.2f, 干扰 p90 %.2f)'
                      % (thr, 100 * np.mean(e > thr), np.percentile(e[quiet], 99),
                         np.percentile(e[~quiet], 90) if (~quiet).any() else -1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
