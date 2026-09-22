# -*- coding: utf-8 -*-
r"""地磁失效门控参数定型：给候选 (THR, HOLD) 算三件事
  ① 干净录像上的误触发占空比（要求 ~0）
  ② 受扰录像上的"门开"最长连续时段（这才是地磁真能拽偏航的时间，要求 < ~1 s）
  ③ 干净录像的 |mag_norm-1| 上界（定幅度门）
用法: python tools/ekf_session/maggate_pick.py
"""
import os
import numpy as np
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calib'))
import cols_162 as C
CH = C.CH_162

DIST = [r'R:\imu_20260922_195513.bin', r'R:\imu_20260922_195551.bin']
CLEAN = [r'R:\imu_20260922_121205.bin', r'R:\imu_20260922_125937.bin',
         r'R:\imu_20260922_120949.bin', r'R:\imu_20260922_124448.bin',
         r'R:\imu_20260922_124607.bin']


def unwr(a):
    return np.degrees(np.unwrap(np.radians(a)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def load(path):
    a, _ = C.load_frames(path)
    tk = a[:, CH['tick_tk']].astype(np.float64)
    d = np.diff(tk); d[d < 0] += 16777216.0
    t = np.concatenate([[0.0], np.cumsum(d)]) * 1e-6
    fps = (len(a) - 1) / t[-1]
    return a, t, fps


def runs_of(mask, fps):
    """连续 True 段长度(s)"""
    if not mask.any():
        return np.array([0.0])
    idx = np.where(np.diff(np.concatenate([[0], mask.view(np.int8), [0]])) != 0)[0]
    p = idx.reshape(-1, 2)
    return (p[:, 1] - p[:, 0]) / fps


def gate(a, fps, T, THR, HOLD, tiltfix=True):
    N = len(a)
    k = max(1, int(round(T * fps)))
    w = a[:, CH['gyro_dps0']:CH['gyro_dps0'] + 3].astype(np.float64)
    if tiltfix:
        q = a[:, CH['ekf_q0']:CH['ekf_q0'] + 4].astype(np.float64)
        qn = q / np.linalg.norm(q, axis=1, keepdims=True)
        ww, xx, yy, zz = qn.T
        # up 在体系 = R^T z_nav
        up = np.stack([2 * (xx * zz - ww * yy), 2 * (yy * zz + ww * xx), 1 - 2 * (xx * xx + yy * yy)], 1)
        gz = (w * up).sum(1)                       # 绕世界竖直的角速率 dps
    else:
        gz = w[:, 2]
    psit = unwr(a[:, CH['psi_true_deg']])
    dt = 1.0 / fps
    gi = np.cumsum(gz) * dt
    e = np.full(N, np.nan)
    e[k:] = np.abs(wrap((psit[k:] - psit[:-k]) - (gi[k:] - gi[:-k]))) / T
    bad = (e > THR)          # 只看速率一致性（幅度门是原有机制，单独看）
    hold = int(round(HOLD * fps))
    h = 0
    trig = np.zeros(N, bool)
    for i in range(N):
        if bad[i]:
            h = hold
        if h > 0:
            trig[i] = True
            h -= 1
    return trig, e


def main():
    print('== 干净录像：误触发占空比 / |mag_norm-1| 上界 ==')
    clean = []
    for p in CLEAN:
        if not os.path.exists(p):
            continue
        a, t, fps = load(p)
        dev = np.abs(a[:, CH['mag_norm']] - 1.0)
        for THR, HOLD in ((15.0, 5.0), (20.0, 5.0), (15.0, 10.0)):
            trig, e = gate(a, fps, 0.25, THR, HOLD)
            print('  %-26s %5.1fs  |mn-1| p99 %.4f max %.4f | THR=%2.0f HOLD=%2.0f -> 误触发 %5.2f%%'
                  % (os.path.basename(p), t[-1], np.percentile(dev, 99), dev.max(),
                     THR, HOLD, 100 * trig.mean()))
        clean.append((os.path.basename(p), 100 * np.percentile(dev, 99)))
    print('  干净 |mag_norm-1| p99 汇总: %s' % ', '.join('%.3f' % c[1] for c in clean))
    print()
    print('== 受扰录像：门开的**最长连续时段**(地磁能拽偏航的时间) ==')
    for p in DIST:
        a, t, fps = load(p)
        for THR, HOLD in ((15.0, 5.0), (20.0, 5.0), (15.0, 10.0), (20.0, 10.0)):
            trig, e = gate(a, fps, 0.25, THR, HOLD)
            r = runs_of(~trig, fps)
            rate = np.full(len(a), np.nan)
            psit = unwr(a[:, CH['psi_true_deg']]); k = max(1, int(round(0.25 * fps)))
            rate[k:] = np.abs(wrap(psit[k:] - psit[:-k])) / 0.25
            interfere = np.nan_to_num(rate) > 20.0
            on_in = 100 * (1 - trig[interfere].mean()) if interfere.any() else -1
            print('  %-26s %5.1fs  THR=%2.0f HOLD=%2.0f -> 门关 %5.1f%% | 干扰帧占比 %4.1f%% | **干扰期内门开 %5.2f%%** | 门开最长 %5.2fs'
                  % (os.path.basename(p), t[-1], THR, HOLD, 100 * trig.mean(),
                     100 * interfere.mean(), on_in, r.max()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
