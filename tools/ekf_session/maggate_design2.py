# -*- coding: utf-8 -*-
r"""陀螺一致性门的定型（按用户要求：用新录的"极限运动"段放宽门限）。

判据: 窗口 T 内 |Δpsi_mag - Δpsi_gyro|/T > THR  ->  地磁失效，保持 HOLD 秒
陀螺航向增量 = (w·up)*dt（up = R^T z_nav，用 EKF 四元数）；
护栏（真机动时不许误触发）：
  - |w_x| 或 |w_y| > 50 dps -> 本次不判（大倾角自转时 body-z 速率不是航向速率）
  - 陀螺原始 LSB 削顶（|lsb| >= 32700）-> 本次不判
用法: python tools/ekf_session/maggate_design2.py
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calib'))
import cols_162 as C
CH = C.CH_162

EXTREME = r'R:\imu_20260922_201451.bin'
DIST = [r'R:\imu_20260922_195513.bin', r'R:\imu_20260922_195551.bin']
CLEAN = [r'R:\imu_20260922_121205.bin']


def unwr(a):
    return np.degrees(np.unwrap(np.radians(a)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def prep(path):
    a, _ = C.load_frames(path)
    tk = a[:, CH['tick_tk']].astype(np.float64)
    d = np.diff(tk); d[d < 0] += 16777216.0
    dtv = np.concatenate([d, d[-1:]]) * 1e-6
    t = np.concatenate([[0.0], np.cumsum(d)]) * 1e-6
    fps = (len(a) - 1) / t[-1]
    w = a[:, CH['gyro_dps0']:CH['gyro_dps0'] + 3].astype(np.float64)
    q = a[:, CH['ekf_q0']:CH['ekf_q0'] + 4].astype(np.float64)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    ww, xx, yy, zz = q.T
    up = np.stack([2 * (xx * zz - ww * yy), 2 * (yy * zz + ww * xx), 1 - 2 * (xx * xx + yy * yy)], 1)
    gz = (w * up).sum(1)
    gi = np.concatenate([[0.0], np.cumsum((gz[1:] + gz[:-1]) * 0.5 * d * 1e-6)])
    psi = unwr(a[:, CH['psi_true_deg']])
    lsb = np.abs(a[:, CH['gyro_lsb0']:CH['gyro_lsb0'] + 3]).max(1)
    return dict(a=a, t=t, dtv=dtv, fps=fps, gz=gz, gi=gi, psi=psi, w=w, lsb=lsb)


def trig_of(D, T, THR, HOLD, guards=True, WGATE=50.0, KS=0.0):
    N = len(D['a']); fps = D['fps']
    k = max(1, int(round(T * fps)))
    e = np.zeros(N); ok = np.zeros(N, bool)
    dm = np.zeros(N); dg = np.zeros(N)
    dm[k:] = wrap(D['psi'][k:] - D['psi'][:-k])
    dg[k:] = D['gi'][k:] - D['gi'][:-k]
    e = np.abs(wrap(dm - dg)) / T
    ok[k:] = True
    if guards:
        gmag = np.linalg.norm(D['w'], axis=1)
        ok &= (gmag < WGATE)
        ok &= (D['lsb'] < 32700.0)
    gmag = np.linalg.norm(D['w'], axis=1)
    thr_eff = THR + KS * gmag                      # 速率自适应门限
    bad = ok & (e > thr_eff)
    hold = int(round(HOLD * fps)); h = 0
    trig = np.zeros(N, bool)
    for i in range(N):
        if bad[i]:
            h = hold
        if h > 0:
            trig[i] = True; h -= 1
    return trig, e, ok


def main():
    Ex = prep(EXTREME)
    print('== 极限运动段 %s ==' % os.path.basename(EXTREME))
    gm = np.linalg.norm(Ex['w'], axis=1)
    print('  时长 %.1f s  陀螺|w| p50 %.0f p90 %.0f p99 %.0f max %.0f dps  (>500dps %.1f%%)  削顶帧 %.1f%%'
          % (Ex['t'][-1], np.percentile(gm, 50), np.percentile(gm, 90), np.percentile(gm, 99), gm.max(),
             100 * np.mean(gm > 500), 100 * np.mean(Ex['lsb'] >= 32700)))
    print('  psi_mag 路径 %.0f°  陀螺路径 %.0f°' % (np.abs(np.diff(Ex['psi'])).sum(),
                                                   np.abs(np.diff(Ex['gi'])).sum()))
    print('  %-8s %-8s %-8s %-9s %-9s %-9s' % ('T(s)', 'THR(dps)', 'HOLD(s)', '误触发%', 'e p90', 'e p99'))
    for T in (0.25, 0.5):
        for THR in (15.0, 20.0, 30.0):
            for KS in (0.0, 0.3, 0.5, 1.0):
                tr, e, ok = trig_of(Ex, T, THR, 3.0, WGATE=100.0, KS=KS)
                print('  T=%.2f THR0=%3.0f KS=%.1f -> 极限段误触发 %5.2f%%' % (T, THR, KS, 100 * tr.mean()))
    print()
    print('== 受扰段：干扰帧内"门开"占比（越低越好，0% 最好）==')
    for p in DIST:
        D = prep(p)
        rate = np.zeros(len(D['a']))
        k = max(1, int(round(0.25 * D['fps'])))
        rate[k:] = np.abs(wrap(D['psi'][k:] - D['psi'][:-k])) / 0.25
        itf = rate > 20.0
        print('  %s  干扰帧占比 %.1f%%' % (os.path.basename(p), 100 * itf.mean()))
        for T, THR, KS in ((0.25, 20.0, 0.5), (0.25, 30.0, 0.5), (0.5, 30.0, 0.5)):
            trig = trig_of(D, T, THR, 3.0, KS=KS)[0]
            on = 100 * (1 - trig[itf].mean()) if itf.any() else -1
            print('     T=%.2f THR0=%3.0f KS=%.1f -> 门关 %5.1f%%  干扰期门开 %6.2f%%' %
                  (T, THR, KS, 100 * trig.mean(), on))
    print()
    print('== 干净段：误触发（应 ~0）==')
    for p in CLEAN:
        if not os.path.exists(p):
            continue
        D = prep(p)
        for T, THR, KS in ((0.25, 20.0, 0.5), (0.25, 30.0, 0.5)):
            trig = trig_of(D, T, THR, 3.0, KS=KS)[0]
            print('  %s T=%.2f THR0=%3.0f KS=%.1f -> 误触发 %5.2f%%' % (os.path.basename(p), T, THR, KS, 100 * trig.mean()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
