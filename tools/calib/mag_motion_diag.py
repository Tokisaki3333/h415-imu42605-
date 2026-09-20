# -*- coding: utf-8 -*-
r"""运动期磁牵引有效性诊断：运动中的残差到底是被拉住了，还是在累积、停稳后一次性放掉。

每帧独立算（不依赖固件内部量）：
  mhat  = 标定后磁场的机体系单位矢量
  gn_b  = 加速度单位矢量（重力）
  b_b   = R(q_ekf)^T b_n（模型磁场在机体系）
  e1    = unit(gn_b x b_b)   -> 对 yaw 敏感（dip 模型无关）
  e2    = unit(b_b x e1)     -> 对 tilt 敏感
  rx = e1.mhat, ry = e2.mhat  (deg)
  残差 rx 就是"运动中被牵引住的量"，它若在运动期持续增大 = 牵引没起作用。

用法:
  python tools/calib/mag_motion_diag.py R:\imu_20260921_030649.bin
  python tools/calib/mag_motion_diag.py <bin> --wmove 100
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import cols_162 as C
import mag360_cal as M

c = C.CH_162
DIP_TAN = 1.70
DECL = -7.53


def load(path):
    a, _ = C.load_frames(path)
    rep = C.frame_report(a)
    A, Cv = M.read_current_AC()
    dt = a[:, c['dt_us']].astype(float) * 1e-6
    t = np.cumsum(dt) - dt[0]
    w = np.linalg.norm(a[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
    clip = ((a[:, c['flags']].astype(int) >> 12) & 1).astype(bool)
    mb = (A @ a[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float).T + Cv[:, None]).T
    mhat = mb / np.linalg.norm(mb, axis=1, keepdims=True)
    gn = a[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
    gn = gn / np.linalg.norm(gn, axis=1, keepdims=True)
    q = a[:, c['ekf_q0']:c['ekf_q0'] + 4].astype(float)
    R = np.stack([M.quat_to_R(r) for r in q])
    ci = 1.0 / np.sqrt(1 + DIP_TAN ** 2)
    bn = np.array([ci * np.sin(np.radians(DECL)), ci * np.cos(np.radians(DECL)), -DIP_TAN * ci])
    bb = np.einsum('nji,j->ni', R, bn)                     # R^T b_n
    bb = bb / np.linalg.norm(bb, axis=1, keepdims=True)
    e1 = np.cross(gn, bb)
    e1 /= np.linalg.norm(e1, axis=1, keepdims=True)
    e2 = np.cross(bb, e1)
    e2 /= np.linalg.norm(e2, axis=1, keepdims=True)
    rx = np.degrees(np.einsum('ni,ni->n', e1, mhat))
    ry = np.degrees(np.einsum('ni,ni->n', e2, mhat))
    u = np.einsum('nij,nj->ni', R, mb)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    umean = u.mean(0); umean /= np.linalg.norm(umean)
    res = np.degrees(np.arccos(np.clip(u @ umean, -1, 1)))
    dipm = np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', mhat, gn), -1, 1))) - 90.0
    dq = a[:, c['ekf_mag_dqx']:c['ekf_mag_dqz'] + 1].astype(float)
    dqy = np.einsum('ni,ni->n', dq, gn)
    dqt = np.sqrt(np.maximum(np.einsum('ni,ni->n', dq, dq) - dqy ** 2, 0.0))
    used = a[:, c['ekf_mag_used']].astype(float) > 0.5
    rej = a[:, c['ekf_mag_rej']].astype(float)
    return dict(rep=rep, n=len(t), t=t, w=w, clip=clip, rx=rx, ry=ry, res=res, dipm=dipm,
                mhat=mhat, ghat=gn,
                dqy=dqy, dqt=dqt, used=used, rej=rej, rs=a[:, c['ekf_mag_rs']].astype(float),
                mnorm=a[:, c['mag_norm']].astype(float),
                syaw=a[:, c['ekf_sigma_yaw']].astype(float),
                stil=a[:, c['ekf_sigma_tilt_deg']].astype(float),
                th=a[:, c['ekf_mag_r_deg']].astype(float),
                nis3=a[:, c['ekf_nis0'] + 3].astype(float),
                nis4=a[:, c['ekf_nis0'] + 4].astype(float))


def p(x, q=50):
    return float(np.percentile(x, q)) if len(x) else float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=r'R:\imu_20260921_030649.bin')
    ap.add_argument('--wmove', type=float, default=100.0, help='判定"运动中"的 |w| 门限 [dps]')
    ap.add_argument('--win', type=float, default=0.25, help='停稳后放大表的窗长 [s]')
    ap.add_argument('--zoom', nargs=2, type=float, default=None, help='逐 20ms 细看区间 t0 t1')
    ap.add_argument('--every', type=float, default=0.02, help='--zoom 的行间隔 [s]')
    a = ap.parse_args()
    C.selfcheck(verbose=False)
    D = load(a.log)
    t, w = D['t'], D['w']
    print('%s  VER=%d  %d 帧  %.1fs  dt=%.0fus'
          % (os.path.basename(a.log), D['rep']['ver'], D['n'], t[-1], 1e6 * np.median(np.diff(t))))

    print('\n=== ① 逐秒总览 ===')
    print('  t     w_p50  w_max clip%  used%  rejΣ  th_p50 nis3_p50 nis4_p50 rs_p50'
          '  Σdq_yaw Σdq_tilt  rx_p50  rx_p90 res_p50 res_p90')
    for s in range(int(t[-1]) + 1):
        m = (t >= s) & (t < s + 1)
        if not m.any():
            continue
        print('%5.1f %6.0f %6.0f %5.1f %6.1f %5.0f %7.2f %8.2f %8.2f %7.0f %8.2f %8.2f'
              ' %7.2f %7.2f %7.2f %7.2f'
              % (s, p(w[m], 50), w[m].max(), 100 * D['clip'][m].mean(), 100 * D['used'][m].mean(),
                 D['rej'][m].max() - D['rej'][m].min() + (D['rej'][m] > 0).sum() * 0,
                 p(D['th'][m]), p(D['nis3'][m]), p(D['nis4'][m]), p(D['rs'][m]),
                 np.abs(D['dqy'][m]).sum(), np.abs(D['dqt'][m]).sum(),
                 p(D['rx'][m]), p(D['rx'][m], 90), p(D['res'][m]), p(D['res'][m], 90)))

    mv = w > a.wmove
    if mv.any():
        tstop = float(t[np.flatnonzero(mv)[-1]])
        tstart = float(t[np.flatnonzero(mv)[0]])
        print('\n=== ② 运动段 %.2f~%.2f s（|w|>%.0f dps），末次 >门限 在 %.2f s ==='
              % (tstart, tstop, a.wmove, tstop))
        mm = mv & (t <= tstop)
        print('  运动段: rx 起 %.2f° 末 %.2f°  最大 %.2f°  Σ|dq_yaw| %.1f°  时长 %.1fs'
              % (D['rx'][np.flatnonzero(mm)[0]], D['rx'][np.flatnonzero(mm)[-1]],
                 np.abs(D['rx'][mm]).max(), np.abs(D['dqy'][mm]).sum(), mm.sum() * np.median(np.diff(t))))
        k = np.flatnonzero(t >= tstop)[0]
        for wlen, tag in ((1.0, '停稳后 1s'), (3.0, '停稳后 3s'), (6.0, '停稳后 6s')):
            m2 = (t > tstop) & (t <= tstop + wlen)
            if m2.any():
                print('  %-10s used %4.1f%%  Σ|dq_yaw| %6.2f°  max|dq_yaw| %5.2f°  rx %.2f->%.2f°'
                      '  res %.2f->%.2f°  nis4_p50 %.2f'
                      % (tag, 100 * D['used'][m2].mean(), np.abs(D['dqy'][m2]).sum(),
                         np.abs(D['dqy'][m2]).max(), D['rx'][m2][0], D['rx'][m2][-1],
                         D['res'][m2][0], D['res'][m2][-1], p(D['nis4'][m2])))
        print('\n=== ③ 停稳后逐 %.2fs 放大（t=%.2f 起）===' % (a.win, tstop))
        print('   t      w_p50  used%   rx     ry    dq_yaw dq_tilt  th    nis4   res')
        e = tstop + 6 * a.win
        for s in np.arange(tstop - a.win, e, a.win):
            m2 = (t >= s) & (t < s + a.win)
            if not m2.any():
                continue
            print(' %6.2f %6.0f %5.0f %7.2f %6.2f %7.2f %7.2f %6.2f %6.2f %6.2f'
                  % (s, p(w[m2], 50), 100 * D['used'][m2].mean(), D['rx'][m2].mean(),
                     D['ry'][m2].mean(), D['dqy'][m2].sum(), D['dqt'][m2].sum(),
                     p(D['th'][m2]), p(D['nis4'][m2]), D['res'][m2].mean()))
        slow = np.flatnonzero((t > tstop + 0.5) & (w < 30))
        if slow.size > 20:
            i0, i1 = slow[0], slow[-1]
            seg = slice(i0, i1)
            print('\n  停稳静置段 t=%.2f~%.2f s（%d 帧）: Σ|dq_yaw| %.2f°  Σ|dq_tilt| %.2f°'
                  % (t[i0], t[i1], i1 - i0, np.abs(D['dqy'][seg]).sum(), np.abs(D['dqt'][seg]).sum()))
            print('     rx %.2f -> %.2f°  res %.2f -> %.2f°  used %.1f%%  nis4_p50 %.2f  nis3_p50 %.2f'
                  % (D['rx'][i0], D['rx'][i1], D['res'][i0], D['res'][i1],
                     100 * D['used'][seg].mean(), p(D['nis4'][seg]), p(D['nis3'][seg])))
    if a.zoom:
        z0, z1 = a.zoom
        print('\n=== ④ 逐 %.0f ms 细看 t=%.2f~%.2f s ===' % (a.every * 1000, z0, z1))
        print('   t      w    gx     gy     gz  |   mx     my     mz  |'
              '   rx     ry    dq_yaw dq_tilt   th   nis4  sig_yaw sig_tilt   rs  used  res')
        tt = z0
        while tt <= z1:
            k = int(np.argmin(np.abs(t - tt)))
            if t[k] >= z0 - a.every and t[k] <= z1 + a.every:
                g = D['ghat'][k]; mm = D['mhat'][k]
                print(' %6.3f %5.0f %6.3f %6.3f %6.3f | %6.3f %6.3f %6.3f |'
                      ' %6.2f %6.2f %7.3f %7.3f %6.2f %6.2f %7.2f %8.2f %4d %6.2f'
                      % (t[k], w[k], g[0], g[1], g[2], mm[0], mm[1], mm[2],
                         D['rx'][k], D['ry'][k], D['dqy'][k], D['dqt'][k], D['th'][k],
                         D['nis4'][k], D['syaw'][k], D['stil'][k], D['used'][k], D['res'][k]))
            tt += a.every
    return 0


if __name__ == '__main__':
    sys.exit(main())
