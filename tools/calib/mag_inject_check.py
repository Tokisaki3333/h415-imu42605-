# -*- coding: utf-8 -*-
r"""从数据反推磁修正的**注入约定**和**每次更新的真实效果**。

数据里 q_k（发布的 EKF 四元数）、dq_k（发布的世界系旋转矢量增量）。静止时传播量≈0，所以
    q_k ≈ dq_k x q_{k-1}   （世界系注入）   或   q_k ≈ q_{k-1} x dq_k  （机体系注入）
用两个假设各自与发布的 q_k 比对，角度误差小的那个就是真实约定。

再看"每次更新到底把姿态拉近还是推远"：模型失配角 theta = angle(m̂_meas, b̂_b(q))，静止时若更新
正确，theta 应当减小；若 theta 增大 = 修正方向反了（正反馈）。

用法:
  python tools/calib/mag_inject_check.py R:\imu_20260921_030649.bin
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


def qmul(a, b):
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], axis=-1)


def qrot(v):
    """旋转矢量(rad, (...,3)) -> 四元数。"""
    th = np.linalg.norm(v, axis=-1, keepdims=True)
    half = 0.5 * th
    s = np.where(th > 1e-12, np.sin(half) / np.maximum(th, 1e-12), 0.5)
    return np.concatenate([np.cos(half), v * s], axis=-1)


def qang(a, b):
    d = np.abs(np.einsum('...i,...i->...', a, b))
    return np.degrees(2 * np.arccos(np.clip(d, -1, 1)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=r'R:\imu_20260921_030649.bin')
    a = ap.parse_args()
    C.selfcheck(verbose=False)
    fr, _ = C.load_frames(a.log)
    rep = C.frame_report(fr)
    A, Cv = M.read_current_AC()
    dt = fr[:, c['dt_us']].astype(float) * 1e-6
    t = np.cumsum(dt) - dt[0]
    q = fr[:, c['ekf_q0']:c['ekf_q0'] + 4].astype(float)
    qn = np.linalg.norm(q, axis=1, keepdims=True)
    ok = (qn[:, 0] > 0.5)
    q = q / np.maximum(qn, 1e-9)
    dq = fr[:, c['ekf_mag_dqx']:c['ekf_mag_dqz'] + 1].astype(float)
    dqn = np.linalg.norm(dq, axis=1)
    used = fr[:, c['ekf_mag_used']].astype(float) > 0.5
    w = np.linalg.norm(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
    mb = (A @ fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float).T + Cv[:, None]).T
    mhat = mb / np.linalg.norm(mb, axis=1, keepdims=True)
    ci = 1.0 / np.sqrt(1 + DIP_TAN ** 2)
    bn = np.array([ci * np.sin(np.radians(DECL)), ci * np.cos(np.radians(DECL)), -DIP_TAN * ci])
    R = np.stack([M.quat_to_R(r) for r in q])
    bb = np.einsum('nji,j->ni', R, bn)
    bb /= np.linalg.norm(bb, axis=1, keepdims=True)
    th = np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', mhat, bb), -1, 1)))
    print('%s  VER=%d  %d 帧  %.1fs  theta(模型失配) p50 %.2f° p90 %.2f°'
          % (os.path.basename(a.log), rep['ver'], len(t), t[-1],
             np.nanpercentile(th[ok], 50), np.nanpercentile(th[ok], 90)))

    for t0, t1, tag in ((0.0, 1.0, '起始静置'), (3.0, 10.0, '剧烈运动'), (13.9, 14.6, '停稳异常段'),
                        (15.0, 29.5, '末尾静置')):
        k0 = int(np.flatnonzero(t >= t0)[0]); k1 = int(np.flatnonzero(t <= t1)[-1])
        idx = np.arange(max(k0, 1), k1)
        idx = idx[ok[idx] & ok[idx - 1] & used[idx] & (dqn[idx] > 0.05)]
        if idx.size < 20:
            print('\n== %s == 有效更新太少 (%d)' % (tag, idx.size)); continue
        dqr = qrot(np.radians(dq[idx]))
        qA = qmul(dqr, q[idx - 1])          # 世界系注入 dq x q
        qB = qmul(q[idx - 1], dqr)          # 机体系注入 q x dq
        eA = np.median(qang(qA, q[idx]))
        eB = np.median(qang(qB, q[idx]))
        dth = th[idx] - th[idx - 1]
        print('\n== %s == t=%.2f~%.2f  |w| p50 %.0f dps  n=%d  |dq| p50 %.3f°  帧间转角 p50 %.3f°'
              % (tag, t0, t1, np.percentile(w[idx], 50), idx.size,
                 np.median(dqn[idx]), np.median(qang(q[idx], q[idx - 1]))))
        print('   约定: dqxq 误差 %.4f°   qxdq 误差 %.4f°  -> %s'
              % (eA, eB, '世界系注入 dqxq' if eA < eB else '机体系注入 qxdq'))
        print('   dtheta: med %+.3f deg  mean %+.3f deg  shrink%% %.0f   theta_after p50 %.2f deg'
              % (np.median(dth), dth.mean(), 100 * (dth < 0).mean(), np.median(th[idx])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
