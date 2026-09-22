# -*- coding: utf-8 -*-
r"""陀螺质量参数（v4，逐帧反解"实际施加的修正量"）：

问题：把设备放回台座时**真实朝向会变**，所以 q(tg-1) 与 q(稳定) 之差混进了真实旋转，
不能当误差；而第一个被信任的重力帧上加速度计仍可能受动态加速度污染（实测 17~56 deg 假残差）。

做法：逐帧精确反解 EKF 实际注入的姿态修正
    q[p] = Exp_nav(dx) * ( q[p-1] * dq_gyro(frame) )
    => dx[p] = rotvec( q[p] * (q[p-1] * dq_gyro)^-1 )        （导航系，deg）
  其中 dq_gyro 用**同一帧的陀螺**按固件约定（q <- q (x) dq_body）积分。
  把 settle 窗（tg .. tg+3 s）内所有 dx 累加 = 该次"重力/地磁牵引量"总量
  —— 这正是"第一次修正之前积攒的误差"，且与设备真实转向无关。

用法: python tools/ekf_session/quality_params4.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def qmul(a, b):
    return np.array([
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


def qconj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])


def rotvec(q):
    q = np.asarray(q, float)
    q = q / max(np.linalg.norm(q), 1e-12)
    if q[0] < 0.0:
        q = -q
    v = q[1:]
    nv = np.linalg.norm(v)
    if nv < 1e-12:
        return np.zeros(3)
    return v / nv * (2.0 * math.atan2(nv, q[0]) * RAD2DEG)


def dq_gyro(w_dps, dt):
    n = float(np.linalg.norm(w_dps))
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    half = 0.5 * math.radians(n) * dt
    return np.array([math.cos(half), *(math.sin(half) / n * w_dps)])


def scan(path, verbose=True, min_closed=0.5, settle=3.0):
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
    w = np.linalg.norm(g, axis=1)
    q = np.asarray(a[:, C['ekf_q0']:C['ekf_q0'] + 4], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    gm = np.asarray(a[:, C['gate_ekf_mag_yaw']], float)
    op = gt > 0.5

    # 逐帧修正量（导航系，度）
    dx = np.zeros((n, 3))
    for p in range(1, n):
        qp = q[p - 1] / max(np.linalg.norm(q[p - 1]), 1e-12)
        pred = qmul(qp, dq_gyro(g[p], dt[p]))
        dx[p] = rotvec(qmul(q[p] / max(np.linalg.norm(q[p]), 1e-12), qconj(pred)))

    rows = []
    i = 0
    while i < n:
        if not op[i] and i > 0:
            j = i
            while j < n and not op[j]:
                j += 1
            if dt[i:j].sum() >= min_closed and j < n:
                tg = j
                p1 = min(n, tg + int(settle / max(np.median(dt), 1e-4)))
                tot = dx[tg:p1].sum(axis=0)
                th = np.array([np.sum(g[i:tg, ax] * dt[i:tg]) for ax in range(3)])
                rows.append(dict(tg=tg, T=dt[i:tg].sum(), th=th, tot=tot,
                                 span=np.sum(w[i:tg] * dt[i:tg]), wmax=w[i:tg].max(),
                                 ver=ver, file=os.path.basename(path)))
            i = j
        else:
            i += 1
    if verbose:
        print('== %-24s VER=%3d  门关段(>=%.1fs) %d 个' % (os.path.basename(path), ver,
              min_closed, len(rows)))
        print('   %-10s %-6s %-7s %-8s %-24s %-24s'
              % ('tg帧', 'T s', '|w|max', '路径角', '净转角 x/y/z', '牵引总量 (nav x,y,z) [deg]'))
        for s in rows:
            print('   %-10d %-6.1f %-7.0f %-8.0f (%7.0f,%7.0f,%7.0f) (%+8.2f,%+8.2f,%+8.2f)'
                  % (s['tg'], s['T'], s['wmax'], s['span'], s['th'][0], s['th'][1], s['th'][2],
                     s['tot'][0], s['tot'][1], s['tot'][2]))
    return rows


def main():
    files = sys.argv[1:] or sorted(glob.glob(r'R:\imu_2026*.bin'))
    rows = []
    for f in files:
        try:
            rows += scan(f)
        except Exception as ex:
            print('!! %s: %s' % (os.path.basename(f), ex))
        print()
    big = [r for r in rows if abs(r['th'][0]) > 500 or abs(r['th'][1]) > 500]
    print('合计 %d 段，其中 |θ|>500 deg 的 %d 段' % (len(rows), len(big)))
    if len(big) < 2:
        return
    Th = np.array([r['th'] for r in big]); T = np.array([r['T'] for r in big])
    E = np.array([r['tot'] for r in big])
    print()
    print('==== 大运动段 %d 个：牵引量 = a*θ + b*T ====' % len(big))
    for ax, nm in enumerate(('x(nav) ~ θx', 'y(nav) ~ θy', 'z(nav) ~ θz')):
        A = np.vstack([Th[:, ax], T]).T
        coef, *_ = np.linalg.lstsq(A, E[:, ax], rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - E[:, ax]) ** 2)))
        cc = float(np.corrcoef(Th[:, ax], E[:, ax])[0, 1]) if np.std(E[:, ax]) > 0 else 0.0
        print('  %-12s ppm = %+8.0f  时间项 = %+7.4f deg/s (= %+8.1f deg/h)  rms %.2f  corr %+.2f'
              % (nm, coef[0] * 1e6, coef[1], coef[1] * 3600.0, rms, cc))
    print()
    print('单参数（只用同轴旋转角）：')
    for ax, nm in enumerate(('x', 'y', 'z')):
        P = Th[:, ax]
        c = float(np.sum(P * E[:, ax]) / max(np.sum(P * P), 1e-12))
        rms = float(np.sqrt(np.mean((c * P - E[:, ax]) ** 2)))
        print('  %-4s ppm = %+8.0f rms %.2f deg' % (nm, c * 1e6, rms))


if __name__ == '__main__':
    main()
