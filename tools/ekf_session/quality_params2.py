# -*- coding: utf-8 -*-
r"""陀螺质量参数（修正版）：误差取"运动后**第一次**重力门打开之前一帧"的残差。

上一版把 t0 取在 |w|<5 dps 的静止窗里 —— 放回台座的过程中（|w| 几十 dps、|a|≈1 g）
重力门可能已经开过、已修正过几次，于是量到的牵引量偏小（用户指出）。
本版：
  1) 找**重力门连续关闭 >= V5F_MAG_... 0.5 s** 的段（= 没有重力牵引的时段，含整个大运动 + 放回过程）；
  2) tg = 该段之后**第一次门打开**的帧（= 第一次修正发生的地方）；
  3) 误差 = q(tg-1) -> q(tg + 2 s 稳定后) 的相对旋转矢量 —— 这才是"第一次修正前积攒的全部误差"；
  4) 旋转角 θ_i = ∫ w_i dt 覆盖同一关闭段，T = 段时长；
  5) 拟合 err = a*θ + b*T。

用法: python tools/ekf_session/quality_params2.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def qrel_rotvec(qb, qa):
    qb = np.asarray(qb, float); qa = np.asarray(qa, float)
    qb = qb / max(np.linalg.norm(qb), 1e-12); qa = qa / max(np.linalg.norm(qa), 1e-12)
    wb, xb, yb, zb = qb
    d = np.array([qa[0]*wb + qa[1]*xb + qa[2]*yb + qa[3]*zb,
                  -qa[0]*xb + qa[1]*wb - qa[2]*zb + qa[3]*yb,
                  -qa[0]*yb + qa[1]*zb + qa[2]*wb - qa[3]*xb,
                  -qa[0]*zb - qa[1]*yb + qa[2]*xb + qa[3]*wb])
    d = d / max(np.linalg.norm(d), 1e-12)
    if d[0] < 0.0:
        d = -d
    v = d[1:]
    nv = np.linalg.norm(v)
    ang = 2.0 * math.atan2(nv, d[0]) * RAD2DEG
    return np.zeros(3) if nv < 1e-12 else v / nv * ang


def scan(path, verbose=True, min_closed=0.5):
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    t = np.cumsum(dt)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
    w = np.linalg.norm(g, axis=1)
    acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
    an = np.linalg.norm(acc, axis=1)
    q = np.asarray(a[:, C['ekf_q0']:C['ekf_q0'] + 4], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    gm = np.asarray(a[:, C['gate_ekf_mag_yaw']], float)
    op = gt > 0.5

    rows = []
    i = 0
    while i < n:
        if not op[i]:
            j = i
            while j < n and not op[j]:
                j += 1
            if dt[i:j].sum() >= min_closed and j < n and i > 0:
                tg = j                      # 第一次修正的帧
                # 稳定值：tg 之后 2~3 s 的中位四元数
                s0 = min(n - 1, tg + 400)
                s1 = min(n, tg + 1000)
                if s1 - s0 >= 50:
                    qs = np.median(q[s0:s1], axis=0)
                    err = qrel_rotvec(q[tg - 1], qs)
                    th = np.array([np.sum(g[i:tg, ax] * dt[i:tg]) for ax in range(3)])
                    rows.append(dict(tg=tg, T=dt[i:tg].sum(), th=th, err=err,
                                     span=np.sum(w[i:tg] * dt[i:tg]),
                                     wmax=w[i:tg].max(), amin=float(np.min(an[i:tg])),
                                     amax=float(np.max(an[i:tg])),
                                     gtduty=100.0 * float(np.mean(op[i:tg])),
                                     gmduty=100.0 * float(np.mean(gm[i:tg] > 0.5)),
                                     ver=ver, file=os.path.basename(path)))
            i = j
        else:
            i += 1
    if verbose:
        print('== %-24s VER=%3d  门关段(>=%.1fs) %d 个' %
              (os.path.basename(path), ver, min_closed, len(rows)))
        print('   %-11s %-6s %-7s %-8s %-24s %-9s %-10s %-9s | 段内门T/门M %%'
              % ('tg帧', 'T s', '|w|max', '路径角', '净转角 x / y / z', 'err_roll', 'err_pitch',
                 'err_yaw'))
        for s in rows:
            print('   %-11d %-6.1f %-7.0f %-8.0f (%7.0f,%7.0f,%7.0f) %-9.2f %-10.2f %-9.2f | %3.0f / %3.0f'
                  % (s['tg'], s['T'], s['wmax'], s['span'], s['th'][0], s['th'][1], s['th'][2],
                     s['err'][0], s['err'][1], s['err'][2], s['gtduty'], s['gmduty']))
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
    rows = [r for r in rows if abs(r['th'][0]) > 500 or abs(r['th'][1]) > 500]
    if len(rows) < 3:
        print('可用样本 %d 个：' % len(rows))
        for s in rows:
            print('  %s tg=%d T=%.1f θ=(%.0f,%.0f,%.0f) err=(%+.2f,%+.2f) yaw %+.2f'
                  % (s['file'], s['tg'], s['T'], s['th'][0], s['th'][1], s['th'][2],
                     s['err'][0], s['err'][1], s['err'][2]))
        return
    Th = np.array([s['th'] for s in rows]); T = np.array([s['T'] for s in rows])
    E = np.array([s['err'] for s in rows])
    print('==== 汇总 %d 个样本（|θ|>500 deg）：err_i = a*θ_i + b*T ====' % len(rows))
    for ax, nm in enumerate(('err_roll  ~ θx', 'err_pitch ~ θy', 'err_yaw   ~ θz')):
        A = np.vstack([Th[:, ax], T]).T
        coef, *_ = np.linalg.lstsq(A, E[:, ax], rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - E[:, ax]) ** 2)))
        cc = float(np.corrcoef(Th[:, ax], E[:, ax])[0, 1]) if np.std(E[:, ax]) > 0 else 0.0
        print('  %-14s ppm = %+8.0f  时间项 = %+7.4f deg/s (= %+8.1f deg/h)  rms %.2f deg  corr %+.2f'
              % (nm, coef[0] * 1e6, coef[1], coef[1] * 3600.0, rms, cc))
    print()
    print('单参数：')
    for ax, nm in enumerate(('err_roll', 'err_pitch', 'err_yaw')):
        P = Th[:, ax]
        c = float(np.sum(P * E[:, ax]) / max(np.sum(P * P), 1e-12))
        rms = float(np.sqrt(np.mean((c * P - E[:, ax]) ** 2)))
        print('  %-10s ppm = %+8.0f  rms %.2f deg' % (nm, c * 1e6, rms))


if __name__ == '__main__':
    main()
