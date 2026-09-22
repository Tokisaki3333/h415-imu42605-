# -*- coding: utf-8 -*-
r"""陀螺 ppm/零偏 提取 v5（抗单帧污染）：用牵引衰减曲线外推到 tg。

单帧法（v3）在"第一个被信任的帧"上取值，但那一帧的加速度计仍可能被残余动态污染
（实测出现 54 deg 的孤立值，而同转角幅度的另一段只有 1.8 deg）。
本版用**衰减曲线**：
  1) 段 = 重力门连续关闭 >= 0.5 s；tg = 其后第一次门开的帧；
  2) 取 tg .. tg+1.5 s 内 **门开且 |am2-1| < 0.02**（比门限更严）的帧，计算
     r(t) = |normalize(accel_g) - up_pred| （= 修正前的倾角残差）；
  3) 对 r(t) 做指数拟合 r = r0*exp(-(t-tg)/tau)（只取 r > 0.5 deg 的点），
     外推到 t = tg 得 r0 = **第一次修正前的积攒误差**；同时给出 tau（牵引速度）。
  4) ppm = r0 / θ_i（分轴净转角）；err = a*θ + b*T 拟合。

用法: python tools/ekf_session/quality_params5.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def scan(path, verbose=True, min_closed=0.5, amtol=0.02, span=1.5, minpts=6):
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
    up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
    thm = np.asarray(a[:, C['ekf_mag_cmp_thm']], float)
    thp = np.asarray(a[:, C['ekf_mag_cmp_thp']], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    op = gt > 0.5
    clean = np.abs(an * an - 1.0) < amtol
    rvec = (acc / np.maximum(an, 1e-6)[:, None] - up) * RAD2DEG

    rows = []
    i = 0
    while i < n:
        if not op[i] and i > 0:
            j = i
            while j < n and not op[j]:
                j += 1
            if dt[i:j].sum() >= min_closed and j < n:
                tg = j
                sel = [p for p in range(tg, min(n, tg + int(span / max(np.median(dt), 1e-4))))
                       if op[p] and clean[p]]
                r_mag = np.array([np.linalg.norm(rvec[p]) for p in sel])
                rr = rvec[sel]
                keep = r_mag > 0.5
                rmax = float(np.max(r_mag)) if len(r_mag) else float('nan')
                r0 = float('nan'); tau = float('nan'); npts = int(keep.sum())
                if npts >= minpts:
                    tt = t[sel][keep] - t[tg]
                    yy = np.log(r_mag[keep])
                    A = np.vstack([np.ones_like(tt), -tt]).T
                    coef, *_ = np.linalg.lstsq(A, yy, rcond=None)
                    r0 = math.exp(coef[0])
                    tau = 1.0 / coef[1] if coef[1] > 1e-6 else float('inf')
                    if r0 > 2.5 * rmax or tau > 30.0:   # 拟合发散 -> 弃用外推，退回首帧值
                        r0 = float(r_mag[0])
                # 分轴符号：取 tg 之后前 100 帧里第一个干净帧的方向
                ex = ey = float('nan')
                for p in range(tg, min(n, tg + 200)):
                    if op[p] and clean[p]:
                        ex, ey = rvec[p][0], rvec[p][1]
                        break
                th = np.array([np.sum(g[i:tg, ax] * dt[i:tg]) for ax in range(3)])
                rows.append(dict(tg=tg, T=dt[i:tg].sum(), th=th, r0=r0, tau=tau, npts=npts,
                                 r_raw=float(r_mag[0]) if len(r_mag) else float('nan'),
                                 ex=ex, ey=ey, sign=np.sign(ex) if ex == ex else 1.0,
                                 span=np.sum(w[i:tg] * dt[i:tg]), wmax=w[i:tg].max(),
                                 ver=ver, file=os.path.basename(path)))
                i = j if j > i else i + 1
            else:
                i = j if j > i else i + 1
        else:
            i += 1
    if verbose:
        print('== %-24s VER=%3d  门关段 %d 个（外推法）' % (os.path.basename(path), ver, len(rows)))
        print('   %-9s %-6s %-7s %-8s %-24s %-9s %-8s %-8s %-6s'
              % ('tg', 'T s', '|w|max', '路径角', '净转角 x/y/z', 'r0(外推)', 'r_raw', 'tau s',
                 'pts'))
        for s in rows:
            print('   %-9d %-6.1f %-7.0f %-8.0f (%7.0f,%7.0f,%7.0f) %-9.2f %-8.2f %-8.2f %-6d'
                  % (s['tg'], s['T'], s['wmax'], s['span'], s['th'][0], s['th'][1], s['th'][2],
                     s['r0'], s['r_raw'], s['tau'], s['npts']))
    return rows


def main():
    files = sys.argv[1:] or [r'R:\imu_20260922_210629.bin']
    rows = []
    for f in files:
        try:
            rows += scan(f)
        except Exception as ex:
            print('!! %s: %s' % (os.path.basename(f), ex))
        print()
    big = [r for r in rows if (abs(r['th'][0]) > 500 or abs(r['th'][1]) > 500)
           and r['r0'] == r['r0']]
    print('可用大运动样本 %d 个' % len(big))
    if not big:
        return
    Th = np.array([r['th'] for r in big])
    T = np.array([r['T'] for r in big])
    E = np.array([r['r0'] * r['sign'] for r in big])   # 用首帧方向给符号（倾角总量）
    print()
    print('==== r0（首修正前积攒的倾角误差）====')
    for r in big:
        print('  %s tg=%-6d T=%-5.1f θx=%-7.0f θy=%-7.0f r0=%6.2f deg  tau=%.2f s  -> ppm_x %+6.0f '
              'ppm_y %+6.0f'
              % (r['file'], r['tg'], r['T'], r['th'][0], r['th'][1], r['r0'], r['tau'],
                 r['r0'] / max(abs(r['th'][0]), 1e-9) * 1e6,
                 r['r0'] / max(abs(r['th'][1]), 1e-9) * 1e6))
    print()
    for ax, nm in enumerate(('θx', 'θy')):
        A = np.vstack([Th[:, ax], T]).T
        coef, *_ = np.linalg.lstsq(A, E, rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - E) ** 2)))
        cc = float(np.corrcoef(Th[:, ax], E)[0, 1]) if np.std(E) > 0 else 0.0
        print('  r0 ~ %s: ppm = %+8.0f  时间项 = %+7.4f deg/s (%+8.1f deg/h)  rms %.2f  corr %+.2f'
              % (nm, coef[0] * 1e6, coef[1], coef[1] * 3600.0, rms, cc))
    P = Th[:, 0]
    c = float(np.sum(P * E) / max(np.sum(P * P), 1e-12))
    print('  单参数 r0 ~ θx: ppm = %+8.0f' % (c * 1e6))
    P = T
    c = float(np.sum(P * E) / max(np.sum(P * P), 1e-12))
    print('  单参数 r0 ~ T : 零偏 = %+7.4f deg/s (%+8.1f deg/h)' % (c, c * 3600.0))


if __name__ == '__main__':
    main()
