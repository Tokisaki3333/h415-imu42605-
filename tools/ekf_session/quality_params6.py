# -*- coding: utf-8 -*-
r"""陀螺质量参数 v6：**每轴路径角**（∫|w_i|dt）而不是净值 —— 用户指出"每个轴都有剧烈转动"。

误差 r0 = 首次重力修正前的倾角残差（衰减曲线外推到 tg，见 quality_params5）。
分母同时给三种，便于看清差异：
    θ_net_i  = ∫ w_i dt        （来回摆动会被抵消，**不能**当比例误差的分母）
    θ_path_i = ∫ |w_i| dt      （每轴实际转过的角度，陀螺不对称标度/交叉轴误差积它）
    θ_path   = ∫ |w| dt

用法: python tools/ekf_session/quality_params6.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def scan(path, min_closed=0.5, amtol=0.02, span=1.5, minpts=6):
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
                step = max(np.median(dt), 1e-4)
                sel = [p for p in range(tg, min(n, tg + int(span / step))) if op[p] and clean[p]]
                rm = np.array([np.linalg.norm(rvec[p]) for p in sel])
                r0 = float('nan')
                if len(rm) >= minpts and np.max(rm) > 0.5:
                    keep = rm > 0.5
                    tt = t[sel][keep] - t[tg]
                    yy = np.log(rm[keep])
                    A = np.vstack([np.ones_like(tt), -tt]).T
                    coef, *_ = np.linalg.lstsq(A, yy, rcond=None)
                    r0f = math.exp(coef[0])
                    r0 = r0f if (r0f <= 2.5 * np.max(rm)) else float(rm[0])
                th_net = np.array([np.sum(g[i:tg, ax] * dt[i:tg]) for ax in range(3)])
                th_path = np.array([np.sum(np.abs(g[i:tg, ax]) * dt[i:tg]) for ax in range(3)])
                rows.append(dict(tg=tg, T=dt[i:tg].sum(), th=th_net, tp=th_path, r0=r0,
                                 span=np.sum(w[i:tg] * dt[i:tg]), wmax=w[i:tg].max(),
                                 ver=ver, file=os.path.basename(path)))
            i = j if j > i else i + 1
        else:
            i += 1
    return rows


def main():
    files = sys.argv[1:] or [r'R:\imu_20260922_210629.bin', r'R:\imu_20260922_205255.bin',
                             r'R:\imu_20260922_201451.bin']
    rows = []
    for f in files:
        try:
            rows += scan(f)
        except Exception as ex:
            print('!! %s: %s' % (os.path.basename(f), ex))
    print('%-24s %-7s %-6s %-7s %-24s %-24s %-8s %-9s'
          % ('文件', 'tg', 'T s', '|w|max', '每轴路径角', '净转角', 'r0 deg', 'ppm_x(路径)'))
    for s in rows:
        px = s['r0'] / max(s['tp'][0], 1e-9) * 1e6 if s['r0'] == s['r0'] else float('nan')
        print('%-24s %-7d %-6.1f %-7.0f (%6.0f,%6.0f,%6.0f) (%6.0f,%6.0f,%6.0f) %-8.2f %-9.0f'
              % (s['file'][:24], s['tg'], s['T'], s['wmax'], s['tp'][0], s['tp'][1], s['tp'][2],
                 s['th'][0], s['th'][1], s['th'][2], s['r0'], px))
    big = [r for r in rows if r['r0'] == r['r0'] and r['wmax'] > 500 and r['T'] >= 5.0]
    print()
    print('大运动样本 %d 个' % len(big))
    for ax, nm in enumerate(('x', 'y', 'z')):
        P = np.array([r['tp'][ax] for r in big]); E = np.array([r['r0'] for r in big])
        c = float(np.sum(P * E) / max(np.sum(P * P), 1e-12))
        print('  r0 ~ 每轴路径(%s): ppm = %+8.0f' % (nm, c * 1e6))
    P = np.array([r['span'] for r in big]); E = np.array([r['r0'] for r in big])
    c = float(np.sum(P * E) / max(np.sum(P * P), 1e-12))
    print('  r0 ~ 总路径    : ppm = %+8.0f' % (c * 1e6))
    T = np.array([r['T'] for r in big])
    c = float(np.sum(T * E) / max(np.sum(T * T), 1e-12))
    print('  r0 ~ T         : 零偏 = %+7.4f deg/s (%+8.1f deg/h)' % (c, c * 3600.0))
    if len(big) >= 3:
        A = np.vstack([np.array([r['span'] for r in big]), T]).T
        coef, *_ = np.linalg.lstsq(A, E, rcond=None)
        print('  两参数 r0 = a*总路径 + b*T: ppm = %+8.0f, 零偏 = %+7.4f deg/s (%+8.1f deg/h)'
              % (coef[0] * 1e6, coef[1], coef[1] * 3600.0))


if __name__ == '__main__':
    main()
