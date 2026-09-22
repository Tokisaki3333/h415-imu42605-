# -*- coding: utf-8 -*-
r"""VER=130 质量参数提取（用户口径）：

"最新一次录制用以读取质量参数：剧烈运动下的 ppm（随旋转角的误差）和随时间的误差；
 注意误差应当取重力开始有效牵引前的一瞬间的残差，即静止后地磁和重力的牵引量就是误差角度。"

方法（全部用调试帧里已发布的量）：
  分段：静态段 = |w| < 5 dps 且连续 >= 0.3 s；运动段 = 上一静态段结束 -> 本静态段开始。
  t0 = 静态段里第一个"观测门打开"的帧（重力门 gate_ekf_tilt 或 地磁门 gate_ekf_mag_yaw）。
  误差（= 随后重力/地磁要牵引掉的角度）：
      倾角 err_tilt = |normalize(accel_g) - (prx,pry,prz)|      [col32..34 vs 141..143]
            e_x ~ 绕 y（俯仰）误差, e_y ~ 绕 x（横滚）误差（小角近似，body 系）
      偏航 nu_yaw   = wrap(thm - thp)                            [col144 - col145]
  旋转角 th_i = ∫ w_i dt（分轴，deg）；时长 T = 运动段时长。
  拟合 err = a·th + b·T  -> a = ppm（相对旋转角），b = 随时间的误差（deg/s, deg/h）。

用法: python tools/ekf_session/quality_extract.py [R:\imu_xxx.bin]
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
DEG = 180.0 / math.pi


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260922_205255.bin'
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], dtype=float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], dtype=float)
    w = np.linalg.norm(g, axis=1)
    acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], dtype=float)
    up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], dtype=float)
    thm = np.asarray(a[:, C['ekf_mag_cmp_thm']], dtype=float)
    thp = np.asarray(a[:, C['ekf_mag_cmp_thp']], dtype=float)
    gate_t = np.asarray(a[:, C['gate_ekf_tilt']], dtype=float)
    gate_m = np.asarray(a[:, C['gate_ekf_mag_yaw']], dtype=float)
    bias = np.asarray(a[:, C['gyro_bias_dps0']:C['gyro_bias_dps0'] + 3], dtype=float)
    an = np.linalg.norm(acc, axis=1)
    an = np.where(an < 1e-3, 1.0, an)
    evec = acc / an[:, None] - up

    print('录像 %s  VER=%d  %.1f s  帧数 %d' % (os.path.basename(path), ver, dt.sum(), n))
    print('|w| p50 %.0f p90 %.0f max %.0f dps   |a| p50 %.3f g   |a|静止 p50 %.3f g'
          % (np.percentile(w, 50), np.percentile(w, 90), w.max(), np.median(an),
             np.median(an[w < 5.0]) if np.any(w < 5.0) else float('nan')))
    print('bias p50 (%.3f %.3f %.3f) dps' % tuple(np.median(bias, axis=0)))
    print()

    # ---- 分段 ----
    stat = w < 5.0
    segs_s = []
    i = 0
    while i < n:
        if stat[i]:
            j = i
            while j < n and stat[j]:
                j += 1
            if j - i >= 100:
                segs_s.append((i, j))
            i = j
        else:
            i += 1

    rows = []
    prev_end = 0
    for (i, j) in segs_s:
        km = prev_end
        prev_end = j
        if i - km < 100:
            continue
        dur = dt[km:i].sum()
        th = np.array([np.sum(g[km:i, ax] * dt[km:i]) for ax in range(3)])
        path = np.sum(w[km:i] * dt[km:i])
        t0 = None
        for p in range(i, min(n, i + 600)):
            if gate_t[p] > 0.5 or gate_m[p] > 0.5:
                t0 = p
                break
        if t0 is None:
            continue
        rows.append(dict(i=i, t0=t0, dur=dur, th=th, path=path, e=evec[t0] * DEG,
                         et=np.degrees(np.linalg.norm(evec[t0])),
                         nu=wrap_deg(thm[t0] - thp[t0]),
                         wmax=w[km:i].max(),
                         gt=100.0 * np.mean(gate_t[km:i] > 0.5),
                         gm=100.0 * np.mean(gate_m[km:i] > 0.5)))

    print('%-7s %-6s %-6s %-8s %-28s %-8s %-8s %-8s %-5s %-5s' %
          ('t0帧', 'T s', '|w|max', '路径角', '净转角 x / y / z', '倾角', 'e_x俯仰', 'e_y横滚',
           '门T%', '门M%'))
    for s in rows:
        print('%-7d %-6.1f %-6.0f %-8.0f (%7.0f,%7.0f,%7.0f) %-8.2f %-8.2f %-8.2f %-5.0f %-5.0f'
              % (s['t0'], s['dur'], s['wmax'], s['path'], s['th'][0], s['th'][1], s['th'][2],
                 s['et'], s['e'][0], s['e'][1], s['gt'], s['gm']))
    print()
    for s in rows:
        print('  t0=%-6d nu_yaw = %+7.2f deg' % (s['t0'], s['nu']))
    print()

    # ---- 拟合 err = a*th + b*T ----
    if len(rows) >= 2:
        Th = np.array([s['th'] for s in rows]) * DEG
        T = np.array([s['dur'] for s in rows])
        Y = np.zeros((len(rows), 3))
        Y[:, 0] = np.array([s['e'][0] for s in rows]) * DEG
        Y[:, 1] = np.array([s['e'][1] for s in rows]) * DEG
        Y[:, 2] = [np.radians(s['nu']) for s in rows]
        print('分轴拟合 err_i = a*th_i + b*T:')
        for ax, nm in enumerate(('e_x / 绕y(俯仰)', 'e_y / 绕x(横滚)', 'nu_yaw / 绕z(偏航)')):
            A = np.vstack([Th[:, ax], T]).T
            coef, *_ = np.linalg.lstsq(A, Y[:, ax], rcond=None)
            rms = np.sqrt(np.mean((A @ coef - Y[:, ax]) ** 2)) * DEG
            print('  %-18s ppm = %+8.0f  时间项 = %+8.4f deg/s (= %+8.2f deg/h)  rms %.2f deg'
                  % (nm, coef[0] * 1e6, coef[1] * DEG, coef[1] * DEG * 3600.0, rms))
        P = np.array([np.radians(s['path']) for s in rows])
        print()
        print('单参数（总路径角 theta_path）：')
        for ax, nm in enumerate(('e_x', 'e_y', 'nu_yaw')):
            coef = float(np.sum(P * Y[:, ax]) / np.sum(P * P))
            rms = np.sqrt(np.mean((coef * P - Y[:, ax]) ** 2)) * DEG
            print('  %-8s ppm = %+8.0f   rms %.2f deg' % (nm, coef * 1e6, rms))
    else:
        print('（样本 <2，给出单点比值）')
        for s in rows:
            print('  t0=%d: 倾角 %.2f / 路径角 %.0f = %.0f ppm；偏航 %.2f / |净z角| %.0f = %.0f ppm'
                  % (s['t0'], s['et'], s['path'], 1e6 * s['et'] / max(s['path'], 1e-9),
                     abs(s['nu']), abs(s['th'][2]),
                     1e6 * abs(s['nu']) / max(abs(s['th'][2]), 1e-9)))
    print()
    bmed = np.median(bias[stat], axis=0) if np.any(stat) else np.zeros(3)
    print('静止段 bias 估计 p50 (x,y,z) = (%.4f, %.4f, %.4f) dps -> (%.2f, %.2f, %.2f) deg/h'
          % (bmed[0], bmed[1], bmed[2], bmed[0] * 3600.0, bmed[1] * 3600.0, bmed[2] * 3600.0))
    print('（设计参考：KS_YAW=1000 ppm @ 满量程 2000 dps = 2 dps；bias 残差 1.23 deg/h；'
          'ARW 0.58 deg/rt-h）')


if __name__ == '__main__':
    main()
