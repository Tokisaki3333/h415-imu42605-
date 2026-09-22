# -*- coding: utf-8 -*-
r"""陀螺质量参数（v3，最终口径）：

"误差应当取重力开始有效牵引前的一瞬间的残差" —— 即**运动后第一个被信任的重力观测帧 tg**
上、修正发生**之前**的残差（调试帧里 tg 帧的 prx/pry/prz 就是修正前的预测 up）：

  倾角误差  e = normalize(accel_g)(tg) - up_pred(tg)      [与 M6 内部用的创新量一致]
            e_x ≈ 绕 body y（俯仰）、e_y ≈ 绕 body x（横滚）的小角分量
  偏航误差  nu = wrap(thm - thp)(tg)                       [M7 的创新量]

为什么不用 q(tg-1)->q(稳定) 之差：把设备放回台座时**真实朝向会变**，
那个差里混进了真实旋转（实测会得到 20~85 deg 的假残差，见旧版输出）。
用"同一帧的真值 - 同帧的预测"就没有这个问题。

旋转角 θ_i = ∫ w_i dt 覆盖 tg 之前的**门关段**（含整个大运动 + 放回过程），T = 段时长。
拟合 err = a*θ + b*T  ->  a = ppm，b = 随时间的误差。

用法: python tools/ekf_session/quality_params3.py [R:\imu_xxx.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def scan(path, verbose=True, min_closed=0.5):
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
    w = np.linalg.norm(g, axis=1)
    acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
    an = np.linalg.norm(acc, axis=1)
    up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
    thm = np.asarray(a[:, C['ekf_mag_cmp_thm']], float)
    thp = np.asarray(a[:, C['ekf_mag_cmp_thp']], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    gm = np.asarray(a[:, C['gate_ekf_mag_yaw']], float)
    op = gt > 0.5
    rows = []
    i = 0
    while i < n:
        if not op[i] and i > 0:
            j = i
            while j < n and not op[j]:
                j += 1
            if dt[i:j].sum() >= min_closed and j < n:
                tg = j
                if abs(an[tg] - 1.0) <= 0.03:
                    e = (acc[tg] / an[tg] - up[tg]) * RAD2DEG
                    # 修正后的残差（tg+2 s）用于确认牵引确实发生
                    k2 = min(n - 1, tg + 670)
                    e2 = (acc[k2] / max(an[k2], 1e-6) - up[k2]) * RAD2DEG
                    nu = (thm[tg] - thp[tg] + 180.0) % 360.0 - 180.0
                    th = np.array([np.sum(g[i:tg, ax] * dt[i:tg]) for ax in range(3)])
                    rows.append(dict(tg=tg, T=dt[i:tg].sum(), th=th, e=e, e2=e2, nu=nu,
                                     span=np.sum(w[i:tg] * dt[i:tg]), wmax=w[i:tg].max(),
                                     gtduty=100.0 * float(np.mean(op[i:tg])),
                                     gmduty=100.0 * float(np.mean(gm[i:tg] > 0.5)),
                                     ver=ver, file=os.path.basename(path)))
            i = j
        else:
            i += 1
    if verbose:
        print('== %-24s VER=%3d  门关段(>=%.1fs) %d 个' %
              (os.path.basename(path), ver, min_closed, len(rows)))
        print('   %-10s %-6s %-7s %-8s %-24s %-8s %-8s %-8s %-8s %-8s'
              % ('tg帧', 'T s', '|w|max', '路径角', '净转角 x/y/z', 'e_x', 'e_y', 'e2_x',
                 'e2_y', 'nu_yaw'))
        for s in rows:
            print('   %-10d %-6.1f %-7.0f %-8.0f (%7.0f,%7.0f,%7.0f) %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f'
                  % (s['tg'], s['T'], s['wmax'], s['span'], s['th'][0], s['th'][1], s['th'][2],
                     s['e'][0], s['e'][1], s['e2'][0], s['e2'][1], s['nu']))
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
    print('合计 %d 个门关段，其中 |θ|>500 deg 的 %d 个' % (len(rows), len(big)))
    if len(big) < 2:
        return
    Th = np.array([r['th'] for r in big]); T = np.array([r['T'] for r in big])
    E = np.array([[r['e'][0], r['e'][1]] for r in big])
    print()
    print('==== 大运动样本 %d 个：err = a*θ + b*T ====' % len(big))
    for ax, nm in enumerate(('e_x ~ θx', 'e_y ~ θy')):
        A = np.vstack([Th[:, ax], T]).T
        coef, *_ = np.linalg.lstsq(A, E[:, ax], rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - E[:, ax]) ** 2)))
        cc = float(np.corrcoef(Th[:, ax], E[:, ax])[0, 1]) if np.std(E[:, ax]) > 0 else 0.0
        print('  %-10s ppm = %+8.0f  时间项 = %+7.4f deg/s (= %+8.1f deg/h)  rms %.2f  corr %+.2f'
              % (nm, coef[0] * 1e6, coef[1], coef[1] * 3600.0, rms, cc))
    print()
    print('单参数（只用相应轴旋转角）：')
    for ax, nm in enumerate(('e_x', 'e_y')):
        P = Th[:, ax]
        c = float(np.sum(P * E[:, ax]) / max(np.sum(P * P), 1e-12))
        rms = float(np.sqrt(np.mean((c * P - E[:, ax]) ** 2)))
        print('  %-6s ppm = %+8.0f  rms %.2f deg' % (nm, c * 1e6, rms))
    print()
    print('所有门关段（含小运动）的详情已在上面各录像表中。')


if __name__ == '__main__':
    main()
