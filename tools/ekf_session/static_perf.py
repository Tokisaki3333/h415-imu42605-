# -*- coding: utf-8 -*-
r"""静止性能评估（长静止录像）：陀螺零偏/ARW/Allan、加速度计噪声、EKF 静止精度、门与磁使用率。

用法: python tools/ekf_session/static_perf.py [R:\imu_xxx.bin]
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def allan(x, dt, taus):
    out = []
    for T in taus:
        n = max(1, int(round(T / dt)))
        m = (len(x) // n) * n
        if m < 2 * n:
            out.append(float('nan'))
            continue
        b = x[:m].reshape(-1, n).mean(axis=1)
        d = np.diff(b)
        out.append(float(np.sqrt(0.5 * np.mean(d * d))))
    return out


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260922_211657.bin'
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
    gm = np.asarray(a[:, C['gate_ekf_mag_yaw']], float)
    sig_yaw = np.asarray(a[:, C['ekf_sigma_yaw']], float)
    sig_tilt = np.asarray(a[:, C['ekf_sigma_tilt_deg']], float)
    pyy = np.asarray(a[:, C['ekf_p_yy']], float)
    magr = np.asarray(a[:, C['ekf_mag_r_deg']], float)
    used = np.asarray(a[:, C['ekf_mag_used']], float)
    rej = np.asarray(a[:, C['ekf_mag_rej']], float)
    q = np.asarray(a[:, C['ekf_q0']:C['ekf_q0'] + 4], float)
    psi = np.asarray(a[:, C['psi_true_deg']], float)

    calm = (w < 5.0) & (np.abs(an - 1.0) < 0.03)
    print('录像 %s  VER=%d  %.1f s  帧数 %d' % (os.path.basename(path), ver, t[-1], n))
    print('静止占 %.1f%%（|w|<5 dps 且 ||a|-1|<0.03）' % (100.0 * calm.mean()))
    print()
    # 最长静止段
    segs = []
    i = 0
    while i < n:
        if calm[i]:
            j = i
            while j < n and calm[j]:
                j += 1
            if dt[i:j].sum() >= 5.0:
                segs.append((i, j))
            i = j
        else:
            i += 1
    segs.sort(key=lambda s: -(s[1] - s[0]))
    print('>=5 s 的静止段 %d 个；最长的 3 个：%s'
          % (len(segs), ', '.join('%.0f s' % dt[i:j].sum() for i, j in segs[:3])))
    print()
    for (i, j) in segs[:2]:
        sg = g[i:j]
        T = dt[i:j].sum()
        dts = float(np.median(dt[i:j]))
        r = np.degrees(np.linalg.norm(acc[i:j] / an[i:j, None] - up[i:j], axis=1))
        nu = (thm[i:j] - thp[i:j] + 180.0) % 360.0 - 180.0
        yaw = np.array([yaw_of(q[k]) for k in range(i, j)])
        yaw = np.unwrap(yaw)
        yaw_drift = (yaw[-1] - yaw[0]) / T * 3600.0 * RAD2DEG
        # 1 s 平均后 sigma（ARW 估算）
        n1 = max(1, int(round(1.0 / dts)))
        m = (len(sg) // n1) * n1
        sd1 = np.std(sg[:m].reshape(-1, n1).mean(axis=1), axis=0) if m >= 2 * n1 else np.zeros(3)
        sd1 = np.atleast_1d(sd1).reshape(-1)[:3]
        if sd1.size < 3:
            sd1 = np.zeros(3)
        al = allan(sg[:, 0], dts, (1.0, 10.0, 100.0))
        print('--- 静止段 [%.1f, %.1f) s  时长 %.1f s' % (t[i], t[j - 1], T))
        print('  陀螺均值 (bias)     = (%+8.4f, %+8.4f, %+8.4f) dps  = (%+7.1f, %+7.1f, %+7.1f) deg/h'
              % (sg[:, 0].mean(), sg[:, 1].mean(), sg[:, 2].mean(),
                 sg[:, 0].mean() * 3600, sg[:, 1].mean() * 3600, sg[:, 2].mean() * 3600))
        print('  陀螺原始 sigma      = (%.4f, %.4f, %.4f) dps' % tuple(sg.std(axis=0)))
        print('  1 s 平均后 sigma    = (%.4f, %.4f, %.4f) dps -> ARW = (%.2f, %.2f, %.2f) deg/rt-h'
              % (sd1[0], sd1[1], sd1[2], sd1[0] * 60, sd1[1] * 60, sd1[2] * 60))
        print('  Allan(x) tau=1/10/100 s = %.4f / %.4f / %.4f dps' % tuple(al))
        print('  加速度计 |a| 均值 %.4f g, sigma (%.4f, %.4f, %.4f) g'
              % (an[i:j].mean(), *acc[i:j].std(axis=0)))
        print('  倾角残差 |acc-uppred|: p50 %.2f  p95 %.2f  max %.2f deg' %
              (np.percentile(r, 50), np.percentile(r, 95), r.max()))
        print('  偏航创新 nu=thm-thp : p50 %+.3f  std %.3f  p95 %.3f deg' %
              (np.median(nu), nu.std(), np.percentile(np.abs(nu), 95)))
        print('  EKF 偏航漂移 %.2f deg/h（段内 unwrap 端点差）' % yaw_drift)
        print('  sigma_yaw(col104) %.3f deg | sigma_tilt(col107) %.3f deg | p_yy(col121) %.3g'
              % (np.median(sig_yaw[i:j]), np.median(sig_tilt[i:j]), np.median(pyy[i:j])))
        print('  mag_R(col119) %.2f deg | 门T %.0f%% | 门M %.0f%% | mag_used %.0f%% | rej 末 %.0f'
              % (np.median(magr[i:j]), 100 * np.mean(gt[i:j] > 0.5), 100 * np.mean(gm[i:j] > 0.5),
                 100 * np.mean(used[i:j] > 0.5), rej[j - 1]))
        print()


if __name__ == '__main__':
    main()
