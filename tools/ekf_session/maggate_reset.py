# -*- coding: utf-8 -*-
r"""地磁失效门控重新设定：用"静止 + 外部磁干扰"新录像量化三件事
  ① |mag_norm-1| 的分布，以及"收紧幅度门"能把方向残差压到多少（幅度门够不够）
  ② 陀螺（积分偏航）vs 磁航向 vs EKF 偏航 —— 三者的发散量（陀螺一致性门的分离度）
  ③ 干扰的时间尺度（多少秒涨到多少度）-> 门控窗口与阈值
用法: python tools/ekf_session/maggate_reset.py R:\imu_20260922_195513.bin [...]
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calib'))
import cols_162 as C

CH = C.CH_162


def yaw_of(q):
    w, x, y, z = q.T
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def unwr(a):
    return np.degrees(np.unwrap(np.radians(a)))


def pct(v):
    return ' '.join('%7.2f' % x for x in np.percentile(v, [50, 90, 99, 100]))


def main():
    for path in sys.argv[1:]:
        a, info = C.load_frames(path)
        N = len(a)
        tk = a[:, CH['tick_tk']].astype(np.float64)
        d = np.diff(tk); d[d < 0] += 16777216.0
        t = np.concatenate([[0.0], np.cumsum(d)]) * 1e-6
        mn = a[:, CH['mag_norm']]
        res = a[:, CH['ekf_mag_r_deg']]          # 磁残差角(deg)
        gz = a[:, CH['gyro_dps0'] + 2]
        q = a[:, CH['ekf_q0']:CH['ekf_q0'] + 4]
        yaw = unwr(yaw_of(q))
        psit = unwr(a[:, CH['psi_true_deg']])    # 固件算的"磁真北航向"
        gi = np.concatenate([[0.0], np.cumsum((gz[1:] + gz[:-1]) * 0.5 * d * 1e-6)])
        dev = np.abs(mn - 1.0)
        print('=' * 100)
        print('%s  %d 帧 %.2f s  VER=%d' % (os.path.basename(path), N, t[-1],
                                           int(np.median(a[:, CH['fw_tag']])) >> 16))
        print('① 幅度门：|mag_norm-1| p50 %.4f p90 %.4f p99 %.4f max %.4f  (>0.05 %.1f%% >0.10 %.1f%%)'
              % (np.percentile(dev, 50), np.percentile(dev, 90), np.percentile(dev, 99), dev.max(),
                 100 * np.mean(dev > 0.05), 100 * np.mean(dev > 0.10)))
        print('   残差 ekf_mag_r_deg  p50/p90/p99/max: %s' % pct(res))
        print('   ---- 收紧幅度门后的残差（若收紧后残差仍大 => 幅度门治不了）----')
        for lim in (0.02, 0.05, 0.10, 0.20, 1.0):
            m = dev < lim
            if m.sum() < 50:
                print('   |mag_norm-1|<%.2f : 帧数 %5d  太少' % (lim, m.sum())); continue
            print('   |mag_norm-1|<%.2f : 帧 %5d (%.1f%%)  残差 p50 %6.2f p90 %6.2f p99 %6.2f'
                  % (lim, m.sum(), 100 * m.mean(), np.percentile(res[m], 50),
                     np.percentile(res[m], 90), np.percentile(res[m], 99)))
        print('② 陀螺 vs 磁航向 vs EKF 偏航（全程变化量）')
        print('   陀螺 w_z: 均值 %+.3f dps  std %.3f  积分偏航 Δ %+.2f°  |  积分路径 Σ|Δ| %.2f°'
              % (gz.mean(), gz.std(), gi[-1], np.abs(np.diff(gi)).sum()))
        print('   磁真北航向 psi_true: Δ %+.2f°   路径 Σ|Δ| %.2f°' % (psit[-1] - psit[0], np.abs(np.diff(psit)).sum()))
        print('   EKF 偏航: Δ %+.2f°   路径 Σ|Δ| %.2f°' % (yaw[-1] - yaw[0], np.abs(np.diff(yaw)).sum()))
        print('   ---- 陀螺一致性门的分离度：窗口 T 内 |Δpsi_mag - Δpsi_gyro| ----')
        for T in (1.0, 2.0, 5.0, 10.0):
            k = max(1, int(round(T * (N / t[-1]))))
            dpsi = psit[k:] - psit[:-k]
            dgi = gi[k:] - gi[:-k]
            e = np.abs((dpsi - dgi + 180) % 360 - 180)
            print('   T=%4.0fs (%3d 帧): |Δpsi_mag-Δpsi_gyro| p50 %6.2f p90 %6.2f p99 %6.2f max %6.2f'
                  % (T, k, np.percentile(e, 50), np.percentile(e, 90), np.percentile(e, 99), e.max()))
        print('③ 干扰时间尺度（世界系磁场方位变化率，用 psi_true）')
        r = np.abs(np.diff(psit)) / np.diff(t)
        print('   |dpsi_true/dt| dps: p50 %.2f p90 %.2f p99 %.2f max %.2f' %
              (np.percentile(r, 50), np.percentile(r, 90), np.percentile(r, 99), r.max()))
        # EKF 偏航是否在跟着磁走
        print('   |d(EKF yaw)/dt| dps: p50 %.3f p90 %.3f max %.3f'
              % (np.percentile(np.abs(np.diff(yaw)) / np.diff(t), 50),
                 np.percentile(np.abs(np.diff(yaw)) / np.diff(t), 90),
                 (np.abs(np.diff(yaw)) / np.diff(t)).max()))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
