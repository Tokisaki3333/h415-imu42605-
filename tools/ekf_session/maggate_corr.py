# -*- coding: utf-8 -*-
r"""地磁失效门控：先验证"总磁大小 |m| 偏离"与"EKF 角度偏移"的相关性，再定阈值。

用户口径：阈值要选到"磁干扰造成的角度偏移 <= 磁本身的固有误差"（S3 实测 σ_mag ~ 2 deg）。

做法：
 ① 相关性：每帧 x = |mag_norm-1|（总磁大小偏离），y = d(EKF 偏航 - 陀螺积分偏航)/dt
    （= 地磁真正把姿态拽走的速度，dps）。只在"地磁被允许"的帧上统计。
 ② 反事实回放：一阶牵引 psi += (psi_mag - psi)*dt/tau（地磁被允许时），否则纯陀螺积分。
    tau 取 VER=122 的静态闭环时间常数 15.85 s。对每个幅度门 L 回放整段，看
      末态偏移 |psi-psi_gyro| 与全程最大偏移  -> 选出 max <= 2 deg 的最小 L。
用法: python tools/ekf_session/maggate_corr.py R:\imu_20260922_195513.bin [...]
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calib'))
import cols_162 as C
CH = C.CH_162
TAU = 15.85          # VER=122 静态闭环时间常数 (s)
SIG_MAG = 2.0        # 地磁自身固有误差 (deg) —— 目标上界


def yaw_of(q):
    w, x, y, z = q.T
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def unwr(a):
    return np.degrees(np.unwrap(np.radians(a)))


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def main():
    for path in sys.argv[1:]:
        a, _ = C.load_frames(path)
        N = len(a)
        tk = a[:, CH['tick_tk']].astype(np.float64)
        d = np.diff(tk); d[d < 0] += 16777216.0
        dt = d * 1e-6
        dtf = np.concatenate([dt, dt[-1:]])          # 长度 N：dtf[i] = 到第 i 帧为止的间隔
        t = np.concatenate([[0.0], np.cumsum(dt)])
        fps = (N - 1) / t[-1]
        dev = np.abs(a[:, CH['mag_norm']] - 1.0)
        wz = a[:, CH['gyro_dps0'] + 2]
        psi_mag = unwr(a[:, CH['psi_true_deg']])
        psi_ekf = unwr(yaw_of(a[:, CH['ekf_q0']:CH['ekf_q0'] + 4]))
        gi = np.concatenate([[0.0], np.cumsum((wz[1:] + wz[:-1]) * 0.5 * dt)])
        psi_gyro = gi + psi_ekf[0]                     # 陀螺积分航向（同起点）
        err = wrap(psi_ekf - psi_gyro)                 # EKF 相对陀螺的偏移
        rate = np.concatenate([[0.0], np.diff(err) / np.maximum(dt, 1e-9)])
        print('=' * 100)
        print('%s  %.1f s  %.1f Hz' % (os.path.basename(path), t[-1], fps))
        print('  当前固件(gate |mn-1|<0.10)：EKF-陀螺 偏移 末态 %+7.2f°  最大 %6.2f°  路径 Σ|Δ| %.1f°'
              % (err[-1], np.abs(err).max(), np.abs(np.diff(err)).sum()))
        print('  磁航向 psi_mag 路径 Σ|Δ| %.0f°   陀螺路径 Σ|Δ| %.1f°' %
              (np.abs(np.diff(psi_mag)).sum(), np.abs(np.diff(psi_gyro)).sum()))
        # ---- ① 相关性：把 |mn-1| 分箱，看"拽偏航"的速率 ----
        print('  ① 相关性（只在 mag_used=1 的帧）：')
        used = a[:, CH['ekf_mag_used']] > 0.5
        bins = [(0.0, 0.01), (0.01, 0.02), (0.02, 0.05), (0.05, 0.10), (0.10, 0.20), (0.20, 9.9)]
        for lo, hi in bins:
            m = used & (dev >= lo) & (dev < hi)
            if m.sum() < 20:
                print('     |mn-1| in [%.2f,%.2f): 帧 %5d  太少' % (lo, hi, m.sum())); continue
            print('     |mn-1| in [%.2f,%.2f): 帧 %5d (%4.1f%%)  EKF偏移速率 p50 %+7.2f p90 %7.2f dps'
                  '   |偏移|最大后段 %6.2f°' %
                  (lo, hi, m.sum(), 100 * m.mean(), np.percentile(rate[m], 50),
                   np.percentile(np.abs(rate[m]), 90), np.abs(err[m]).max()))
        if used.sum() > 100:
            x = dev[used]; y = np.abs(rate[used])
            r = np.corrcoef(x, y)[0, 1]
            print('     Pearson r(|mn-1|, |EKF偏移速率|) = %+.3f   (n=%d)' % (r, used.sum()))
        # ---- ② 反事实回放：不同幅度门 ----
        print('  ② 反事实回放（一阶牵引 tau=%.2fs，门 = |mn-1| < L）：' % TAU)
        for L in (0.01, 0.015, 0.02, 0.03, 0.05, 0.10, 9.9):
            psi = psi_ekf[0]
            mx = 0.0
            for i in range(1, N):
                psi += wz[i] * dtf[i]
                if dev[i] < L:
                    psi += wrap(psi_mag[i] - psi) * dtf[i] / TAU
                e = abs(wrap(psi - psi_gyro[i]))
                if e > mx:
                    mx = e
            tag = '  <== 最大偏移 <= 2°' if mx <= SIG_MAG else ''
            print('     L=%.3f: 末态偏移 %+7.2f°  最大偏移 %6.2f°  允许帧占比 %5.1f%%%s'
                  % (L, wrap(psi - psi_gyro[-1]), mx, 100 * np.mean(dev < L), tag))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
