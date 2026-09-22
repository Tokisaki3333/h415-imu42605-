# -*- coding: utf-8 -*-
r"""新录像(静止 + 外部磁干扰)探查：地磁把姿态拽走多少、门控在哪几列上看得见。
用法: python tools/ekf_session/magdist_probe.py R:\imu_20260922_195513.bin [R:\..._195551.bin]
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'calib'))
import cols_162 as C

C_ = C.CH_162


def yaw_of(q):
    w, x, y, z = q.T
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def tilt_up(q):
    w, x, y, z = q.T
    return np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], 1)


def unwrap(a):
    return np.degrees(np.unwrap(np.radians(a)))


def main():
    for path in sys.argv[1:]:
        a, info = C.load_frames(path)
        N = len(a)
        t = np.arange(N) / (N / (N * 0.0 + 1))  # placeholder
        tk = a[:, C_['tick_tk']].astype(np.float64)
        # tick 是 1us 低位 24 位 -> 相对时间
        dt = np.diff(tk)
        dt[dt < 0] += 16777216.0
        t = np.concatenate([[0.0], np.cumsum(dt)]) * 1e-6
        fps = (N - 1) / t[-1] if t[-1] > 0 else 0
        print('=' * 96)
        print('%s  %d 帧  %.2f s  %.1f Hz  fw_tag=%.0f (VER=%d ch=%d)'
              % (os.path.basename(path), N, t[-1], fps, np.median(a[:, C_['fw_tag']]),
                 int(np.median(a[:, C_['fw_tag']])) >> 16,
                 (int(np.median(a[:, C_['fw_tag']])) >> 8) & 0xFF))

        g = np.degrees(np.linalg.norm(a[:, C_['gyro_dps0']:C_['gyro_dps0'] + 3], axis=1))
        print('陀螺 |w| dps: p50 %.3f p90 %.3f p99 %.3f max %.3f   (>15dps 占比 %.1f%%)'
              % (np.percentile(g, 50), np.percentile(g, 90), np.percentile(g, 99), g.max(),
                 100 * np.mean(g > 15)))
        mf = a[:, C_['mag_f0']:C_['mag_f0'] + 3]
        mn = a[:, C_['mag_norm']]
        print('mag_norm: p50 %.4f  p01 %.4f p99 %.4f   |mag_norm-1|>0.10 占比 %.1f%%'
              % (np.median(mn), np.percentile(mn, 1), np.percentile(mn, 99),
                 100 * np.mean(np.abs(mn - 1) > 0.10)))
        print('|mag_f|: p50 %.4f p99 %.4f' % (np.median(np.linalg.norm(mf, axis=1)),
                                              np.percentile(np.linalg.norm(mf, axis=1), 99)))
        # 门控占空比
        for nm, col in (('gate_ekf_mag_yaw', 155), ('mag_ok', 154), ('ekf_mag_used', 120),
                        ('gate_ekf_tilt', 156), ('ekf_grav_ok', 152), ('ekf_mag_hold', 151)):
            v = a[:, C_[nm]]
            print('  %-18s 均值 %.3f  非零占比 %.1f%%' % (nm, v.mean(), 100 * np.mean(v > 0.5)))
        # 磁残差与累加器
        for nm in ('ekf_mag_r_deg', 'ekf_mag_rs', 'ekf_nis4', 'ekf_mag_rej',
                   'ekf_mag_dqx', 'ekf_mag_dqy', 'ekf_mag_dqz',
                   'ekf_mag_cmp_thm', 'ekf_mag_cmp_thp', 'ekf_mag_cmp_amn', 'ekf_mag_cmp_mhn'):
            v = a[:, C_[nm]]
            print('  %-18s p50 %+9.3f p99 %+9.3f min %+9.3f max %+9.3f' %
                  (nm, np.median(v), np.percentile(v, 99), v.min(), v.max()))
        # 模式反推
        dqx, dqy, dqz = a[:, 135], a[:, 136], a[:, 137]
        used = a[:, 120] > 0.5
        A = used & (np.abs(dqx) < 1e-9) & (np.abs(dqy) < 1e-9) & (np.abs(dqz) > 0)
        B = used & ((np.abs(dqx) > 0) | (np.abs(dqy) > 0))
        print('  模式反推: used %.1f%%  -> 入口A %.1f%%  入口B %.1f%%  两者都不是 %.1f%%'
              % (100 * used.mean(), 100 * A.mean(), 100 * B.mean(),
                 100 * (used & ~A & ~B).mean()))
        # 姿态：EKF vs 纯陀螺积分（静止时陀螺应≈0）
        q = a[:, C_['ekf_q0']:C_['ekf_q0'] + 4]
        yaw = unwrap(yaw_of(q))
        up = tilt_up(q)
        tilt = np.degrees(np.arccos(np.clip(up[:, 2], -1, 1)))
        print('  EKF 偏航: 起 %.3f 末 %.3f  变化 %+.3f°  |  路径 Σ|Δyaw| %.2f°'
              % (yaw[0], yaw[-1], yaw[-1] - yaw[0], np.abs(np.diff(yaw)).sum()))
        print('  EKF 倾角: 起 %.3f 末 %.3f  变化 %+.3f°  |  最大 %.3f°'
              % (tilt[0], tilt[-1], tilt[-1] - tilt[0], tilt.max()))
        # 世界系磁场方向（mag_Bw）
        bw = a[:, C_['mag_Bw']:C_['mag_Bw'] + 3]
        azi = unwrap(np.degrees(np.arctan2(bw[:, 0], bw[:, 1])))
        print('  世界系磁场方位(mag_Bw): 起 %.3f 末 %.3f 变化 %+.3f°  | 路径 Σ|Δ| %.2f°  | psi_true 变化 %+.3f°'
              % (azi[0], azi[-1], azi[-1] - azi[0], np.abs(np.diff(azi)).sum(),
                 unwrap(a[:, C_['psi_true_deg']])[-1] - unwrap(a[:, C_['psi_true_deg']])[0]))
        # 陀螺积分偏航（z）
        wz = a[:, C_['gyro_dps0'] + 2]
        gi = np.concatenate([[0], np.cumsum((wz[1:] + wz[:-1]) * 0.5 * np.diff(t))])
        print('  陀螺积分偏航 Δ = %+.3f°   vs EKF Δyaw = %+.3f°   vs 磁场方位 Δ = %+.3f°'
              % (gi[-1], yaw[-1] - yaw[0], azi[-1] - azi[0]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
