# -*- coding: utf-8 -*-
r"""零漂溯源（600 s 静止录像）：把偏航/倾角的漂移拆成"地磁侧游走"与"估计器侧问题"。

判据（静止、无转动）：
  A 地磁侧   : 体坐标系下磁场水平方向的变化（用**固定**姿态把 mag 矢量投到导航系）
               —— 设备没动，磁场方向变了 ⇒ 外部磁场/温度/软硬铁漂移（"地磁误差内的游走"）
  B 估计器侧 : EKF 偏航 ψ_ekf 与 地磁指示航向之差（= 新息 ν）随时间的**趋势**
               —— ν 不涨 ⇒ 滤波器跟着地磁走（漂移来自 A）；ν 单调涨 ⇒ 估计器自身问题
  C 陀螺侧   : ∫w dt（未补偿积分）与 ekf.bg（131 后应已收敛）的趋势
  D 倾角侧   : 体坐标系下重力方向的变化 vs EKF up 与加速度计之差（静差/漂移）

用法: python tools/ekf_session/drift_budget.py [R:\imu_xxx.bin]
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def q_to_R(q):
    w, x, y, z = q / max(np.linalg.norm(q), 1e-12)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260923_060100.bin'
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    n = len(a)
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    t = np.cumsum(dt)
    g = np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float)
    w = np.linalg.norm(g, axis=1)
    acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
    an = np.linalg.norm(acc, axis=1)
    m = np.asarray(a[:, C['mag_lsb0']:C['mag_lsb0'] + 3], float)
    mn = np.linalg.norm(m, axis=1)
    q = np.asarray(a[:, C['ekf_q0']:C['ekf_q0'] + 4], float)
    up = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
    thm = np.asarray(a[:, C['ekf_mag_cmp_thm']], float)
    thp = np.asarray(a[:, C['ekf_mag_cmp_thp']], float)
    nu = (thm - thp + 180.0) % 360.0 - 180.0
    psi = np.asarray(a[:, C['psi_true_deg']], float)
    bg = np.asarray(a[:, C['ekf_bg0']:C['ekf_bg0'] + 3], float)
    ba = np.asarray(a[:, C['ekf_ba0']:C['ekf_ba0'] + 3], float)
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    gm = np.asarray(a[:, C['gate_ekf_mag_yaw']], float)
    pyy = np.asarray(a[:, C['ekf_p_yy']], float)
    sy = np.asarray(a[:, C['ekf_sigma_yaw']], float)
    mr = np.asarray(a[:, C['ekf_mag_r_deg']], float)
    use = np.asarray(a[:, C['ekf_mag_used']], float)
    mnorm = np.asarray(a[:, C['mag_norm']], float) if 'mag_norm' in C else np.ones(n)

    R0 = q_to_R(q[0])                       # t=0 的姿态（静止，固定用它投影）
    mh = m / np.maximum(mn, 1e-9)[:, None]
    mnav = mh @ R0.T                        # 体->导航（固定姿态）
    psi_field = np.degrees(np.arctan2(mnav[:, 1], mnav[:, 0]))   # 磁场水平方向（导航系）
    psi_field = np.unwrap(psi_field) - psi_field[0]
    upnav = acc / np.maximum(an, 1e-9)[:, None] @ R0.T
    tilt_field = np.degrees(np.arccos(np.clip(upnav[:, 2], -1, 1)))

    psi_ekf = np.unwrap(np.array([yaw_of(q[k]) for k in range(n)]))
    psi_ekf = np.degrees(psi_ekf - psi_ekf[0])
    psi_ekf[psi_ekf > 180.0] -= 360.0 * np.round(psi_ekf[psi_ekf > 180.0] / 360.0)
    psi_true = psi - psi[0]
    gyro_int = np.cumsum(g[:, 2] * dt)      # z 轴未补偿积分（deg）
    tilt_res = np.degrees(np.linalg.norm(acc / np.maximum(an, 1e-6)[:, None] - up, axis=1))

    print('录像 %s  VER=%d  %.0f s  帧 %d' % (os.path.basename(path), ver, t[-1], n))
    print('静止占比(|w|<5dps) %.1f%%   |a| p50 %.4f g   |m| 均值 %.1f  |m| 变化 %+.2f%%'
          % (100 * np.mean(w < 5.0), np.median(an), mn.mean(),
             100 * (mn[-1] / mn[0] - 1)))
    print('ekf.bg 首/末 (dps) = (%+.5f,%+.5f,%+.5f) / (%+.5f,%+.5f,%+.5f)'
          % (bg[0, 0], bg[0, 1], bg[0, 2], bg[-1, 0], bg[-1, 1], bg[-1, 2]))
    print('ekf.ba 首/末 (g)   = (%+.5f,%+.5f,%+.5f) / (%+.5f,%+.5f,%+.5f)'
          % (ba[0, 0], ba[0, 1], ba[0, 2], ba[-1, 0], ba[-1, 1], ba[-1, 2]))
    print()
    print('%-8s %-10s %-10s %-10s %-10s %-10s %-9s %-9s' %
          ('t(min)', 'A 磁场航向', 'B EKF偏航', 'B 新息ν', 'C 陀螺积分', 'D 倾角静差',
           'p_yy', 'sigma_yaw'))
    for s in range(0, int(t[-1]) + 1, 60):
        k = np.searchsorted(t, s)
        k = min(k, n - 1)
        print('%-8d %-10.2f %-10.2f %-10.2f %-10.2f %-10.2f %-9.3g %-9.2f'
              % (s // 60, psi_field[k], psi_ekf[k], nu[k], gyro_int[k], tilt_res[k],
                 pyy[k], sy[k]))
    k = n - 1
    print()
    print('600 s 总账：')
    print('  A 磁场水平方向漂移（固定姿态投影）  = %+.2f deg  = %+.1f deg/h'
          % (psi_field[k], psi_field[k] / t[k] * 3600))
    print('  B EKF 偏航漂移                     = %+.2f deg  = %+.1f deg/h'
          % (psi_ekf[k], psi_ekf[k] / t[k] * 3600))
    print('    新息 ν 首/末                     = %+.2f / %+.2f deg（趋势 %+.2f deg）'
          % (nu[0], nu[k], nu[k] - nu[0]))
    print('  C 陀螺 z 未补偿积分                = %+.1f deg  = %+.0f deg/h；ekf.bg_z 末 %+.4f dps'
          % (gyro_int[k], gyro_int[k] / t[k] * 3600, bg[k, 2]))
    print('  D 倾角静差 首/末                   = %.2f / %.2f deg' % (tilt_res[0], tilt_res[k]))
    print('    磁场水平方向 vs EKF偏航 之差      = %+.2f deg'
          % (psi_field[k] - psi_ekf[k]))
    print('  门 T/M 占比 %.0f%% / %.0f%%   mag_used %.0f%%   |m| 变化 %+.2f%%'
          % (100 * np.mean(gt > 0.5), 100 * np.mean(gm > 0.5), 100 * np.mean(use > 0.5),
             100 * (mn[-1] / mn[0] - 1)))
    print('  mag_norm 首/末 %.4f / %.4f   mag_R(deg) p50 %.2f'
          % (mnorm[0], mnorm[k], np.median(mr)))
    print()
    print('判读：A≈B 且 ν 不涨 ⇒ 漂移在**地磁误差内的游走**；')
    print('      B 明显大于 A 或 ν 单调增大 ⇒ **估计器/陀螺侧的真问题**。')


if __name__ == '__main__':
    main()
