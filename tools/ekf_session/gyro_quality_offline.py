# -*- coding: utf-8 -*-
r"""陀螺质量参数：**离线积分法**（不依赖任何固件修正，直接量陀螺自身的误差）

对每个"静止 -> 剧烈运动 -> 静止"的窗口：
  q(t) 从窗口起点的 EKF 四元数出发，用**原始陀螺**逐帧积分（q <- q ⊗ dq_body）；
  终点用静止时的**加速度真值**给倾角误差、用磁航向的**变化量**给偏航误差：
       err_roll/pitch = 终点 normalize(accel_g) 与 积分预测的 up_body 之差（体轴 x/y）
       err_yaw        = (积分偏航变化) - (磁航向变化)      [psi_true_deg，两端都静止]
  同时给出该窗口的分轴净旋转角 theta_i = ∫ w_i dt 与时长 T。
  拟合 err_i = a*theta_i + b_i*T  ->  a = ppm（随旋转角），b = 随时间的零偏误差（deg/s、deg/h）

这样得到的是**陀螺自身的** ppm 与零偏，不含固件牵引/门控的影响。

用法: python tools/ekf_session/gyro_quality_offline.py [R:\imu_xxx.bin ...]
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def qmul(a, b):
    return np.array([
        a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]])


def q_to_R(q):
    w, x, y, z = q / max(np.linalg.norm(q), 1e-12)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))


def integrate(q0, w_rad, dt):
    q = np.array(q0, float)
    for k in range(len(dt)):
        wv = w_rad[k]
        n = float(np.linalg.norm(wv))
        if n < 1e-9:
            continue
        half = 0.5 * n * dt[k]
        dq = np.array([math.cos(half), *(math.sin(half)/n * wv)])
        q = qmul(q, dq)
        q /= np.linalg.norm(q)
    return q


def scan(path, verbose=True):
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
    q = np.asarray(a[:, C['ekf_q0']:C['ekf_q0'] + 4], float)
    psi = np.asarray(a[:, C['psi_true_deg']], float)

    calm = (w < 5.0) & (np.abs(an - 1.0) < 0.03)
    wins = []
    i = 0
    while i < n:
        if calm[i]:
            j = i
            while j < n and calm[j]:
                j += 1
            if j - i >= 167:
                wins.append((i, j))
            i = j
        else:
            i += 1

    rows = []
    for win_i in range(1, len(wins)):
        i0, j0 = wins[win_i - 1]
        i1, j1 = wins[win_i]
        km = j0                      # 运动段起点（静止窗结束）
        t0 = i1                      # 运动段终点（下一个静止窗开始）
        if t0 - km < 300:
            continue
        if abs(an[t0] - 1.0) > 0.03:
            continue
        q0 = q[km - 1]
        qi = integrate(q0, np.radians(g[km:t0]), dt[km:t0])
        # 倾角误差（体轴）：真值 vs 积分预测
        up_pred = q_to_R(qi).T @ np.array([0.0, 0.0, 1.0])
        up_true = acc[t0] / an[t0]
        e = (up_true - up_pred) * RAD2DEG
        # 偏航误差：积分偏航变化 vs 磁航向变化（两端静止）
        dy_int = math.degrees(yaw_of(qi) - yaw_of(q0))
        dy_mag = psi[t0] - psi[km - 1]
        dy_mag = (dy_mag + 180.0) % 360.0 - 180.0
        err_yaw = (dy_int - dy_mag + 180.0) % 360.0 - 180.0
        th = np.array([np.sum(g[km:t0, ax] * dt[km:t0]) for ax in range(3)])
        rows.append(dict(km=km, t0=t0, T=dt[km:t0].sum(), th=th, e=e, ey=err_yaw,
                         span=np.sum(w[km:t0] * dt[km:t0]), wmax=w[km:t0].max(),
                         file=os.path.basename(path), ver=ver))
    if verbose:
        print('== %-24s VER=%3d  可用窗口 %d 个' % (os.path.basename(path), ver, len(rows)))
        print('   %-12s %-6s %-7s %-8s %-24s %-9s %-10s %-9s'
              % ('区间', 'T s', '|w|max', '路径角', '净转角 x / y / z', 'e_x(倾)', 'e_y(倾)',
                 'err_yaw'))
        for s in rows:
            print('   %-12s %-6.1f %-7.0f %-8.0f (%7.0f,%7.0f,%7.0f) %-9.2f %-10.2f %-9.2f'
                  % ('[%d,%d)' % (s['km'], s['t0']), s['T'], s['wmax'], s['span'], s['th'][0],
                     s['th'][1], s['th'][2], s['e'][0], s['e'][1], s['ey']))
    return rows


def main():
    files = sys.argv[1:] or [r'R:\imu_20260922_205255.bin', r'R:\imu_20260922_201451.bin']
    rows = []
    for f in files:
        try:
            rows += scan(f)
        except Exception as ex:
            print('!! %s: %s' % (os.path.basename(f), ex))
        print()
    if len(rows) < 3:
        print('样本 %d 个（不足 3），给出单点比值：' % len(rows))
        for s in rows:
            print('  %s [%d,%d): ppm_x %+.0f ppm_y %+.0f ppm_z %+.0f | 零偏(deg/h) '
                  'x %+.0f y %+.0f z %+.0f'
                  % (s['file'], s['km'], s['t0'],
                     1e6 * s['e'][0] / max(abs(s['th'][0]), 1e-9),
                     1e6 * s['e'][1] / max(abs(s['th'][1]), 1e-9),
                     1e6 * s['ey'] / max(abs(s['th'][2]), 1e-9),
                     s['e'][0] / s['T'] * 3600.0, s['e'][1] / s['T'] * 3600.0,
                     s['ey'] / s['T'] * 3600.0))
        return
    Th = np.array([s['th'] for s in rows])
    T = np.array([s['T'] for s in rows])
    E = np.array([[s['e'][0], s['e'][1], s['ey']] for s in rows])
    print('==== 离线积分法汇总 %d 个窗口：err_i = a*theta_i + b*T ====' % len(rows))
    for ax, nm in enumerate(('倾角 e_x ~ 绕x', '倾角 e_y ~ 绕y', '偏航 err_yaw ~ 绕z')):
        A = np.vstack([Th[:, ax], T]).T
        coef, *_ = np.linalg.lstsq(A, E[:, ax], rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - E[:, ax]) ** 2)))
        print('  %-16s ppm = %+9.0f   零偏 = %+8.4f deg/s (= %+9.1f deg/h)   rms %.2f deg'
              % (nm, coef[0] * 1e6, coef[1], coef[1] * 3600.0, rms))
    print()
    print('单参数（只对相应轴旋转角）：')
    for ax, nm in enumerate(('e_x', 'e_y', 'err_yaw')):
        P = Th[:, ax]
        c = float(np.sum(P * E[:, ax]) / max(np.sum(P * P), 1e-12))
        print('  %-8s ppm = %+9.0f' % (nm, c * 1e6))
    print()
    print('只用时间（零偏）：')
    for ax, nm in enumerate(('e_x', 'e_y', 'err_yaw')):
        c = float(np.sum(T * E[:, ax]) / max(np.sum(T * T), 1e-12))
        print('  %-8s 零偏 = %+8.4f deg/s (= %+8.1f deg/h)' % (nm, c, c * 3600.0))


if __name__ == '__main__':
    main()
