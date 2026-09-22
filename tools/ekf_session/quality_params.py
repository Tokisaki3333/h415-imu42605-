# -*- coding: utf-8 -*-
r"""陀螺质量参数提取（用户口径）—— "静止后重力/地磁的牵引量 = 运动期间积攒的误差角"

做法（不依赖具体固件版本，只依赖 M6/M7 会把误差拉掉这一事实）：
  静止窗：|w| < 5 dps 且 ||a|-1| < 0.03 且两门皆开，连续 >= 0.5 s
  t0     ：静止窗里**重力门第一次打开**的帧（= 重力开始有效牵引的瞬间）
  误差   ：err = 四元数 q(t0-1) -> q(窗尾+2 s) 的相对旋转矢量（导航系）
               err_roll  ~ 绕 x           （重力牵引量）
               err_pitch ~ 绕 y
               err_yaw   ~ 绕 z           （地磁牵引量）
  旋转角 ：运动窗 = 上一静止窗结束 -> t0，分轴 theta_i = ∫ w_i dt (deg)，时长 T
  拟合   ：err_i = a*theta_i + b*T   ->  a = ppm（随旋转角），b = 随时间的误差 (deg/s, deg/h)

用法: python tools/ekf_session/quality_params.py [R:\imu_a.bin R:\imu_b.bin ...]
"""
import sys, os, math, glob
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def qrel_rotvec(qb, qa):
    """导航系相对旋转矢量（deg）：qa = dq ⊗ qb，返回 dq 的旋转矢量。"""
    qb = np.asarray(qb, float); qa = np.asarray(qa, float)
    qb = qb / max(np.linalg.norm(qb), 1e-12); qa = qa / max(np.linalg.norm(qa), 1e-12)
    # dq = qa * conj(qb)  （同一约定：world = q ⊗ body）
    wb, xb, yb, zb = qb
    d = np.array([
        qa[0] * wb + qa[1] * xb + qa[2] * yb + qa[3] * zb,
        -qa[0] * xb + qa[1] * wb - qa[2] * zb + qa[3] * yb,
        -qa[0] * yb + qa[1] * zb + qa[2] * wb - qa[3] * xb,
        -qa[0] * zb - qa[1] * yb + qa[2] * xb + qa[3] * wb])
    d = d / max(np.linalg.norm(d), 1e-12)
    if d[0] < 0.0:
        d = -d
    v = d[1:]
    nv = np.linalg.norm(v)
    ang = 2.0 * math.atan2(nv, d[0]) * RAD2DEG
    if nv < 1e-12:
        return np.zeros(3)
    return v / nv * ang


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
    gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
    gm = np.asarray(a[:, C['gate_ekf_mag_yaw']], float)

    calm = (w < 5.0) & (np.abs(an - 1.0) < 0.03)
    # 静止窗（连续 >= 0.5 s ≈ 167 帧）
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
    prev_end = 0
    for (i, j) in wins:
        km = prev_end
        prev_end = j
        if i - km < 100:
            continue
        t0 = None
        for p in range(i, min(n, i + 400)):
            if gt[p] > 0.5 and gm[p] > 0.5:
                t0 = p
                break
        if t0 is None or t0 < 2:
            continue
        tend = min(n - 1, j + 667)                  # 窗尾再等 2 s 让牵引收敛
        err = qrel_rotvec(q[t0 - 1], q[tend])
        dur = dt[km:t0].sum()
        tclosed = float(np.sum(dt[km:t0][gt[km:t0] < 0.5]))
        th = np.array([np.sum(g[km:t0, ax] * dt[km:t0]) for ax in range(3)])
        span = np.sum(w[km:t0] * dt[km:t0])
        rows.append(dict(t0=t0, dur=dur, tc=tclosed, th=th, path=span, err=err,
                         wmax=w[km:t0].max(), n=n, ver=ver,
                         file=os.path.basename(path)))
    if verbose:
        print('== %-26s VER=%3d  %.0f s  静止窗 %d 个 -> 可用切换 %d 个'
              % (os.path.basename(path), ver, dt.sum(), len(wins), len(rows)))
        print('   %-9s %-6s %-7s %-8s %-26s %-9s %-9s %-9s'
              % ('t0帧', 'T s', '|w|max', '路径角', '净转角 x / y / z', 'err_roll', 'err_pitch',
                 'err_yaw'))
        for s in rows:
            print('   %-9d %-6.1f %-7.0f %-8.0f (%7.0f,%7.0f,%7.0f) %-9.2f %-9.2f %-9.2f'
                  % (s['t0'], s['dur'], s['wmax'], s['path'], s['th'][0], s['th'][1],
                     s['th'][2], s['err'][0], s['err'][1], s['err'][2]))
    return rows


def main():
    args = sys.argv[1:]
    files = args if args else sorted(glob.glob(r'R:\imu_2026*.bin'))
    allrows = []
    for f in files:
        try:
            allrows += scan(f)
        except Exception as e:
            print('!! %s: %s' % (os.path.basename(f), e))
        print()
    if len(allrows) < 3:
        print('样本不足（%d），给出单点比值：' % len(allrows))
        for s in allrows:
            print('  %s t0=%d: err=(%+.2f,%+.2f) / 净角(x %.0f, y %.0f) -> ppm_x %+.0f ppm_y %+.0f'
                  % (s['file'], s['t0'], s['err'][0], s['err'][1], s['th'][0], s['th'][1],
                     1e6 * s['err'][0] / max(abs(s['th'][0]), 1e-9),
                     1e6 * s['err'][1] / max(abs(s['th'][1]), 1e-9)))
        return
    Th = np.array([s['th'] for s in allrows])
    T = np.array([s['tc'] for s in allrows])
    E = np.array([s['err'] for s in allrows])
    print('==== 汇总 %d 个样本，拟合 err_i = a*theta_i + b*T_closed（重力门关时长）====' % len(allrows))
    for ax, nm in enumerate(('err_roll  ~ 绕x', 'err_pitch ~ 绕y', 'err_yaw   ~ 绕z')):
        A = np.vstack([Th[:, ax], T]).T
        coef, *_ = np.linalg.lstsq(A, E[:, ax], rcond=None)
        rms = float(np.sqrt(np.mean((A @ coef - E[:, ax]) ** 2)))
        # 相关系数
        cc = float(np.corrcoef(Th[:, ax], E[:, ax])[0, 1]) if np.std(E[:, ax]) > 0 else 0.0
        print('  %-14s ppm = %+9.0f   时间项 = %+8.4f deg/s (= %+9.1f deg/h)   rms %.2f deg  '
              'corr(θ,err) = %+.2f' % (nm, coef[0] * 1e6, coef[1], coef[1] * 3600.0, rms, cc))
    print()
    print('单参数（只用对应轴旋转角）：')
    for ax, nm in enumerate(('err_roll', 'err_pitch', 'err_yaw')):
        P = Th[:, ax]
        coef = float(np.sum(P * E[:, ax]) / max(np.sum(P * P), 1e-12))
        rms = float(np.sqrt(np.mean((coef * P - E[:, ax]) ** 2)))
        print('  %-10s ppm = %+9.0f   rms %.2f deg' % (nm, coef * 1e6, rms))
    print('（设计参考：KS_YAW=1000 ppm @2000 dps = 2 dps；bias 残差 1.23 deg/h；ARW 0.58 deg/rt-h）')


if __name__ == '__main__':
    main()
