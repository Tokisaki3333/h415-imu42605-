# -*- coding: utf-8 -*-
r"""VER=129 复盘：进入静止后，地磁对"运动中积攒的偏航残差"的修正速度到底是多少？

用户口径："进入静止状态后，地磁对姿态角依然有运动中积攒的残差，此时其被迫慢慢修正。"

本脚本从实测录像里直接量三件事（不猜）：
  1) 静止段里 残差 = wrap(psi_mag - psi_ekf) 的衰减时间常数；
  2) 同期滤波器自己认为的 P88 / sigma_yaw / 地磁 R / 隐含 K = P/(P+R) / tau = dt_mag/K；
  3) 修正是否被"拒收"（ekf_mag_used / rej / nis4），即卡在 NIS 而不是 K 小。
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

REC = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260922_201451.bin'
C = CH_162
DT_MAG = 0.00526
DEG2RAD = math.pi / 180.0


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):          # body->nav 四元数 -> 偏航（rad, ENU，0=+x 东）
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    fr = load_frames(REC)
    a = fr[0] if isinstance(fr, tuple) else fr
    ver = int(np.median(a[:, C['fw_tag']])) >> 16
    g = a[:, C['gyro_dps0']:C['gyro_dps0'] + 3]
    w = np.linalg.norm(g, axis=1)
    q = a[:, C['ekf_q0']:C['ekf_q0'] + 4]
    yaw_e = yaw_of(q)
    yaw_m = np.asarray(a[:, C['psi_true_deg']], dtype=float) * DEG2RAD
    p88 = np.asarray(a[:, C['ekf_p_yy']], dtype=float)
    sig_yaw = np.asarray(a[:, C['ekf_sigma_yaw']], dtype=float)
    r_deg = np.asarray(a[:, C['ekf_mag_r_deg']], dtype=float)
    gate = np.asarray(a[:, C['ekf_mag_gate']], dtype=float)
    used = np.asarray(a[:, C['ekf_mag_used']], dtype=float)
    rej = np.asarray(a[:, C['ekf_mag_rej']], dtype=float)
    nis4 = np.asarray(a[:, C['ekf_nis4']], dtype=float)
    rs = np.asarray(a[:, C['ekf_mag_rs']], dtype=float)
    res = np.array([wrap_pi(m - e) for m, e in zip(yaw_m, yaw_e)])

    print('录像 %s  fw VER=%d  帧数 %d  (%.1f s @335Hz)'
          % (os.path.basename(REC), ver, len(w), len(w) / 335.0))
    print('|w| p50 %.1f  p90 %.1f  max %.1f dps' % (np.percentile(w, 50),
          np.percentile(w, 90), w.max()))
    print('残差 wrap(psi_mag-psi_ekf): p50 %.2f  p90 %.2f  max %.2f deg'
          % tuple(np.degrees(np.percentile(np.abs(res), p)) for p in (50, 90, 100)))
    print('p_yy p50 %.3g   sigma_yaw p50 %.3f deg   mag_R p50 %.2f deg'
          % (np.median(p88), np.median(sig_yaw), np.median(r_deg)))
    print()

    # 静止段：|w| < 5 dps 连续 >= 300 帧（约 0.9 s）
    stat = w < 5.0
    segs = []
    i = 0
    while i < len(stat):
        if stat[i]:
            j = i
            while j < len(stat) and stat[j]:
                j += 1
            if j - i >= 300:
                segs.append((i, j))
            i = j
        else:
            i += 1
    print('静止段(|w|<5 dps, >=0.9 s): %d 段' % len(segs))
    for (i, j) in segs:
        d = j - i
        r = res[i:j]
        # 用末端 20% 的中位数作为"残余平台"，看前段衰减
        tail = np.median(np.abs(r[int(d * 0.8):])) * np.degrees(1.0)
        head = np.median(np.abs(r[:5])) * np.degrees(1.0)
        p = np.median(p88[i:j]) if j > i else 0.0
        rr = np.median(r_deg[i:j])
        k = p / (p + (rr * DEG2RAD) ** 2) if rr > 0 else float('nan')
        print('  [%5d,%5d)  %.2f s | 残差 首 %.2f -> 尾 %.2f deg | 中位 p_yy %.3g '
              'R %.2f deg K %.3g tau %.2f s | gate %.0f%% used %.0f%% rej 末 %.0f'
              % (i, j, d / 335.0, head, tail, p, rr, k,
                 (DT_MAG / k) if k else float('nan'),
                 100.0 * gate[i:j].mean(), 100.0 * used[i:j].mean(), rej[j - 1]))
    print()

    # 逐段：从"刚进入静止"起，残差 |res| 的衰减曲线（按 0.5 s 分箱）
    for (i, j) in segs[:3]:
        n = min(j - i, 3350)
        print('  段 [%d,%d) 残差衰减（0.5 s 分箱，deg）:' % (i, j))
        line = []
        for k in range(0, n, 167):
            blk = np.abs(res[i + k:i + k + 167])
            if len(blk) == 0:
                break
            line.append('%.2f' % np.degrees(np.median(blk)))
        print('    ' + '  '.join(line))
        line2 = []
        for k in range(0, n, 167):
            blk = p88[i + k:i + k + 167]
            if len(blk) == 0:
                break
            line2.append('%.2g' % np.median(blk))
        print('    p_yy: ' + '  '.join(line2))
        line3 = []
        for k in range(0, n, 167):
            blk = r_deg[i + k:i + k + 167]
            if len(blk) == 0:
                break
            line3.append('%.2f' % np.median(blk))
        print('    R_deg: ' + '  '.join(line3))


if __name__ == '__main__':
    main()
