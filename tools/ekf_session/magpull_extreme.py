# -*- coding: utf-8 -*-
"""VER=128 分析：极限运动下地磁偏航牵引速度是否还有效？

问题：VER=125 的 R 地板把 K 钉在 K_max = dt_mag/tau_min（常数，与 |w| 无关），
      于是 1 kHz 级陀螺残差（KS_YAW*|w|）在极限运动下累积，而地磁仍以
      15.85 s 的时间常数牵引 -> 稳态滞后 = drift_rate * tau。

本脚本用实测极限运动录像（R:\\imu_20260922_201451.bin）的 |w| 时间序列，
比较两条定律：
  A) 现版（VER=125/127）：K = dt_mag / 15.85                （常数）
  B) VER=128 设计曲线    ：K = K_nat(Q(|w|))，Q = min(q0^2+(KS*|w|)^2, QMAX)*dt_mag
     K_nat = P/(P+R)，P = (Q + sqrt(Q^2+4QR))/2            （1 维 KF 稳态）
输出：tau 与稳态滞后 drift*tau 的分位数（deg）。
"""
import sys, os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

REC = r'R:\imu_20260922_201451.bin'
KS = 1.0e-3              # 满量程 1000 ppm -> 比例残差
ARW = 0.58 / 60.0        # dps/rt-hz -> dps/rt-s? (与固件同名口径: rad/rt-s)
DEG2RAD = 0.017453292
QYAW_MAX = 7.14e-6       # V5F_EKF_Q_YAW_MAX
SIG_MAG_DEG = 2.0
DT_MAG = 0.00526         # V5F_MAG_EPOCH_DT_S
TAU_MIN_S = 15.85        # V5F_MAG_YAW_TAU_MIN_S

q0 = ARW * DEG2RAD       # rad/rt-s
R = (SIG_MAG_DEG * DEG2RAD) ** 2


def k_nat(w_dps):
    """1 维 KF 稳态增益（与固件 Q_MAX 同口径）。"""
    kry = KS * w_dps * DEG2RAD
    qd = np.minimum(q0 * q0 + kry * kry, QYAW_MAX)
    Q = qd * DT_MAG
    P = 0.5 * (Q + np.sqrt(Q * Q + 4.0 * Q * R))
    return P / (P + R)


def k_curve(w_dps):
    """VER=128 实际落盘律：以 V5F_MAG_YAW_TAU_MIN_S 为静止锚点，
    按与传播同一 Q 口径缩放 tau：tau_min(|w|) = tau0*sqrt(q0^2/Q(|w|))。"""
    kry = KS * w_dps * DEG2RAD
    qd = np.minimum(q0 * q0 + kry * kry, QYAW_MAX)
    tau = TAU_MIN_S * np.sqrt(q0 * q0 / qd)
    return DT_MAG / tau


def main():
    fr = load_frames(REC)
    a = fr[0] if isinstance(fr, tuple) else fr
    g = a[:, CH_162['gyro_dps0']:CH_162['gyro_dps0'] + 3]
    w = np.linalg.norm(g, axis=1)
    print('录像 %s  帧数 %d' % (os.path.basename(REC), len(w)))
    for p in (50, 90, 99, 100):
        print('  |w| p%-4.0f = %7.1f dps' % (p, np.percentile(w, p)))
    print('  |w|>500 dps 占比 %.1f%%   >1000 dps %.1f%%'
          % (100.0 * np.mean(w > 500), 100.0 * np.mean(w > 1000)))
    print()

    kA = np.full_like(w, DT_MAG / TAU_MIN_S)   # A) 现版常数地板
    kB = k_nat(w)                              # B) 精确 1 维 KF 稳态增益
    kC = k_curve(w)                            # C) VER=128 落盘（A 的锚点 + 设计曲线）
    tauA, tauB, tauC = DT_MAG / kA, DT_MAG / kB, DT_MAG / kC
    drift = KS * w                     # dps，偏航角速度残差（设计口径）
    lagA, lagB, lagC = drift * tauA, drift * tauB, drift * tauC

    print('  静止基准(A 律在 |w|->0 的取值): tau_A=%.2f s' % tauA[0])
    print('  设计曲线静止点: K_nat(0)=%.4e -> tau=%.2f s；C 律静止点 tau=%.2f s'
          % (kB[0], tauB[0], tauC[0]))
    print()
    print('  %-34s %10s %10s %10s %10s' % ('量', 'p50', 'p90', 'p99', 'max'))
    for name, arr in (('K  (A 现版常数)', kA), ('K  (C VER=128)', kC),
                      ('tau s (A 现版常数)', tauA), ('tau s (C VER=128)', tauC),
                      ('稳态滞后 deg (A 现版)', lagA),
                      ('稳态滞后 deg (C VER=128)', lagC)):
        print('  %-34s %10.4g %10.4g %10.4g %10.4g' % (name, np.percentile(arr, 50),
              np.percentile(arr, 90), np.percentile(arr, 99), arr.max()))
    print()
    # 只看剧烈段（|w|>500 dps）与削顶段
    for lim in (500.0, 1000.0, 2000.0):
        m = w > lim
        if not np.any(m):
            continue
        print('  |w|>%4.0f dps 段(占 %.1f%%): 滞后 A p50/max = %.1f/%.1f deg, '
              'C p50/max = %.1f/%.1f deg, tau C p50 = %.2f s'
              % (lim, 100.0 * np.mean(m), np.percentile(lagA[m], 50), lagA[m].max(),
                 np.percentile(lagC[m], 50), lagC[m].max(), np.percentile(tauC[m], 50)))
    print()
    print('  C 律注入的高频偏航噪声 sigma_mag*sqrt(K/2): p50 %.3f deg, p90 %.3f deg, max %.3f deg'
          % tuple(np.percentile(SIG_MAG_DEG * np.sqrt(kC / 2.0), p) for p in (50, 90, 100)))
    print('  A 律同上: p50 %.3f deg' % np.percentile(SIG_MAG_DEG * np.sqrt(kA / 2.0), 50))


if __name__ == '__main__':
    main()
