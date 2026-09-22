# -*- coding: utf-8 -*-
r"""VER=122 复核：地磁修正速度（EKF 标准数学）—— tau、速率上限、稳态滞后。

模型（与 proc_ekf.c 偏航通道同构，标量近似；h^2*P << R 时）：
  预测 P- = P + Qd,  Qd = Q*dt
  观测 r = h*e + v,  S = h^2*P- + R
  更新 K = P-*h/S,   P = P-*R/S
  稳态 P_ss = sqrt(Qd*R)/h,  闭环 tau = dt/(K*h) = dt*sqrt(R/(Q*dt))/h
  修正速度（对误差 e）= e/tau
  稳态滞后（漂移率 w_d）= w_d*tau

VER=122 的取值（全部来自 v5f_tune.h）：
  R      = (2 deg)^2                      地磁自身误差（VER=118 诚实值，不动）
  ARW    = 0.58 deg/rt-h = 0.009667 deg/rt-s      S3 实测随机游走
  KS_YAW = 1000 ppm                        用户口径：满量程 1000 ppm 漂移
  Q_MAX  = 7.14e-06 rad^2/s                <=> tau >= 1 s <=> 修正速度 <= 2 dps（e<=2 deg）
  h      = cos(dip) = 0.5831（e1 方向对偏航的灵敏度）, dt_e = 1/501.4 s
"""
import math

DEG = math.pi / 180.0
F_EPOCH = 8021.9 / 16.0
DT = 1.0 / F_EPOCH
DIP_TAN = 1.3932
H = 1.0 / math.sqrt(1.0 + DIP_TAN * DIP_TAN)
R_YAW = (2.0 * DEG) ** 2
ARW = 0.58 / 60.0 * DEG            # rad/rt-s
KS_YAW = 1.0e-3
Q_MAX = 7.14e-6
FS_DPS = 2000.0
DRIFT_FS_DPS = KS_YAW * FS_DPS     # 2 dps 最坏不可建模漂移率（1000 ppm x FS）
E_SIG = 2.0                        # 地磁自身误差 2 deg


def q_yaw(w_dps):
    kry = KS_YAW * w_dps * DEG
    return min(ARW * ARW + kry * kry, Q_MAX)


def tau_of_q(q):
    return DT * math.sqrt(R_YAW / (q * DT)) / H


def main():
    print('==== VER=122 常数 ====')
    print('R_yaw = (2 deg)^2 = %.4e rad^2   ARW = %.4e rad/rt-s (= %.2f deg/rt-h)'
          % (R_YAW, ARW, ARW / DEG * 60))
    print('KS_YAW = 1000 ppm -> 满量程(%.0f dps) 不可建模漂移率 = %.1f dps = %.0f deg/h'
          % (FS_DPS, DRIFT_FS_DPS, DRIFT_FS_DPS * 3600))
    print('Q_MAX = %.2e rad^2/s  <=> tau_min = %.3f s' % (Q_MAX, tau_of_q(Q_MAX)))
    print('h = cos(dip) = %.4f   dt_e = %.6f s (%.1f Hz)' % (H, DT, F_EPOCH))
    print()
    print('==== 与 VER=118 原状对比（静止）====')
    q118 = 4.5637e-6 + 1.0e-5 * F_EPOCH        # SIG_G_RADS^2 + Q_YAW_MIN*f
    t118 = tau_of_q(q118)
    print('  VER=118 : Q = %.3e rad^2/s -> tau = %.3f s  -> 1 deg 偏差修正速度 %.2f dps'
          % (q118, t118, (1.0 * DEG) / t118 / DEG))
    t122 = tau_of_q(q_yaw(0.0))
    print('  VER=122 : Q = %.3e rad^2/s -> tau = %.2f s   -> 1 deg 偏差修正速度 %.3f dps'
          % (q_yaw(0.0), t122, (1.0 * DEG) / t122 / DEG))
    print()
    print('==== 工作点（Q_MAX = %.2e 截断）====' % Q_MAX)
    print('%-9s %-13s %-10s %-13s %-13s %-11s' %
          ('|w| dps', 'Q rad^2/s', 'tau (s)', '2deg 修正速度', '1deg 修正速度', '2dps 滞后'))
    for w in (0.0, 5.0, 20.0, 50.0, 100.0, 153.0, 300.0, 1000.0, 2000.0):
        q = q_yaw(w)
        tau = tau_of_q(q)
        v2 = (E_SIG * DEG) / tau / DEG            # deg/s
        v1 = (1.0 * DEG) / tau / DEG
        print('%-9.0f %-13.3e %-10.3f %-13.3f %-13.3f %-11.3f'
              % (w, q, tau, v2, v1, DRIFT_FS_DPS * tau))
    print('  "2dps 滞后" = 满量程漂移(2 dps) x tau = 稳态落后角度（设计目标 <= 地磁自身误差 2 deg）')
    print()
    print('==== 速率上限核对（用户"先前讨论值"= 2 dps）====')
    ok = True
    for w in (0.0, 20.0, 50.0, 100.0, 153.0, 300.0, 1000.0, 2000.0):
        tau = tau_of_q(q_yaw(w))
        v2 = E_SIG / tau                      # dps（误差 = 地磁自身误差 2 deg 时）
        ok &= (tau >= 1.0 - 1e-9)
        print('  |w|=%6.0f dps: tau=%8.3f s >= 1 s : %s ; 2deg 误差修正速度 %.3f dps <= 2 dps : %s'
              % (w, tau, 'OK' if tau >= 1.0 - 1e-9 else 'BAD', v2,
                 'OK' if v2 <= 2.0 + 1e-6 else 'BAD'))
    print('  结论:', '全部满足（tau>=1 s <=> 修正速度 <= 2 dps @ e<=2 deg）' if ok else '有违反')
    print()
    print('==== 静态稳态量 ====')
    q = q_yaw(0.0)
    Pss = math.sqrt(q * DT * R_YAW) / H
    print('  P[8][8] 稳态 = %.3e rad^2 -> sigma_yaw 上报 = %.4f deg' % (Pss, math.sqrt(Pss) / DEG))
    K = Pss * H / (H * H * Pss + R_YAW)
    print('  K 稳态 = %.3e /次 -> tau = %.2f s' % (K, DT / (K * H)))
    print('  地磁白噪声对偏航的贡献 1sigma = %.5f deg' % (E_SIG * math.sqrt(K / 2.0)))
    print('  静止 1.23 deg/h 零偏漂移的滞后 = %.4f deg' % (1.23 / 3600.0 * tau_of_q(q)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
