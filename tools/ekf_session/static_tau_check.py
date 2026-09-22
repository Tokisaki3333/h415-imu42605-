# -*- coding: utf-8 -*-
r"""VER=130 设计校核：静止态偏航牵引 tau = 1.5 s 的数学与数值验证。

机制（proc_ekf.c）：静止时把偏航 P88 地板抬到 P_min = R_yaw*K/(1-K)，K = dt_mag/tau。
本脚本：
 1) 对若干实际 R_yaw（2/4/10/20 deg）给出 P_min、静止 sigma_yaw、NIS 16.27 下可接受的创新上限；
 2) **逐帧仿真** 一维 KF（prop 地板 + 190 Hz 更新），验证闭环 tau 确实 = 1.5 s（不是只有解析式）；
 3) 给出"运动中积攒 5/10/20 deg 残差"从静止开始的收敛时间；
 4) 地磁噪声（2 deg）经 K 之后注入的姿态抖动。
"""
import math

DT = 0.00526          # V5F_MAG_EPOCH_DT_S
TAU = 1.5             # V5F_MAG_STATIC_TAU_S
NIS_MAX = 16.27       # V5F_EKF_NIS_MAX_MAG
P_MIN = 1.0e-12       # V5F_EKF_YAW_P_MIN（非静止默认）
DEG = math.pi / 180.0

K = DT / TAU


def sim(r_yaw, e0_deg, n=4000):
    """逐帧仿真：P 每步被地板抬到 P_min，随后做 1 维 KF 更新（真值 = 磁航向 0）。"""
    pmin = max(r_yaw * (K / (1.0 - K)), P_MIN)
    e = e0_deg * DEG
    p = pmin
    out = [e / DEG]
    for _ in range(n):
        p += (K * K / (1.0 - K)) * r_yaw * 0.02          # 传播步给一点 Q（不影响地板结果）
        if p < pmin:
            p = pmin
        kk = p / (p + r_yaw)
        e -= kk * e
        p = (1.0 - kk) * p
        out.append(e / DEG)
    return out, pmin


def main():
    print('dt_mag = %.5f s   tau* = %.2f s   ->  K* = P/(P+R) = %.5g  (= dt/tau)'
          % (DT, TAU, K))
    print()
    print('  %-10s %-12s %-12s %-12s %-12s' % ('R_yaw', 'P_min', 'sigma_yaw', 'nu_max(NIS)', '抖动'))
    for rd in (2.0, 4.0, 10.0, 20.0):
        r = (rd * DEG) ** 2
        pmin = max(r * (K / (1.0 - K)), P_MIN)
        nu = math.sqrt(NIS_MAX * (pmin + r)) / DEG
        jit = 2.0 * math.sqrt(K * (1.0 - K) / 2.0)
        print('  %-10s %-12.3g %-12.4f %-12.2f %-12.3f'
              % ('%.0f deg' % rd, pmin, math.sqrt(pmin) / DEG, nu, jit))
    print('      (R=%.0f..%.0f deg 时都能通过 NIS：可接受创新 %d..%d deg；抖动 = sigma_mag*sqrt(K(1-K)/2))'
          % (2, 20, math.sqrt(NIS_MAX * ((2 * DEG) ** 2 / (1 - K))) / DEG,
             math.sqrt(NIS_MAX * ((20 * DEG) ** 2 / (1 - K))) / DEG, ))
    print()
    # 逐帧仿真（R = 2 deg 与 4 deg 两种）
    for rd in (2.0, 4.0):
        r = (rd * DEG) ** 2
        curve, pmin = sim(r, 10.0)
        def t_to(x):
            for i, v in enumerate(curve):
                if abs(v) <= x:
                    return i * DT
            return float('nan')
        fit_tau = None
        # 用 0.5*e0 与 0.25*e0 两点估时间常数
        t1, t2 = t_to(5.0), t_to(2.5)
        if t1 == t1 and t2 == t2:
            fit_tau = (t2 - t1) / math.log(2.0)
        print('R=%4.0f deg: P_min=%.3g  10 deg 残差 -> 到 1 deg 用 %.2f s，到 0.2 deg 用 %.2f s'
              % (rd, pmin, t_to(1.0), t_to(0.2)))
        print('             实测时间常数(仿真) tau = %.3f s   (目标 %.2f s)' % (fit_tau, TAU))
    print()
    print('参考：VER=129（无地板）静止 K=3.7e-4 -> tau=14.2 s，10 deg 残差降到 1 deg 需 ~32 s。')


if __name__ == '__main__':
    main()
