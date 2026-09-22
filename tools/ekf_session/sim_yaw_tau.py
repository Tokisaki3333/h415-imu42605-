# -*- coding: utf-8 -*-
r"""偏航通道 PC 仿真 v4（物理一致 + 标量实现，可跑完）

与 v3 同样的物理与指标，只把 KF 内环改写成标量递推（原来 3x3 矩阵每步太慢），
并把时长压到 25 s / 30 次蒙特卡洛以便秒级跑完。

物理参数（300 s 静止录像实测）：ARW=0.0060 deg/√s, σ_m=0.33 deg, b_m=2.0 deg,
σ_b0=0.0087 dps, σ_bg(RW)=2e-4 dps/√s, 初始失配 mis~N(0,3°)。

用法: python tools/ekf_session/sim_yaw_tau.py
"""
import math
import numpy as np

DT = 1.0 / 190.0
FS = 1.0 / DT
ARW = 0.0060
SIG_M = 0.33
B_M = 2.0
SIG_B0 = 0.0087
SIG_BRW = 2.0e-4
SIG_MIS = 3.0
T_END = 25.0
N_RUN = 30
T_EDGE = 4.0
T_STEP = 12.0
COMP = ((0.02, 60.0), (0.08, 40.0), (0.25, 15.0), (0.6, 4.0))


def envelope(t):
    w = np.ones_like(t)
    w[t < T_EDGE] = 0.5 * (1 - np.cos(math.pi * t[t < T_EDGE] / T_EDGE))
    te = T_END - t
    w[te < T_EDGE] = 0.5 * (1 - np.cos(math.pi * te[te < T_EDGE] / T_EDGE))
    return w


def truth(n):
    t = np.arange(n) * DT
    env = envelope(t)
    om = np.zeros(n)
    for f, A in COMP:
        om += A * 2 * math.pi * f * np.cos(2 * math.pi * f * t)
    om *= env
    psi = np.cumsum(om) * DT
    psi -= psi[0]
    return psi, om


def run_once(mode, tau=None, Tb=None, T_c=60.0, seed=0):
    rng = np.random.default_rng(seed)
    n = int(T_END / DT)
    t = np.arange(n) * DT
    psi, om = truth(n)
    bg = rng.normal(0.0, SIG_B0) + np.cumsum(rng.normal(0.0, SIG_BRW * math.sqrt(DT), n))
    bm = np.zeros(n)
    if math.isfinite(T_c):
        a = math.exp(-DT / T_c)
        s = B_M * math.sqrt(1 - a * a)
        x = rng.normal(0.0, B_M)
        for k in range(n):
            x = a * x + s * rng.normal()
            bm[k] = x
    else:
        bm[:] = B_M
    bm[t >= T_STEP] += B_M
    gyro = om + bg + rng.normal(0.0, ARW * math.sqrt(DT), n)
    mag = psi + bm + rng.normal(0.0, SIG_M, n)
    mis = rng.normal(0.0, SIG_MIS)
    e = np.zeros(n)
    if mode == 'pure':
        est = psi[0] + mis
        for k in range(n):
            est += gyro[k] * DT
            e[k] = est - psi[k]
    elif mode == 'M1':
        K = 1.0 - math.exp(-DT / tau)
        est = psi[0] + mis
        for k in range(n):
            est += gyro[k] * DT
            est += K * (mag[k] - est)
            e[k] = est - psi[k]
    elif mode == 'M2':
        K = 1.0 - math.exp(-DT / tau)
        kb = DT / Tb
        est, bgh = psi[0] + mis, 0.0
        for k in range(n):
            est += (gyro[k] - bgh) * DT
            nu = mag[k] - est
            est += K * nu
            bgh += kb * K * nu / DT
            e[k] = est - psi[k]
    elif mode in ('M3', 'M4'):
        ns = 3 if mode == 'M3' else 2
        q0 = (ARW ** 2) * DT
        q1 = (SIG_BRW * DT) ** 2 * DT
        q2 = ((B_M ** 2) * 2 * DT / T_c) if (ns == 3 and math.isfinite(T_c)) else (1e-14 if ns == 3 else 0.0)
        x0, x1, x2 = psi[0] + mis, 0.0, 0.0
        P00, P01, P11 = SIG_MIS ** 2, 0.0, (SIG_B0 * DT) ** 2
        P02 = P12 = P22 = 0.0
        if ns == 3:
            P22 = B_M ** 2
        for k in range(n):
            # 传播: psi <- psi - bg*DT ; P <- F P F' + Q
            x0 += (gyro[k] - x1) * DT
            P00 = P00 - 2 * DT * P01 + DT * DT * P11 + q0
            P02 = P02 - DT * P12
            P01 = P01 - DT * P11
            P11 = P11 + q1
            P22 = P22 + q2
            # 观测 H=[1,0,1]
            if ns == 3:
                HP0, HP1, HP2 = P00 + P02, P01 + P12, P02 + P22
                S = HP0 + P22 + SIG_M ** 2
                nu = mag[k] - (x0 + x2)
                K0, K1, K2 = HP0 / S, HP1 / S, HP2 / S
                x0 += K0 * nu; x1 += K1 * nu; x2 += K2 * nu
                P00 -= K0 * HP0; P01 -= K0 * HP1; P02 -= K0 * HP2
                P11 -= K1 * HP1; P12 -= K1 * HP2
                P22 -= K2 * HP2
                P10 = P01; P20 = P02; P21 = P12
                _ = (P10, P20, P21)
            else:
                HP0, HP1 = P00, P01
                S = P00 + SIG_M ** 2
                nu = mag[k] - x0
                K0, K1 = HP0 / S, HP1 / S
                x0 += K0 * nu; x1 += K1 * nu
                P00 -= K0 * HP0; P01 -= K0 * HP1
                P11 -= K1 * HP1
            e[k] = x0 - psi[k]
    return e


def metrics(acc):
    m = int(FS)
    h = int(T_EDGE / DT)
    ks = int(T_STEP / DT)
    inc, tail, peak, resid = [], [], [], []
    for e in acc:
        d = e[h:-h]
        inc.append(np.mean((d[m:] - d[:-m]) ** 2))
        tail.append(np.mean(e[-h:] ** 2))
        after = e[ks:ks + int(4.0 / DT)]
        before = float(np.mean(e[ks - int(1.0 / DT):ks]))
        peak.append(np.max(np.abs(after - before)))
        resid.append(abs(float(np.mean(after[-int(1.0 / DT):])) - before))
    return (math.sqrt(np.mean(inc)), math.sqrt(np.mean(tail)),
            float(np.mean(peak)), float(np.mean(resid)))


CASES = [('纯陀螺（无地磁）', 'pure', None, None),
         ('M1 τ=1.5 s', 'M1', 1.5, None),
         ('M1 τ=15.85 s', 'M1', 15.85, None),
         ('M1 τ=60 s', 'M1', 60.0, None),
         ('M1 τ=300 s', 'M1', 300.0, None),
         ('M2 τ=1.5 s,Tb=30 s', 'M2', 1.5, 30.0),
         ('M2 τ=60 s,Tb=30 s', 'M2', 60.0, 30.0),
         ('M3 3状态 KF(含 b_m)', 'M3', None, None),
         ('M4 2状态 KF(固件型)', 'M4', None, None)]


def main():
    for T_c, tag in ((60.0, '地磁偏置 GM(T_c=60 s)'), (float('inf'), '地磁偏置常值')):
        print('==== %s ；t=%.0f s 处再加 2° 偏置阶跃' % (tag, T_STEP))
        print('   %-24s %-15s %-14s %-14s %-14s'
              % ('方案', '机动 1s 增量 rms', '末段绝对误差', '阶跃后峰值', '阶跃后残差'))
        for label, mode, tau, Tb in CASES:
            acc = [run_once(mode, tau=tau, Tb=Tb, T_c=T_c, seed=s) for s in range(N_RUN)]
            i, tl, pk, rs = metrics(acc)
            print('   %-24s %-15.4f %-14.3f %-14.3f %-14.3f' % (label, i, tl, pk, rs))
        print()
    print('陀螺自身 1 s 增量精度（物理底）σ_inc = sqrt(ARW²+σ_b0²) = %.4f deg'
          % math.sqrt(ARW ** 2 + SIG_B0 ** 2))


if __name__ == '__main__':
    main()
