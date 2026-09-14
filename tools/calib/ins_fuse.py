# -*- coding: utf-8 -*-
"""
自稳定融合：陀螺 + 加速度计互相校准，角度与速度漂移有界。

结构照飞控（Betaflight Mahony / iNav 加权）：
    1) 误差   e = â_b × ĝ_est         （实测比力方向 × 估计重力方向，都在机体系）
    2) PI     ω_c = ω_gyro + w*(Kp*e + ∫Ki*e dt)     <- 反馈进角速度，再积分四元数
               w = bellCurve(|a_b|/g - 1, 0.2) × 转速降权     <- iNav 的连续权重，不是开关
    3) ZUPT   静止门（陀螺门 AND 加速度动静门）内 v≡0
    4) 互相校准
         陀螺零偏  <- PI 的积分项（倾角误差驱动）
         加速度零偏 沿重力分量 <- 静止段 |a_b|=1g 约束
         加速度零偏 全部三分量 <- ZUPT 速度残差（这是水平分量唯一的观测途径）

验收按需求：角度与速度漂移有界；位置是高次项，只报告不苛求。

用法：python tools/calib/ins_fuse.py <记录> [输出图]
"""
import os, sys
import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
for _p in (_here, os.path.join(_here, 'h415-imu42605-', 'tools', 'calib')):
    if os.path.isfile(os.path.join(_p, 'jf_load.py')):
        sys.path.insert(0, _p)
        break
from jf_load import load_jf

G = 9.7985                       # 本地重力 m/s^2
FPS = 8027.0
DT = 1.0/FPS

# ---- 离线标定（v5f_proc.h） ----
ACC_B = np.array([-10.10, -15.42, 43.72])            # LSB
ACC_S = np.array([2028.48, 2040.78, 2016.07])        # LSB/g
GYRO_S = np.array([16.2753, 16.4366, 16.4235])       # LSB/(deg/s)
GYRO_K = np.array([[1.0, 0.001528, -0.001923],
                   [0.001528, 1.0, 0.000005],
                   [-0.001923, 0.000005, 1.0]])
GYRO_B0 = np.array([1.0334, 0.8494, 12.0910])        # LSB：固件离线零偏 GB_BIAS0_*
BG0 = GYRO_B0/GYRO_S                                 # dps：raw 通道是未校正的，必须自己减
BG_TRIM = np.zeros(3)                                # 在线 PI 只负责修残余零偏

# ---- 融合参数 ----
KP = 0.25                        # Mahony 比例（与飞控同量级）
KI = KP/30.0                     # 积分：约 30 s 收敛陀螺零偏
ACC_BELL = 0.20                  # |a|/g-1 的 bell 半宽（iNav MAX_ACC_NEARNESS）
RATE_LO, RATE_HI = 20.0, 60.0    # 转速降权区间 dps
T_ON_A, T_OFF_A = 1.92, 1.15     # 加速度动静门 mg（accel_gate_sim.py 实测标定）
DEB_ON_A = 16
DEB_OFF_A = int(os.environ.get('DEB_OFF', '12000'))   # 1.49 s；拐角停顿比它短就会漏检
W_A = 4000                       # 0.5 s
KZ = float(os.environ.get('KZ', '0.35'))   # ZUPT 速度残差 -> 加速度零偏 的步长
KTRIM = float(os.environ.get('TRIM_KP', '0.5'))   # 连续牵引比例增益 (1/s)，tau = 1/KTRIM
KITRIM = float(os.environ.get('TRIM_KI', '0.0'))  # 连续牵引积分增益：吃掉 PC 侧残余零偏
TRIM_RATE = float(os.environ.get('TRIM_RATE', '60'))  # 转速降权上限 dps（超过就完全不牵引）
SA_LO = float(os.environ.get('SA_LO', '0.0'))     # s_a 权重因子：默认关闭
SA_HI = float(os.environ.get('SA_HI', '1e9'))
# 试过加"比力平稳度 s_a"当第三个权重因子，**实测更差**，故默认关闭（SA_HI 取无穷即不生效）：
#   SA_HI=1e9(关) 1.404 m / 1.564 m   SA_HI=200 1.906 / 2.069   SA_HI=60 3.632 / 3.802   SA_HI=20 6.087 / 6.254
# 原因：手抖是零均值的，对速度的贡献自己抵消；压掉牵引后姿态净漂移(31 deg/h)无人管，
# 而漏重力是单方向的 —— 净漂移远比抖动致命。iNav 原版两因子（bell x 转速）就是对的。


def qmul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                     a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])


def Rm(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def expq(v):
    """旋转矢量 -> 四元数"""
    t = np.linalg.norm(v)
    if t < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.concatenate([[np.cos(t*0.5)], np.sin(t*0.5)*v/t])


def bell(x, r):
    """iNav 式钟形权重：0 处 1，±r 处约 0.5，超出快速衰减"""
    return 1.0/(1.0 + (x/r)**4)


def main():
    fn = sys.argv[1] if len(sys.argv) > 1 else 'serial_runtime_20260914_003448_145_export.txt'
    out = sys.argv[2] if len(sys.argv) > 2 else 'ins_fuse.png'
    A = np.asarray(load_jf(fn), dtype=np.float64)
    N = len(A)
    acc_lsb = A[:, 4:7]
    gyro_lsb = A[:, 7:10]
    print("记录 %s   %d 帧  %.1f s" % (os.path.basename(fn), N, N/FPS))

    # 陀螺 -> dps：**先减 LSB 离线零偏、再除标度、再过对称交叉**（与 proc_gyro_bias.c 同序；
    # 上报的原始 LSB 是未校正的，不减这一项就会带着 ~0.74 dps 的 z 零偏一路漂）
    dps = (GYRO_K @ (((gyro_lsb - GYRO_B0)/GYRO_S).T)).T
    wr = np.radians(dps)

    # ---- 第一次：静态门（用离线标定的加速度）----
    a_b0 = (acc_lsb - ACC_B)/ACC_S                       # 单位 g
    M = np.cumsum(np.vstack([np.zeros((1, 3)), a_b0]), 0)
    kk = np.arange(N)
    sa = np.abs((M[np.minimum(kk+1, N)]-M[np.maximum(kk+1-W_A, 0)]) /
                np.maximum(np.minimum(kk+1, N)-np.maximum(kk+1-W_A, 0), 1)[:, None]
                - (M[np.maximum(kk+1-W_A, 0)]-M[np.maximum(kk+1-2*W_A, 0)]) /
                np.maximum(np.maximum(kk+1-W_A, 0)-np.maximum(kk+1-2*W_A, 0), 1)[:, None]
                ).max(1)*1000

    rate = np.linalg.norm(dps, axis=1)
    g_still = rate < 10.0
    acc_still = np.ones(N, bool)
    c_on = c_off = 0
    for k in range(2*W_A, N):
        acc_still[k] = acc_still[k-1]
        if acc_still[k]:
            if sa[k] > T_ON_A:
                c_on += 1
                if c_on >= DEB_ON_A:
                    acc_still[k] = False; c_on = 0
            else:
                c_on = 0
        else:
            if sa[k] < T_OFF_A:
                c_off += 1
                if c_off >= DEB_OFF_A:
                    acc_still[k] = True; c_off = 0
            else:
                c_off = 0
    still = g_still & acc_still
    print("静止门：陀螺 %.1f s / 加速度 %.1f s / 与 %.1f s"
          % (g_still.sum()/FPS, acc_still.sum()/FPS, still.sum()/FPS))

    # ---- 本场次零偏清零（用起始静止段，把 |a_b| 归一化到 1 g）----
    k0 = np.flatnonzero(still)
    s0 = k0[0]
    e0 = s0
    while e0 < N and still[e0]:
        e0 += 1
    sl = slice(s0+int(0.5*FPS), max(s0+int(0.5*FPS)+1, e0-int(0.3*FPS)))
    m0 = acc_lsb[sl].mean(0)
    # 要 |(m0-ba)/ACC_S| = 1：令 L = |(m0-ACC_B)/ACC_S|，则 ba = m0 - (m0-ACC_B)/L
    L = np.linalg.norm((m0-ACC_B)/ACC_S)
    ba = m0 - (m0-ACC_B)/L
    print("零偏清零：起始静止段 %.2f~%.2f s，|a_b|-1g 由 %+.3f mg 归零，ba=[%s]"
          % (s0/FPS, e0/FPS, (L-1)*1000, np.array2string(ba, precision=2)))

    # ---- 静止段列表（姿态对准与 ZUPT 都用）----
    segs, i = [], 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/FPS > 0.8:
                segs.append((i, j))
            i = j
        else:
            i += 1

    # ---- 先决步骤：把姿态对准到重力方向 ----
    # 静止时加速度计给的就是重力方向，所以 R(q)^T ẑ 与 â_b 的夹角就是姿态的绝对误差。
    # 这个错位必须**先**消掉再积分，否则 ∫R(q)·a_b 里会一直漏 g·sin(θ)。
    # 做法：每个静止段解一个 dq（把 â_b 转到 R(q)^T ẑ，即让 R(q⊗dq)^T ẑ = â_b），
    #       然后向后传播到下一个静止段（段内不再改，靠陀螺自己走）。
    q_rec = A[:, :4].copy(); q_rec /= np.linalg.norm(q_rec, axis=1, keepdims=True)
    q_al = q_rec.copy()
    dq_cur = np.array([1.0, 0.0, 0.0, 0.0])
    misalign = []
    prev = 0
    for (s, e) in segs:
        q_al[prev:s] = np.array([qmul(q_rec[k], dq_cur) for k in range(prev, s)]) if s > prev else q_al[prev:s]
        sl2 = slice(s+int(0.2*FPS), max(s+int(0.2*FPS)+1, e-int(0.2*FPS)))
        m = (acc_lsb[sl2].mean(0)-ba)/ACC_S
        m = m/np.linalg.norm(m)                       # 实测重力方向（机体系）
        qm = q_rec[sl2].mean(0); qm = qm/np.linalg.norm(qm)
        u = Rm(qm).T @ np.array([0.0, 0.0, 1.0])      # 姿态给出的重力方向
        cr = np.cross(m, u)
        n = np.linalg.norm(cr)
        ang = np.arctan2(n, np.dot(m, u))
        misalign.append(np.degrees(ang))
        # 从原始 q 解本段总修正（不做累积组合，避免重复计入）
        dq_cur = expq(ang*cr/n) if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])
        q_al[s:e] = np.array([qmul(q_rec[k], dq_cur) for k in range(s, e)])
        prev = e
    if prev < N:
        q_al[prev:] = np.array([qmul(q_rec[k], dq_cur) for k in range(prev, N)])
    q_al /= np.linalg.norm(q_al, axis=1, keepdims=True)
    print("重力方向对准：各静止段解出的错位量 %s deg（这些是必须在积分前消掉的）"
          % np.array2string(np.array(misalign), precision=3))
    print("   对准后各静止段的残余错位：")

    def _tilt_after(QQ):
        out = []
        for (s, e) in segs:
            sl2 = slice(s+int(0.2*FPS), max(s+int(0.2*FPS)+1, e-int(0.2*FPS)))
            m = (acc_lsb[sl2].mean(0)-ba)/ACC_S
            m = m/np.linalg.norm(m)
            qm = QQ[sl2].mean(0); qm = qm/np.linalg.norm(qm)
            u = Rm(qm).T @ np.array([0.0, 0.0, 1.0])
            out.append(np.degrees(np.arccos(np.clip(np.dot(m, u), -1, 1))))
        return np.array(out)
    print("      原始记录姿态 %s" % np.array2string(_tilt_after(q_rec), precision=4))
    print("      对准后       %s" % np.array2string(_tilt_after(q_al), precision=4))

    # ---- 主循环 ----
    # 姿态基准：**直接用固件记录的四元数**（f0~f3），并**先做重力方向对准**（q_al）。
    # 固件姿态已过完整的静止牵引（20 ms 粒度 + 回滚）与零偏/标度/交叉修正、用 DRDY tick
    # 在 8 kHz 积分；自己拿原始陀螺重积分必然更差（实测固件漂 0.44 deg，自积分漂 2.73 deg）。
    #   原始陀螺的用途：① 对照量化固件处理链的价值；② 需要时在 PC 侧做慢速细调。
    # ATT=level 固件姿态 + 重力方向对准（默认，先决步骤）
    # ATT=trim  固件姿态 + 加速度计连续慢速微调
    # ATT=rec   纯用固件姿态（不做对准，用作对照）
    # ATT=gyro  自己用原始陀螺重积分（对照）
    ATT = os.environ.get('ATT', 'level')

    qg = np.array([1.0, 0.0, 0.0, 0.0])                  # 自积分对照姿态
    dq_trim = np.array([1.0, 0.0, 0.0, 0.0])             # 连续牵引的累积修正
    integ = np.zeros(3)                                  # 牵引积分项（PC 侧残余零偏）
    bg = BG_TRIM.copy()                                  # dps
    v = np.zeros(3); p = np.zeros(3)
    P = np.zeros((N, 3)); V = np.zeros((N, 3))
    Q = np.zeros((N, 4)); QG = np.zeros((N, 4))
    TILT = np.zeros(N); WGT = np.zeros(N); BA = np.zeros((N, 3)); BG = np.zeros((N, 3))
    prev_still = bool(still[0])
    zupt_list = []
    seg_start = 0
    for k in range(N):
        if ATT == 'gyro':
            q = qg
        elif ATT == 'rec':
            q = q_rec[k]
        else:
            q = q_al[k]
        if ATT == 'trim':
            q = qmul(q, dq_trim)
            q = q/np.linalg.norm(q)
        R = Rm(q)
        a_b = (acc_lsb[k] - ba)/ACC_S                    # 单位 g
        an = np.linalg.norm(a_b)
        up_est = R.T @ np.array([0.0, 0.0, 1.0])         # 估计重力方向（机体系）
        e_err = np.cross(a_b/an, up_est) if an > 1e-9 else np.zeros(3)
        # iNav 式连续权重，再加第三个因子（比力平稳度 s_a）：
        #   bell(|a|-1g)   抓"线性加速度大到改变了模长"（对水平加速只有二阶敏感）
        #   转速降权        抓"转弯的向心加速度污染"（iNav 的原意）
        #   s_a 平稳度      抓"手抖/振动"——这个既不改变模长也不是转动，前两个因子都抓不到，
        #                  而它正是本记录里悬空段的主导污染（s_a 中位 30.6 mg vs 桌面段 3~7 mg）
        wgt = (bell(an - 1.0, ACC_BELL)
               * np.clip((TRIM_RATE-rate[k])/(TRIM_RATE-RATE_LO), 0.0, 1.0)
               * np.clip((SA_HI-sa[k])/(SA_HI-SA_LO), 0.0, 1.0))
        WGT[k] = wgt
        TILT[k] = np.degrees(np.arccos(np.clip(np.dot(a_b/an, up_est), -1, 1)))
        if ATT == 'trim':
            # 连续牵引：把姿态连续地拉向实测重力方向，权重 = bell(|a|-1g) × 转速降权。
            # 与固件对陀螺零偏的牵引同构（连续指数牵引 + 只在可信时施加）。
            #   e_err = â_b × ĝ_est（机体系，把 â_b 转到 u 的旋转轴）
            #   dq_trim <- dq_trim ⊗ exp( (Kp*e + ∫Ki*e) * w * dt )
            integ += KITRIM*e_err*wgt*DT
            dq_trim = qmul(dq_trim, expq((KTRIM*e_err + integ)*wgt*DT))
            dq_trim /= np.linalg.norm(dq_trim)
        # 自积分对照姿态（只吃离线零偏，不含任何加速度计修正）
        qg = qmul(qg, expq((wr[k] - bg)*DT))
        qg /= np.linalg.norm(qg)

        # ZUPT
        if still[k]:
            if not prev_still:
                # 刚进入静止：用速度残差反观加速度零偏（水平分量的唯一观测途径）
                T = (k - seg_start)*DT
                if T > 0.5 and np.linalg.norm(v) > 1e-6:
                    Rm_ = Rm(q)
                    # 符号：ba 偏小 δ(LSB) 时比力偏大 δ/ACC_S，速度朝 +R·δ 漂，
                    # 故修正量 = +ACC_S·R^T·v/(G·T)（写成减号会变成正反馈，实测把零偏推到 +97 LSB）
                    ba = ba + KZ*(ACC_S*(Rm_.T @ v)/(G*T))
                    Ba_lim = 200.0
                    ba = np.clip(ba, ACC_B-Ba_lim, ACC_B+Ba_lim)
                    zupt_list.append((k/FPS, np.linalg.norm(v), T))
            v = np.zeros(3)
            seg_start = k
        else:
            if prev_still:
                seg_start = k
            v = v + (R @ (a_b*G) + np.array([0.0, 0.0, -G]))*DT
        p = p + v*DT
        P[k] = p; V[k] = v; BA[k] = ba; BG[k] = bg
        Q[k] = q; QG[k] = qg
        prev_still = bool(still[k])

    # 融合姿态与纯陀螺姿态的夹角 = 加速度计这一路替陀螺挡掉的漂移；有界即达标
    drift = np.degrees(2*np.arccos(np.clip(np.abs((Q*QG).sum(1)), -1.0, 1.0)))
    print("\n静止段 %d 个，共 %.1f s（%.0f%%）" % (len(segs), still.sum()/FPS, 100.0*still.sum()/N))
    print("\n验收 1：角度漂移是否有界")
    print("   指标 = 固件姿态与「自己用原始陀螺重积分」姿态的夹角（= 固件处理链挡掉的漂移）")
    print("   全程最大 %.4f deg   末值 %.4f deg   中位 %.4f deg"
          % (drift.max(), drift[-1], np.median(drift)))
    print("   静止段上的姿态倾角误差（对加速度计参考，单位 deg）   当前姿态源 ATT=%s:" % ATT)
    print("      %-18s %12s %12s" % ("段", "自积分", "当前姿态"))
    for (s, e) in segs:
        sl2 = slice(s+int(0.2*FPS), max(s+int(0.2*FPS)+1, e-int(0.2*FPS)))
        m = (acc_lsb[sl2].mean(0)-ba)/ACC_S
        m = m/np.linalg.norm(m)
        row = []
        for QQ in (QG, Q):
            qm = QQ[sl2].mean(0); qm = qm/np.linalg.norm(qm)
            u = Rm(qm).T @ np.array([0.0, 0.0, 1.0])
            row.append(np.degrees(np.arccos(np.clip(np.dot(m, u), -1, 1))))
        print("      %6.2f~%-9.2f s %12.4f %12.4f" % (s/FPS, e/FPS, row[0], row[1]))
    print("\n验收 2：速度是否有界 —— 每次进入静止时的速度残差（应逐次收敛）:")
    for (t, ve, T) in zupt_list:
        print("   t=%6.2f s  进入静止前 |v|=%.4f m/s（已积 %.1f s）  -> 折算零偏修正 %.2f LSB"
              % (t, ve, T, KZ*ve/G/T*np.linalg.norm(ACC_S)))
    print("   全程最大 |v| = %.4f m/s" % np.linalg.norm(V, axis=1).max())
    print("   陀螺零偏: 离线初值 %s dps（在线修正由固件处理函数 1 完成，此处不再重复）"
          % np.array2string(BG0, precision=4))
    print("   加速度零偏: 清零后 %s -> 末尾 %s LSB（在线修正 %s）"
          % (np.array2string(BA[0], precision=2), np.array2string(BA[-1], precision=2),
             np.array2string(BA[-1]-BA[0], precision=2)))

    print("\n验收 3：位置（高次项，参考）")
    print("   末端 |p| = %.3f m   路径 = %.3f m   最大位移 = %.3f m"
          % (np.linalg.norm(P[-1]), np.abs(np.diff(P, axis=0)).sum(), np.linalg.norm(P, axis=1).max()))
    print("   真值：正方形 2 圈 = 8 条 10 cm 边 = 0.80 m，另加抬 10 cm + 降 10 cm = 0.20 m，合计 1.00 m；末端 0")
    print("\n逐运动段位移（真值：每条边水平 0.10 m，抬/降垂直 ±0.10 m）:")
    for k in range(1, len(segs)):
        ps, pe = segs[k-1][1], segs[k][0]
        if pe - ps < 0.2*FPS:
            continue
        dp = P[pe-1] - P[ps]
        print("   %6.2f~%-6.2f s (%5.2f s)  dp=[%+7.4f %+7.4f %+7.4f]  水平 %.4f m  垂直 %+7.4f m  末速 %.4f  s_a中位 %.2f mg"
              % (ps/FPS, pe/FPS, (pe-ps)/FPS, dp[0], dp[1], dp[2],
                 np.hypot(dp[0], dp[1]), dp[2], np.linalg.norm(V[pe-1]),
                 np.median(sa[ps:pe])))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa
    t = np.arange(N)/FPS
    fig = plt.figure(figsize=(15, 9))
    a1 = fig.add_subplot(2, 3, 1, projection='3d')
    a1.plot(P[:, 0], P[:, 1], P[:, 2], lw=1.0)
    a1.scatter(P[0, 0], P[0, 1], P[0, 2], c='g', s=40, label='start')
    a1.scatter(P[-1, 0], P[-1, 1], P[-1, 2], c='r', s=40, label='end')
    a1.set_xlabel('X [m]'); a1.set_ylabel('Y [m]'); a1.set_zlabel('Z [m]')
    a1.set_title('3D'); a1.legend(fontsize=8)
    a2 = fig.add_subplot(2, 3, 2)
    # 自动缩放到"前 90% 位移"的量程：失控段会把量程拉到米级，把真正的 10 cm 正方形压成一个点
    lim = np.percentile(np.abs(P - P[0]), 60, axis=0)
    lim = max(0.15, float(np.max(lim[:2]))*1.6)
    a2.plot(P[:, 0], P[:, 1], lw=1.0)
    a2.plot(P[0, 0], P[0, 1], 'go'); a2.plot(P[-1, 0], P[-1, 1], 'rs')
    a2.set_xlim(P[0, 0]-lim, P[0, 0]+lim); a2.set_ylim(P[0, 1]-lim, P[0, 1]+lim)
    a2.set_xlabel('X [m]'); a2.set_ylabel('Y [m]')
    a2.set_title('top view, zoom +-%.2f m (full end %.2f m)' % (lim, np.linalg.norm(P[-1])))
    a2.grid(alpha=.3); a2.axis('equal')
    a3 = fig.add_subplot(2, 3, 3)
    for j, c in enumerate('xyz'):
        a3.plot(t, P[:, j], label=c)
    for (s, e) in segs:
        a3.axvspan(s/FPS, e/FPS, color='0.88')
    a3.set_ylim(-lim, lim)
    a3.set_xlabel('t [s]'); a3.set_ylabel('p [m]'); a3.legend(fontsize=8)
    a3.set_title('position (zoom +-%.2f m, gray=static)' % lim); a3.grid(alpha=.3)
    a4 = fig.add_subplot(2, 3, 4)
    a4.plot(t, np.linalg.norm(V, axis=1)); a4.set_yscale('log')
    for (s, e) in segs:
        a4.axvspan(s/FPS, e/FPS, color='0.88')
    a4.set_xlabel('t [s]'); a4.set_ylabel('|v| [m/s]'); a4.set_title('speed (log)'); a4.grid(alpha=.3)
    a5 = fig.add_subplot(2, 3, 5)
    a5.plot(t, TILT); a5.plot(t, WGT*10, lw=.7)
    for (s, e) in segs:
        a5.axvspan(s/FPS, e/FPS, color='0.88')
    a5.set_xlabel('t [s]'); a5.set_ylabel('deg / w*10')
    a5.set_title('tilt err vs accel reference / weight'); a5.grid(alpha=.3)
    a6 = fig.add_subplot(2, 3, 6)
    for j, c in enumerate('xyz'):
        a6.plot(t, BA[:, j]-ACC_B[j], label='dba_'+c)
    a6.set_xlabel('t [s]'); a6.set_ylabel('LSB'); a6.legend(fontsize=8)
    a6.set_title('accel bias online correction'); a6.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print("\n图: %s" % out)


if __name__ == '__main__':
    main()
