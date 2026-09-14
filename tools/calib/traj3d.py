# -*- coding: utf-8 -*-
"""
从本次连续六面记录反解陀螺的空间路线（轨迹）。

输入：7 路 float = q(w,x,y,z) + accel_raw(3) LSB（±16 g 档，标称 2048 LSB/g）
标定：零偏 b、倍率 S 取自 accel_ellipse_cal.py（绕 x/y/z 三条单轴旋转记录，椭圆法，
      每轴两个独立来源；本地 g=9.7985）。旧的六面法值仅作对照。
输出：traj3d.png

算法（四步）
  1) 加速度校正      a_b = (accel_lsb - b) / S                      [m/s²] 机体系比力
  2) 静态段拉平      静止时加速度计给出真实"上"方向，用它把四元数的倾角拉正
                     （只修倾角，不动 yaw —— 加速度计看不到 yaw）
  3) 去重力          a_nav = R(q)·a_b + g_nav,  g_nav = [0,0,-g]   导航系线加速度
                     削顶帧（|lsb|>=32760）与冲击窗口标记无效，不参与积分
  4) 积分 + ZUPT     v += a·dt; p += v·dt;  静止段内 v≡0
"""
import os, re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa

_here = os.path.dirname(os.path.abspath(__file__))
for _p in (_here, os.path.join(_here, 'h415-imu42605-', 'tools', 'calib')):
    if os.path.isfile(os.path.join(_p, 'jf_load.py')):
        sys.path.insert(0, _p)
        break
from jf_load import load_jf      # 流式解析 + npy 缓存

G = 9.7985
LSB_PER_G = 2048.0
FN = sys.argv[1] if len(sys.argv) > 1 else 'serial_runtime_20260913_225736_268_export.txt'
OUT = sys.argv[2] if len(sys.argv) > 2 else 'traj3d.png'
fps = 8027.0
dt = 1.0/fps
# accel_ellipse_cal.py 解出的标定（±16 g，绕 x/y/z 三条记录，每轴两源均值）
B_ELL = np.array([-10.10, -15.42, 43.72])           # LSB
S_ELL = np.array([2028.48, 2040.78, 2016.07])/G     # LSB/(m/s^2)
# sixface16.py 的旧值，作对照（OPT_CAL=six 时启用）
B_SIX = np.array([-10.31, -13.66, 45.68])
S_SIX = np.array([2027.501, 2039.529, 2015.479])/G
_use_six = os.environ.get('OPT_CAL', 'ell') == 'six'
B, S = (B_SIX, S_SIX) if _use_six else (B_ELL, S_ELL)
print("标定组: %s   B=%s  S*G=%s" % ('六面法(旧)' if _use_six else '椭圆法(新)',
      np.array2string(B, precision=2), np.array2string(np.array(S)*G, precision=3)))

a = np.asarray(load_jf(FN), dtype=np.float64)
q = a[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
acc = a[:, 4:7]
N = len(q)


def R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def qmul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                     a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])


def rotvec_to_q(v):
    t = np.linalg.norm(v)
    if t < 1e-15:
        return np.array([1.0, 0, 0, 0])
    return np.concatenate([[np.cos(t/2)], np.sin(t/2)*v/t])


# ---- 1) 加速度校正 ----
a_b = (acc - B)/S                                  # m/s², 机体系比力

# ---- 静态段：陀螺门 AND 加速度门 ----
# 陀螺判不了平动（平动时 |w|≈0），必须与加速度门取"与"。
# 加速度门参数取自 accel_gate_sim.py 的实测标定：W=0.5 s、T_ON=10sigma_s、OFF去抖 1.5 s。
rate = np.degrees(2*np.arccos(np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)))/dt
g_still = np.concatenate([[rate[0] < 10.0], rate < 10.0])

W_A, K_A, DEB_ON_A, DEB_OFF_A = 4000, 10.0, 16, 12000
# 阈值用 accel_gate_sim.py 实测标定的定值（不应在一条数据上现算：
# 单窗 σ_s 会把慢变成分剔掉，得到 0.047 mg -> T_ON=0.47 mg，低于静止 p99.9=1.45 mg，
# 虚警把门一直关着。实测分布：静止 p99.9=1.45 mg、运动中位=2.89 mg。）
T_ON, T_OFF = 1.92, 1.15                   # mg


def gate_and_segs(a_b):
    """加速度门 + 与门静止段。返回 (still, segs, acc_still, s_a)"""
    M = np.cumsum(np.vstack([np.zeros((1, 3)), a_b]), axis=0)
    kk = np.arange(N)
    a1 = np.maximum(kk+1-W_A, 0); b1 = kk+1
    m_now = (M[b1]-M[a1])/np.maximum(b1-a1, 1)[:, None]
    a2 = np.maximum(kk+1-2*W_A, 0); b2 = np.maximum(kk+1-W_A, 0)
    m_del = (M[b2]-M[a2])/np.maximum(b2-a2, 1)[:, None]
    s_a = np.abs(m_now-m_del).max(axis=1)*1000/G
    acc_still = np.ones(N, dtype=bool)
    c_on = c_off = 0
    for k in range(2*W_A, N):
        acc_still[k] = acc_still[k-1]
        if acc_still[k]:
            if s_a[k] > T_ON:
                c_on += 1
                if c_on >= DEB_ON_A:
                    acc_still[k] = False; c_on = 0
            else:
                c_on = 0
        else:
            if s_a[k] < T_OFF:
                c_off += 1
                if c_off >= DEB_OFF_A:
                    acc_still[k] = True; c_off = 0
            else:
                c_off = 0
    st = g_still & acc_still
    sg = []
    i = 0
    while i < N:
        if st[i]:
            j = i
            while j < N and st[j]:
                j += 1
            if (j-i)/fps > 0.8:
                sg.append((i, j))
            i = j
        else:
            i += 1
    return st, sg, acc_still, s_a


print("加速度门: T_ON=%.2f mg  T_OFF=%.2f mg  (定值, from accel_gate_sim.py)" % (T_ON, T_OFF))
still, segs, acc_still, s_a = gate_and_segs(a_b)
print("陀螺门静止 %.1f s / 加速度门静止 %.1f s / 与门静止 %.1f s"
      % (g_still.sum()/fps, acc_still.sum()/fps, still.sum()/fps))

# ---- 1b) 可选：用起始静止段对零偏做本场次清零（OPT_ZERO=1）----
# 离线标定管得住标度（三轴两源互差 0.004%~0.06%），但管不住零偏：零偏是每场次量，
# 实测各场次有效 b_z 相差可达 ~2 LSB（1 mg），足以让 35 s 的积分偏出 0.3 m/s。
# 这里只用**运动开始前的第一段静止**（真实系统能拿到的信息）把 |a_b| 归一化到 1 g，
# 不碰标度，也不看后面的静止段。
OPT_ZERO = os.environ.get('OPT_ZERO', '0') == '1'
if OPT_ZERO and segs:
    s0, e0 = segs[0]
    sl = slice(s0+int(0.2*fps), e0-int(0.2*fps))
    if sl.stop <= sl.start:
        sl = slice(s0, e0)
    m0 = acc[sl].mean(0)
    e_rel = np.linalg.norm((m0-B)/S) - G          # 当前标定在该姿态下的模长误差
    B_new = m0 - (m0 - B)*G/np.linalg.norm((m0-B)/S)
    print("\n本场次零偏清零：用 %.2f~%.2f s 的起始静止段" % (s0/fps, e0/fps))
    print("  清零前 |a_b|-g = %+.3f mg   B = [%s]" % (e_rel/G*1000, np.array2string(B, precision=3)))
    print("  清零后 |a_b|-g = %+.3f mg   B = [%s]"
          % ((np.linalg.norm((m0-B_new)/S)-G)/G*1000, np.array2string(B_new, precision=3)))
    B = B_new
    a_b = (acc - B)/S
    still, segs, acc_still, s_a = gate_and_segs(a_b)
    print("  清零后：陀螺门 %.1f s / 加速度门 %.1f s / 与门静止 %.1f s，静止段 %d 个"
          % (g_still.sum()/fps, acc_still.sum()/fps, still.sum()/fps, len(segs)))

print("静止段 %d 个，共 %.1f s（占 %.0f%%）"
      % (len(segs), still.sum()/fps, 100*still.sum()/N))


# ---- 2) 用加速度计修正姿态（两种模式）----
# 重力泄露 = g*sin(姿态倾角误差)。1 mg 的零偏只值 0.0098 m/s²，而 0.1 deg 的倾角就值
# 17 mg —— 所以运动段的漂移只可能来自这里，不可能来自零偏。
#   模式 A（OPT_CF=0，旧做法）：只在静止段用加速度计拉平，运动期间靠陀螺裸推。
#   模式 B（OPT_CF>0，默认）：陀螺 + 加速度计互补滤波，时间常数 OPT_CF 秒。
#       只在 ||a_b|-g| < OPT_CF_BAND 时才把倾角往实测比力方向拉 —— 这个条件意味着
#       "线性加速度小"，此时比力方向就是重力方向，修正不会把真实加速度当重力吃掉。
OPT_CF = float(os.environ.get('OPT_CF', '1.0'))
CF_BAND = float(os.environ.get('OPT_CF_BAND', '30'))     # mg
CF_GATE = os.environ.get('OPT_CF_GATE', 'sa')            # 'sa' | 'mag'

if OPT_CF > 0:
    # 符号：要让 R(q⊗dq)^T z 等于 â_b，需 R(dq)*â_b = u，即 dq 是 â_b -> u 的旋转。
    # 两种门控：
    #   'mag' —— ||a_b|-g| < CF_BAND。标准做法。对**水平**线性加速度不敏感（只有二阶
    #            影响：0.3 m/s² 才值 0.46 mg），所以它不区分"倾斜"和"水平加速"，
    #            靠 τ 够长把瞬时加速平均掉。
    #   'sa'  —— 加速度动静统计量 s_a < T_OFF。只在真静止时开，不会被水平加速骗，
    #            但运动段完全靠陀螺裸推，泄漏照旧。
    gain = 1.0 - np.exp(-dt/OPT_CF)
    dq_cf = np.array([1.0, 0, 0, 0])
    q_corr = np.empty_like(q)
    ab_norm = np.linalg.norm(a_b, axis=1)
    open_cnt = 0
    for k in range(N):
        op = (abs(ab_norm[k] - G) < CF_BAND/1000.0*G) if CF_GATE == 'mag' else bool(acc_still[k])
        if op:
            open_cnt += 1
            u = R(qmul(q[k], dq_cf)).T @ np.array([0.0, 0, 1.0])
            ub_ = a_b[k]/ab_norm[k]
            cr = np.cross(ub_, u)                  # â_b -> u
            n = np.linalg.norm(cr)
            if n > 1e-12:
                ang = np.arctan2(n, np.dot(ub_, u))*gain
                dq_cf = qmul(dq_cf, rotvec_to_q(ang*cr/n))
        q_corr[k] = qmul(q[k], dq_cf)
    q = q_corr
    print("姿态修正：互补滤波 tau=%.2f s，门控 %s（%s），修正开启时间占 %.0f%%"
          % (OPT_CF, CF_GATE, ('%.0f mg 模长带' % CF_BAND) if CF_GATE == 'mag' else 's_a',
             100.0*open_cnt/N))
else:
    # 只施加在静止段内部不够：段之间的运动帧若不带上一段解出的修正，
    # 运动期间会一直用那个倾斜的姿态去重力 → 漏重力。所以修正必须向后传播。
    q_corr = q.copy()
    dq_cur = np.array([1.0, 0, 0, 0])
    tilt_before = []
    prev = 0
    for (s, e) in segs:
        for k in range(prev, s):                   # 本节之前的运动帧：带上一段的修正
            q_corr[k] = qmul(q[k], dq_cur)
        sl = slice(s+int(0.2*fps), e-int(0.2*fps))
        if sl.stop <= sl.start:
            sl = slice(s, e)
        u_meas = a_b[sl].mean(0); u_meas /= np.linalg.norm(u_meas)
        qm_raw = q[sl].mean(0); qm_raw /= np.linalg.norm(qm_raw)
        u_est_raw = R(qm_raw).T @ np.array([0.0, 0, 1.0])
        cr = np.cross(u_meas, u_est_raw)
        n = np.linalg.norm(cr)
        tilt_before.append(np.degrees(np.arccos(np.clip(np.dot(u_meas, u_est_raw), -1, 1))))
        dq_cur = rotvec_to_q(np.arctan2(n, np.dot(u_meas, u_est_raw))*cr/n) if n > 1e-12 \
            else np.array([1.0, 0, 0, 0])
        for k in range(s, e):
            q_corr[k] = qmul(q[k], dq_cur)
        prev = e
    for k in range(prev, N):
        q_corr[k] = qmul(q[k], dq_cur)
    q = q_corr
    print("各静止段拉平前的倾角残差（相对原始陀螺姿态）: %s deg"
          % np.array2string(np.array(tilt_before), precision=4))

# 拉平后复核
after = []
for (s, e) in segs:
    sl = slice(s+int(0.2*fps), e-int(0.2*fps))
    if sl.stop <= sl.start:
        sl = slice(s, e)
    u_meas = a_b[sl].mean(0); u_meas /= np.linalg.norm(u_meas)
    qm = q[sl].mean(0); qm /= np.linalg.norm(qm)
    u_est = R(qm).T @ np.array([0.0, 0, 1.0])
    after.append(np.degrees(np.arccos(np.clip(np.dot(u_meas, u_est), -1, 1))))
print("拉平后各静止段倾角误差: %s deg"
      % np.array2string(np.array(after), precision=4))

# ---- 标定误差的直接量度：静止段 |a_b| 与 1 g 的差 ----
# 静止时 a_b 还原的就是 -重力矢量，模长恒为 g，与姿态无关。
# 所以 (|a_b| - g) 只反映"这一刻的零偏/标度误差"，不受倾角、不受拉平影响，
# 是判断某套标定在这条记录上好不好用的唯一干净指标。
print("\n静止段 |a_b| 与 g 的差（与姿态无关，直接量度标定误差）:")
print("   %-16s %10s %10s %10s   %s" % ("段(s)", "|a_b|-g", "沿a_b分量", "倾角deg", "a_b单位方向"))
for (s, e) in segs:
    sl = slice(s+int(0.2*fps), e-int(0.2*fps))
    if sl.stop <= sl.start:
        sl = slice(s, e)
    m = a_b[sl].mean(0)
    mag = np.linalg.norm(m)
    u = m/mag
    print("   %5.2f~%-9.2f %+9.3f mg %+9.3f mg %9.3f   (%+.4f %+.4f %+.4f)"
          % (s/fps, e/fps, (mag-G)/G*1000, (mag-G)/G*1000,
             np.degrees(np.arccos(np.clip(u[2], -1, 1))), u[0], u[1], u[2]))
print("   => 若这一列整体偏离 0，说明本场次的有效零偏与标定值不同（温度/时段漂移）")

# ---- 3) 去重力 ----
g_nav = np.array([0.0, 0.0, -G])
a_nav = np.empty((N, 3))
for k in range(N):
    a_nav[k] = R(q[k]) @ a_b[k] + g_nav

# 削顶 / 冲击窗口失效
clip = (np.abs(acc) >= 32760).any(axis=1)
bad = np.zeros(N, dtype=bool)
w = int(0.05*fps)                      # 冲击前后各 50 ms 一并作废
idx = np.flatnonzero(clip)
for k in idx:
    bad[max(0, k-w):min(N, k+w)] = True
print("削顶帧 %d，冲击窗口失效 %.2f s" % (clip.sum(), bad.sum()/fps))

# ---- 3b) 逐运动段去掉零均值加速度误差（ZUPT 约束）----
# 门控已经告诉系统"每段的起止都是静止"，所以段内 ∫a·dt 必须为 0。
# 这里把每段的 a_nav 减去它的均值，等价于用段两端的速度约束去估这一段的有效零偏。
# 注意这不是拿真值作弊：门控边界是系统自己实时判出来的，属于 ZUPT 的标准用法。
# 它只能消掉段内的**常值**误差，零均值部分仍然残留 —— 那部分才是真正的积分噪声。
OPT_TREND = os.environ.get('OPT_TREND', '1') == '1'
if OPT_TREND:
    print("\n逐运动段零均值化（ZUPT 约束）:")
    for k in range(1, len(segs)):
        ps, pe = segs[k-1][1], segs[k][0]
        if pe <= ps:
            continue
        T = (pe-ps)*dt
        dv = a_nav[ps:pe].sum(0)*dt
        a_nav[ps:pe] -= dv/T
        print("   %5.2f~%5.2f s (%5.2f s)  扣除 dv=%s  |%.4f| m/s  -> 均值误差 %+.3f mg"
              % (ps/fps, pe/fps, T, np.array2string(dv, precision=4), np.linalg.norm(dv),
                 np.linalg.norm(dv)/T/G*1000))


# ---- 4) 积分 + ZUPT ----
v = np.zeros(3); p = np.zeros(3)
P = np.zeros((N, 3)); V = np.zeros((N, 3))
for k in range(N):
    if not still[k] and not bad[k]:
        v = v + a_nav[k]*dt
        p = p + v*dt
    if still[k]:
        v = np.zeros(3)
    P[k] = p; V[k] = v

print("\n各运动段 ∫a·dt（起止都静止，应≈0）—— 这是加速度系统误差的直接量度：")
tot = np.zeros(3)
for k in range(1, len(segs)):
    ps, pe = segs[k-1][1], segs[k][0]
    if pe <= ps:
        continue
    dv = a_nav[ps:pe].sum(0)*dt
    tot += dv
    print("   %5.2f~%5.2f s (%5.2f s)  dv=[%+8.4f %+8.4f %+8.4f]  |%.4f| m/s"
          % (ps/fps, pe/fps, (pe-ps)/fps, dv[0], dv[1], dv[2], np.linalg.norm(dv)))
print("   合计 dv = %s  |%.4f| m/s" % (np.array2string(tot, precision=4), np.linalg.norm(tot)))

print("\n末端位置 = %s m   |p| = %.3f m   末端速度 |v| = %.4f m/s"
      % (np.array2string(P[-1], precision=4), np.linalg.norm(P[-1]), np.linalg.norm(V[-1])))
print("路径长度 = %.3f m   最大位移 = %.3f m"
      % (np.abs(np.diff(P, axis=0)).sum(), np.linalg.norm(P, axis=1).max()))

# 逐运动段位移：真值应是 4 条 10 cm 水平边 + 抬 0.105 m + 降 0.105 m
print("\n逐运动段位移（真值：正方形 4 条 10 cm 边，再原地 +0.105 m 抬、-0.105 m 降）:")
for k in range(1, len(segs)):
    ps, pe = segs[k-1][1], segs[k][0]
    if pe - ps < 0.2*fps:
        continue
    dp = P[pe-1] - P[ps]
    print("   %5.2f~%5.2f s (%5.2f s)  dp=[%+7.4f %+7.4f %+7.4f]  水平 %.4f m  垂直 %+7.4f m  三维 %.4f m"
          % (ps/fps, pe/fps, (pe-ps)/fps, dp[0], dp[1], dp[2],
             np.hypot(dp[0], dp[1]), dp[2], np.linalg.norm(dp)))

# ---- 重力泄露体检 ----
# 静止段拉平后 a_nav 必须≈0（否则就是没拉干净）；运动段的水平分量若不是重力泄露，
# 量级只可能是 mg —— 一旦看到 0.01~0.1 m/s² 的水平常值，等价倾角就是 0.06~0.6 deg，
# 那只能是重力泄露，不可能是零偏（1 mg 才 0.0098 m/s²，且拉平已经把该方向吃掉）。
print("\n静止段 a_nav（拉平后应≈0）:")
for (s, e) in segs:
    sl = slice(s+int(0.2*fps), e-int(0.2*fps))
    if sl.stop <= sl.start:
        sl = slice(s, e)
    m = a_nav[sl].mean(0)
    print("   %5.2f~%5.2f s  a_nav=[%+8.4f %+8.4f %+8.4f] m/s^2  水平 %.4f (等价倾角 %.3f deg)"
          % (s/fps, e/fps, m[0], m[1], m[2], np.hypot(m[0], m[1]),
             np.degrees(np.arcsin(min(1.0, np.hypot(m[0], m[1])/G)))))

    seg = int(os.environ.get('OPT_SEG', len(segs)-1))
if 1 <= seg < len(segs):
    ps, pe = segs[seg-1][1], segs[seg][0]
    # 姿态倾角体检：R(q)^T z 与实测比力方向的夹角。重力泄露 = g*sin(这个角)。
    ub = a_b/np.linalg.norm(a_b, axis=1, keepdims=True)
    up = np.empty((N, 3))
    for k in range(N):
        up[k] = R(q[k]).T @ np.array([0.0, 0.0, 1.0])
    tilt = np.degrees(np.arccos(np.clip((ub*up).sum(1), -1, 1)))
    print("\n第 %d 段 %.2f~%.2f s 逐 0.5 s 明细:" % (seg, ps/fps, pe/fps))
    print("   %7s %7s %6s %9s %9s %9s %8s %9s %8s %9s" %
          ("t(s)", "s_a[mg]", "still", "a_x", "a_y", "a_z", "|a_b|g", "倾角deg", "泄露mg", "z[m]"))
    k0 = max(0, ps-int(1.5*fps))
    for k in range(k0, min(N-1, pe), int(0.5*fps)):
        print("   %7.2f %7.2f %6d %+9.4f %+9.4f %+9.4f %8.4f %9.4f %8.2f %+9.4f"
              % (k/fps, s_a[k], still[k], a_nav[k, 0], a_nav[k, 1], a_nav[k, 2],
                 np.linalg.norm(a_b[k]), tilt[k], np.sin(np.radians(tilt[k]))*1000, P[k, 2]))




# ---- 绘图 ----
t = np.arange(N)/fps
fig = plt.figure(figsize=(14, 9))

ax1 = fig.add_subplot(2, 3, 1, projection='3d')
ax1.plot(P[:, 0], P[:, 1], P[:, 2], lw=1.2)
ax1.scatter(P[0, 0], P[0, 1], P[0, 2], c='g', s=40, label='start')
ax1.scatter(P[-1, 0], P[-1, 1], P[-1, 2], c='r', s=40, label='end')
for s, e in segs:
    ax1.scatter(P[s, 0], P[s, 1], P[s, 2], c='k', s=10)
ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]'); ax1.set_zlabel('Z [m]')
ax1.set_title('3D trajectory (black dots = ZUPT static segments)')
ax1.legend(fontsize=8)

ax2 = fig.add_subplot(2, 3, 2)
ax2.plot(P[:, 0], P[:, 1], lw=1.2)
ax2.plot(P[0, 0], P[0, 1], 'go'); ax2.plot(P[-1, 0], P[-1, 1], 'rs')
ax2.set_xlabel('X [m]'); ax2.set_ylabel('Y [m]'); ax2.set_title('top view (XY)')
ax2.grid(alpha=.3); ax2.axis('equal')

ax3 = fig.add_subplot(2, 3, 3)
ax3.plot(t, P[:, 0], label='X'); ax3.plot(t, P[:, 1], label='Y'); ax3.plot(t, P[:, 2], label='Z')
for s, e in segs:
    ax3.axvspan(s/fps, e/fps, color='0.85')
ax3.set_xlabel('t [s]'); ax3.set_ylabel('position [m]'); ax3.legend(fontsize=8)
ax3.set_title('position vs time (gray = static)'); ax3.grid(alpha=.3)

ax4 = fig.add_subplot(2, 3, 4)
ax4.plot(t, np.linalg.norm(V, axis=1))
for s, e in segs:
    ax4.axvspan(s/fps, e/fps, color='0.85')
ax4.set_xlabel('t [s]'); ax4.set_ylabel('|v| [m/s]'); ax4.set_title('speed'); ax4.grid(alpha=.3)

ax5 = fig.add_subplot(2, 3, 5)
amag = np.linalg.norm(a_nav, axis=1)
ax5.semilogy(t, np.maximum(amag, 1e-6))
ax5.axhline(G, color='r', ls='--', lw=.8)
for s, e in segs:
    ax5.axvspan(s/fps, e/fps, color='0.85')
ax5.set_xlabel('t [s]'); ax5.set_ylabel('|a_lin| [m/s^2]')
ax5.set_title('linear accel (red = 1 g)'); ax5.grid(alpha=.3, which='both')

ax6 = fig.add_subplot(2, 3, 6)
ax6.plot(t, np.degrees(np.arccos(np.clip((q[:, 0]**2+q[:, 1]**2-q[:, 2]**2-q[:, 3]**2), -1, 1))), label='roll')
ax6.plot(t, np.degrees(np.arcsin(np.clip(2*(q[:, 0]*q[:, 2]-q[:, 3]*q[:, 1]), -1, 1))), label='pitch')
ax6.set_xlabel('t [s]'); ax6.set_ylabel('deg'); ax6.legend(fontsize=8)
ax6.set_title('roll / pitch from quaternion'); ax6.grid(alpha=.3)

fig.tight_layout()
fig.savefig(OUT, dpi=120)
print("\n图已保存: %s" % OUT)
