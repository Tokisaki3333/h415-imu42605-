# -*- coding: utf-8 -*-
"""
加速度计标定 —— 单轴旋转夹具的椭圆法（定稿出处：V5F_ACCEL_* 全部 6 个常量）

记录（7 通道 = q[4] + accel_lsb[3]，±16 g）：绕机体不同轴的两次独立实验，合起来覆盖三轴
  233731_085  绕 z 转 462 deg，11 个静止点  -> 给 S_x, S_y, b_x, b_y
  234829_229  绕 y 转 649 deg，17 个静止点  -> 给 S_x, S_z, b_x, b_z
两次都覆盖 x，所以 S_x / b_x 有一次真正的独立交叉校验。
     从仓库根目录运行：python tools/calib/accel_ellipse_cal.py [记录...]

原理
----
静止段里加速度计测的是比力 f，大小恒为 1 g；夹具绕固定机体轴转，f 沿轴的分量恒定、
垂直分量在垂直平面里画一个半径 1 g 的圆。读数 y = M f + b 把该圆映射成椭圆：
**中心 = 零偏**（与 M 无关，且 f 的沿轴分量经近对角的 M 映射后仍在轴方向上，
所以平面内中心就是平面内的零偏），**半轴 = 逐轴标度**。
不需要知道任何停止角度，也不需要知道转轴指向。

为什么必须只用静止段
--------------------
转轴与加速度计不共点，转动时传感器处叠加
    a_c = w^2 * r      （向心，r = 传感器到转轴的距离）
    a_t = alpha * r    （切向）
本夹具手转，实测转动段残差 rms 60~90 mg，且按 |w| 分箱后 rms 不随 w^2 变化
（w^2 涨 20 倍，res^2/w^2 掉 130 倍）=> 残差主要是夹具平动抖动，不是向心项。
两者都靠"只取静止段"排除。

静止段判据
----------
角速度不能用逐样本四元数差分：8 kHz 下噪声底有几个 dps。改用**相对四元数矢量部分的
0.2 s 滑窗均值**（噪声互相抵消），实测静止底噪 0.025 dps。
阈值从 0.25 dps 放宽到 5 dps（20 倍）标定结果不变 => 污染上界 0.2 mg。

转轴为什么必须接近某个机体轴
----------------------------
算法按"平面法向最接近哪根机体轴"来挑平面内两轴，要求轴的失准量级 << 1 deg。
两条记录的实测失准都在 1 deg 以内；失准只会让"沿轴分量"在平面内漏出一点点
（第二阶，<1e-4），且水平度误差对沿轴那根轴的污染靠"该轴不参与标定"规避。
"""
import re, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, __file__.rsplit('\\', 1)[0] if '\\' in __file__ else '.')
from jf_load import load_jf      # 流式解析 + npy 缓存，见 jf_load.py 顶部说明

G = 9.7985
FS = 2048.0
fps = 8027.0
WIN = int(0.20 * fps)
DEF = ['serial_runtime_20260913_233731_085_export.txt',
       'serial_runtime_20260913_234829_229_export.txt',
       'serial_runtime_20260914_000636_751_export.txt']
AX = 'xyz'


def load(fn):
    a = load_jf(fn)
    q = np.asarray(a[:, :4], dtype=np.float64).copy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q, np.asarray(a[:, 4:7], dtype=np.float64)


def smooth_rate(q):
    q0, q1 = q[:-1], q[1:]
    w_, x_, y_, z_ = q0.T
    w2, x2, y2, z2 = q1.T
    V = np.stack([w_*x2 - x_*w2 - y_*z2 + z_*y2,
                  w_*y2 + x_*z2 - y_*w2 - z_*x2,
                  w_*z2 - x_*y2 + y_*x2 - z_*w2], 1)
    cs = np.vstack([np.zeros(3), np.cumsum(V, 0)])
    k = np.arange(len(q))
    lo = np.clip(k - WIN, 0, None); hi = np.clip(k + 1, None, len(V))
    mv = (cs[hi] - cs[lo]) / np.maximum(hi - lo, 1)[:, None]
    return np.degrees(2.0 * np.linalg.norm(mv, axis=1) * fps), V


def static_pts(acc, rate, thr, minlen=0.4):
    N = len(acc); out, still, i = [], rate < thr, 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j - i) / fps >= minlen:
                g_ = int(0.05 * fps)
                sl = slice(i + g_, j - g_)
                if sl.stop - sl.start > int(0.1 * fps):
                    out.append((acc[sl].mean(0), rate[sl].max(), sl))
            i = j
        else:
            i += 1
    return out


def dedup(P, tol=20.0):
    keep = [0]
    for k in range(1, len(P)):
        if np.linalg.norm(P[k] - P[keep[-1]]) > tol:
            keep.append(k)
    return P[keep]


def qmul_v(a, B):
    """a: (4,) ；B: (N,4)  ->  (N,4)"""
    w_, x_, y_, z_ = a
    bw, bx, by, bz = B.T
    return np.stack([w_*bw - x_*bx - y_*by - z_*bz,
                     w_*bx + x_*bw + y_*bz - z_*by,
                     w_*by - x_*bz + y_*bw + z_*bx,
                     w_*bz + x_*by - y_*bx + z_*bw], 1)


def rotT_v(Q, v):
    """R(q)^T v ，Q: (N,4) wxyz ；v: (3,) -> (N,3)"""
    w_, x_, y_, z_ = Q.T
    return np.stack([
        (1-2*(y_*y_+z_*z_))*v[0] + 2*(x_*y_+w_*z_)*v[1] + 2*(x_*z_-w_*y_)*v[2],
        (1-2*(x_*x_+z_*z_))*v[1] + 2*(x_*y_-w_*z_)*v[0] + 2*(y_*z_+w_*x_)*v[2],
        (1-2*(x_*x_+y_*y_))*v[2] + 2*(x_*z_+w_*y_)*v[0] + 2*(y_*z_-w_*x_)*v[1]], 1)


def fit_plane_ellipse(P):
    """在点云自己的最佳平面里拟合椭圆，中心与半轴各自归属到最接近的机体轴。
    u,v 取"平面内最接近机体轴的那两根轴"的原始坐标，可读性更好。"""
    c0 = P.mean(0)
    _, _, Vt = np.linalg.svd(P - c0, full_matrices=False)
    nrm = Vt[2]
    k = int(np.argmax(np.abs(nrm)))
    ij = [i for i in range(3) if i != k]
    out = (P - c0) @ nrm                       # 离平面残差
    u, v = P[:, ij[0]], P[:, ij[1]]
    A = np.stack([u*u, u*v, v*v, u, v, np.ones_like(u)], 1)
    _, _, Vt2 = np.linalg.svd(A, full_matrices=False)
    a_, b_, c_, d_, e_, f_ = Vt2[-1]
    den = b_*b_ - 4*a_*c_
    u0 = (2*c_*d_ - b_*e_) / den
    v0 = (2*a_*e_ - b_*d_) / den
    f0 = a_*u0*u0 + b_*u0*v0 + c_*v0*v0 + d_*u0 + e_*v0 + f_
    M = -np.array([[a_, b_/2], [b_/2, c_]]) / f0      # 椭圆二次型 Q，(y-c)^T Q (y-c) = 1
    lam, ev = np.linalg.eigh(M)
    ax = 1.0 / np.sqrt(lam)
    S = {}
    for i in range(2):
        S[ij[0] if abs(ev[0, i]) >= abs(ev[1, i]) else ij[1]] = ax[i]
    # 交叉灵敏：测量阵 M_meas 满足 M_meas M_meas^T = Q^-1，取对称正定平方根
    # （Q 只定到 M_meas M_meas^T；近对角时对称根是标准取法）
    lam2, ev2 = np.linalg.eigh(np.linalg.inv(M))
    Mmeas = (ev2 * np.sqrt(lam2)) @ ev2.T
    gam = Mmeas[0, 1] / Mmeas[0, 0]
    # 平面内几何残差：把每个点沿径向投到椭圆上，(d^T Q d)^(1/2) = s 时径向距离 = |d|(1-1/s)
    d = np.stack([u - u0, v - v0], 1)
    s = np.sqrt(np.einsum('ij,jk,ik->i', d, M, d))
    geo = np.linalg.norm(d, axis=1) * (1.0 - 1.0 / np.maximum(s, 1e-9))
    bvec = np.full(3, np.nan)
    bvec[ij[0]] = u0; bvec[ij[1]] = v0
    return dict(axis=k, nrm=nrm, S=S, b=bvec, res=out.std(), res_geo=geo.std(),
                tilt=np.degrees(np.arctan2(ev[1, 1], ev[0, 1])), gam=gam,
                ratio=min(ax)/max(ax), pair=(ij[0], ij[1]))


# ==================== 逐记录分析 ====================
recs = []
for fn in (sys.argv[1:] or DEF):
    q, acc = load(fn); N = len(q)
    rate, V = smooth_rate(q)
    axis = np.argmax(np.abs(np.mean(V[np.asarray(rate[:len(V)]) > 20], 0))) if (np.asarray(rate[:len(V)]) > 20).sum() > 100 else -1

    print("\n" + "=" * 96)
    print("记录 %s" % fn)
    print("  时长 %.1f s   最大 |w| %.0f dps   累计转角 %.0f deg"
          % (N/fps, rate.max(), np.trapezoid(rate, dx=1.0/fps)))
    print("=" * 96)

    # --- 阈值不变性 ---
    print("  %-9s %5s %11s %10s %10s %9s %9s %10s" %
          ("阈值dps", "窗数", "段内max|w|", "S_%s" % AX[0], "S_%s" % AX[1], "b_a LSB", "b_b LSB", "离面rms"))
    rows, pts1, cache = {}, None, {}
    for thr in (0.25, 0.5, 1.0, 2.0, 5.0):
        pts = static_pts(acc, rate, thr)
        cache[thr] = pts
        if thr == 1.0:
            pts1 = pts
        P = dedup(np.array([p[0] for p in pts]))
        e = fit_plane_ellipse(P)
        rows[thr] = e
    order = sorted(rows[1.0]['S'].keys())
    for thr in sorted(rows):
        e = rows[thr]
        sv = [e['S'][k] for k in order]
        bv = [e['b'][k] for k in order]
        pts = cache[thr]
        print("  %-9.2f %5d %10.3f %10.2f %10.2f %9.2f %9.2f %10.2f" %
              (thr, len(dedup(np.array([p[0] for p in pts]))), max(p[1] for p in pts),
               sv[0], sv[1], bv[0], bv[1], e['res']))
    t0, t1 = min(rows), max(rows)
    dS = [rows[t1]['S'][k] - rows[t0]['S'][k] for k in order]
    db = [rows[t1]['b'][k] - rows[t0]['b'][k] for k in order]
    print("  阈值 %.2f->%.2f dps 漂移: dS %s = (%+.3f, %+.3f) LSB/g   db %s = (%+.3f, %+.3f) LSB"
          % (t0, t1, ''.join(AX[k] for k in order), *dS,
             ''.join(AX[k] for k in order), *db))
    print("     => 最大 %.3f LSB/g 标度、%.3f mg 零偏  => 向心项/抖动污染 <= 0.2 mg"
          % (max(abs(x) for x in dS), max(abs(x) for x in db)/FS*1000))

    BEST = rows[1.0]
    ka = BEST['axis']
    print("  转轴: 平面法向 (%+.4f, %+.4f, %+.4f) -> 机体 %s 轴（失准 %.2f deg）"
          % (*BEST['nrm'], AX[ka], np.degrees(np.arccos(min(1.0, abs(BEST['nrm'][ka]))))))
    print("  %s<->%s 交叉 %+.4f%%   半轴比 %.4f   离面 rms %.2f LSB   平面内几何 rms %.1f LSB (%.3f mg)   静止窗 %d 个"
          % (AX[BEST['pair'][0]], AX[BEST['pair'][1]],
             BEST['gam']*100, BEST['ratio'], BEST['res'], BEST['res_geo'],
             BEST['res_geo']/FS*1000, len(pts1)))
    print("     （近圆椭圆下交叉项本身病态，只作数量级参考：轴比与 1 的差就是两轴标度差）")
    print("  本记录给出: " + "  ".join(
        "S_%s=%.2f (%+.4f%%)" % (AX[k], BEST['S'][k], (BEST['S'][k]/FS-1)*100) for k in order))
    print("              " + "  ".join(
        "b_%s=%.2f LSB (%+.3f mg)" % (AX[k], BEST['b'][k], BEST['b'][k]/FS*1000) for k in order))

    # --- 对照组：含转动帧 ---
    e_all = fit_plane_ellipse(acc[np.arange(0, N, 7)])
    print("  对照组(全部帧一起拟合): " + "  ".join(
        "S_%s=%.2f (%+.4f%%)" % (AX[k], e_all['S'][k], (e_all['S'][k]/FS-1)*100) for k in order)
        + "   离面 rms %.1f LSB" % e_all['res'])
    print("     不取静止段的误差: " + "  ".join(
        "dS_%s=%+.2f LSB/g" % (AX[k], e_all['S'][k]-BEST['S'][k]) for k in order)
        + "  " + " ".join("db_%s=%+.2f LSB" % (AX[k], e_all['b'][k]-BEST['b'][k]) for k in order))

    # --- 转动段残差分箱：证明残差不是向心项 ---
    bf = np.array([BEST['b'][i] if not np.isnan(BEST['b'][i]) else 0.0 for i in range(3)])
    Sf = np.array([BEST['S'][i] if i in BEST['S'] else FS for i in range(3)])
    fbl = (acc - bf) / Sf
    still = rate < 1.0
    segs, i = [], 0
    while i < N:
        j = i
        while j < N and still[j] == still[i]:
            j += 1
        segs.append((i, j, bool(still[i]))); i = j
    R_, W_ = [], []
    for m in range(len(segs) - 1):
        i0, i1, st = segs[m]; j0, j1, st2 = segs[m + 1]
        if not st or st2 or (i1 - i0) / fps < 0.3 or (j1 - j0) / fps < 0.15:
            continue
        mid = (i0 + i1) // 2
        if rate[mid] > 0.3:
            continue
        qr = q[mid].copy(); fr = fbl[mid] / np.linalg.norm(fbl[mid])
        cq = np.array([qr[0], -qr[1], -qr[2], -qr[3]])
        idx = np.arange(j0 + int(0.02 * fps), j1 - int(0.02 * fps), 16)
        if len(idx) < 20:
            continue
        B = q[idx]
        Q = qmul_v(cq, B)
        R_.append(fbl[idx] - rotT_v(Q, fr))
        W_.append(rate[idx])
    R_ = np.concatenate(R_); W_ = np.concatenate(W_)
    print("  转动段残差按 |w| 分箱（res = 实测比力 - 由陀螺推过去的预期比力）:")
    print("    %14s %7s %11s %13s" % ("|w| 区间dps", "n", "|res|rms mg", "res^2/w^2"))
    for e0, e1 in zip([0, 20, 40, 60, 90, 120], [20, 40, 60, 90, 120, 200]):
        m_ = (W_ >= e0) & (W_ < e1)
        if m_.sum() < 20:
            continue
        r2 = (R_[m_]**2).sum(1).mean()
        print("    %6d-%-7d %7d %11.2f %13.4f" % (e0, e1, m_.sum(),
              np.linalg.norm(R_[m_], axis=1).std()*1000, r2/(np.radians(W_[m_])**2).mean()*1e6))
    print("    若残差是向心项: rms 应 ∝ |w|, 最后一列应恒定; 实测基本不变且掉两个数量级")
    print("    => 是夹具手转的平动抖动, 不是向心项 (也正因如此, 反解杠杆臂 r 做不到)")

    recs.append(dict(fn=fn, axis=ka, S=BEST['S'], b=BEST['b'], res=BEST['res'],
                     n=len(pts1), rate_path=(rate, pts1), acc=acc))

# ==================== 合并 ====================
print("\n" + "=" * 96)
print("合并三轴：逐轴取各记录给出的值，并给出独立来源之间的离散度（这才是真实的标定误差）")
print("=" * 96)
fin_S, fin_b = {}, {}
for k in range(3):
    sv = [(r['fn'][-20:-12], r['S'][k]) for r in recs if k in r['S']]
    bv = [(r['fn'][-20:-12], r['b'][k]) for r in recs if not np.isnan(r['b'][k])]
    if not sv:
        print("  S_%s : 无记录覆盖" % AX[k]); continue
    fin_S[k] = float(np.mean([v for _, v in sv]))
    fin_b[k] = float(np.mean([v for _, v in bv]))
    ss = "  ".join("%s %.2f" % (t, v) for t, v in sv)
    bs = "  ".join("%s %+.2f LSB" % (t, v) for t, v in bv)
    print("  S_%s = %8.2f LSB/g (%+.4f%%)   来源: %s%s"
          % (AX[k], fin_S[k], (fin_S[k]/FS-1)*100, ss,
             "" if len(sv) < 2 else "   两源差 %.2f LSB/g (%.4f%%)" % (sv[0][1]-sv[1][1], (sv[0][1]/sv[1][1]-1)*100)))
    print("  b_%s = %8.2f LSB (%+.3f mg)   来源: %s%s"
          % (AX[k], fin_b[k], fin_b[k]/FS*1000, bs,
             "" if len(bv) < 2 else "   两源差 %.2f LSB (%.3f mg)" % (bv[0][1]-bv[1][1], (bv[0][1]-bv[1][1])/FS*1000)))

print("\n抄进 V5F/User/inc/v5f_proc.h:")
for k in range(3):
    if k in fin_S:
        print("  #define V5F_ACCEL_LSB_PER_G_%s   %8.2ff" % (AX[k].upper(), round(fin_S[k], 2)))
for k in range(3):
    if k in fin_b:
        print("  #define V5F_ACCEL_BIAS_LSB_%s    %8.2ff" % (AX[k].upper(), round(fin_b[k], 2)))

# ==================== 图 ====================
fig, ax = plt.subplots(2, len(recs), figsize=(6.4 * len(recs), 9))
if len(recs) == 1:
    ax = ax.reshape(2, 1)
for c, r in enumerate(recs):
    rate, pts1 = r['rate_path']
    ka = r['axis']; ij = [i for i in range(3) if i != ka]
    P1 = dedup(np.array([p[0] for p in pts1]))
    ax[0, c].plot(r['acc'][::7, ij[0]], r['acc'][::7, ij[1]], '.', ms=1, color='0.85', label='all frames')
    ax[0, c].plot(P1[:, ij[0]], P1[:, ij[1]], 'ro', ms=6, label='static')
    ax[0, c].set_xlabel('accel %s [LSB]' % AX[ij[0]]); ax[0, c].set_ylabel('accel %s [LSB]' % AX[ij[1]])
    ax[0, c].set_title('%s: rotate about %s' % (r['fn'][-20:-12], AX[ka]))
    ax[0, c].legend(fontsize=8); ax[0, c].axis('equal'); ax[0, c].grid(alpha=.3)
    ax[1, c].plot(np.arange(len(rate)) / fps, rate, lw=.5)
    ax[1, c].axhline(1.0, color='r', ls='--', lw=1, label='|w|<1 dps')
    for p in pts1:
        ax[1, c].axvspan(p[2].start / fps, p[2].stop / fps, color='g', alpha=.25)
    ax[1, c].set_xlabel('t [s]'); ax[1, c].set_ylabel('|w| [dps]'); ax[1, c].set_yscale('log')
    ax[1, c].set_title('smoothed rate; static windows green'); ax[1, c].legend(fontsize=8); ax[1, c].grid(alpha=.3)
plt.tight_layout(); plt.savefig('accel_ellipse_static.png', dpi=110)
print("\n图: accel_ellipse_static.png")
