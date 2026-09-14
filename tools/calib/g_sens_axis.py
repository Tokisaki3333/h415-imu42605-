# -*- coding: utf-8 -*-
"""
按轴测陀螺的加速度敏感度 S_g。

判据：剧烈摇晃记录里所有静止点的**姿态其实相同**（加速度计给出的重力方向一致），
      所以扣对 S_g 之后，各静止点由姿态推出的重力方向 R(q)^T w 应该都等于实测 u。
      做法：对候选 (Sgx,Sgy,Sgz) 重新积分姿态，量各静止点倾角误差的**散布**。
      散布最小 = S_g 正确。

用法：python g_sens_axis.py <记录> [<记录>...]
"""
import sys, os
import numpy as np
sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
from jf_load import load_jf

GB = np.array([1.0334, 0.8494, 12.0910])
GS = np.array([16.2753, 16.4793, 16.4235])
FPS = 8027.0
DT = 1.0/FPS
W_A = 4000
ACC_B0 = np.array([-10.10, -15.42, 43.72])
ACC_S0 = np.array([2028.48, 2040.78, 2016.07])
AX = 'xyz'


def Rm(qq):
    w_, x_, y_, z_ = qq
    return np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-w_*z_), 2*(x_*z_+w_*y_)],
                     [2*(x_*y_+w_*z_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-w_*x_)],
                     [2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)]])


def prepare(fn):
    A = np.asarray(load_jf(fn), dtype=np.float64)
    N = len(A)
    q = A[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
    ab = (A[:, 4:7] - ACC_B0)/ACC_S0
    rate = np.linalg.norm((A[:, 7:10]-GB)/GS, axis=1)
    M = np.cumsum(np.vstack([np.zeros((1, 3)), ab]), 0)
    kk = np.arange(N)
    sa = np.abs((M[np.minimum(kk+1, N)]-M[np.maximum(kk+1-W_A, 0)]) /
                np.maximum(np.minimum(kk+1, N)-np.maximum(kk+1-W_A, 0), 1)[:, None]
                - (M[np.maximum(kk+1-W_A, 0)]-M[np.maximum(kk+1-2*W_A, 0)]) /
                np.maximum(np.maximum(kk+1-W_A, 0)-np.maximum(kk+1-2*W_A, 0), 1)[:, None]
                ).max(1)*1000
    still = (sa < 1.15) & (rate < 1.0)
    segs, i = [], 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/FPS > 0.3:
                segs.append((i, j))
            i = j
        else:
            i += 1
    # 增量（弧度）
    qc = q.copy(); qc[:, 1:] *= -1.0
    aw, ax_, ay, az = qc[:-1].T
    bw, bx, by, bz = q[1:].T
    vx = aw*bx+ax_*bw+ay*bz-az*by
    vy = aw*by-ax_*bz+ay*bw+az*bx
    vz = aw*bz+ax_*by-ay*bx+az*bw
    ww = aw*bw-ax_*bx-ay*by-az*bz
    n = np.sqrt(vx*vx+vy*vy+vz*vz)
    ang = 2.0*np.arctan2(n, ww)
    rv = np.stack([vx, vy, vz], 1)/np.maximum(n, 1e-15)[:, None]*ang[:, None]
    return q, ab, rv, segs


def qmul_arr(a, b):
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw], 1)


def qprod_scan(dq):
    """按顺序把所有增量四元数乘起来（倍增归约，向量化）。空数组返回单位四元数。"""
    if len(dq) == 0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    cur = dq
    while len(cur) > 1:
        n = len(cur)//2
        prod = qmul_arr(cur[:2*n:2], cur[1:2*n:2])
        if len(cur) % 2:
            prod = np.vstack([prod, cur[-1:]])
        cur = prod
    return cur[0]


def expq_arr(v):
    t = np.linalg.norm(v, axis=1)
    out = np.zeros((len(v), 4))
    out[:, 0] = 1.0
    nz = t > 1e-13
    half = t[nz]*0.5
    out[nz, 0] = np.cos(half)
    out[nz, 1:] = np.sin(half)[:, None]*v[nz]/t[nz][:, None]
    return out


def tilt_spread(q, ab, rv, segs, sg):
    """按 sg(dps/g, 三轴) 重新积分，返回各静止点倾角误差的散布与均值。

    性能：只在**静止点**处需要姿态，所以用倍增归约算"起点到该静止点"的累积旋转，
    不做逐样本 Python 循环（那个在 6^3 x 多记录扫描下会直接卡死）。"""
    s0, e0 = segs[0]
    sl = slice(s0+int(0.1*FPS), max(s0+int(0.1*FPS)+1, e0-int(0.1*FPS)))
    u0 = ab[sl].mean(0); u0 /= np.linalg.norm(u0)
    qm = q[sl].mean(0); qm /= np.linalg.norm(qm)
    w = Rm(qm) @ u0
    w /= np.linalg.norm(w)
    # 用**未修正**姿态估 a_lin（一次近似足够，修正量本身很小）
    w_, x_, y_, z_ = q.T
    gx = (1-2*(y_*y_+z_*z_))*w[0] + 2*(x_*y_+w_*z_)*w[1] + 2*(x_*z_-w_*y_)*w[2]
    gy = 2*(x_*y_-w_*z_)*w[0] + (1-2*(x_*x_+z_*z_))*w[1] + 2*(y_*z_+w_*x_)*w[2]
    gz = 2*(x_*z_+w_*y_)*w[0] + 2*(y_*z_-w_*x_)*w[1] + (1-2*(x_*x_+y_*y_))*w[2]
    a_lin = ab - np.stack([gx, gy, gz], 1)
    rvc = rv - (a_lin[:-1]*sg[None, :])*DT*np.pi/180.0
    dq = expq_arr(rvc.astype(np.float64))
    errs = []
    for (s, e) in segs:
        Q = qmul_arr(q[:1], qprod_scan(dq[:s])[None, :])[0]
        Q /= np.linalg.norm(Q)
        sl = slice(s+int(0.1*FPS), max(s+int(0.1*FPS)+1, e-int(0.1*FPS)))
        u = ab[sl].mean(0); u /= np.linalg.norm(u)
        up = Rm(Q).T @ w
        errs.append(np.degrees(np.arccos(np.clip(np.dot(up/np.linalg.norm(up), u), -1, 1))))
    errs = np.array(errs)
    return errs.max()-errs.min(), errs.mean()


data = [(os.path.basename(f), prepare(f if os.path.exists(f) else os.path.join('..', f)))
        for f in sys.argv[1:]]
if os.environ.get('GRID', '0') == '1':
    # 三维粗扫：每条记录单点方程少（静止点 2~5 个），只能给量级与符号，不能给精确值
    for tag, (q, ab, rv, segs) in data:
        print("\n== %s  静止段 %d 个   三维粗扫" % (tag, len(segs)))
        b0 = tilt_spread(q, ab, rv, segs, np.zeros(3))
        print("   S_g=0 基准散布 %.3f deg" % b0[0])
        best = None
        vals_x = [-1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        vals_yz = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0]
        for sx in vals_x:
            for sy in vals_yz:
                for sz in vals_yz:
                    sp, mn = tilt_spread(q, ab, rv, segs, np.array([sx, sy, sz]))
                    if best is None or sp < best[0]:
                        best = (sp, sx, sy, sz)
        print("   最优 (Sgx,Sgy,Sgz) = (%+.1f, %+.1f, %+.1f) -> 散布 %.3f deg（改善 %.1f 倍）"
              % (best[1], best[2], best[3], best[0], b0[0]/best[0] if best[0] > 1e-9 else 0))
        print("   候选三元组的散布:")
        for cand in [(0, 0, 0), (0.83, 0.83, 0.83), (2, 1, 0), (3, 1, 0), (3, 1, -2), (6, 3, -2)]:
            sp, mn = tilt_spread(q, ab, rv, segs, np.array(cand, float))
            print("      %-18s 散布 %6.3f deg   均值 %6.3f" % (str(cand), sp, mn))
    sys.exit(0)
print("基准（S_g=0）与按轴单扫（dps/g）—— 看哪个轴把散布压下去")
for tag, (q, ab, rv, segs) in data:
    print("\n== %s  静止段 %d 个" % (tag, len(segs)))
    b0 = tilt_spread(q, ab, rv, segs, np.zeros(3))
    print("   S_g=0        散布 %.3f deg  均值 %.3f" % b0)
    for ax in range(3):
        best = None
        for v in [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, -0.5, -1.0]:
            sg = np.zeros(3); sg[ax] = v
            sp, mn = tilt_spread(q, ab, rv, segs, sg)
            if best is None or sp < best[0]:
                best = (sp, mn, v)
        print("   只动 %s 轴: 最优 S_g=%+.2f -> 散布 %.3f  均值 %.3f" % (AX[ax], best[2], best[0], best[1]))
    # 三轴同值
    best = None
    for v in [0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0]:
        sp, mn = tilt_spread(q, ab, rv, segs, np.full(3, v))
        if best is None or sp < best[0]:
            best = (sp, mn, v)
    print("   三轴同值:     最优 S_g=%+.2f -> 散布 %.3f  均值 %.3f" % (best[2], best[0], best[1]))
