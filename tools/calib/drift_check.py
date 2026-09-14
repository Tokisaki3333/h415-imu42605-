# -*- coding: utf-8 -*-
"""
往复摇晃记录的漂移体检。

关键：这条记录每个静止段的加速度读数几乎完全相同（点云 std ~1 LSB），说明每次摇完都
回到**同一姿态**。于是任意两个静止点之间的相对旋转 R_ij = q_i^-1 q_j 必须绕竖直轴
（它要以重力方向为不动点）。把 theta_ij 分解成
    沿重力方向的分量  -> 偏航漂移（加速度计看不见）
    垂直于重力方向的分量 -> **倾角漂移**（这就是重力泄露的来源）
后者是干净的、可判定的漂移量度。

用法：python drift_check.py <记录>
"""
import sys, os
import numpy as np
sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
from jf_load import load_jf

FPS = 8027.0
W_A = 4000
ACC_B0 = np.array([-10.10, -15.42, 43.72])
ACC_S0 = np.array([2028.48, 2040.78, 2016.07])
GB = np.array([1.0334, 0.8494, 12.0910])


def qmul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                     a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def logq(q):
    q = q/np.linalg.norm(q)
    w = float(np.clip(q[0], -1.0, 1.0))
    v = q[1:]
    n = np.linalg.norm(v)
    if n < 1e-13:
        return np.zeros(3)
    return v/n*(2.0*np.arctan2(n, w))


fn = sys.argv[1]
p = fn if os.path.exists(fn) else os.path.join('..', fn)
A = np.asarray(load_jf(p), dtype=np.float64)
N = len(A)
q = A[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
ab = (A[:, 4:7] - ACC_B0)/ACC_S0
M = np.cumsum(np.vstack([np.zeros((1, 3)), ab]), 0)
kk = np.arange(N)
sa = np.abs((M[np.minimum(kk+1, N)]-M[np.maximum(kk+1-W_A, 0)]) /
            np.maximum(np.minimum(kk+1, N)-np.maximum(kk+1-W_A, 0), 1)[:, None]
            - (M[np.maximum(kk+1-W_A, 0)]-M[np.maximum(kk+1-2*W_A, 0)]) /
            np.maximum(np.maximum(kk+1-W_A, 0)-np.maximum(kk+1-2*W_A, 0), 1)[:, None]
            ).max(1)*1000
if A.shape[1] >= 10:
    rate = np.linalg.norm((A[:, 7:10]-GB)/16.4, axis=1)
else:
    r0 = np.degrees(2*np.arccos(np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)))*FPS
    rate = np.concatenate([[r0[0]], r0])
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

print("记录 %s  %d 帧 %.1f s  静止段 %d 个" % (os.path.basename(fn), N, N/FPS, len(segs)))
U, Q, T = [], [], []
for (s, e) in segs:
    sl = slice(s+int(0.1*FPS), max(s+int(0.1*FPS)+1, e-int(0.1*FPS)))
    u = ab[sl].mean(0); u /= np.linalg.norm(u)
    qm = q[sl].mean(0); qm /= np.linalg.norm(qm)
    U.append(u); Q.append(qm); T.append(0.5*(s+e)/FPS)
U = np.array(U); Q = np.array(Q); T = np.array(T)

print("   各静止点重力方向（机体系）:")
for i in range(len(U)):
    print("      t=%6.2f s  u=(%+.4f %+.4f %+.4f)  |a|-1g=%+.3f mg"
          % (T[i], U[i][0], U[i][1], U[i][2],
             (np.linalg.norm(ab[int(T[i]*FPS)])-1.0)*1000))

# ---- 正确做法：解出"世界竖直在固件参考系里的方向" w，再看每点残差 ----
# 注意：相对旋转的"沿重力/垂直重力"分解只对小转角有效（旋转不可交换），
#      像往复摇晃这种单次 342 deg 的大转角，那个分解没有意义。
# 正确量度：找单位向量 w 使 R(q_i)^T w 尽量等于 u_i（u_i 是加速度计给的重力方向）。
#     最小化 sum|R_i^T w - u|^2 = 3 - 2 w·(sum R_i u)  =>  w = normalize(sum R_i u)
# 若姿态正确，所有静止点的 R_i^T w 应都等于同一个 u，残差为 0。
def Rm(qq):
    w_, x_, y_, z_ = qq
    return np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-w_*z_), 2*(x_*z_+w_*y_)],
                     [2*(x_*y_+w_*z_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-w_*x_)],
                     [2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)]])

um = U.mean(0); um /= np.linalg.norm(um)
acc = np.zeros(3)
for i in range(len(Q)):
    acc += Rm(Q[i]) @ um
w = acc/np.linalg.norm(acc)
print("\n   解出：世界竖直在固件参考系里的方向 w=(%+.4f %+.4f %+.4f)，与固件 z 轴夹角 %.2f deg"
      % (w[0], w[1], w[2], np.degrees(np.arccos(np.clip(w[2], -1, 1)))))
print("\n   各静止点的倾角误差（R(q)^T w 与实测重力方向的夹角）:")
errs = []
for i in range(len(Q)):
    up = Rm(Q[i]).T @ w
    e = np.degrees(np.arccos(np.clip(np.dot(up/np.linalg.norm(up), U[i]), -1, 1)))
    errs.append(e)
    print("      t=%6.2f s  倾角误差 %8.3f deg" % (T[i], e))
errs = np.array(errs)
print("\n   倾角误差：首点 %.3f -> 末点 %.3f deg，全程最大 %.3f deg（%.1f s）"
      % (errs[0], errs[-1], errs.max(), T[-1]-T[0]))
print("   即 %.1f s 内漂了 %.2f deg => %.1f deg/h"
      % (T[-1]-T[0], errs.max()-errs.min(),
         (errs.max()-errs.min())/(T[-1]-T[0])*3600 if T[-1] > T[0] else 0))

