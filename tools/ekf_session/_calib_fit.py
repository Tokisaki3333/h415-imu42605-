# -*- coding: utf-8 -*-
"""硬磁/软磁标定（用原始 mag_lsb + 旧链姿态当真值）。

椭球拟合:  F(m)= m^T Q m + p^T m + c = 0  (SVD 最小奇异向量)
  中心 b = -0.5 Q^-1 p ;  k = 0.25 p^T Q^-1 p - c
  球化 A:  |A (m-b)| = 1  <=  A^T A = Q/k
然后:  u = A(m-b) (单位),  Bw = R(q_legacy) u
判据:  标定后 |Bw| 与方位角/磁倾角在全段（含运动）应收窄成常数，
       且方位角应 ≈ 磁偏角 D、倾角 ≈ 当地磁倾角。
"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4'); n = 144
b = r[:r.size//n*n].reshape(-1, n).astype(np.float64)
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def gg(nm, dflt):
    m = re.search(r'#define\s+' + nm + r'\s+\(?\s*([-\d.]+)', T)
    return float(m.group(1)) if m else dflt
DIP = gg('V5F_EKF_DIP_TAN', 2.08)
DECL = gg('V5F_MAG_DECL_DEG', -7.53)
if not re.search(r'#define\s+V5F_MAG_DECL', T):
    print('  (未找到 V5F_MAG_DECL_DEG 定义，用默认 -7.53)')
print('数据 %s 帧%d   DIP_TAN=%.3f (倾角 %.2f 度)  DECL=%.2f' %
      (os.path.basename(P), len(b), DIP, np.degrees(np.arctan(DIP)), DECL))

m = b[:, 38:41].copy()          # 原始 LSB
q = b[:, 0:4].copy()
g = np.linalg.norm(b[:, 26:29], axis=1)
mm = np.linalg.norm(m, axis=1)
ok = (mm > 80) & (mm < 300) & np.isfinite(m).all(axis=1)
print('原始 |m|: p1 %.1f p50 %.1f p99 %.1f ; 有效样本 %d/%d'
      % (np.percentile(mm, 1), np.median(mm), np.percentile(mm, 99), ok.sum(), len(b)))


def qn(x): return x/np.linalg.norm(x)


def Rq(qq):
    w, x, y, z = qn(qq)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


sub = np.flatnonzero(ok)[::37]
X = m[sub]
print('用于拟合 %d 点' % len(X))
D = np.column_stack([X[:, 0]**2, X[:, 1]**2, X[:, 2]**2, 2*X[:, 0]*X[:, 1],
                     2*X[:, 0]*X[:, 2], 2*X[:, 1]*X[:, 2], 2*X[:, 0], 2*X[:, 1], 2*X[:, 2],
                     np.ones(len(X))])
_, _, Vt = np.linalg.svd(D, full_matrices=False)
v = Vt[-1]
Q = np.array([[v[0], v[3], v[4]], [v[3], v[1], v[5]], [v[4], v[5], v[2]]])
p = np.array([v[6], v[7], v[8]]); c = v[9]
if np.linalg.det(Q) < 0 or np.trace(Q) < 0:
    Q, p, c = -Q, -p, -c
try:
    Qi = np.linalg.inv(Q)
except np.linalg.LinAlgError:
    raise SystemExit('Q 奇异')
bc = -0.5*Qi@p
k = 0.25*(p@Qi@p) - c
print('\n硬磁偏移 b (LSB) = [%.2f  %.2f  %.2f]' % tuple(bc))
print('椭球中心相对均值 = [%.2f  %.2f  %.2f]' % tuple(bc - X.mean(axis=0)))
w2, V2 = np.linalg.eigh(Q/k)
print('软磁特征值(1/k 归一) = %s   -> 轴比 %.3f : %.3f : 1'
      % (np.round(w2, 6), np.sqrt(w2[0]/w2[2]), np.sqrt(w2[1]/w2[2])))
A = V2 @ np.diag(np.sqrt(w2)) @ V2.T          # A^T A = Q/k
if np.linalg.det(A) < 0:
    A = -A
print('软磁矩阵 A =\n%s' % np.array2string(A, precision=6, suppress_small=False))

u = np.einsum('ij,nj->ni', A, m - bc)
un = np.linalg.norm(u, axis=1)
Bw = np.einsum('nij,nj->ni', np.stack([Rq(x) for x in q[::11]]), u[::11])
un11 = un[::11]
az = np.degrees(np.arctan2(Bw[:, 0], Bw[:, 1]))
mod = np.linalg.norm(Bw, axis=1)
dip = np.degrees(np.arcsin(np.clip(Bw[:, 2]/np.maximum(mod, 1e-9), -1, 1)))
gg = g[::11]
still = gg < 5.0
f = np.isfinite(az) & np.isfinite(mod)
print('\n=== 标定后（旧链姿态 + 标定过的磁矢量）===')
print('  球化后 |u|    : p10 %.3f p50 %.3f p90 %.3f  (目标 1.000)' %
      (np.percentile(un11[f], 10), np.median(un11[f]), np.percentile(un11[f], 90)))
print('  |Bw|          : 静止 p50 %.3f (p10 %.3f p90 %.3f)  运动 p50 %.3f (p10 %.3f p90 %.3f)' %
      (np.median(mod[still & f]), np.percentile(mod[still & f], 10), np.percentile(mod[still & f], 90),
       np.median(mod[~still & f]), np.percentile(mod[~still & f], 10), np.percentile(mod[~still & f], 90)))
print('  方位角        : 静止 p50 %+8.2f (p10 %+7.2f p90 %+7.2f)  运动 p50 %+8.2f (p10 %+7.2f p90 %+7.2f)' %
      (np.median(az[still & f]), np.percentile(az[still & f], 10), np.percentile(az[still & f], 90),
       np.median(az[~still & f]), np.percentile(az[~still & f], 10), np.percentile(az[~still & f], 90)))
print('  磁倾角        : 静止 p50 %+8.2f  运动 p50 %+8.2f   (模型 %+.2f / WMM 约 +53.7)' %
      (np.median(dip[still & f]), np.median(dip[~still & f]), np.degrees(np.arctan(DIP))))
print('  方位角与 DECL 之差: 静止 p50 %+8.2f  运动 p50 %+8.2f' %
      (np.median(az[still & f]) - DECL, np.median(az[~still & f]) - DECL))
# 标定前对照
Bw0 = np.einsum('nij,nj->ni', np.stack([Rq(x) for x in q[::11]]), m[::11])
az0 = np.degrees(np.arctan2(Bw0[:, 0], Bw0[:, 1])); mod0 = np.linalg.norm(Bw0, axis=1)
print('\n=== 标定前对照 ===')
print('  |Bw|   静止 p10 %.1f p90 %.1f | 方位角 静止 p10 %+.2f p90 %+.2f' %
      (np.percentile(mod0[still], 10), np.percentile(mod0[still], 90),
       np.percentile(az0[still], 10), np.percentile(az0[still], 90)))
