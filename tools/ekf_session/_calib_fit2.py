# -*- coding: utf-8 -*-
"""用旧链姿态当真值的**仿射标定**：求 A(3x3), d(3) 使 R(q_i)(A m_i + d) 尽量恒定。
   迭代：B0 = mean(R(q_i)(A m_i+d))  ->  最小二乘解  R(q_i)^T B0 = A m_i + d  -> 重复
每步都是线性最小二乘，良态；A 一次吸收 硬磁/软磁/尺度/装配旋转 等一切静态畸变。
残差（世界方向还能剩多少抖动）直接判定畸变是"静态可标定"还是"动态"。
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
DIP, DECL = gg('V5F_EKF_DIP_TAN', 2.08), gg('V5F_MAG_DECL_DEG', -7.53)
print('帧%d  DIP_TAN=%.3f(倾角%.2f)  DECL=%.2f' % (len(b), DIP, np.degrees(np.arctan(DIP)), DECL))

m = b[:, 38:41].copy(); q = b[:, 0:4].copy()
g = np.linalg.norm(b[:, 26:29], axis=1)
mm = np.linalg.norm(m, axis=1)
ok = (mm > 100) & (mm < 260) & np.isfinite(m).all(axis=1) & np.isfinite(q).all(axis=1)
idx = np.flatnonzero(ok)[::11]


def qn(x):
    nn = np.linalg.norm(x, axis=1); nn = np.where(nn < 1e-9, 1, nn); return x/nn[:, None]


def Rq(qq):
    qq = qn(qq); w, x, y, z = qq[:, 0], qq[:, 1], qq[:, 2], qq[:, 3]
    R = np.empty((len(qq), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z); R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y); R[:, 2, 1] = 2*(y*z+w*x); R[:, 2, 2] = 1-2*(x*x+y*y)
    return R


M = m[idx]; R = Rq(q[idx]); gg2 = g[idx]
N = len(M)
A = np.eye(3); d = np.zeros(3)
Xd = np.column_stack([M, np.ones(N)])
for it in range(30):
    W = M @ A.T + d
    Bw = np.einsum('nij,nj->ni', R, W)
    B0 = Bw.mean(axis=0)
    Tgt = np.einsum('nji,j->ni', R, B0)
    Th, *_ = np.linalg.lstsq(Xd, Tgt, rcond=None)
    A = Th[:3].T; d = Th[3]
W = M @ A.T + d
Bw = np.einsum('nij,nj->ni', R, W)
res = Bw - Bw.mean(axis=0)
resn = np.linalg.norm(res, axis=1)
B0 = Bw.mean(axis=0)
print('\n=== 拟合结果 ===')
print('B0 (世界磁场, LSB) = [%.2f  %.2f  %.2f]   |B0| = %.2f' % (*B0, np.linalg.norm(B0)))
print('方位角 = %+8.2f 度   磁倾角 = %+8.2f 度' %
      (np.degrees(np.arctan2(B0[0], B0[1])), np.degrees(np.arcsin(B0[2]/np.linalg.norm(B0)))))
print('硬磁偏移 b = -A^-1 d = [%s]' % np.array2string(-np.linalg.solve(A, d), precision=2))
print('软磁/尺度矩阵 A =\n%s' % np.array2string(A, precision=6))
w, V = np.linalg.eigh(A.T @ A)
print('A^T A 特征值 %s  -> 轴比 %.4f : %.4f : 1' % (np.round(w, 4), np.sqrt(w[0]/w[2]), np.sqrt(w[1]/w[2])))
still = gg2 < 5.0
print('\n=== 标定后 世界的抖动（残差）===')
print('|Bw - mean| : 全段 p50 %.2f  p90 %.2f   |B0|=%.1f  -> 折合角度 p50 %.2f 度  p90 %.2f 度'
      % (np.median(resn), np.percentile(resn, 90), np.linalg.norm(B0),
         np.degrees(np.median(resn)/np.linalg.norm(B0)), np.degrees(np.percentile(resn, 90)/np.linalg.norm(B0))))
az = np.degrees(np.arctan2(Bw[:, 0], Bw[:, 1])); mod = np.linalg.norm(Bw, axis=1)
dip = np.degrees(np.arcsin(np.clip(Bw[:, 2]/mod, -1, 1)))
print('  |Bw|   静止 p50 %.1f (p10 %.1f p90 %.1f)   运动 p50 %.1f (p10 %.1f p90 %.1f)'
      % (np.median(mod[still]), np.percentile(mod[still], 10), np.percentile(mod[still], 90),
         np.median(mod[~still]), np.percentile(mod[~still], 10), np.percentile(mod[~still], 90)))
print('  方位角 静止 p50 %+8.2f (p10 %+7.2f p90 %+7.2f)  运动 p50 %+8.2f (p10 %+7.2f p90 %+7.2f)'
      % (np.median(az[still]), np.percentile(az[still], 10), np.percentile(az[still], 90),
         np.median(az[~still]), np.percentile(az[~still], 10), np.percentile(az[~still], 90)))
print('  磁倾角 静止 p50 %+8.2f  运动 p50 %+8.2f   (WMM 约 +53.7)' % (np.median(dip[still]), np.median(dip[~still])))
# 标定前
Bw0 = np.einsum('nij,nj->ni', R, M)
az0 = np.degrees(np.arctan2(Bw0[:, 0], Bw0[:, 1])); mod0 = np.linalg.norm(Bw0, axis=1)
r0 = np.linalg.norm(Bw0 - Bw0.mean(axis=0), axis=1)
print('\n=== 标定前 ===')
print('  |Bw-mean| 折合角度 p50 %.2f   p90 %.2f 度' %
      (np.degrees(np.median(r0)/np.median(mod0)), np.degrees(np.percentile(r0, 90)/np.median(mod0))))
print('  |Bw| 静止 p10 %.1f p90 %.1f   方位角 静止 p10 %+.2f p90 %+.2f' %
      (np.percentile(mod0[still], 10), np.percentile(mod0[still], 90),
       np.percentile(az0[still], 10), np.percentile(az0[still], 90)))
np.save(r'C:\Users\33\Documents\v2\_calib_A.npy', A)
np.save(r'C:\Users\33\Documents\v2\_calib_d.npy', d)
print('\n已存 _calib_A.npy / _calib_d.npy')
