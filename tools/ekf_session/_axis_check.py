# -*- coding: utf-8 -*-
"""轴错位检查（离散穷举，不做拟合）。

原理：磁三轴若与 IMU 三轴一致，则 Bw = R(q_legacy) m 的方向在任何姿态下相同。
     于是逐个试 6 种轴排列 x 8 种符号，用"集中度" |mean(Bw)| / mean(|Bw|) 打分：
     1.0 = 世界矢量完全恒定（轴对齐）；越小 = 世界矢量随姿态乱转（轴错位）。
     并对最优组合报告世界矢量的 方位角/磁倾角/模，与模型对照。
"""
import glob, os, re, itertools
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4'); n = 144
b = r[:r.size//n*n].reshape(-1, n).astype(np.float64)
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def gg(nm, d):
    m = re.search(r'#define\s+' + nm + r'\s+\(?\s*([-\d.]+)', T)
    return float(m.group(1)) if m else d
DIP, DECL = gg('V5F_EKF_DIP_TAN', 2.08), gg('V5F_MAG_DECL_DEG', -7.53)

m = b[:, 38:41].copy(); q = b[:, 0:4].copy(); g = np.linalg.norm(b[:, 26:29], axis=1)
mm = np.linalg.norm(m, axis=1)
ok = (mm > 100) & (mm < 260) & np.isfinite(m).all(axis=1) & np.isfinite(q).all(axis=1)
idx = np.flatnonzero(ok)[::11]
M = m[idx]; gg2 = g[idx]; st = gg2 < 5.0
print('帧%d  用于检查 %d 点（静止 %d）' % (len(b), len(M), st.sum()))


def qn(x):
    nn = np.linalg.norm(x, axis=1); nn = np.where(nn < 1e-9, 1, nn); return x/nn[:, None]


def Rq(qq):
    qq = qn(qq); w, x, y, z = qq[:, 0], qq[:, 1], qq[:, 2], qq[:, 3]
    R = np.empty((len(qq), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z); R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y); R[:, 2, 1] = 2*(y*z+w*x); R[:, 2, 2] = 1-2*(x*x+y*y)
    return R


R = Rq(q[idx])
res = []
for perm in itertools.permutations(range(3)):
    for sg in itertools.product((1, -1), repeat=3):
        Mp = M[:, perm] * np.array(sg)
        Bw = np.einsum('nij,nj->ni', R, Mp)
        mean = Bw.mean(axis=0)
        conc = np.linalg.norm(mean)/max(np.mean(np.linalg.norm(Bw, axis=1)), 1e-9)
        res.append((conc, perm, sg, mean))
res.sort(key=lambda x: -x[0])
print('\n=== 集中度排序（越大 = 世界矢量越恒定 = 轴越可能是这样接的）===')
print('  集中度   排列(0=x,1=y,2=z)   符号          |Bw|均值   方位角    磁倾角')
for conc, perm, sg, mean in res[:8]:
    mm_ = np.linalg.norm(mean)
    f = np.linalg.norm(mean)
    az = np.degrees(np.arctan2(mean[0], mean[1]))
    di = np.degrees(np.arcsin(np.clip(mean[2]/f, -1, 1)))
    print('  %.4f   [%d %d %d]        %+d%+d%+d     %8.1f   %+8.2f  %+8.2f'
          % (conc, perm[0], perm[1], perm[2], sg[0], sg[1], sg[2], mm_, az, di))
print('\n  参照: 模型 方位角 %+.2f 度, 磁倾角 %+.2f 度 (WMM 约 +53.7)' % (DECL, np.degrees(np.arctan(DIP))))
best = res[0]
print('\n=== 最优组合的细节 ===')
Mp = M[:, best[1]] * np.array(best[2])
Bw = np.einsum('nij,nj->ni', R, Mp)
mean = Bw.mean(axis=0)
dev = np.degrees(np.arccos(np.clip((Bw @ mean)/(np.linalg.norm(Bw, axis=1)*np.linalg.norm(mean)), -1, 1)))
print('  世界矢量均值 = [%.1f  %.1f  %.1f]   |Bw| = %.1f' % (*mean, np.linalg.norm(mean)))
print('  每点与均值的夹角: 静止 p50 %6.2f  p90 %6.2f 度 | 运动 p50 %6.2f  p90 %6.2f 度'
      % (np.median(dev[st]), np.percentile(dev[st], 90), np.median(dev[~st]), np.percentile(dev[~st], 90)))
print('  |Bw| 每点: 静止 p10 %.1f p50 %.1f p90 %.1f | 运动 p10 %.1f p50 %.1f p90 %.1f'
      % (np.percentile(np.linalg.norm(Bw[st], axis=1), 10), np.median(np.linalg.norm(Bw[st], axis=1)),
         np.percentile(np.linalg.norm(Bw[st], axis=1), 90),
         np.percentile(np.linalg.norm(Bw[~st], axis=1), 10), np.median(np.linalg.norm(Bw[~st], axis=1)),
         np.percentile(np.linalg.norm(Bw[~st], axis=1), 90)))
