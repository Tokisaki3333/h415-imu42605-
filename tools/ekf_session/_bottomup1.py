# -*- coding: utf-8 -*-
"""自下而上第 1 段：原始 LSB -> 底层标定 (y=A*raw+C) -> 归一化 -> 世界磁场。
   A、C 直接取 v5f_tune.h 的 V5F_MAG_A_INIT / V5F_MAG_C_INIT（真实值，不拟合）。
   判据：世界磁场矢量是否恒定（集中度）+ 方向 vs 模型。
   同时对照：再加一层 EKF 里我做的 (y,x,-z) 会变成什么。
"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4'); n = 144
b = r[:r.size//n*n].reshape(-1, n).astype(np.float64)
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def nums(nm):
    m = re.search(r'#define\s+' + nm + r'\s+(\{.*?\}\s*\})', T, re.S)
    if not m:
        m = re.search(r'#define\s+' + nm + r'\s+((?:\{[^}]*\}\s*)+)', T, re.S)
    return [float(x) for x in re.findall(r'[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?', m.group(1))]
A = np.array(nums('V5F_MAG_A_INIT')).reshape(3, 3)
C = np.array(nums('V5F_MAG_C_INIT')).reshape(3)
DIP = float(re.search(r'#define\s+V5F_EKF_DIP_TAN\s+\(?\s*([-\d.]+)', T).group(1))
DECL = float(re.search(r'#define\s+V5F_MAG_DECL_DEG\s+\(?\s*([-\d.]+)', T).group(1))
print('底层矩阵 A =\n%s' % np.array2string(A, precision=8))
print('底层偏移 C = %s' % np.array2string(C, precision=8))
print('模型: 磁倾角 %.2f 度(内), 磁偏角 %.2f 度' % (np.degrees(np.arctan(DIP)), DECL))
# 反解底层等效轴映射：看每一行最大元素落在哪一列、符号如何
for i in range(3):
    j = int(np.argmax(np.abs(A[i])))
    print('  f[%d] 主要由 raw[%d] 决定, 符号 %+d, 系数 %.6e' % (i, j, 1 if A[i, j] > 0 else -1, A[i, j]))

m = b[:, 38:41].copy(); q = b[:, 0:4].copy(); g = np.linalg.norm(b[:, 26:29], axis=1)
mm = np.linalg.norm(m, axis=1)
ok = (mm > 100) & (mm < 260) & np.isfinite(m).all(axis=1)
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


M = m[idx]; R = Rq(q[idx]); gg = g[idx]; st = gg < 5.0
y = M @ A.T + C
fn = np.linalg.norm(y, axis=1)
f = y/np.where(fn < 1e-9, 1, fn)[:, None]
print('\n底层标定后 |y|: p10 %.4f p50 %.4f p90 %.4f  (应在 1.0 附近)' %
      (np.percentile(fn, 10), np.median(fn), np.percentile(fn, 90)))
print('底层标定后 |f| = %.6f (恒为 1)' % np.median(np.linalg.norm(f, axis=1)))


def score(F, nm):
    Bw = np.einsum('nij,nj->ni', R, F)
    mean = Bw.mean(axis=0); mm_ = np.linalg.norm(mean)
    conc = mm_/max(np.mean(np.linalg.norm(Bw, axis=1)), 1e-9)
    az = np.degrees(np.arctan2(mean[0], mean[1])); di = np.degrees(np.arcsin(np.clip(mean[2]/mm_, -1, 1)))
    dev = np.degrees(np.arccos(np.clip((Bw @ mean)/(np.linalg.norm(Bw, axis=1)*mm_), -1, 1)))
    print('  %-26s 集中度 %.4f  世界方位角 %+8.2f  磁倾角 %+7.2f  |Bw|=%.1f  抖动: 静止p90 %5.2f 运动p90 %5.2f 度'
          % (nm, conc, az, di, mm_, np.percentile(dev[st], 90), np.percentile(dev[~st], 90)))


print('\n=== 世界磁场（用旧链姿态投影）===')
score(f, '底层标定后(不加变换)')
score(np.column_stack([-f[:, 0], -f[:, 1], f[:, 2]]), '再加 -x,-y,z')
score(np.column_stack([f[:, 1], f[:, 0], -f[:, 2]]), '再加 y,x,-z (=我VER=68)')
score(np.column_stack([-f[:, 1], -f[:, 0], f[:, 2]]), '再加 -y,-x,z')
score(np.column_stack([f[:, 1], f[:, 0], f[:, 2]]), '再加 y,x,z (只互换)')
print('\n参照: 模型要求 方位角 %+.2f 度, 磁倾角 %+.2f' % (DECL, np.degrees(np.arctan(DIP))))
