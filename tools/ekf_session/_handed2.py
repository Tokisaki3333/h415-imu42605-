# -*- coding: utf-8 -*-
# 局部手性判据（无需姿态、无需长积分、无需零偏）:
#   正常链路: 世界磁场恒定 -> f_j = Exp(-w*dt) f_i   (机体坐标系)
#   镜像链路: 该关系被反射破坏 -> 残差很大
# 扫 48 个带符号置换, 看哪个使残差最小 = 正确的 磁->机体 映射
import numpy as np, glob, os, itertools

def load():
    P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
            key=os.path.getmtime)
    r = np.fromfile(P, dtype='<f4'); n = 144
    b = r[:r.size // n * n].reshape(-1, n).astype(np.float64)
    return P, b

def rot_exp(th):
    """th: (n,3) 旋转矢量(rad) -> (n,3,3) 旋转矩阵"""
    a = np.linalg.norm(th, axis=1); h = 0.5 * a
    s = np.where(a > 1e-12, np.sin(h) / np.maximum(a, 1e-12), 0.5)
    w = th * s[:, None]
    K = np.zeros((len(th), 3, 3))
    K[:, 0, 1] = -w[:, 2]; K[:, 0, 2] = w[:, 1]
    K[:, 1, 0] = w[:, 2];  K[:, 1, 2] = -w[:, 0]
    K[:, 2, 0] = -w[:, 1]; K[:, 2, 1] = w[:, 0]
    return np.eye(3)[None] + 2.0 * (K @ K) + 2.0 * np.cos(h)[:, None, None] * K

P, b = load()
N = len(b); dt = 124.56e-6
t = np.cumsum(b[:, 25] * 1e-6)
gyro = b[:, 26:29]; f = b[:, 42:45]
fn = np.linalg.norm(f, axis=1)
clean = (fn > 0.9) & (fn < 1.1)
fu = np.where(clean[:, None], f / np.maximum(fn, 1e-9)[:, None], 0.0)

K = 240                                   # 30 ms
cg = np.vstack([np.zeros(3), np.cumsum(gyro, axis=0)])   # 累积
wbar = (cg[K:] - cg[:-K])[:N - K] / K      # dps
theta = np.radians(wbar) * (K * dt)
ang = np.degrees(np.linalg.norm(theta, axis=1))
i = np.arange(N - K); j = i + K
sel = (ang > 10.0) & clean[i] & clean[j]
ii, jj, th = i[sel], j[sel], theta[sel]
print('%s N=%d %.2fs  间隔%.1fms  合格对 %d (净转角>10度且两端磁干净)' % (
    os.path.basename(P), N, t[-1], K * dt * 1e3, len(ii)))
print('  净转角 p10 %.1f p50 %.1f p90 %.1f 度' % tuple(np.percentile(ang[sel], [10, 50, 90])))
Rinv = rot_exp(-th)

def err_of(M):
    fi = fu[ii] @ M.T; fj = fu[jj] @ M.T
    p = np.einsum('nij,nj->ni', Rinv, fi)
    c = np.clip(np.sum(p * fj, axis=1), -1, 1)
    return np.degrees(np.arccos(c))

rows = []
for perm in itertools.permutations(range(3)):
    for sg in itertools.product([1, -1], repeat=3):
        M = np.zeros((3, 3))
        for k in range(3): M[k, perm[k]] = sg[k]
        e = err_of(M)
        rows.append((np.median(e), np.percentile(e, 90), np.linalg.det(M), perm, sg))
rows.sort()
print()
print('排名  残差中位(度)   p90     det   置换   符号')
for n, (md, p9, det, perm, sg) in enumerate(rows):
    print('%3d %12.3f %8.2f  %+5.0f   %s   %s' % (n + 1, md, p9, det,
          ''.join('xyz'[p] for p in perm), ''.join('+' if x > 0 else '-' for x in sg)))
idr = [r for r in rows if r[3] == (0, 1, 2) and r[4] == (1, 1, 1)][0]
print()
print('★ 恒等映射(当前固件输出): 残差中位 %.3f 度, p90 %.3f 度  <- 若很大则确有轴/手性错位'
      % (idr[0], idr[1]))
best = rows[0]
print('★ 最优映射: %s%s  det=%+.0f  残差中位 %.3f 度 (比恒等小 %.1f 倍)'
      % (''.join('xyz'[p] for p in best[3]), ''.join('+' if x > 0 else '-' for x in best[4]),
         best[2], best[0], idr[0] / max(best[0], 1e-9)))
