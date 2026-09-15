# -*- coding: utf-8 -*-
# 手性判据: 纯陀螺积分姿态(与磁无关) + 磁矢量
#   正常旋转 -> w = R(q) f 在短窗内恒定(任意初始偏航被常数吸收)
#   镜像     -> w 随姿态游走
# 在 48 个带符号置换中找使 w 最集中者 = 正确的磁->机体 映射
import numpy as np, glob, os, itertools, sys

def load():
    P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
            key=os.path.getmtime)
    r = np.fromfile(P, dtype='<f4'); n = 144
    b = r[:r.size // n * n].reshape(-1, n).astype(np.float64)
    return P, b

def quat_mul(a, b):
    aw, ax, ay, az = a[...,0], a[...,1], a[...,2], a[...,3]
    bw, bx, by, bz = b[...,0], b[...,1], b[...,2], b[...,3]
    return np.stack([aw*bw - ax*bx - ay*by - az*bz,
                     aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw], -1)

def quat_to_R(q):
    w,x,y,z = q[...,0], q[...,1], q[...,2], q[...,3]
    return np.stack([
        np.stack([1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)], -1),
        np.stack([2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)], -1),
        np.stack([2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)], -1)], -2)

def propagate(w_dps, bg_dps, dt, sign=+1.0):
    w = (w_dps - bg_dps) * (np.pi/180.0) * sign
    ang = np.linalg.norm(w, axis=1)
    half = 0.5 * ang
    s = np.where(ang > 1e-12, np.sin(half)/np.maximum(ang,1e-12), 0.5)
    dq = np.stack([np.cos(half), s*w[:,0], s*w[:,1], s*w[:,2]], -1)
    N = len(w); q = np.empty((N,4)); q[0] = [1.0,0,0,0]
    for i in range(1, N):
        q[i] = quat_mul(q[i-1:i], dq[i:i+1])[0]
    return q / np.linalg.norm(q, axis=1, keepdims=True)

def spread(w):
    """窗内单位矢量集中度: 到平均方向的 RMS 角(度) 与 平均合矢量长"""
    m = w.mean(0); Rbar = np.linalg.norm(m)
    if Rbar < 1e-9: return 180.0, 0.0
    m = m/Rbar
    c = np.clip(w @ m, -1, 1)
    return float(np.degrees(np.arccos(c)).std()), float(Rbar)

P, b = load()
N = len(b); t = np.cumsum(b[:,25]*1e-6)
gyro = b[:,26:29]; bg = b[:,99:102]; f = b[:,42:45]
fn = np.linalg.norm(f, axis=1)
fu = f / np.maximum(fn,1e-9)[:,None]
dt_ms = b[:,25]*1e-6
print('%s  N=%d  %.2fs   |f| p50 %.4f   |gyro| p50 %.1f dps  bg p50 %s'
      % (os.path.basename(P), N, t[-1], np.median(fn), np.median(np.linalg.norm(gyro,axis=1)),
         np.round(np.median(bg,axis=0),3)))

W = int(0.30 / np.median(dt_ms))     # 0.30 s 窗
ROT = 20.0                            # 窗内净转动 > 20 度才计入
valid = fn > 0.9
wins = []
for k in range(0, N-W, W):
    s = slice(k, k+W)
    if valid[s].mean() < 0.98: continue
    q = propagate(gyro[s], bg[s], dt_ms[s])
    R = quat_to_R(q)
    net = np.degrees(np.linalg.norm(np.trapezoid(gyro[s]-bg[s], t[s], axis=0)))
    if net < ROT: continue
    wins.append((k, R, s, net))
print('合格窗 %d / %d (窗内净转动>%.0f度)' % (len(wins), N//W, ROT))
if not wins:
    print('!! 无合格窗'); sys.exit()

cands = []
for perm in itertools.permutations(range(3)):
    for sg in itertools.product([1,-1], repeat=3):
        M = np.zeros((3,3))
        for i in range(3): M[i, perm[i]] = sg[i]
        cands.append((perm, sg, M, np.linalg.det(M)))
res = []
for perm, sg, M, det in cands:
    fm = fu @ M.T
    sp = []
    for k, R, s, net in wins:
        w = np.einsum('nij,nj->ni', R, fm[s])
        w = w/np.maximum(np.linalg.norm(w,axis=1,keepdims=True),1e-9)
        sp.append(spread(w)[0])
    sp = np.array(sp)
    res.append((np.median(sp), np.percentile(sp,90), det, perm, sg))
res.sort()
print()
print('排名  中位窗内散布(度)  p90散布   det  置换    符号')
for i,(md,p9,det,perm,sg) in enumerate(res):
    print('%3d %14.3f %9.3f %+5.0f  %s  %s' % (i+1, md, p9, det, ''.join('xyz'[p] for p in perm),
                                                ''.join('+' if x>0 else '-' for x in sg)))
print()
print('最优(含镜像) det=%+.0f ; 最优det=+1 -> %s' %
      (res[0][2], str([r for r in res if r[2] > 0][0][:5])))
