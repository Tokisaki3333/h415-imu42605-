# -*- coding: utf-8 -*-
"""校验集：头部参数 19:37:25 写入，这条 19:38:16 开始 -> 真样本外。"""
import re, sys
import numpy as np
np.set_printoptions(precision=6, suppress=True, linewidth=160)

NEW = np.array([16.2753, 16.4366, 16.4235])
KN = np.array([[1, 0.001528, -0.001923], [0.001528, 1, 0.000005], [-0.001923, 0.000005, 1]])
PREV = np.array([16.2970, 16.4356, 16.4382])
KP = np.array([[1, 0.001914, -0.002047], [0.001914, 1, -0.001009], [-0.002047, -0.001009, 1]])

def load_q(fn):
    raw = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){20})').findall(
        open(fn, 'r', errors='ignore').read())
    b = np.frombuffer(b''.join(bytes.fromhex(s.replace(' ', '')) for s in raw), dtype='<f4')
    q = b.reshape(-1, 5)[:, :4].astype(np.float64)
    return q / np.linalg.norm(q, axis=1, keepdims=True)

def tens(fn):
    q = load_q(fn); N = len(q)
    q0, q1 = q[:-1], q[1:]
    w0, x0, y0, z0 = q0.T; w1, x1, y1, z1 = q1.T
    dw = w0*w1+x0*x1+y0*y1+z0*z1; dx = w0*x1-x0*w1-y0*z1+z0*y1
    dy = w0*y1+x0*z1-y0*w1-z0*x1; dz = w0*z1-x0*y1+y0*x1-z0*w1
    ng = dw < 0; dw[ng], dx[ng], dy[ng], dz[ng] = -dw[ng], -dx[ng], -dy[ng], -dz[ng]
    nv = np.sqrt(dx*dx+dy*dy+dz*dz)
    k = np.where(nv > 1e-15, 2*np.arctan2(nv, np.clip(dw, -1, 1))/np.maximum(nv, 1e-300), 0)
    d = np.stack([dx*k, dy*k, dz*k], 1)
    v = q[-1, 1:]*(1 if q[-1, 0] >= 0 else -1)
    ang = 2*np.arctan2(np.linalg.norm(v), abs(q[-1, 0]))
    e = ang*v/np.linalg.norm(v)
    qNi = q[-1].copy(); qNi[1:] *= -1
    aw, ax_, ay_, az_ = qNi
    H = np.zeros((3, 3, 3))
    for s in range(0, N-1, 200000):
        t = min(N-1, s+200000); bw, bx, by, bz = q1[s:t].T
        rw = aw*bw-ax_*bx-ay_*by-az_*bz; rx = aw*bx+ax_*bw+ay_*bz-az_*by
        ry = aw*by-ax_*bz+ay_*bw+az_*bx; rz = aw*bz+ax_*by-ay_*bx+az_*bw
        col = np.stack([np.stack([1-2*(ry*ry+rz*rz), 2*(rx*ry+rw*rz), 2*(rx*rz-rw*ry)], 1),
                        np.stack([2*(rx*ry-rw*rz), 1-2*(rx*rx+rz*rz), 2*(ry*rz+rw*rx)], 1),
                        np.stack([2*(rx*rz+rw*ry), 2*(ry*rz-rw*rx), 1-2*(rx*rx+ry*ry)], 1)], 2)
        dd = d[s:t]
        for j in range(3):
            H[:, j, :] += col[:, :, j].T @ dd
    rr = np.linalg.norm(d, axis=1)*8032.7*180/np.pi
    return dict(H=H, e=e, ang=np.degrees(ang), N=N, rr=rr, d=d)

for fn in sys.argv[1:]:
    r = tens(fn)
    mv = r['rr'] > 50
    print("=== %s" % fn)
    print("  N=%d  %.1f s   运动段 %.1f s   速率 均值%4.0f p99 %5.0f 最大%5.0f dps"
          % (r['N'], r['N']/8032.7, mv.sum()/8032.7, r['rr'][mv].mean(),
             np.percentile(r['rr'], 99), r['rr'].max()))
    print("  累计行程 %s deg   净体轴转角 %s deg"
          % (np.array2string(np.degrees(np.abs(r['d']).sum(0)), precision=0),
             np.array2string(np.degrees(r['d'].sum(0)), precision=0)))
    print("  >>> 末态偏差 = %.3f deg   轴 %s"
          % (r['ang'], np.array2string(r['e']/np.linalg.norm(r['e']), precision=3)))
    # 样本外误差就是原始值；再算"若用上一版参数"会是多少，作为对照
    C = (KP @ np.diag(1.0/PREV)) @ np.linalg.inv(KN @ np.diag(1.0/NEW))
    print("      对照：同样这条运动，若装的是上一版(16.2970/16.4356/16.4382 + 旧 K) -> %.3f deg"
          % np.degrees(np.linalg.norm(r['e'] + np.einsum('ijl,jl->i', r['H'], C-np.eye(3)))))
    G = np.stack([r['H'][:, j, j] for j in range(3)], 1)
    sd = np.linalg.solve(G, -r['e'])
    print("      这条运动单独反解需要的等效标度 %s （当前 %s）"
          % (np.array2string(NEW/(1+sd), precision=4), np.array2string(NEW, precision=4)))
