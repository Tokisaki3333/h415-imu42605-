# -*- coding: utf-8 -*-
"""对任意四元数记录：算末态偏差，并比较各套标度下的预测偏差。"""
import re, sys, os, time
import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth=160)

FN = sys.argv[1]
OLD = np.array([16.3182, 16.5382, 16.4849])
NEW = np.array([16.2760, 16.4486, 16.4184])

raw = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){20})').findall(
    open(FN, 'r', errors='ignore').read())
buf = np.frombuffer(b''.join(bytes.fromhex(s.replace(' ', '')) for s in raw), dtype='<f4')
q = buf.reshape(-1, 5)[:, :4].astype(np.float64)
q /= np.linalg.norm(q, axis=1, keepdims=True)
N = len(q)

v = q[-1, 1:] * (1.0 if q[-1, 0] >= 0 else -1.0)
ang = 2.0 * np.arctan2(np.linalg.norm(v), abs(q[-1, 0]))
axis = v / max(np.linalg.norm(v), 1e-30)
e_vec = ang * axis
print("%s   N=%d  %.1f s" % (FN, N, N / 8032.7))
print("末态 q = %s" % np.array2string(q[-1], precision=6))
print("末态偏差 |e| = %.3f deg  轴 = %s   分量占比 %.0f%%/%.0f%%/%.0f%%"
      % (np.degrees(ang), np.array2string(axis, precision=3), *(100 * axis ** 2)))

q0, q1 = q[:-1], q[1:]
w0, x0, y0, z0 = q0.T; w1, x1, y1, z1 = q1.T
dw = w0*w1 + x0*x1 + y0*y1 + z0*z1
dx = w0*x1 - x0*w1 - y0*z1 + z0*y1
dy = w0*y1 + x0*z1 - y0*w1 - z0*x1
dz = w0*z1 - x0*y1 + y0*x1 - z0*w1
neg = dw < 0
dw[neg], dx[neg], dy[neg], dz[neg] = -dw[neg], -dx[neg], -dy[neg], -dz[neg]
nv = np.sqrt(dx*dx + dy*dy + dz*dz)
kk = np.where(nv > 1e-15, 2*np.arctan2(nv, np.clip(dw, -1, 1)) / np.maximum(nv, 1e-300), 0)
d = np.stack([dx*kk, dy*kk, dz*kk], 1)
print("累计转角 deg：净 %s / 绝对 %s"
      % (np.array2string(np.degrees(d.sum(0)), precision=0),
         np.array2string(np.degrees(np.abs(d).sum(0)), precision=0)))

qNi = q[-1].copy(); qNi[1:] *= -1.0
aw, ax_, ay_, az_ = qNi
H = np.zeros((3, 3, 3))
for s in range(0, N-1, 200000):
    t = min(N-1, s+200000)
    bw, bx, by, bz = q1[s:t].T
    rw = aw*bw - ax_*bx - ay_*by - az_*bz
    rx = aw*bx + ax_*bw + ay_*bz - az_*by
    ry = aw*by - ax_*bz + ay_*bw + az_*bx
    rz = aw*bz + ax_*by - ay_*bx + az_*bw
    col = np.stack([
        np.stack([1-2*(ry*ry+rz*rz), 2*(rx*ry+rw*rz), 2*(rx*rz-rw*ry)], 1),
        np.stack([2*(rx*ry-rw*rz), 1-2*(rx*rx+rz*rz), 2*(ry*rz+rw*rx)], 1),
        np.stack([2*(rx*rz+rw*ry), 2*(ry*rz-rw*rx), 1-2*(rx*rx+ry*ry)], 1)], 2)
    dd = d[s:t]
    for j in range(3):
        H[:, j, :] += col[:, :, j].T @ dd

# 记录里跑的标度若为 S_load，则"真标度 = S"等价于把增量乘 (S_load/S)
def err(S_load, S):
    C = np.diag(S_load / S)
    return np.degrees(np.linalg.norm(e_vec + np.einsum('ijl,jl->i', H, C - np.eye(3))))

print("\n用增量张量的线性预测（同一激励、换标度）：")
for name_load, L in (("旧标度", OLD), ("新标度", NEW)):
    for name_true, S in (("旧标度", OLD), ("新标度", NEW), ("真值", None)):
        if S is None:
            # 本记录自身的最优逐轴解
            G = np.stack([H[:, j, j] for j in range(3)], 1)
            sd = np.linalg.solve(G, -e_vec)
            c = np.diag(L / (1 + sd))
            print("  [假设记录用%s] 换成 '本记录最优' %s -> %.3f deg"
                  % (name_load, np.array2string(c, precision=4), 0.0))
            continue
        print("  [假设记录用%s] 真标度=%s -> 预测末态 %.3f deg" % (name_load, name_true, err(L, S)))
