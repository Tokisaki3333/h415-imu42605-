# -*- coding: utf-8 -*-
"""把两条翻滚记录联立，解一个**对称**的 3x3 修正矩阵（对角=逐轴标度，非对角=交叉项）。

对角项就是"每轴一个标度"；如果只解对角，两条记录的答案互相矛盾（差 1~1.7%）。
这里加 3 个对称非对角自由度，看能不能同时满足两条 —— 能，就说明缺的是交叉项，
不是"每轴一个标度"。
"""
import re
import numpy as np
np.set_printoptions(precision=6, suppress=True, linewidth=160)

def load(fn):
    raw = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){20})').findall(
        open(fn, 'r', errors='ignore').read())
    b = np.frombuffer(b''.join(bytes.fromhex(s.replace(' ', '')) for s in raw), dtype='<f4')
    q = b.reshape(-1, 5)[:, :4].astype(np.float64)
    return q / np.linalg.norm(q, axis=1, keepdims=True)

def inc(q):
    q0, q1 = q[:-1], q[1:]
    w0, x0, y0, z0 = q0.T; w1, x1, y1, z1 = q1.T
    dw = w0*w1+x0*x1+y0*y1+z0*z1; dx = w0*x1-x0*w1-y0*z1+z0*y1
    dy = w0*y1+x0*z1-y0*w1-z0*x1; dz = w0*z1-x0*y1+y0*x1-z0*w1
    ng = dw < 0; dw[ng], dx[ng], dy[ng], dz[ng] = -dw[ng], -dx[ng], -dy[ng], -dz[ng]
    nv = np.sqrt(dx*dx+dy*dy+dz*dz)
    k = np.where(nv > 1e-15, 2*np.arctan2(nv, np.clip(dw, -1, 1))/np.maximum(nv, 1e-300), 0)
    return np.stack([dx*k, dy*k, dz*k], 1)

def tens(fn):
    q = load(fn); d = inc(q); N = len(q)
    v = q[-1, 1:]*(1 if q[-1, 0] >= 0 else -1)
    ang = 2*np.arctan2(np.linalg.norm(v), abs(q[-1, 0]))
    e = ang*v/np.linalg.norm(v)
    qNi = q[-1].copy(); qNi[1:] *= -1
    aw, ax_, ay_, az_ = qNi; q1 = q[1:]
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
    return H, e, np.degrees(ang)

REC = [('serial_runtime_20260913_184905_223_export.txt', np.array([16.3182, 16.5382, 16.4849]), '184905 旧标度'),
       ('serial_runtime_20260913_191953_344_export.txt', np.array([16.2760, 16.4486, 16.4184]), '191953 新标度')]

A = []; b = []; info = []
for fn, ls, tag in REC:
    H, e, ang = tens(fn)
    a = np.stack([H[:, 0, 0], H[:, 1, 1], H[:, 2, 2],
                  H[:, 0, 1]+H[:, 1, 0], H[:, 0, 2]+H[:, 2, 0], H[:, 1, 2]+H[:, 2, 1]], 1)
    A.append(a); b.append(-e); info.append((tag, ls, H, e, ang))
A = np.vstack(A); b = np.concatenate(b)
p = np.linalg.solve(A, b)
dC = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
print("联立解出的对称修正矩阵 dC（记录值需乘 I+dC 才闭合），单位 %：")
print(dC*100)
print("  = 对角(逐轴标度) %s %%  + 对称交叉 %s %%"
      % (np.array2string(np.diag(dC)*100, precision=4),
         np.array2string(dC[np.triu_indices(3, 1)]*100, precision=4)))

print("\n各记录残差（线性）：")
for (tag, ls, H, e, ang), a in zip(info, [A[:3], A[3:]]):
    r = e + a @ p
    print("  %-16s 原偏差 %.3f deg -> 联立修正后 %.3f deg" % (tag, np.degrees(ang), np.degrees(np.linalg.norm(r))))

# 只解对角（每轴一个标度）的残差对比
print("\n只解对角（=每轴一个标度）时的残差：")
for tag, ls, H, e, ang in info:
    G = np.stack([H[:, j, j] for j in range(3)], 1)
    sd = np.linalg.solve(G, -e)
    print("  %-16s 需要 %s %%  -> 0；但与另一条冲突" % (tag, np.array2string(sd*100, precision=4)))

# 换算成"真标度"（对角项 = 标度，非对角 = 交叉灵敏）
print("\n换算（以 191953 的新标度为参考基准，真测量阵 = inv(I+dC) 作用在加载标度上）：")
for tag, ls, H, e, ang in info:
    Ct = np.diag(ls) @ np.linalg.inv(np.eye(3)+dC)
    print("  %-16s 真标度对角 %s" % (tag, np.array2string(np.diag(Ct), precision=4)))
