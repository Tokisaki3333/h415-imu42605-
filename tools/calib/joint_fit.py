# -*- coding: utf-8 -*-
"""
联立全部 6 条四元数记录，解"真实测量阵"：
    LSB = diag(S_load) · dps_load
    dps_true = K · diag(1/S) · LSB        K = 对称、对角=1（交叉灵敏），S = 逐轴真标度
    => dps_true = [K · diag(S_load/S)] · dps_load
未知量 6 个：Sx,Sy,Sz（逐轴标度）+ kxy,kxz,kyz（对称交叉）
方程 6 条记录 x 3 = 18 个 -> 过约束，残差就是判据。
逐条记录单独用对角模型都能归零（3 未知 3 方程），联立才分得开。
"""
import re
import numpy as np
np.set_printoptions(precision=6, suppress=True, linewidth=160)

OLD = np.array([16.3182, 16.5382, 16.4849])
NEW = np.array([16.2760, 16.4486, 16.4184])

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
    return H, e, np.degrees(ang), d, q

REC = [('serial_runtime_20260913_190158_640_export.txt', OLD, 'x整圈+'),
       ('serial_runtime_20260913_190359_575_export.txt', OLD, 'x整圈-'),
       ('serial_runtime_20260913_190713_772_export.txt', OLD, 'y整圈+'),
       ('serial_runtime_20260913_190745_795_export.txt', OLD, 'y整圈-'),
       ('serial_runtime_20260913_184905_223_export.txt', OLD, '翻滚184905'),
       ('serial_runtime_20260913_191953_344_export.txt', NEW, '翻滚191953')]

data = []
for fn, ls, tag in REC:
    H, e, ang, d, q = tens(fn)
    data.append(dict(tag=tag, H=H, e=e, ang=ang, ls=ls))
    print("  %-10s 末态偏差 %7.3f deg   加载标度 %s" % (tag, ang, np.array2string(ls, precision=4)))

def model(p):
    S = p[:3]; K = np.array([[1, p[3], p[4]], [p[3], 1, p[5]], [p[4], p[5], 1]])
    out = []
    for dd in data:
        C = K @ np.diag(dd['ls']/S)
        out.append(dd['e'] + np.einsum('ijl,jl->i', dd['H'], C-np.eye(3)))
    return np.concatenate(out)

p = np.array([16.28, 16.45, 16.41, 0.0, 0.0, 0.0])
for it in range(60):
    r = model(p); J = np.zeros((18, 6))
    for j in range(6):
        dp = p.copy(); dp[j] += 1e-6*(1 if j >= 3 else 1e-2)
        J[:, j] = (model(dp)-r)/(1e-6*(1 if j >= 3 else 1e-2))
    p = p - np.linalg.lstsq(J, r, rcond=None)[0]

S = p[:3]; K = np.array([[1, p[3], p[4]], [p[3], 1, p[5]], [p[4], p[5], 1]])
print("\n联立解（18 方程 / 6 未知）：")
print("  逐轴真标度 S = %s" % np.array2string(S, precision=4))
print("  对称交叉灵敏 K（对角=1）= \n%s" % np.array2string(K, precision=6))
print("  相对现在的加载值：")
for i in range(3):
    print("    %s: %.4f -> %.4f  (%+.4f%%)" % ('xyz'[i], OLD[i] if i else OLD[i], S[i], (S[i]/16.4-1)*100))
print("\n各记录残差（这条才是判据）：")
for dd in data:
    C = K @ np.diag(dd['ls']/S)
    r = dd['e'] + np.einsum('ijl,jl->i', dd['H'], C-np.eye(3))
    print("  %-10s 原 %7.3f deg  ->  联立后残差 %7.3f deg" % (dd['tag'], dd['ang'], np.degrees(np.linalg.norm(r))))
