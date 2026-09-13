# -*- coding: utf-8 -*-
"""一条常数逐轴标度能不能同时满足所有记录？——最小二乘给出唯一答案。"""
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
    # 转成"每轴独立标度"的 3 个需求（相对加载值）
    G = np.stack([H[:, j, j] for j in range(3)], 1)
    sd = np.linalg.solve(G, -e)
    return H, e, np.degrees(ang), sd

REC = [('serial_runtime_20260913_184905_223_export.txt', np.array([16.3182, 16.5382, 16.4849]), '翻滚184905(旧)'),
       ('serial_runtime_20260913_191953_344_export.txt', np.array([16.2760, 16.4486, 16.4184]), '翻滚191953(新)')]

L = np.array([26.0, 26.0, 21.0])          # 慢整圈各自需要的标度（x,y,z 三个轴分别测的）
data = []
print("每一条激励各自需要的逐轴标度（3 个自由度，恒能归零；信息在数值里）：")
for fn, ls, tag in REC:
    H, e, ang, sd = tens(fn)
    data.append((H, e, ls))
    print("  %-14s 原偏差 %6.3f deg   需求校正 %s %%  ->  绝对标度 %s"
          % (tag, ang, np.array2string(sd*100, precision=4),
             np.array2string(ls/(1+sd), precision=4)))
print("  %-14s %s" % ('慢整圈(实测)', np.array2string(L, precision=4)))

# 最小二乘：找一条常数逐轴标度 S，同时满足两条翻滚
def resid(S):
    out = []
    for H, e, ls in data:
        C = np.diag(ls/S)
        out.append(e + np.einsum('ijl,jl->i', H, C-np.eye(3)))
    return np.concatenate(out)

S = np.array([16.30, 16.55, 16.42])
for _ in range(40):
    r = resid(S); J = np.zeros((6, 3))
    for j in range(3):
        dS = S.copy(); dS[j] += 1e-5
        J[:, j] = (resid(dS)-r)/1e-5
    S = S - np.linalg.lstsq(J, r, rcond=None)[0]
print("\n最小二乘同时满足两条翻滚的唯一条数标度： %s" % np.array2string(S, precision=4))
for (H, e, ls), tag in zip(data, [t for _, _, t in REC]):
    C = np.diag(ls/S)
    rr = e + np.einsum('ijl,jl->i', H, C-np.eye(3))
    print("  %-14s 残差 %.3f deg （原 %.3f，单条自解时 0.000）"
          % (tag, np.degrees(np.linalg.norm(rr)), np.degrees(np.linalg.norm(e))))
print("\n与慢整圈实测值的差： %s %%" % np.array2string((S/L-1)*100, precision=4))
