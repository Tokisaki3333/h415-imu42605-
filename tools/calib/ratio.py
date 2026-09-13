# -*- coding: utf-8 -*-
"""角度积分误差比例：误差 / 累计行程，逐条记录；再拟合"固定底 + 比例项"。"""
import re
import numpy as np
np.set_printoptions(precision=6, suppress=True, linewidth=160)

OLD = np.array([16.3182, 16.5382, 16.4849])
PREV = np.array([16.2760, 16.4486, 16.4184])
NEW = np.array([16.2970, 16.4356, 16.4382])
CUR = np.array([16.2753, 16.4366, 16.4235])
KL = np.array([[1, 0.001914, -0.002047], [0.001914, 1, -0.001009], [-0.002047, -0.001009, 1]])
KC = np.array([[1, 0.001528, -0.001923], [0.001528, 1, 0.000005], [-0.001923, 0.000005, 1]])

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
    return dict(H=H, e=e, ang=np.degrees(ang),
                travel=np.degrees(np.abs(d).sum()), N=N, d=d)

# (文件, 记录时固件传递阵, 是否拟合集)
def mfw(kind, S):
    return (KL @ np.diag(1.0/S)) if kind == 2 else np.diag(1.0/S)
REC = [('serial_runtime_20260913_190158_640_export.txt', 0, OLD,  'x整圈+', 1),
       ('serial_runtime_20260913_190359_575_export.txt', 0, OLD,  'x整圈-', 1),
       ('serial_runtime_20260913_190713_772_export.txt', 0, OLD,  'y整圈+', 1),
       ('serial_runtime_20260913_190745_795_export.txt', 0, OLD,  'y整圈-', 1),
       ('serial_runtime_20260913_184905_223_export.txt', 0, OLD,  '翻滚184905', 1),
       ('serial_runtime_20260913_191953_344_export.txt', 0, PREV, '翻滚191953', 1),
       ('serial_runtime_20260913_193108_876_export.txt', 2, NEW,  '翻滚A', 1),
       ('serial_runtime_20260913_193303_307_export.txt', 2, NEW,  '翻滚B', 1),
       ('serial_runtime_20260913_193339_436_export.txt', 2, NEW,  '翻滚C', 1),
       ('serial_runtime_20260913_193816_365_export.txt', 2, CUR,  '校验(样本外)', 0)]

rows = []
print("%-12s %10s %10s %9s   %s" % ('记录', '行程deg', '残余deg', '比例', '集'))
for fn, kind, S, tag, infit in REC:
    r = tens(fn)
    C = (KC @ np.diag(1.0/CUR)) @ np.linalg.inv(mfw(kind, S))
    res = np.degrees(np.linalg.norm(r['e'] + np.einsum('ijl,jl->i', r['H'], C-np.eye(3))))
    if not infit:
        res = r['ang']          # 样本外：原始偏差就是答案
    ratio = res/r['travel']
    rows.append((tag, r['travel'], res, ratio, infit))
    print("%-12s %10.0f %10.3f %8.3f%%   %s" % (tag, r['travel'], res, ratio*100,
                                                '拟合' if infit else '★样本外'))

T = np.array([x[1] for x in rows]); E = np.array([x[2] for x in rows])
A = np.stack([np.ones_like(T), T], 1)
coef, *_ = np.linalg.lstsq(A, E, rcond=None)
print("\n拟合 误差 = %.3f deg + %.3e x 行程  (即比例 %.4f%% + 固定底 %.3f deg)"
      % (coef[0], coef[1], coef[1]*100, coef[0]))
print("样本外那条的比例（误差/行程）: %.4f %%  = %.1f ppm"
      % (rows[-1][3]*100, rows[-1][3]*1e6))
fit = [x for x in rows if x[4]]
print("拟合集 9 条的比例范围: %.4f%% ~ %.4f%%"
      % (min(x[3] for x in fit)*100, max(x[3] for x in fit)*100))
print("对照：修之前（初版固件）长翻滚 23.505/19372 = %.4f%%" % (23.505/19372*100))
