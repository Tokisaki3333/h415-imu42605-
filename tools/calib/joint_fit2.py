# -*- coding: utf-8 -*-
"""
9 条记录联立，正确地处理"记录时固件里装的是哪一版"：

  dps_recorded = M_fw · LSB        M_fw = 当时固件的传递阵
  dps_true     = M_true · LSB      M_true = K · diag(1/S)   （6 个未知）
  把记录值换算到真值： C = M_true · inv(M_fw)

  M_fw = diag(1/S_load)                 初版 / 上一版（只逐轴）
  M_fw = K_loaded · diag(1/S_load)      已载入版（逐轴 + 对称交叉）
"""
import re
import numpy as np
np.set_printoptions(precision=6, suppress=True, linewidth=160)

OLD = np.array([16.3182, 16.5382, 16.4849])
PREV = np.array([16.2760, 16.4486, 16.4184])
NEW = np.array([16.2970, 16.4356, 16.4382])
KL = np.array([[1, 0.001914, -0.002047], [0.001914, 1, -0.001009], [-0.002047, -0.001009, 1]])

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
    return dict(H=H, e=e, ang=np.degrees(ang))

def mfw(kind, S):
    return (KL @ np.diag(1.0/S)) if kind else np.diag(1.0/S)

REC = [('serial_runtime_20260913_190158_640_export.txt', False, OLD,  'x整圈+'),
       ('serial_runtime_20260913_190359_575_export.txt', False, OLD,  'x整圈-'),
       ('serial_runtime_20260913_190713_772_export.txt', False, OLD,  'y整圈+'),
       ('serial_runtime_20260913_190745_795_export.txt', False, OLD,  'y整圈-'),
       ('serial_runtime_20260913_184905_223_export.txt', False, OLD,  '翻滚184905'),
       ('serial_runtime_20260913_191953_344_export.txt', False, PREV, '翻滚191953'),
       ('serial_runtime_20260913_193108_876_export.txt', True,  NEW,  '新翻滚A'),
       ('serial_runtime_20260913_193303_307_export.txt', True,  NEW,  '新翻滚B'),
       ('serial_runtime_20260913_193339_436_export.txt', True,  NEW,  '新翻滚C')]

data = []
for fn, hasK, S, tag in REC:
    r = tens(fn); r['M'] = mfw(hasK, S); r['tag'] = tag
    r['Minv'] = np.linalg.inv(r['M'])
    data.append(r)
    print("  %-10s 原始末态偏差 %8.3f deg" % (tag, r['ang']))

def model(p):
    S = p[:3]; K = np.array([[1, p[3], p[4]], [p[3], 1, p[5]], [p[4], p[5], 1]])
    Mt = K @ np.diag(1.0/S)
    out = []
    for r in data:
        C = Mt @ r['Minv']
        out.append(r['e'] + np.einsum('ijl,jl->i', r['H'], C-np.eye(3)))
    return np.concatenate(out)

p = np.array([16.30, 16.44, 16.44, 0.002, -0.002, -0.001])
last = None
for it in range(100):
    res = model(p); J = np.zeros((3*len(data), 6))
    for j in range(6):
        h = 1e-7*(1 if j >= 3 else 1e-3)
        dp = p.copy(); dp[j] += h
        J[:, j] = (model(dp)-res)/h
    step = np.linalg.lstsq(J, res, rcond=None)[0]
    p = p - step
    if last is not None and np.linalg.norm(step) < 1e-12: break
    last = p.copy()

S = p[:3]; K = np.array([[1, p[3], p[4]], [p[3], 1, p[5]], [p[4], p[5], 1]])
print("\n=== 9 条记录联立（27 方程 / 6 未知）===")
print("  S = %s   相对标称 %s %%" % (np.array2string(S, precision=4),
                                     np.array2string((S/16.4-1)*100, precision=4)))
print("  K = \n%s" % np.array2string(K, precision=6))
print("  （当前固件装的是 S=%s, K=KL）" % np.array2string(NEW, precision=4))
rms = []
print("\n  各记录残差（含记录时固件版本的正确换算）：")
for r in data:
    C = (K @ np.diag(1.0/S)) @ r['Minv']
    rr = r['e'] + np.einsum('ijl,jl->i', r['H'], C-np.eye(3))
    v = np.degrees(np.linalg.norm(rr)); rms.append(v*v)
    print("    %-10s 原 %8.3f -> 残差 %7.3f deg" % (r['tag'], r['ang'], v))
print("  RMS = %.3f deg" % np.sqrt(np.mean(rms)))
