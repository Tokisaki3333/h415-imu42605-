# -*- coding: utf-8 -*-
"""
反解 / 正向验证：在这条复杂长运动激励下，多大的角标度误差能造出记录到的末态四元数偏差。

原理
  记录里的 q_k 是固件输出姿态。相邻帧的机体增量 dq_k = conj(q_k) (x) q_{k+1}
  与 dt 无关（就是"这一帧转过的角度"），主机丢帧不影响结论。
  把每帧增量按常矩阵 C 重标（w_true = C w_meas 等价于 d -> C d），重积：
      q_N(C) = q_N (x) exp(e(C)/2),   e(C) = sum_k A_k (C-I) d_k
      A_k = R( conj(q_N) (x) q_{k+1} )   且 e(I) = 0，故 e 即"相对原样的改变量"
  用三阶张量写成线性形式：e[i] = sum_{j,l} H[i,j,l] (C[j,l]-delta_jl)
      H[i,j,l] = sum_k A_k[i,j] * d_k[l]
"""
import re
import numpy as np

FN = 'serial_runtime_20260913_184905_223_export.txt'
np.set_printoptions(precision=6, suppress=True, linewidth=160)

print("parsing %s ..." % FN)
raw = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){20})').findall(
    open(FN, 'r', errors='ignore').read())
buf = np.frombuffer(b''.join(bytes.fromhex(s.replace(' ', '')) for s in raw), dtype='<f4')
q = buf.reshape(-1, 5)[:, :4].astype(np.float64)      # 20 B/帧 = 4 分量 + 4 B 帧尾
q /= np.linalg.norm(q, axis=1, keepdims=True)
N = len(q)
print("  N = %d frames, q0 = %s" % (N, np.array2string(q[0], precision=7)))

# ---- 末态偏差 ----
v = q[-1, 1:] * (1.0 if q[-1, 0] >= 0 else -1.0)
ang = 2.0 * np.arctan2(np.linalg.norm(v), abs(q[-1, 0]))
axis = v / np.linalg.norm(v)
e_vec = ang * axis
print("\n末态 q = %s" % np.array2string(q[-1], precision=6))
print("末态偏差 = %.3f deg  轴 = %s" % (np.degrees(ang), np.array2string(axis, precision=4)))
print("误差矢量 = %s deg" % np.array2string(np.degrees(e_vec), precision=3))
print("分量占比 = %.1f%% / %.1f%% / %.1f%%" % tuple(100 * axis ** 2))

# ---- 逐帧机体增量 ----
q0, q1 = q[:-1], q[1:]
w0, x0, y0, z0 = q0.T
w1, x1, y1, z1 = q1.T
dw = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
dx = w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1
dy = w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1
dz = w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1
neg = dw < 0.0
dw[neg], dx[neg], dy[neg], dz[neg] = -dw[neg], -dx[neg], -dy[neg], -dz[neg]
nv = np.sqrt(dx * dx + dy * dy + dz * dz)
kk = np.where(nv > 1e-15, 2.0 * np.arctan2(nv, np.clip(dw, -1, 1)) / np.maximum(nv, 1e-300), 0.0)
d = np.stack([dx * kk, dy * kk, dz * kk], axis=1)                 # (N-1,3) rad
print("\n累计转角 deg：净 %s / 绝对 %s"
      % (np.array2string(np.degrees(d.sum(axis=0)), precision=0),
         np.array2string(np.degrees(np.abs(d).sum(axis=0)), precision=0)))

# ---- 张量 H ----
qNi = q[-1].copy(); qNi[1:] *= -1.0
aw, ax_, ay_, az_ = qNi
H = np.zeros((3, 3, 3))
for s in range(0, N - 1, 200000):
    t = min(N - 1, s + 200000)
    bw, bx, by, bz = q1[s:t].T
    rw = aw * bw - ax_ * bx - ay_ * by - az_ * bz
    rx = aw * bx + ax_ * bw + ay_ * bz - az_ * by
    ry = aw * by - ax_ * bz + ay_ * bw + az_ * bx
    rz = aw * bz + ax_ * by - ay_ * bx + az_ * bw
    col = np.stack([
        np.stack([1 - 2 * (ry * ry + rz * rz), 2 * (rx * ry + rw * rz), 2 * (rx * rz - rw * ry)], 1),
        np.stack([2 * (rx * ry - rw * rz), 1 - 2 * (rx * rx + rz * rz), 2 * (ry * rz + rw * rx)], 1),
        np.stack([2 * (rx * rz + rw * ry), 2 * (ry * rz - rw * rx), 1 - 2 * (rx * rx + ry * ry)], 1),
    ], axis=2)                                                    # (n,3,3) A[i,j]
    dd = d[s:t]
    for j in range(3):
        H[:, j, :] += col[:, :, j].T @ dd                         # sum_k A[i,j] d[l]

def e_of(C):
    return np.einsum('ijl,jl->i', H, C - np.eye(3))

def reintegrate(C):
    qi = np.array([1.0, 0.0, 0.0, 0.0])
    dd = d @ C.T
    nv2 = np.linalg.norm(dd, axis=1)
    for i in range(len(dd)):
        t = nv2[i]
        if t < 1e-15:
            continue
        sn = np.sin(0.5 * t) / t
        bw, bx, by, bz = np.cos(0.5 * t), dd[i, 0] * sn, dd[i, 1] * sn, dd[i, 2] * sn
        aw_, ax2, ay2, az2 = qi
        qi = np.array([aw_ * bw - ax2 * bx - ay2 * by - az2 * bz,
                       aw_ * bx + ax2 * bw + ay2 * bz - az2 * by,
                       aw_ * by - ax2 * bz + ay2 * bw + az2 * bx,
                       aw_ * bz + ax2 * by - ay2 * bx + az2 * bw])
    return qi / np.linalg.norm(qi)

def report(tag, C):
    el = e_vec + e_of(C)
    qn = reintegrate(C)
    a = 2 * np.arctan2(np.linalg.norm(qn[1:]), abs(qn[0]))
    print("  %-34s 线性预测 %7.3f deg   精确重积 %7.3f deg   (原 %.3f)"
          % (tag, np.degrees(np.linalg.norm(el)), np.degrees(a), np.degrees(ang)))
    return a

print("\n=== 基线 ===")
report("C = I（原样）", np.eye(3))

print("\n=== (1) 单轴标度误差：只动一个轴，多大的偏差能造出这个末态 ===")
for j in range(3):
    gj = H[:, j, j]
    need = -np.dot(gj, e_vec) / np.dot(gj, gj)
    res = e_vec + gj * need
    print("  仅 %s 轴标度偏差 %+8.4f %%  -> 残余 %.2f deg (从 %.2f 降下来)"
          % ('xyz'[j], need * 100, np.degrees(np.linalg.norm(res)), np.degrees(ang)))

print("\n=== (2) 公共标度误差（三轴同改）===")
g1 = H[:, 0, 0] + H[:, 1, 1] + H[:, 2, 2]
c = -np.dot(g1, e_vec) / np.dot(g1, g1)
report("公共 %+.4f%%" % (c * 100), np.eye(3) * (1 + c))

print("\n=== (3) 三轴对角联合解 ===")
G = np.stack([H[:, j, j] for j in range(3)], axis=1)
sd = np.linalg.solve(G, -e_vec)
report("对角 %s %%" % np.array2string(sd * 100, precision=4), np.diag(1 + sd))
SC_OLD = np.array([16.3182, 16.5382, 16.4849])
print("  => 隐含的标度应为 %s （当时加载的 %s）"
      % (np.array2string(SC_OLD / (1 + sd), precision=4), np.array2string(SC_OLD, precision=4)))

print("\n=== (4) 灵敏度：各轴标度偏 0.1% 造成的末态偏差 ===")
for j in range(3):
    ev = H[:, j, j] * 1e-3
    print("  %s 轴 +0.1%% -> 末态偏差 %6.3f deg  (x %+6.3f y %+6.3f z %+6.3f)"
          % ('xyz'[j], np.degrees(np.linalg.norm(ev)),
             np.degrees(ev[0]), np.degrees(ev[1]), np.degrees(ev[2])))
ev = g1 * 1e-3
print("  公共 +0.1%% -> 末态偏差 %6.3f deg  (x %+6.3f y %+6.3f z %+6.3f)"
      % (np.degrees(np.linalg.norm(ev)), np.degrees(ev[0]), np.degrees(ev[1]), np.degrees(ev[2])))

print("\n=== (5) 正向验证：载入定稿标度后这条独立记录剩多少 ===")
report("C = 定稿 (0.259/0.545/0.405 %)", np.diag([1.00259, 1.00545, 1.00405]))
report("C = 定稿 + 旧整转非对角", np.array([
    [1.00259, -0.0048392, -0.0069472],
    [0.0055836, 1.00545, -0.0044272],
    [-0.0043264, 0.0105391, 1.00405]]))
report("C = 旧公共项 I*1.0036", np.eye(3) * 1.0036)
report("C = 旧整转 M", np.array([
    [1.0036414, -0.0048392, -0.0069472],
    [0.0055836, 1.0036139, -0.0044272],
    [-0.0043264, 0.0105391, 1.0036189]]))

print("\n=== (6) 固定实测 x/y，反解这条激励隐含的 z（一致性检查）===")
ex, ey = 0.00259, 0.00545
rest = e_vec + H[:, 0, 0] * ex + H[:, 1, 1] * ey
ez = -np.dot(H[:, 2, 2], rest) / np.dot(H[:, 2, 2], H[:, 2, 2])
print("  eps_z = %+.4f %%   ->  z 标度 = %.4f  (定稿 16.4184, 差 %+.4f)"
      % (ez * 100, 16.4849 / (1 + ez), 16.4849 / (1 + ez) - 16.4184))
report("C = 实测 x/y + 反解 z", np.diag([1 + ex, 1 + ey, 1 + ez]))
