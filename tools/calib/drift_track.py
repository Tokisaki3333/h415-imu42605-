# -*- coding: utf-8 -*-
"""
逐面追踪四元数的倾角误差（方向无关），定位误差是在哪一次搬运里跳的。
★ 绑定 ±4 g 档（8192 LSB/g）的旧记录；固件已改 ±16 g，新数据要改 8192->2048 并重解 S/B。

倾角误差定义：加速度计测到的"上"方向 vs 四元数推算的"上"方向，两者夹角。
   上方向(机体系, 实测) = f/|f|              f 由六面标定的标度+零偏校正
   上方向(机体系, 估计) = R(q)^T · [0,0,1]
这个量与朝向无关，所以 7 个面可以直接串起来比。

同时查：加速度有没有打到 ±4g 削顶、陀螺有没有打到 ±2000 dps 削顶。
"""
import re
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=160)
G0 = 9.80665
RX = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*)+)')
PRE = 'serial_runtime_20260913_'
F = [('222919_239', 'z+'), ('222934_128', 'z-'), ('222951_145', 'y-'),
     ('223007_913', 'y+'), ('223026_247', 'x+'), ('223107_311', 'x-'),
     ('223149_605', 'z+ 回初始')]
# 六面成对法解出的标度/零偏（见 sixface_accel.py）
S = np.array([8114.619, 8163.620, 8067.570]) / G0      # LSB/(m/s^2)
B = np.array([-38.357, -56.534, 184.037])              # LSB


def load(tag):
    frames = RX.findall(open(PRE + tag + '_export.txt', errors='ignore').read())
    b = b''.join(bytes.fromhex(s.replace(' ', '')) for s in frames)
    a = np.frombuffer(b, dtype='<f4').reshape(-1, 8)[:, :7].astype(np.float64)
    q = a[:, :4].copy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q, a[:, 4:7]


def Rmat(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]])


print("=" * 100)
print("一、有没有削顶（搬运磕碰会打满量程）")
print("=" * 100)
print("%-14s %10s %10s %12s %12s %10s" %
      ('记录', '|a|max LSB', '|a|max g', '打到±4g帧', '|w|max dps', '打到2000帧'))
allw = []
for tag, face in F:
    q, acc = load(tag)
    amax = np.abs(acc).max()
    rail_a = int((np.abs(acc) >= 32760).sum())
    dt = 1.0/8027.0
    d = np.degrees(2*np.arccos(np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)))/dt
    allw.append(d.max())
    print("%-14s %10.1f %10.4f %12d %12.0f %10d"
          % (tag, amax, amax/8192.0, rail_a, d.max(), int((d >= 1990).sum())))

print("\n" + "=" * 100)
print("二、逐面倾角误差（方向无关）—— 看它在哪一次搬运后跳变")
print("=" * 100)
print("%-14s %-12s %10s %10s %10s   误差矢量(机体系, deg)" %
      ('记录', '面', 'a倾角', 'q倾角', '误差deg'))
rows = []
for tag, face in F:
    q, acc = load(tag)
    qs, as_ = q[4000:-4000], acc[4000:-4000]
    f = (as_.mean(0) - B) / S
    u_meas = f / np.linalg.norm(f)
    qm = qs.mean(0); qm /= np.linalg.norm(qm)
    u_est = Rmat(qm).T @ np.array([0.0, 0.0, 1.0])
    c = np.clip(np.dot(u_meas, u_est), -1, 1)
    err = np.degrees(np.arccos(c))
    v = np.cross(u_meas, u_est)
    n = np.linalg.norm(v)
    ax = np.degrees(np.arctan2(n, c)) * (v/n if n > 1e-12 else v)
    a_rp = (np.degrees(np.arctan2(f[1], f[2])),
            np.degrees(np.arctan2(-f[0], np.hypot(f[1], f[2]))))
    w, x, y, z = qm
    q_rp = (np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1))))
    rows.append((tag, face, err, ax))
    print("%-14s %-12s %+9.4f %+9.4f %10.4f   [%+7.4f %+7.4f %+7.4f]"
          % (tag, face, np.hypot(*a_rp), np.hypot(*q_rp), err, ax[0], ax[1], ax[2]))

print("\n  相邻两次搬运造成的误差增量（前一面的误差 -> 后一面的误差）：")
prev = None
for tag, face, err, ax in rows:
    if prev is not None:
        print("    %-12s -> %-12s   %.4f -> %.4f deg   增量 %+.4f deg"
              % (prev[1], face, prev[2], err, err - prev[2]))
    prev = (tag, face, err, ax)
print("\n  注意：第 1 面本身已带误差（测量开始前就在累积），所以增量才是单次搬运的贡献。")
