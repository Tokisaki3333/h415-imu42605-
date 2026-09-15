# -*- coding: utf-8 -*-
# 泄漏的第一性测量：纯倾角误差 eps（无偏航误差）时, 磁新息 r 会读到多少?
#   磁观测 = 把实测场与模型场都投到 姿态自己的重力法向量平面 ⊥ ab, 取平面内方位角之差
#   若 H=[0..0,1] 成立, 纯倾角误差应给 r=0。实测 r 就是"倾角误差污染偏航新息"的量。
import numpy as np

DIP = np.radians(64.32); DECL = np.radians(-7.53)
B0 = np.array([np.cos(DIP)*np.cos(DECL), np.cos(DIP)*np.sin(DECL), -np.sin(DIP)])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def expq(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    h = 0.5*th
    return np.array([np.cos(h), *(np.sin(h)/th*v)])


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def q2R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def mag_innov(R_hat, R_true):
    """按固件口径算磁新息：同一平面 ⊥ ab 内, 实测方位 - 模型方位"""
    ab = R_hat.T @ np.array([0.0, 0.0, 1.0])
    m_body = R_true.T @ B0                       # 实测场(机体系, 无噪声)
    fp = R_hat.T @ B0                            # 模型场(机体系)
    mh = m_body - np.dot(m_body, ab)*ab
    mh2 = fp - np.dot(fp, ab)*ab
    xb = np.array([1.0, 0.0, 0.0])
    xb = xb - np.dot(xb, ab)*ab
    crs = np.cross(ab, xb)
    thm = np.arctan2(np.dot(crs, mh), np.dot(mh, xb))
    thp = np.arctan2(np.dot(crs, mh2), np.dot(mh2, xb))
    return (thm - thp + np.pi) % (2*np.pi) - np.pi


def attitude(yaw, tilt_ax, tilt_ang):
    """先绕竖直转 yaw，再绕水平轴 tilt_ax 倾斜 tilt_ang"""
    qy = expq(np.array([0.0, 0.0, yaw]))
    qt = expq(np.array(tilt_ax)*tilt_ang)
    return q2R(qmul(qy, qt))


print('b0 = %s  |b0| = %.6f' % (np.round(B0, 6), np.linalg.norm(B0)))
print()
print('=== 纯倾角误差 eps (无偏航误差), 磁新息 r 读到多少 ===')
print('   倾角误差轴: 绕 x / 绕 y / 绕 与"水平面内磁场垂直"的水平轴(最坏轴)')
print()
hdir = B0[:2]/np.linalg.norm(B0[:2])
worst_ax = np.array([-hdir[1], hdir[0], 0.0])
print('  %6s | %12s %12s %14s | %12s' % ('eps', '绕x r', '绕y r', '最坏轴 r', 'r/eps(最坏)'))
for eps_deg in [1, 2, 5, 10, 15, 20, 30, 40]:
    eps = np.radians(eps_deg)
    row = []
    for ax in [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), worst_ax]:
        rs = []
        for yaw in np.linspace(0, 2*np.pi, 24, endpoint=False):
            Rt = attitude(yaw, ax, eps)
            Rh = attitude(yaw, np.zeros(3), 0.0)      # 估计: 只有倾角错, 偏航对
            rs.append(mag_innov(Rh, Rt))
        row.append(np.degrees(np.sqrt(np.mean(np.square(rs)))))
    print('  %5.0f° | %11.3f° %11.3f° %13.3f° | %11.4f'
          % (eps_deg, row[0], row[1], row[2], row[2]/eps_deg))
print()
print('  说明: r 应=0 (H 里没有倾角列)。上面非零即"倾角误差污染偏航新息"。')
print()
print('=== 等价地: 倾角误差 eps 会被滤波器当成多大的偏航激励 ===')
for eps_deg in [5, 10, 20, 30]:
    eps = np.radians(eps_deg)
    # 取使 r 最大的相对方位, 扫 yaw 与倾角轴角
    best = 0.0
    for yaw in np.linspace(0, 2*np.pi, 48, endpoint=False):
        for axang in np.linspace(0, 2*np.pi, 24, endpoint=False):
            ax = np.array([np.cos(axang), np.sin(axang), 0.0])
            Rt = attitude(yaw, ax, eps)
            Rh = attitude(yaw, np.zeros(3), 0.0)
            best = max(best, abs(mag_innov(Rh, Rt)))
    print('   倾角误差 %2.0f° -> 最坏偏航等效激励 %7.3f°  (系数 %.3f)'
          % (eps_deg, np.degrees(best), np.degrees(best)/eps_deg))
