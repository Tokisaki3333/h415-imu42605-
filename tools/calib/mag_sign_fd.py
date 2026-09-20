# -*- coding: utf-8 -*-
r"""用数值微分直接检验固件里 H = e_i^T [b^_b]x R^T 是否等于真实的 dr/dx（dq x q 注入约定）。

固件事实（proc_ekf.c VER=107）：
  bb = R^T b_n           (b^_b)
  e1 = normalize(gn_b x bb)       gn_b = 归一化加速度
  e2 = bb x e1
  r_i = e_i . m^                  (m^ = 标定后磁场机体系单位矢量)
  H_i(a) = e_i . ( [bb]x R^T e_a )   for a = 0,1,2  (四元数虚部)
  注入:  q <- dq (x) q   =>   R' = R(dq) R     （已由实测帧比对确认，误差 0.06°）

数值微分：把 dq = eps*e_a（世界系）代入 R' = R(dq) R，重算 e1/e2/r，得到真实 Jacobian J。
  J 与 H 一致  ->  H 正确；否则 H 的符号/因子错，闭环会发散或削弱。
再看反馈方向：dx = +K*r（固件就是 K = P H^T(...)^{-1}，clamp 到 K_MAX）
  r.(H dx) < 0 才是负反馈（牵引有效）。

用法:  python tools/calib/mag_sign_fd.py
"""
import numpy as np

DIP_TAN = 1.70
DECL = -7.53


def Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rotvec(v):
    th = np.linalg.norm(v)
    if th < 1e-15:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def resid(R, bn, gn, mh):
    """按固件公式算 (e1, e2, r1, r2)。"""
    bb = R.T @ bn
    bb = bb / np.linalg.norm(bb)
    e1 = np.cross(gn, bb)
    n1 = np.linalg.norm(e1)
    e1 = e1 / n1
    e2 = np.cross(bb, e1)
    e2 = e2 / np.linalg.norm(e2)
    return bb, e1, e2, np.array([e1 @ mh, e2 @ mh])


def code_H(R, bb, e1, e2):
    Sx = np.array([[0, -bb[2], bb[1]], [bb[2], 0, -bb[0]], [-bb[1], bb[0], 0]])
    G = Sx @ R.T                                  # 3x3
    return np.vstack([e1 @ G, e2 @ G])            # H_i(a)


def main():
    ci = 1.0 / np.sqrt(1 + DIP_TAN ** 2)
    bn = np.array([ci * np.sin(np.radians(DECL)), ci * np.cos(np.radians(DECL)), -DIP_TAN * ci])
    print('b_n = %s   |b_n| = %.6f' % (np.round(bn, 4), np.linalg.norm(bn)))
    print('\n 情形                 H[0] 与 J[0]                          H[1] 与 J[1]')
    for tag, Rt_, tilt in (('水平，yaw 误差 3°', Rz(np.radians(3.0)) @ Rx(np.radians(0.0)), 0.0),
                           ('水平，yaw 误差 12°', Rz(np.radians(12.0)), 0.0),
                           ('倾斜 25° + yaw 5°', Rz(np.radians(5.0)) @ Rx(np.radians(25.0)), 25.0)):
        gn = np.array([0.0, 0.0, 1.0])                      # 机体系重力（水平器件）
        if tilt:
            gn = Rx(np.radians(tilt)) @ gn                  # 器件倾斜
        mh = Rt_.T @ bn                                     # 真实姿态下的测量 = 模型场
        mh = mh / np.linalg.norm(mh)
        # 用一个"估计姿态"= 真值，但 m^ 里带上误差：直接令 m^ = R_true^T b_n 而估计 R = I 不够
        # 这里反过来：估计姿态 = Rt_，测量 = R_true^T b_n 且 R_true = Rt_（无误差）时 r=0；
        # 为了看到非零 r，让测量绕竖直转一个角度。
        mh = (Rz(np.radians(6.0)) @ Rt_).T @ bn
        mh = mh / np.linalg.norm(mh)
        bb, e1, e2, r = resid(Rt_, bn, gn, mh)
        H = code_H(Rt_, bb, e1, e2)
        J = np.zeros((2, 3))
        eps = 1e-7
        for a in range(3):
            d = np.zeros(3); d[a] = eps
            R2 = rotvec(d) @ Rt_
            _, _, _, r2 = resid(R2, bn, gn, mh)
            J[:, a] = (r2 - r) / eps
        print(' %-18s r=%s' % (tag, np.round(np.degrees(r), 3)))
        print('   H0 %s   J0 %s   |H0+J0| %.2e' % (np.round(H[0], 3), np.round(J[0], 3),
                                                 np.abs(H[0] + J[0]).max()))
        print('   H1 %s   J1 %s   |H1+J1| %.2e' % (np.round(H[1], 3), np.round(J[1], 3),
                                                 np.abs(H[1] + J[1]).max()))
        # 固件约定 r = z - h（h = e_i.b^_b = 0），所以 H 必须 = dh/dx = -dr/dx = -J。
        # K = H^T (H H^T + R)^-1（P 取单位阵，只判符号），dx = +K r，
        # 真实残差变化 dr = J dx；r.dr < 0 才是负反馈（牵引有效）。
        Rm = np.eye(2) * (np.radians(0.9) ** 2)
        K = H.T @ np.linalg.inv(H @ H.T + Rm)
        for sc in (0.1, 0.5, 1.0):
            dr = J @ (sc * (K @ r))
            print('   Kx%.1f : |dr| %.4f°  r.dr %+.3e  %s'
                  % (sc, np.degrees(np.linalg.norm(dr)), r @ dr,
                     '负反馈 OK' if r @ dr < 0 else '*** 正反馈 发散 ***'))
    return 0


if __name__ == '__main__':
    main()
