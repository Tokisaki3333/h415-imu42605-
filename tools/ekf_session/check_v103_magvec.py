# -*- coding: utf-8 -*-
"""VER=103 真牵引补丁的验证（本机无 C 编译器，用数学等价 + C 结构检查替代）。

A. 数学等价性：把补丁里那段 C 逐行重写成 numpy，验证
   1) e1,e2 正交且 ⊥ b^_b
   2) H·b^_n ≡ 0 —— 结构零空间。固件用**世界系扰动**（ekf_inject: q <- dq (x) q），
      故 H = -e_i^T [b^_b]x R^T，零空间方向是世界系磁场方向 b^_n（绕世界磁场轴无观测）
   3) H 与 r2v(q) 的中心差分 Jacobian 一致（同一世界系扰动约定）
   4) 姿态绕 b^_n 旋转 -> 残差不变（不可观测方向）
   5) 量测与预测一致时 r2v ≈ 0

B. C 结构检查：去注释/字符串后括号平衡、新变量已声明、M7 内 #if/#endif 配对、
   两个模式分支都在、运动禁磁痕迹已清、H 含 R^T、tune 常量齐全。

用法: python tools/ekf_session/check_v103_magvec.py
"""
import os
import re
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')


def quat_to_R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    q = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                  w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                  w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                  w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    return q / np.linalg.norm(q)


def expq(dx):
    """Exp(dx)：ekf_inject 里 dq=[1, dx/2] 后归一化。"""
    q = np.array([1.0, 0.5 * dx[0], 0.5 * dx[1], 0.5 * dx[2]])
    return q / np.linalg.norm(q)


def mag_block(q, mf, bn):
    """补丁里那段 C 的逐行等价实现（世界系扰动约定）。"""
    R = quat_to_R(q)
    bb = R.T @ bn
    bb = bb / np.linalg.norm(bb)
    c1 = float(mf @ bb)
    pr = mf - c1 * bb                       # P_⊥ m^（⊥b^ 分量）；等价于 (I-b^b^T)(m^-b^)
    e1 = np.array([-bb[1], bb[0], 0.0])
    n1 = np.linalg.norm(e1)
    e1 = np.array([1.0, 0.0, 0.0]) if n1 < 1e-4 else e1 / n1
    e2 = np.cross(bb, e1)
    r2 = np.array([e1 @ pr, e2 @ pr])
    Sx = np.array([[0.0, -bb[2], bb[1]], [bb[2], 0.0, -bb[0]], [-bb[1], bb[0], 0.0]])
    H = np.array([-c1*(e1 @ Sx) @ R.T, -c1*(e2 @ Sx) @ R.T])   # 世界系扰动 + c1
    return bb, e1, e2, r2, H, pr


def main():
    rng = np.random.default_rng(3)
    ci = 1.0 / np.sqrt(1 + 2.08 ** 2)
    bn = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -2.08 * ci])
    fails = []
    worst = dict(H=0.0, orth=0.0, fd=0.0, spin=0.0, perfect=0.0)
    eps = 1e-6

    for _ in range(400):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = quat_to_R(q)
        mb = R.T @ bn + rng.normal(0, 0.01, 3)          # 实测（1% 噪声）
        mb = mb / np.linalg.norm(mb)
        bb, e1, e2, r2, H, pr = mag_block(q, mb, bn)

        worst['H'] = max(worst['H'], np.abs(H @ bn).max())
        worst['orth'] = max(worst['orth'], abs(e1 @ e2), abs(e1 @ bb), abs(e2 @ bb),
                            abs(np.linalg.norm(e1) - 1), abs(np.linalg.norm(e2) - 1))
        J = np.zeros((2, 3))
        for j in range(3):
            dxp = np.zeros(3); dxp[j] = eps * 0.5
            dxm = np.zeros(3); dxm[j] = -eps * 0.5
            # 与固件一致：基 e1,e2 在当前估计处**冻结**，只让预测场随姿态变
            prp = mag_block(qmul(expq(dxp), q), mb, bn)[5]
            prm = mag_block(qmul(expq(dxm), q), mb, bn)[5]
            rp = np.array([e1 @ prp, e2 @ prp])
            rm = np.array([e1 @ prm, e2 @ prm])
            J[:, j] = (rp - rm) / eps
        worst['fd'] = max(worst['fd'], np.abs(J - H).max())
        dxs = eps * bn
        r2s = mag_block(qmul(expq(dxs), q), mb, bn)[3]
        worst['spin'] = max(worst['spin'], np.abs(r2s - r2).max() / eps)
        worst['perfect'] = max(worst['perfect'], np.abs(mag_block(q, R.T @ bn, bn)[3]).max())

    print('A. 数学等价性（400 组随机姿态 + 1%% 量测噪声）')
    print('   1) e1,e2 正交且 ⊥ b^_b          最大偏差 %.2e' % worst['orth'])
    print('   2) |H·b^_n|                      %.2e   <- 结构零空间（绕世界磁场轴无观测）' % worst['H'])
    print('   3) H vs 中心差分 Jacobian        最大偏差 %.2e   <- 约定/符号与 ekf_inject 一致' % worst['fd'])
    print('   4) 姿态绕 b^_n 转 -> 残差变化     %.2e /rad' % worst['spin'])
    print('   5) 完美量测下 |r2v|              %.2e' % worst['perfect'])
    for n, k, tol in (('A1 基正交', 'orth', 1e-12), ('A2 H·b^_n=0', 'H', 1e-12),
                      ('A3 H=Jacobian', 'fd', 1e-6), ('A4 绕 b^_n 不变', 'spin', 1e-6),
                      ('A5 完美量测', 'perfect', 1e-12)):
        if not (worst[k] < tol):
            fails.append('%s 超差 %.3e > %.1e' % (n, worst[k], tol))

    src = open(EKF, 'rb').read().decode('gbk', errors='replace')
    tune = open(TUNE, 'rb').read().decode('gbk', errors='replace')

    def strip_c(t):
        t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
        t = re.sub(r'//[^\n]*', ' ', t)
        t = re.sub(r'"(\\.|[^"\\])*"', '""', t)
        t = re.sub(r"'(\\.|[^'\\])*'", "''", t)
        return t

    code = strip_c(src)
    b_ok = code.count('{') == code.count('}') and code.count('(') == code.count(')')
    decl_ok = all(('float ' + v) in src or (', ' + v) in src for v in
                  ('bb[3]', 'e1[3]', 'e2[3]', 'r2v[2]', 'RRv[4]'))
    i0 = src.find('static void ekf_m7_mag')
    i1 = src.find('static void ekf_m1m2_gps')
    seg = strip_c(src[i0:i1])
    pair_ok = seg.count('#if') == seg.count('#endif')
    mode_ok = '#if (V5F_EKF_MAG_MODE == 0u)' in src and '#if (V5F_EKF_MAG_MODE != 0u)' in src
    ban_gone = 'V5F_EKF_MAG_GT_HOLD_S' not in code and 's_mag_tilt_bad_s' not in code
    r_ok = 'Rt[ax2][0]' in src and 's_H[0][IX_Q + ax2]' in src
    tune_ok = all(k in tune for k in ('#define V5F_EKF_MAG_MODE          1u',
                                      '#define V5F_EKF_MAG_VEC_SIG_DEG   0.9f',
                                      '#define V5F_EKF_MAG_VEC_K_MAX     0.10f',
                                      '#define V5F_FW_VER        103u'))
    print()
    print('B. C 结构检查')
    for n, v in (('去注释后括号平衡', b_ok), ('新变量已声明', decl_ok),
                 ('M7 内 #if/#endif 配对', pair_ok), ('两模式分支都在', mode_ok),
                 ('运动禁磁痕迹已清', ban_gone), ('H 含 R^T（世界系）', r_ok),
                 ('tune 常量齐全', tune_ok)):
        print('   %-22s %s' % (n, 'OK' if v else 'FAIL'))
        if not v:
            fails.append(n)

    print()
    if fails:
        print('CHECK FAIL:')
        for f in fails:
            print('  -', f)
        return 1
    print('CHECK OK（数学等价 + C 结构；实机编译仍需你那边做）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
