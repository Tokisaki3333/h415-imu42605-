# -*- coding: utf-8 -*-
"""VER=103 真牵引补丁的验证（本机无 C 编译器，用数学等价 + C 结构检查替代）。

固件约定（与 ekf_update 的标准形式一致）：
    H = dh/dx,  r = z - h,  dx = K r,  S = H P H^T + R
    h = e_i·b^_b（i=1,2，e_i ⊥ b^_b），z = e_i·m^，
    于是 r = e_i·m^ - e_i·b^_b = e_i·m^（实现里写 e_i·(m^ - (m^·b^)b^)，数值同一）
    姿态扰动是**世界系**（ekf_inject: q <- dq (x) q）=> H = +e_i^T [b^_b]x R^T
    （体坐标系形式会是 -e_i^T[b^]x；两者差一个 R^T，弄错=方向全错）

A. 数学检查
   1) e1,e2 正交且 ⊥ b^_b
   2) H·b^_n ≡ 0（结构零空间：绕世界磁场轴无观测）
   3) H = -J（J = 实现残差对姿态的差分导数）—— 标准形式的符号自洽
   4) 姿态绕 b^_n 旋转 -> 残差不变
   5) 完美量测下 |r2v| ≈ 0
   6) 上报夹角 theta = atan2(|r|, m^·b^) == m^ 与 b^_b 的真实夹角
   7) 符号自检：姿态有小误差 d 时 r·(H d) < 0（更新朝收敛方向）

B. C 结构检查（括号平衡 / 变量声明 / #if-#endif / 禁磁已清 / 常量齐全）

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
    q = np.array([1.0, 0.5 * dx[0], 0.5 * dx[1], 0.5 * dx[2]])
    return q / np.linalg.norm(q)


def mag_block(q, mf, bn):
    R = quat_to_R(q)
    bb = R.T @ bn
    bb = bb / np.linalg.norm(bb)
    c1 = float(mf @ bb)
    pr = mf - c1 * bb                      # = P_perp m
    e1 = np.array([-bb[1], bb[0], 0.0])
    n1 = np.linalg.norm(e1)
    e1 = np.array([1.0, 0.0, 0.0]) if n1 < 1e-4 else e1 / n1
    e2 = np.cross(bb, e1)
    r2 = np.array([e1 @ pr, e2 @ pr])
    Sx = np.array([[0.0, -bb[2], bb[1]], [bb[2], 0.0, -bb[0]], [-bb[1], bb[0], 0.0]])
    H = np.array([(e1 @ Sx) @ R.T, (e2 @ Sx) @ R.T])      # 世界系扰动，正号，无 c1
    return bb, e1, e2, r2, H, pr


def main():
    rng = np.random.default_rng(3)
    ci = 1.0 / np.sqrt(1 + 2.08 ** 2)
    bn = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -2.08 * ci])
    fails, worst = [], dict(H=0., orth=0., fd=0., spin=0., perfect=0., theta=0., sign=0.)
    eps = 1e-6

    for _ in range(400):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        R = quat_to_R(q)
        mb = R.T @ bn + rng.normal(0, 0.01, 3)
        mb = mb / np.linalg.norm(mb)
        bb, e1, e2, r2, H, pr = mag_block(q, mb, bn)

        worst['H'] = max(worst['H'], np.abs(H @ bn).max())
        worst['orth'] = max(worst['orth'], abs(e1 @ e2), abs(e1 @ bb), abs(e2 @ bb),
                            abs(np.linalg.norm(e1) - 1), abs(np.linalg.norm(e2) - 1))
        # J = d(h)/dx，h = e_i·b^_b（基冻结）；标准形式要求 H = J
        J = np.zeros((2, 3))
        for j in range(3):
            dp = np.zeros(3); dp[j] = eps * 0.5
            dm = np.zeros(3); dm[j] = -eps * 0.5
            bp = mag_block(qmul(expq(dp), q), mb, bn)[0]     # 扰动后的 b^_b
            bm = mag_block(qmul(expq(dm), q), mb, bn)[0]
            J[:, j] = (np.array([e1 @ bp, e2 @ bp]) - np.array([e1 @ bm, e2 @ bm])) / eps
        worst['fd'] = max(worst['fd'], np.abs(J - H).max())
        r2s = mag_block(qmul(expq(eps * bn), q), mb, bn)[3]
        worst['spin'] = max(worst['spin'], np.abs(r2s - r2).max() / eps)
        worst['perfect'] = max(worst['perfect'], np.abs(mag_block(q, R.T @ bn, bn)[3]).max())
        bb2 = R.T @ bn / np.linalg.norm(R.T @ bn)
        c1t = float(mb @ bb2)
        th_true = np.degrees(np.arccos(np.clip(c1t, -1, 1)))
        th_rep = np.degrees(np.arctan2(np.linalg.norm(r2), c1t))
        worst['theta'] = max(worst['theta'], abs(th_rep - th_true))
        # 符号：用**无噪**量测（有噪时 1e-3 量级的 d 项会被 1e-2 的噪声淹没）
        mb0 = R.T @ bn
        mb0 = mb0 / np.linalg.norm(mb0)
        bb0, e10, e20, r20, H0, _ = mag_block(q, mb0, bn)
        d = rng.normal(size=3) * 1e-3
        rp = mag_block(qmul(expq(d), q), mb0, bn)[3]         # 估计比真值超前 d -> r ≈ -H d
        if not (float(rp @ (H0 @ d)) < 0.0):
            worst['sign'] = 1.0

    print('A. 数学检查（400 组随机姿态 + 1%% 量测噪声）')
    print('   1) e1,e2 正交且 ⊥ b^_b           最大偏差 %.2e' % worst['orth'])
    print('   2) |H·b^_n|                      %.2e   <- 结构零空间' % worst['H'])
    print('   3) H vs dh/dx（标准形式自洽）     最大偏差 %.2e' % worst['fd'])
    print('   4) 绕 b^_n 旋转 -> 残差变化       %.2e /rad' % worst['spin'])
    print('   5) 完美量测 |r2v|                %.2e' % worst['perfect'])
    print('   6) 上报 theta vs 真实夹角        最大偏差 %.2e deg' % worst['theta'])
    print('   7) 符号自检 r·(H d) < 0          %s' % ('通过' if worst['sign'] == 0 else '反号!'))
    for n, k, tol in (('A1 基正交', 'orth', 1e-12), ('A2 H·b^_n=0', 'H', 1e-12),
                      ('A3 H=dh/dx', 'fd', 1e-6), ('A4 绕 b^_n 不变', 'spin', 1e-6),
                      ('A5 完美量测', 'perfect', 1e-12), ('A6 theta 一致', 'theta', 1e-9),
                      ('A7 符号收敛', 'sign', 0.5)):
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
    pos_ok = 's_H[0][IX_Q + ax2] = e1[0]*w0' in src
    theta_ok = 'atan2f(sqrtf(r2v[0]*r2v[0] + r2v[1]*r2v[1]), c1)' in src and 'r2v[0] = e1[0]*mf[0]' in src
    tune_ok = all(k in tune for k in ('#define V5F_EKF_MAG_MODE          1u',
                                      '#define V5F_EKF_MAG_VEC_SIG_DEG   0.9f',
                                      '#define V5F_EKF_MAG_VEC_K_MAX     0.10f',
                                      '#define V5F_FW_VER        104u'))
    print()
    print('B. C 结构检查')
    for n, v in (('去注释后括号平衡', b_ok), ('新变量已声明', decl_ok),
                 ('M7 内 #if/#endif 配对', pair_ok), ('两模式分支都在', mode_ok),
                 ('运动禁磁痕迹已清', ban_gone), ('H 为正号', pos_ok),
                 ('新息报夹角 theta', theta_ok), ('tune 常量齐全', tune_ok)):
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
