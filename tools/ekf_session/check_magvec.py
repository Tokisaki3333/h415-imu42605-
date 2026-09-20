# -*- coding: utf-8 -*-
"""地磁真牵引（VER=107）验证：数学等价 + C 结构（本机无 C 编译器时用）。

固件约定（ekf_update 标准形式）：H = dh/dx, r = z - h, dx = K r, S = H P H^T + R
世界系扰动（ekf_inject: q <- dq (x) q）=> H_i = +e_i^T [b^_b]x R^T，H_i·b^_n ≡ 0

VER=107 的两点：
  * 面内基的**竖直基准是重力矢量** gn（加计，体坐标），不假设夹具水平：
      e1 = normalize(gn × b^_b)   绕重力转 = 航向；其方向与磁倾角模型无关
      e2 = b^_b × e1              携带倾斜信息
  * 倾斜行 R 按**置信度**放大：ddip = |实测倾角 - 模型倾角|，
      c = 1/(1+(ddip/REF)^2)，sigma_tilt = sigma_yaw / c；s_mag_rs 上报其平方

用法: python tools/ekf_session/check_magvec.py
"""
import glob
import os
import re
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
VER = 107
DIP_TAN = 1.70          # v5f_tune.h 的值
SIG_YAW = 0.9           # V5F_EKF_MAG_VEC_SIG_DEG
DDIP_REF = 1.0          # V5F_EKF_MAG_TILT_DDIP_REF_DEG
CONF_MIN = 0.02         # V5F_EKF_MAG_TILT_CONF_MIN


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


def block(q, mf, bn, gn):
    """补丁里那段 C 的逐行等价实现（返回 bb,e1,e2,r2,H,ddip,conf,sig_tilt,rimg）。"""
    R = quat_to_R(q)
    bb = R.T @ bn
    bb = bb / np.linalg.norm(bb)
    c1 = float(mf @ bb)
    e1 = np.cross(gn, bb)
    n1 = np.linalg.norm(e1)
    if n1 < 1e-4:
        e1 = np.array([-bb[1], bb[0], 0.0])
        n1 = np.linalg.norm(e1)
        e1 = np.array([1.0, 0.0, 0.0]) if n1 < 1e-6 else e1 / n1
    else:
        e1 = e1 / n1
    e2 = np.cross(bb, e1)
    r2 = np.array([e1 @ mf, e2 @ mf])
    Sx = np.array([[0.0, -bb[2], bb[1]], [bb[2], 0.0, -bb[0]], [-bb[1], bb[0], 0.0]])
    H = np.array([(e1 @ Sx) @ R.T, (e2 @ Sx) @ R.T])
    cm = float(np.clip(mf @ gn, -1, 1))
    dip_m = np.degrees(np.arccos(cm)) - 90.0    # 磁场朝下: angle = 90 + I
    ddip = abs(dip_m - np.degrees(np.arctan(DIP_TAN)))
    conf = max(1.0 / (1.0 + (ddip / DDIP_REF) ** 2), CONF_MIN)
    st = SIG_YAW / conf
    return bb, e1, e2, r2, H, ddip, conf, st, (st / SIG_YAW) ** 2


def main():
    rng = np.random.default_rng(3)
    ci = 1.0 / np.sqrt(1 + DIP_TAN ** 2)
    bn = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -DIP_TAN * ci])
    fails, w = [], dict(H=0., orth=0., fd=0., spin=0., perfect=0., theta=0., sign=0., dipfree=0.)
    eps = 1e-6

    for _ in range(400):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        R = quat_to_R(q)
        mb = R.T @ bn + rng.normal(0, 0.01, 3)
        mb = mb / np.linalg.norm(mb)
        gn = R.T @ np.array([0.0, 0.0, 1.0])          # 体坐标里的重力方向（真值）
        bb, e1, e2, r2, H, ddip, conf, st, rimg = block(q, mb, bn, gn)

        w['H'] = max(w['H'], np.abs(H @ bn).max())
        w['orth'] = max(w['orth'], abs(e1 @ e2), abs(e1 @ bb), abs(e2 @ bb),
                        abs(np.linalg.norm(e1) - 1), abs(np.linalg.norm(e2) - 1))
        J = np.zeros((2, 3))
        for j in range(3):
            dp = np.zeros(3); dp[j] = eps * 0.5
            dm = np.zeros(3); dm[j] = -eps * 0.5
            bp = block(qmul(expq(dp), q), mb, bn, gn)[0]
            bm = block(qmul(expq(dm), q), mb, bn, gn)[0]
            J[:, j] = (np.array([e1 @ bp, e2 @ bp]) - np.array([e1 @ bm, e2 @ bm])) / eps
        w['fd'] = max(w['fd'], np.abs(J - H).max())
        w['spin'] = max(w['spin'], np.abs(block(qmul(expq(eps * bn), q), mb, bn, gn)[3] - r2).max() / eps)
        w['perfect'] = max(w['perfect'], np.abs(block(q, R.T @ bn, bn, gn)[3]).max())
        c1t = float(mb @ bb)
        w['theta'] = max(w['theta'], abs(np.degrees(np.arctan2(np.linalg.norm(r2), c1t))
                                         - np.degrees(np.arccos(np.clip(c1t, -1, 1)))))
        mb0 = R.T @ bn; mb0 = mb0 / np.linalg.norm(mb0)
        _, _, _, _, H0, _, _, _, _ = block(q, mb0, bn, gn)
        d = rng.normal(size=3) * 1e-3
        if not (float(block(qmul(expq(d), q), mb0, bn, gn)[3] @ (H0 @ d)) < 0.0):
            w['sign'] = 1.0
        # 偏航行对 dip 模型误差不敏感：把**模型场**在 gn-bb 平面内转 ±5 度
        for sgn in (1.0, -1.0):
            k = np.cross(gn, bb); k = k / (np.linalg.norm(k) + 1e-12)
            k = R @ k                     # 体坐标轴换算到世界系（模型场是世界系向量）
            th = np.radians(5.0) * sgn
            bn_r = bn * np.cos(th) + np.cross(k, bn) * np.sin(th) + k * (k @ bn) * (1 - np.cos(th))
            b2, e1b, _, r2b, _, _, _, _, _ = block(q, mb0, bn_r, gn)
            _, e1a, _, r2a, _, _, _, _, _ = block(q, mb0, bn, gn)
            w['dipfree'] = max(w['dipfree'], abs(r2b[0] - r2a[0]), np.abs(e1b - e1a).max())

    print('A. 数学检查（400 组随机姿态 + 1%% 量测噪声）')
    print('   1) e1,e2 正交且 ⊥ b^_b            %.2e' % w['orth'])
    print('   2) |H·b^_n|                      %.2e   <- 结构零空间' % w['H'])
    print('   3) H vs dh/dx                     %.2e' % w['fd'])
    print('   4) 绕 b^_n 旋转 -> 残差变化       %.2e /rad' % w['spin'])
    print('   5) 完美量测 |r2v|                %.2e' % w['perfect'])
    print('   6) theta 上报一致                 %.2e deg' % w['theta'])
    print('   7) 符号自检 r·(H d) < 0          %s' % ('通过' if w['sign'] == 0 else '反号!'))
    print('   8) 偏航行对 dip 误差不敏感        %.2e （模型场在竖直面内转 ±5 度）' % w['dipfree'])
    q0 = np.array([1.0, 0, 0, 0]); gn0 = np.array([0.0, 0.0, 1.0])

    def conf_for(extra_deg):
        dip = np.degrees(np.arctan(DIP_TAN)) + extra_deg
        mf = np.array([np.cos(np.radians(dip)), 0.0, -np.sin(np.radians(dip))])
        return block(q0, mf, bn, gn0)[6]

    c0, c1_ = conf_for(0.0), conf_for(2.3)
    print('   9) 置信度: ddip=0 -> c=%.3f（全权） | ddip=2.3 -> c=%.3f（方差降 %.0f 倍）'
          % (c0, c1_, 1.0 / c1_ ** 2))
    for n, k, tol in (('A1 基正交', 'orth', 1e-12), ('A2 H·b^_n=0', 'H', 1e-12),
                      ('A3 H=dh/dx', 'fd', 1e-6), ('A4 绕 b^_n 不变', 'spin', 1e-6),
                      ('A5 完美量测', 'perfect', 1e-12), ('A6 theta 一致', 'theta', 1e-9),
                      ('A7 符号收敛', 'sign', 0.5), ('A8 dip 无关', 'dipfree', 1e-9)):
        if not (w[k] < tol):
            fails.append('%s 超差 %.3e > %.1e' % (n, w[k], tol))
    if not (c0 >= 0.999):
        fails.append('A9 模型吻合时应全权 c=%.3f' % c0)
    if not (c1_ < 0.25):
        fails.append('A9 ddip=2.3 时降权不足 c=%.3f' % c1_)

    src = open(EKF, 'rb').read().decode('gbk', errors='replace')
    tune = open(TUNE, 'rb').read().decode('gbk', errors='replace')

    def strip_c(t):
        t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
        t = re.sub(r'//[^\n]*', ' ', t)
        t = re.sub(r'"(\\.|[^"\\])*"', '""', t)
        t = re.sub(r"'(\\.|[^'\\])*'", "''", t)
        return t

    code = strip_c(src)
    i0, i1 = src.find('static void ekf_m7_mag'), src.find('static void ekf_m1m2_gps')
    m7 = strip_c(src[i0:i1])
    b_ok = code.count('{') == code.count('}') and code.count('(') == code.count(')')
    cmt_ok = src.count('/*') == src.count('*/') and tune.count('/*') == tune.count('*/')
    no_cond = ('V5F_EKF_MAG_MODE' not in src) and ('#if' not in m7)
    no_legacy = all(k not in code for k in ('V5F_EKF_MAG_K_MAX', 'V5F_EKF_MAG_GT_HOLD_S',
                                            's_tilt_sig_deg', '0x0100u', 'sig2'))
    gn_ok = ('gn_b[1]*bb[2] - gn_b[2]*bb[1]' in m7) and ('h->imu.accel_g' in m7)
    conf_ok = ('V5F_EKF_MAG_TILT_DDIP_REF_DEG' in m7 and 'V5F_EKF_MAG_VEC_SIG_DEG / conf' in m7)
    vec_ok = ('ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG' in m7
              and m7.count('V5F_EKF_NIS_MAX_MAG') == 1)
    hpos_ok = 's_H[0][IX_Q + ax2] = e1[0]*w0' in m7
    tune_ok = all(k in tune for k in ('#define V5F_EKF_MAG_VEC_SIG_DEG   0.9f',
                                      '#define V5F_EKF_MAG_VEC_K_MAX     0.10f',
                                      '#define V5F_EKF_MAG_TILT_DDIP_REF_DEG  1.0f',
                                      '#define V5F_EKF_MAG_TILT_CONF_MIN      0.02f',
                                      '#define V5F_EKF_DIP_TAN          1.70f',
                                      '#define V5F_FW_VER        %du' % VER))
    hdr = ''.join(open(f, 'rb').read().decode('gbk', errors='replace')
                  for f in glob.glob(os.path.join(ROOT, 'V5F/User/inc/*.h'))
                  + glob.glob(os.path.join(ROOT, 'Common/Common/*.h')))
    defined = set(re.findall(r'#define\s+(V5F_[A-Z0-9_]+)', hdr))
    undef = sorted({n for n in re.findall(r'\b(V5F_[A-Z0-9_]+)\b', code) if n not in defined})
    print()
    print('B. C 结构检查（VER=%d：重力基准 + 置信度倾斜权）' % VER)
    for n, v in (('去注释后括号平衡', b_ok), ('注释配平(两文件)', cmt_ok),
                 ('M7 无条件编译/无 MODE', no_cond), ('旧模式零残留', no_legacy),
                 ('基用重力 gn', gn_ok), ('置信度加权在', conf_ok),
                 ('向量量测更新唯一', vec_ok), ('H 为正号', hpos_ok),
                 ('tune 常量齐全', tune_ok), ('无未定义宏', not undef)):
        print('   %-24s %s%s' % (n, 'OK' if v else 'FAIL',
                                 '' if (v or n != '无未定义宏') else ' %s' % undef[:4]))
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
