# -*- coding: utf-8 -*-
"""地磁 360/全球面标定：用旧链姿态(att.q)当真值，做 9 参数"姿态辅助"拟合。

模型（与固件一致）：y = A * raw + C，要求 R_i * y_i ≈ w0（世界系定值，w0 未知）。
未知量 x = [vec(A)(9), C(3), w0(3)]，每样本 3 个齐次方程 => SVD 零空间；
标度用 median|y| = 1 固定（与固件 mag.ok / mag_norm 约定一致）。

数据充分性判据（本工具的核心，都是**尺度无关**的）：
  1) 帧自检：fw_tag(VER/CH/flags) + 逐帧校验和通过率；
  2) 秩判据：把设计矩阵**逐列归一化**后做 SVD，σ_i < 1e-4·σ0 的个数 = 零空间维数；
     期望只有 1 个（标度自由度）。>1 说明有参数组合没被激励，并打印它落在哪些参数上；
  3) 半分一致性：前后半各自独立拟合，比较 A/C 的差异与交叉残差（真正回答"这组数据能不能定出 A"）；
  4) 姿态激励范围（roll/pitch/yaw 跨度）——平面 360 的 pitch/roll 跨度为 0，一眼可见。

用法：
  python tools/calib/mag360_cal.py <log.txt> [--wmax 50] [--amag 0.02]
                                   [--static-only] [--out magcal.txt]
"""
import argparse
import os
import re
import sys

import numpy as np

try:                      # GBK 控制台编不出组合符(m-hat)等字符
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cols_162 as C   # noqa: E402

TUNE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..',
                    'V5F', 'User', 'inc', 'v5f_tune.h')

RANK_TOL = 1e-3        # σ_i/σ0 低于它视为零空间方向。实测标度自由度 ≈1e-4，
                       # 真实"最弱可辨识方向" ≈1.5e-3，中间有 ~15 倍空隙，1e-3 落在空隙里。
MIN_SAMPLES = 3000
MIN_EIG_COVER = 0.02   # 仅信息性：Σm̂m̂ᵀ 最小特征值（不要求各向同性）


def dip_deg(w0):
    v = w0 / (np.linalg.norm(w0) + 1e-30)
    return float(np.degrees(np.arcsin(np.clip(-v[2], -1, 1))))


# ------------------------- 基础工具 -------------------------

def quat_to_R(q):
    """att.q = (w,x,y,z)，返回体->世界的 3x3（与 hold_poll 里 mag.Bw 的算式一致）。"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def read_current_AC(path=TUNE):
    """从 v5f_tune.h 抓现用 V5F_MAG_A_INIT / V5F_MAG_C_INIT（GBK）。"""
    txt = open(path, 'rb').read().decode('gbk', errors='replace')
    ma = re.search(r'#define\s+V5F_MAG_A_INIT\s*\{(.*?)\}\s*\}', txt, re.S)
    mc = re.search(r'#define\s+V5F_MAG_C_INIT\s*\{([^}]*)\}', txt, re.S)
    if not (ma and mc):
        return None, None

    def nums(seg):
        out = []
        for t in re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?f', seg):
            out.append(float(t[:-1]))
        return out

    a = nums(ma.group(1))
    c = nums(mc.group(1))
    if len(a) < 9 or len(c) < 3:
        return None, None
    return np.array(a[:9]).reshape(3, 3), np.array(c[:3])


def euler_zyx(R):
    pitch = np.arcsin(np.clip(-R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# ------------------------- 采样 -------------------------

def select(a, wmax, amag_tol, static_only):
    c = C.CH_162
    q = a[:, c['att_q0']:c['att_q0'] + 4].astype(np.float64)
    qn = np.linalg.norm(q, axis=1)
    ok_q = np.abs(qn - 1.0) < 5e-3
    q = q / np.where(qn[:, None] > 0, qn[:, None], 1.0)

    gyro = a[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(np.float64)
    w = np.linalg.norm(gyro, axis=1)
    acc = a[:, c['accel_g0']:c['accel_g0'] + 3].astype(np.float64)
    am = np.linalg.norm(acc, axis=1)

    m = a[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(np.float64)
    ist = a[:, c['ist_cnt']].astype(np.int64)
    fresh = np.concatenate(([True], np.diff(ist) != 0))
    flags = a[:, c['flags']].astype(np.int64)
    valid = (flags & 0x04) != 0

    sel = ok_q & fresh & valid & (np.abs(am - 1.0) < amag_tol)
    sel &= (w < (2.0 if static_only else wmax))
    return sel, q, gyro, w, am, m


# ------------------------- 拟合 -------------------------

def design(q, gyro, m, w, w_soft=20.0, idx=None):
    if idx is None:
        idx = np.arange(len(m))
    N = len(idx)
    rows = np.zeros((3 * N, 15))
    for t, i in enumerate(idx):
        R = quat_to_R(q[i])
        for r in range(3):
            row = rows[3 * t + r]
            for j in range(3):
                for k in range(3):
                    row[j * 3 + k] = R[r, j] * m[i, k]
            row[9:12] = R[r, :]
            row[12 + r] = -1.0
    wts = 1.0 / (1.0 + (w[idx] / w_soft) ** 2)
    return rows * np.sqrt(np.repeat(wts, 3))[:, None]


def fit_raw(M):
    """返回未定标的 (A, C, w0, sv, Vt)。"""
    _, sv, Vt = np.linalg.svd(M, full_matrices=False)
    x = Vt[-1]
    return x[:9].reshape(3, 3).copy(), x[9:12].copy(), x[12:15].copy(), sv, Vt


def normalize_scale(A, Cv, w0, m):
    y = A @ m.T + Cv[:, None]
    s = 1.0 / np.median(np.linalg.norm(y, axis=0))
    A, Cv, w0 = A * s, Cv * s, w0 * s
    if w0[2] > 0:
        A, Cv, w0 = -A, -Cv, -w0
    return A, Cv, w0


def residual_deg(R, y, w0):
    u = R @ y
    n = np.linalg.norm(u)
    if n < 1e-12:
        return 180.0
    return float(np.degrees(np.arccos(np.clip(np.dot(u / n, w0 / (np.linalg.norm(w0) + 1e-30)), -1, 1))))


def eval_resid(A, Cv, q, m, idx, w0):
    e = np.empty(len(idx))
    for t, i in enumerate(idx):
        e[t] = residual_deg(quat_to_R(q[i]), A @ m[i] + Cv, w0)
    return e


def rank_diag(M):
    """秩诊断（**不归一化**：标度自由度在这里恰好是精确零空间）。

    返回 (n_null, 逐组能量, σ 谱)。n_null = σ_i < 1e-4·σ0 的个数，期望恰好 1（纯标度）。
    n_null > 1 时，把零空间里**扣掉标度方向**后的投影能量按参数分组打印出来，
    这才回答"哪一维没被激励"。实测：全球面(多轴整圈)=1；纯水平偏航 360 = 5
    （标度 + A 的 z 行 3 + C_z 1）。
    """
    _, sv, Vt = np.linalg.svd(M, full_matrices=False)
    null = np.flatnonzero(sv / sv[0] < RANK_TOL)
    g = Vt[-1]                                  # 标度方向 ≡ 解本身
    basis = Vt[null]
    basis = basis - np.outer(basis @ g, g)      # 扣掉标度分量
    keep = np.linalg.norm(basis, axis=1) > 1e-8
    basis = basis[keep]
    P = basis.T @ basis if len(basis) else np.zeros((15, 15))
    grp = {'A[0,:](x行)': np.trace(P[0:3, 0:3]), 'A[1,:](y行)': np.trace(P[3:6, 3:6]),
           'A[2,:](z行)': np.trace(P[6:9, 6:9]), 'C(偏置)': np.trace(P[9:12, 9:12]),
           'w0(世界场)': np.trace(P[12:15, 12:15])}
    return len(null), grp, sv


def halfsplit(q, gyro, m, w, idx, w_soft, w0):
    """前后半独立拟合 + 交叉残差：直接回答"这组数据能不能定出 A"。"""
    h = len(idx) // 2
    i1, i2 = idx[:h], idx[h:]
    if len(i1) < 200 or len(i2) < 200:
        return None
    A1, C1, w1, _, _ = fit_raw(design(q, gyro, m, w, w_soft, i1))
    A2, C2, w2, _, _ = fit_raw(design(q, gyro, m, w, w_soft, i2))
    A1, C1, w1 = normalize_scale(A1, C1, w1, m[i1])
    A2, C2, w2 = normalize_scale(A2, C2, w2, m[i2])
    relA = float(np.linalg.norm(A1 - A2) / (0.5 * (np.linalg.norm(A1) + np.linalg.norm(A2))))
    e12 = eval_resid(A1, C1, q, m, i2, w0)
    e21 = eval_resid(A2, C2, q, m, i1, w0)
    return relA, float(np.percentile(np.concatenate([e12, e21]), 90))


# ------------------------- 输出 -------------------------

def c_block(A, Cv):
    f = lambda v: '%+.8e' % v
    return '\n'.join([
        '#define V5F_MAG_A_INIT  { { %sf, %sf, %sf }, \\' % (f(A[0, 0]), f(A[0, 1]), f(A[0, 2])),
        '                          { %sf, %sf, %sf }, \\' % (f(A[1, 0]), f(A[1, 1]), f(A[1, 2])),
        '                          { %sf, %sf, %sf } }' % (f(A[2, 0]), f(A[2, 1]), f(A[2, 2])),
        '#define V5F_MAG_C_INIT  { %sf, %sf, %sf }' % (f(Cv[0]), f(Cv[1]), f(Cv[2])),
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--wmax', type=float, default=1000.0)
    ap.add_argument('--amag', type=float, default=1.0)   # 不要用它判覆盖：姿态辅助标定不需要重力有效
    ap.add_argument('--static-only', action='store_true')
    ap.add_argument('--soft-w', type=float, default=20.0)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    C.selfcheck(verbose=False)
    a, info = C.load_frames(args.log)
    rep = C.frame_report(a)
    lines = []
    P = lambda s='': (print(s), lines.append(s))

    P('==== 帧自检 ====')
    P('  格式 %s  帧数 %d  相位 %s' % (info.get('format'), rep['frames'], info.get('phase', '-')))
    P('  fw_tag %.0f -> VER=%d CH=%d flags=%d (编译期望 %.0f)'
      % (rep['fw_tag'], rep['ver'], rep['ch'], rep['flags'], rep['compiled_tag']))
    P('  校验和通过率 %.2f%%   |att.q|-1 max %.2e   |a| 中位 %.4f g'
      % (100 * rep['checksum_ok'], rep['att_q_norm_err'], rep['accel_abs_median']))
    P('  地磁原始 LSB 范围 %s (中位|m| %.1f)'
      % (['%.0f' % v for v in rep['mag_lsb_range']], rep['mag_lsb_abs_median']))
    tag_ok = (rep['ch'] == C.NCH and rep['ver'] >= 101) and rep['checksum_ok'] > 0.99
    if abs(rep['fw_tag'] - C.FW_TAG_EXPECT) > 0.5:
        P('  注：VER=%d ≠ 编译时假设的 %d（标定本身不影响判据，仅提示版本已 bump）'
          % (rep['ver'], C.VER_EXPECT))

    sel, q, gyro, w, am, m = select(a, args.wmax, args.amag, args.static_only)
    idx = np.flatnonzero(sel)
    P('')
    P('==== 选样 ====')
    P('  通过 %d / %d (%.1f%%)   |w| 中位 %.1f dps'
      % (len(idx), len(a), 100.0 * len(idx) / max(len(a), 1), float(np.median(w))))
    if len(idx) < 200:
        P('  !! 有效样本太少，先补录')
        return 1
    R_all = np.stack([quat_to_R(q[i]) for i in idx])
    rpy = np.array([euler_zyx(R) for R in R_all])
    yaw_un = np.unwrap(np.radians(rpy[:, 2])) * 57.29578
    P('  姿态激励: roll %+.0f..%+.0f (%.0f)  pitch %+.0f..%+.0f (%.0f)  yaw 累计 %.0f deg'
      % (rpy[:, 0].min(), rpy[:, 0].max(), np.ptp(rpy[:, 0]),
         rpy[:, 1].min(), rpy[:, 1].max(), np.ptp(rpy[:, 1]), float(np.ptp(yaw_un))))
    md = m[idx] / np.maximum(np.linalg.norm(m[idx], axis=1, keepdims=True), 1e-9)
    ev_m = np.linalg.eigvalsh(md.T @ md / len(md))
    P('  信息性: Σm̂m̂ᵀ 特征值 %s（不要求各向同性，只反映方向分布）' % np.round(ev_m, 3))

    M = design(q, gyro, m, w, args.soft_w, idx)
    A, Cv, w0, sv, Vt = fit_raw(M)
    A, Cv, w0 = normalize_scale(A, Cv, w0, m[idx])

    P('')
    P('==== 秩判据（未归一化：标度自由度恰好是精确零空间）====')
    n_null, grp, svn = rank_diag(M)
    P('  σ/σmax = %s' % np.round(svn / svn[0], 5))
    P('  零空间维数 = %d  (期望 1 = 纯标度自由度；>1 即有参数组合没被激励)' % n_null)
    P('  次小 σ[-2]/σmax = %.4f（可辨识裕度）' % (svn[-2] / svn[0]))
    if n_null > 1:
        P('  ★ 未激励方向（扣掉标度后）的能量分布: ' + '  '.join(
            '%s %.2f' % (k, v) for k, v in sorted(grp.items(), key=lambda kv: -kv[1]) if v > 0.05))
        P('    注：A/C/w0 三者强耦合，能量常同时落在 C 与 w0 上；只要维数 >1 就说明竖直')
        P('    （或某个出平面方向）没被激励 —— 平面 360 转再多圈也补不上，必须加俯仰/翻滚/翻转')

    hs = halfsplit(q, gyro, m, w, idx, args.soft_w, w0)
    if hs:
        P('  半分一致性: ‖A1-A2‖/‖A‖ = %.3f  交叉残差 p90 = %.3f deg' % hs)

    P('')
    P('==== 拟合结果 ====')
    P('  w0(世界系场, nav x右 y前 z上) = %s  |w0| = %.4f' % (np.round(w0, 4), np.linalg.norm(w0)))
    P('  参考模型(decl=-7.53deg, tan(dip)=2.08) = [-0.0567  0.4308 -0.9008]')
    det = abs(np.linalg.det(A))
    P('  A 等轴标度 %.5f  det %.3e' % (np.cbrt(det), np.linalg.det(A)))

    A0, C0 = read_current_AC()
    if A0 is not None:
        e0 = eval_resid(A0, C0, q, m, idx, w0)
        P('  [现用标定] 方向残差 p50 %.3f p90 %.3f max %.3f deg'
          % (np.percentile(e0, 50), np.percentile(e0, 90), e0.max()))
    e1 = eval_resid(A, Cv, q, m, idx, w0)
    yn = np.linalg.norm(A @ m[idx].T + Cv[:, None], axis=0)
    P('  [本次拟合] 方向残差 p50 %.3f p90 %.3f max %.3f deg   |y|-1 中位 %.3f%%'
      % (np.percentile(e1, 50), np.percentile(e1, 90), e1.max(),
         100 * np.median(np.abs(yn - 1))))

    P('')
    P('==== 结论 ====')
    good = True
    if not tag_ok:
        P('  !! 帧指纹/校验和不对：不是 VER=%d 的 162 通道调试固件，或定相位失败' % C.VER_EXPECT)
        good = False
    if len(idx) < MIN_SAMPLES:
        P('  !! 有效样本 %d < %d' % (len(idx), MIN_SAMPLES))
        good = False
    if n_null > 1:
        P('  !! 有 %d 个未被激励的参数方向（见上），需补俯仰/翻滚/翻转（全球面）' % (n_null - 1))
        good = False
    if hs and hs[0] > 0.15:
        P('  !! 半分一致性差（%.3f）：数据不足以唯一确定 A' % hs[0])
        good = False
    if np.ptp(rpy[:, 1]) < 25.0 and np.ptp(rpy[:, 0]) < 25.0:
        P('  !! 姿态激励不足：roll/pitch 跨度都 < 25 deg（只转了平面 360）')
        good = False
    if good:
        P('  OK: 数据足以唯一确定 9 参数，可以写回固件')

    P('')
    P('==== 写回固件 ====')
    for l in c_block(A, Cv).split('\n'):
        P('  ' + l)

    out = args.out or (os.path.splitext(args.log)[0] + '.magcal.txt')
    open(out, 'w', encoding='utf-8').write('\n'.join(lines) + '\n')
    print('\n报告已写入 %s' % out)
    return 0 if good else 2


if __name__ == '__main__':
    sys.exit(main())
