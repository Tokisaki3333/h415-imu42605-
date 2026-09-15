# -*- coding: utf-8 -*-
"""
VER=75：偏航方差下限（治"环路增益被抽干"的真正方法）

VER=74 录制实测暴露的机理（36~40 s 那段静置最干净）：
  t=36.00  |psi|=21.1  p_yy=0.000000  sigma_yaw=0.00  dqz=+0.074
  t=36.75  |psi|=41.8  p_yy=0.000000  sigma_yaw=0.00  dqz=-0.0004
  t=37.25  |psi|=49.2  p_yy=0.000000  sigma_yaw=0.00  dqz=-0.034
  t=37.50  |psi|=52.1  p_yy=0.000209  sigma_yaw=0.83  dqz=+0.010
  t=38.00  |psi|=44.2  p_yy=0.021535  sigma_yaw=8.41  dqz=-0.241
  t=38.50  |psi|=21.1  p_yy=0.034680  sigma_yaw=10.67 dqz=-0.204
  t=39.25  |psi|= 4.78 p_yy=0.036381  sigma_yaw=10.93 dqz=-0.053
  t=40.00  |psi|= 0.78 p_yy=0.035745  sigma_yaw=10.83 dqz=-0.009
即：P[8][8] 被削到 P_FLOOR=1e-12（好帧 min 恰为 1.000e-12，从未为负），
K = P*h/(h^2 P + R) = 0 于是环路**完全没有力气**，静置中残差反而从 21 度涨到 49 度；
之后 Q 用约 1.5 s 把 P 养回 0.036（sigma 10.9 度），环路才恢复，再用 1.5 s 拉回 0.78 度。
同样地曲线运动段（12~20 s）实测 dqz 只有 k_cap 允许值的 1/5~1/12
（16 s：实测 0.145 度 vs 上限 1.712 度），那段 87~107 度也是同一个原因。

为什么上一版用 Q 下限补是错的：削减是 ∝ P 的指数塌陷，而 Q 只贡献增长的约 5%，
托不住；而且 Q 是"加进去"的，塌陷快于补充时结果仍然是塌到地板。

正解：直接给 P[8][8] 一个下限。判据是增益是否还在 k_cap 上限：
  K = P*h/(h^2 P + R)（h=|H_yaw|=0.4334，R=(0.5 度)^2=7.6154e-05）
  P=1e-12 -> K=0.0000（死） ; P=7.615e-05 -> K=0.3649（满） ; 阈值是 P>1.8e-05
取 V5F_EKF_YAW_P_MIN = V5F_EKF_MAG_SIG_RAD^2 = (0.5 度)^2：
  "滤波器对偏航的不确定度永不小于磁力计自身的测量噪声" —— 这是测量论上必须成立的，
  不可能靠融合把航向做得比测量本身更准。
效果（49 度误差）：单步 0.05*|r|=1.030 度，193.8 Hz -> 200 度/秒，
从 49 度收回约 0.25 s（实测现在是 ~12 度/秒、4 s）。
"""
import os, shutil

ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')

def load(p):
    with open(p, 'rb') as f:
        return f.read().decode(ENC)

def save(p, text, tag):
    data = text.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f:
        f.write(data)
    with open(p, 'rb') as f:
        assert f.read().decode(ENC) == text, '回读失败 ' + p
    return len(data)

def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '[%s] 锚点命中 %d 次' % (what, n)
    return t.replace(old, new, 1)

# ---------------- 1. v5f_tune.h ----------------
t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        74u', '#define V5F_FW_VER        75u', 'VER')
t = sub1(t,
    '#define V5F_EKF_MAG_SIG_RAD      (V5F_MAG_YAW_R_DEG * 0.017453292f)  /* 0.7 度 */',
    '#define V5F_EKF_MAG_SIG_RAD      (V5F_MAG_YAW_R_DEG * 0.017453292f)  /* 0.5 度 */\n'
    '/* ★VER=75 偏航方差上限下限（rad^2）—— 治"环路增益被抽干"的正解。\n'
    ' * M7 的 k_cap 限幅更新对 P[8][8] 的削减 ∝ P（S 约等于 |H|^2 P），是指数塌陷；\n'
    ' * 实测 VER=74：P[8][8] 被削到 P_FLOOR=1e-12（好帧 min 恰为 1.000e-12），\n'
    ' * K = P*h/(h^2 P + R) = 0 -> 环路完全没有力气，静置中残差反而从 21 度涨到 49 度，\n'
    ' * 直到 Q 把它养回 0.036（约 1.5 s）才恢复。曲线段 12~20 s 的 87~107 度同因。\n'
    ' * 用 Q 下限补是错的（Q 只占增长的约 5%，托不住）；直接给方差一个下限才对。\n'
    ' * 判据：K 是否仍在上限。h=0.4334, R=(0.5 度)^2=7.6154e-05 时\n'
    ' *   P=1e-12 -> K=0.000（死）; P=7.615e-05 -> K=0.365（满）; 阈值 P>1.8e-05。\n'
    ' * 取 (0.5 度)^2 = 磁力计自身的测量噪声：滤波器对偏航的不确定度不可能比测量更小。 */\n'
    '#define V5F_EKF_YAW_P_MIN        (V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD)',
    'YAW_P_MIN')
n1 = save(TUNE, t, 'v75')

# ---------------- 2. proc_ekf.c ----------------
e = load(EKF)
e = sub1(e,
    '            float a = 0.5f * (s_Pn[i][j] + s_Pn[j][i]);\n'
    '            if (i == j && a < V5F_EKF_P_FLOOR) a = V5F_EKF_P_FLOOR;\n',
    '            float a = 0.5f * (s_Pn[i][j] + s_Pn[j][i]);\n'
    '            if (i == j && a < V5F_EKF_P_FLOOR) a = V5F_EKF_P_FLOOR;\n'
    '            /* ★VER=75 偏航方差下限：M7 的 k_cap 限幅更新对 P[8][8] 的削减 ∝ P，\n'
    '             * 是指数塌陷，实测会被削到 1e-12 使偏航增益归零、环路彻底失去力气\n'
    '             * （VER=74 录制：静置中残差从 21 度涨到 49 度，P 养回来才拉回）。\n'
    '             * 下限取磁力计自身测量噪声的平方，保证 K 恒在 k_cap 上限。 */\n'
    '            if (i == 8u && j == 8u && a < V5F_EKF_YAW_P_MIN) a = V5F_EKF_YAW_P_MIN;\n',
    'floor')
n2 = save(EKF, e, 'v75')

# ---------------- 3. 自检 ----------------
t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        75u' in t2
assert 'V5F_EKF_YAW_P_MIN' in t2 and 'V5F_EKF_YAW_P_MIN' in e2
ob, cb = e2.count('{'), e2.count('}')
oc, cc = e2.count('/*'), e2.count('*/')
op, cp = e2.count('('), e2.count(')')
print('proc_ekf.c  { } %d/%d 差 %+d   /* */ %d/%d 差 %+d   ( ) %d/%d 差 %+d' % (ob, cb, ob-cb, oc, cc, oc-cc, op, cp, op-cp))
assert ob == cb and oc == cc
print('v5f_tune.h %d 字节 ; proc_ekf.c %d 字节' % (n1, n2))
print()
for src, nm in ((e2, 'proc_ekf.c'), (t2, 'v5f_tune.h')):
    for ln in src.split('\n'):
        if 'V5F_EKF_YAW_P_MIN' in ln or 'V5F_FW_VER' in ln:
            print('%-12s %s' % (nm, ln.strip()))
print()
print('PASS: VER=75 已写入')
