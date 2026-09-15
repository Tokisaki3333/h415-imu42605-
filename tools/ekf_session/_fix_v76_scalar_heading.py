# -*- coding: utf-8 -*-
"""
VER=76：地磁观测由"导航系水平二维残差"改为"绕实测重力轴的 1 维标量磁航向"

为什么必须改（数值算出的雅可比，不是推测）：
  旧观测 (Bn_x-b0x, Bn_y-b0y) 对姿态误差的雅可比（Bn_true ≈ Bn + dth×Bn 线性化）
        dth_x      dth_y      dth_z
  Bn_x   0        -0.9012    -0.4296
  Bn_y +0.9012     0        -0.0568
  => 倾角灵敏度 1.2745、偏航 0.4333，**倾角是偏航的 2.94 倍**。
  原因：磁场陡峭下倾 b0z=-0.9012，机体一倾斜就把巨大垂直分量倒进水平面。
  所以"水平二维投影"并没有抹掉倾角那一维；再配 mask=0x0100(禁止修倾角)，
  倾角失配就经 P[8][7] 全部泄放到偏航上 -> 磁环把机体四元数往磁北拽、转不动。

新观测（1 维）：
  â  = 加计实测重力方向(机体系) —— 测量量, 不在状态里
  mh = mf - (mf·â)â        机体系水平磁场(指向磁北)
  xv = x̂_b - (x̂_b·â)â      机体 x 轴的水平投影
  thm= 绕 â 从 mh 转到 xv 的方位角        —— 纯测量
  thp= psi_hat - atan2(b0y,b0x)           —— 只用状态的偏航
  r  = wrap_pi(thm - thp)
  H  = [0 ... 0, 1]   ← 倾角列**按构造恒为 0**
  => ∂r/∂dth_tilt ≡ 0（因为 â 是测量量）、∂r/∂dth_z = 1。
  磁永远拿不到倾角信息；倾角也不再干涉偏航；偏航可时刻被磁修正。

同时按用户指示：
  · 删掉 V5F_EKF_MAG_BH_MIN 死点 —— 其实际触发条件是重力法向与磁北重合(真磁极),
    本地磁倾 64.3 度、水平分量恒 0.4333, 不可能发生; 只会在倾角误差大时误触发。
  · 删掉 V5F_EKF_MAG_DEAD_DEG 死区 —— 用户要求磁环时刻工作。
  · R_MAX 保留, 只作数值野值门(新息已定义为航向误差, 单位就是度)。
  · 几何诊断量与整角对齐(snap)仍用原来的 2 维残差 rg[], 日志语义不变。
  · 版本号跳到 76: 74/75 已被两次已知坏的固件占用, 复用会造成指纹歧义。
"""
import os, shutil

ENC = 'gbk'

# GBK 里没有的符号一律换成 ASCII（∂ â ≡ 等），保证 encode(ENC) 不炸
_SUB = {'\u2202': 'd', '\u2261': '==', '\u00e2': 'a_up', '\u2192': '->',
        '\u00b7': '.', '\u00b1': '+/-', '\u00d7': 'x', '\u2264': '<=',
        '\u2265': '>=', '\u2248': '~', '\u0302': ''}
def san(s):
    for k, v in _SUB.items():
        s = s.replace(k, v)
    s.encode(ENC)          # 立即验证
    return s
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')

NEW_M7 = san('''static void ekf_m7_mag(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float R[1], r[1], rg[2], Rt[3][3], Bn[3], mf[3], fb[3], ab[3], mh[3], xv[3], crs3[3];
    float fhb2, ci, si, b0x, b0y, b0z, sig2;
    float an, dpar, mhn, xvn, thm, thp, psih, azi0;
    uint8_t st;
    uint32_t i;

    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;
    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    /* ★VER=72 量纲守卫：mag.f 由驱动归一化，正常帧 |f| 恒为 1.0000（实测）。
     * 上报里实测有 0.35% 的帧 |f| 达 1948~3430，一次 |f|=1954 的新息经 k_cap
     * 会给 dx[8] 约 97 rad，注入后四元数直接废掉。按模长直接拒绝。*/
    {
        float f2 = mf[0]*mf[0] + mf[1]*mf[1] + mf[2]*mf[2];
        if (f2 < 0.25f || f2 > 2.25f) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
            return;
        }
    }
    /* ★VER=45 机体系水平占比（向量有效性诊断量） */
    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];
    s_mag_fhb = sqrtf(fhb2);
    /* ★VER=45 补样本陈旧：IST 约 187 Hz，2229 dps 下一个样本最多陈旧 63 度。
     * v(t) = Exp(-dth) v(t_s)，dth = 自采样以来的**机体**转动量。 */
    {
        float am = sqrtf(s_mag_dth[0]*s_mag_dth[0] + s_mag_dth[1]*s_mag_dth[1]
                       + s_mag_dth[2]*s_mag_dth[2]);
        if (am > 1e-4f) {
            float half = 0.5f * am;
            float sc = sinf(half) / am;
            float dq[4];
            float tmp[3];
            dq[0] = cosf(half);
            dq[1] = -sc * s_mag_dth[0];
            dq[2] = -sc * s_mag_dth[1];
            dq[3] = -sc * s_mag_dth[2];
            q_rot_vec(dq, mf, tmp);
            mf[0] = tmp[0]; mf[1] = tmp[1]; mf[2] = tmp[2];
        }
    }
    /* 模型磁场常量：水平分量 v0=(b0x,b0y)、垂直分量 b0z（陡峭下倾，-0.9013） */
    ci = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);
    si = V5F_EKF_DIP_TAN * ci;
    b0x = ci * sinf(V5F_MAG_DECL_RAD);
    b0y = ci * cosf(V5F_MAG_DECL_RAD);
    b0z = -si;
    sig2 = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;

    /* ---- 几何诊断：导航系水平二维残差（定义不变，供日志与整角对齐用） ---- */
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);
    s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);
    s_mag_vx = Bn[0]; s_mag_vy = Bn[1];
    s_mag_v0x = b0x; s_mag_v0y = b0y;
    rg[0] = Bn[0] - b0x;
    rg[1] = Bn[1] - b0y;
    s_mag_rx = rg[0];
    s_mag_ry = rg[1];

    /* ---- ★VER=76 新观测：绕**实测重力轴**的 1 维标量磁航向 ----
     * 旧观测是导航系水平二维残差，其雅可比对倾角的灵敏度(1.2745)是偏航(0.4333)
     * 的 2.94 倍 —— 因为 b0z=-0.9012，倾斜会把巨大垂直分量倒进水平面。
     * 所以"水平投影"根本没抹掉倾角那一维；再配 mask=0x0100（禁止修倾角），
     * 倾角失配就经 P[8][7] 全泄放到偏航 -> 磁环把四元数往磁北拽、转不动。
     * 新观测把**加计实测的重力方向**当水平面法向（测量量，不在状态里），
     * 只比较"磁场水平方向"与"机体 x 轴水平投影"的方位角；预测侧只用状态偏航。
     * 于是 ∂r/∂dth_tilt 按构造恒为 0，∂r/∂dth_z = 1。 */
    raw_f_mps2(h, fb);                          /* 只用原始 LSB + 离线标定常量 */
    an = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);
    if (an < 1e-6f) return;
    ab[0] = fb[0]/an; ab[1] = fb[1]/an; ab[2] = fb[2]/an;      /* â：机体系"上" */
    dpar = mf[0]*ab[0] + mf[1]*ab[1] + mf[2]*ab[2];
    mh[0] = mf[0] - dpar*ab[0];
    mh[1] = mf[1] - dpar*ab[1];
    mh[2] = mf[2] - dpar*ab[2];                 /* 水平磁场，指向磁北 */
    xv[0] = 1.0f - ab[0]*ab[0];
    xv[1] =      - ab[0]*ab[1];
    xv[2] =      - ab[0]*ab[2];                 /* 机体 x 轴的水平投影 */
    mhn = sqrtf(mh[0]*mh[0] + mh[1]*mh[1] + mh[2]*mh[2]);
    xvn = sqrtf(xv[0]*xv[0] + xv[1]*xv[1] + xv[2]*xv[2]);
    if (mhn < 1e-3f || xvn < 1e-3f) {           /* 数值保护：磁场或 x 轴与重力平行 */
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    crs3[0] = ab[1]*mh[2] - ab[2]*mh[1];
    crs3[1] = ab[2]*mh[0] - ab[0]*mh[2];
    crs3[2] = ab[0]*mh[1] - ab[1]*mh[0];        /* â × mh */
    thm = atan2f((crs3[0]*xv[0] + crs3[1]*xv[1] + crs3[2]*xv[2]) / (mhn*xvn),
                 (mh[0]*xv[0] + mh[1]*xv[1] + mh[2]*xv[2]) / (mhn*xvn));
    /* 预测航向：状态偏航（机体 x 轴在导航系的方位角）减模型磁场水平方位角 */
    {
        float qw = s_x[IX_Q], qx = s_x[IX_Q+1], qy = s_x[IX_Q+2], qz = s_x[IX_Q+3];
        psih = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
        s_mag_yawpre = psih * RAD2DEG;
    }
    azi0 = atan2f(b0y, b0x);
    thp  = psih - azi0;
    r[0] = wrap_pi(thm - thp);
    s_mag_r = fabsf(r[0]) * RAD2DEG;            /* 新息就是航向误差，单位度 */

    /* H 只有偏航项：倾角列按构造恒为 0，磁拿不到倾角的信息 */
    H_zero(1u);
    s_H[0][8] = 1.0f;

    /* ★VER=62 开机窗内一次精确角整角修正（用几何残差 rg，与上面新息无关） */
    if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {
        float n2 = b0x * b0x + b0y * b0y;
        float dpsi = 0.0f, e0, e1, h2;
        float qb[4], dq[4], qt[4];
        uint32_t i2;
        e0 = sqrtf(rg[0] * rg[0] + rg[1] * rg[1]);
        /* 精确角：线性化 cross/n2 在 180 度处为 0（鞍点），
         * 必须用 atan2(cross, dot) 才能一步到位：
         *   cross = b0x*rg1 - b0y*rg0；dot = b0x*rg0 + b0y*rg1 + n2 */
        if (n2 > 1e-6f) {
            float crs = b0x * rg[1] - b0y * rg[0];
            float dt2 = b0x * rg[0] + b0y * rg[1] + n2;
            dpsi = atan2f(crs, dt2);
        }
        if (dpsi >  3.14159265f) dpsi =  3.14159265f;
        if (dpsi < -3.14159265f) dpsi = -3.14159265f;
        for (i2 = 0u; i2 < 4u; i2++) qb[i2] = s_x[IX_Q + i2];
        h2 = 0.5f * dpsi;
        dq[0] = cosf(h2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(h2);
        q_mul(dq, &s_x[IX_Q], qt);
        for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
        q_norm(&s_x[IX_Q]);
        q_to_R(&s_x[IX_Q], Rt);
        rot_bn(Rt, mf, Bn);
        {
            float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y;
            e1 = sqrtf(rx2 * rx2 + ry2 * ry2);   /* 度量用完整新息 */
        }
        if (e1 > e0) {
            for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qb[i2];
            h2 = -0.5f * dpsi;
            dq[0] = cosf(h2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(h2);
            q_mul(dq, &s_x[IX_Q], qt);
            for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
            q_norm(&s_x[IX_Q]);
            dpsi = -dpsi;
        }
        s_mag_anchor = 1u;
        s_mag_used = 1u;
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = dpsi * RAD2DEG;
        s_gate_bits |= V5F_EKF_GB_MAG;
        rg[0] = 0.0f; rg[1] = 0.0f;
        s_mag_rx = 0.0f; s_mag_ry = 0.0f;
        s_mag_r = 0.0f;
        return;
    }

    R[0] = sig2;
    /* R_MAX 只作数值野值门：s_mag_r 现在就是航向误差（度） */
    if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
    } else {
        /* ★VER=73 只在**磁样本真的更新**时施加一次卡尔曼更新。
         * ist_cnt 变化沿 = 193.8 Hz（磁物理率），而 M7 每 23 帧被调用一次 =
         * 351.7 Hz -> 同一个样本会被重复施加 1.81 次。被挡住的周期把三个 dq
         * 清零，使日志语义唯一：dqz == 0 就是"本周期没有施加修正"。 */
        {
            uint32_t icm = g_shm ? g_shm->ist.hdr.cnt : 0u;
            if (icm == s_mag_cnt_upd) {
                s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
                return;
            }
            s_mag_cnt_upd = icm;
        }
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG_NOSW, &s_nis[4], &s_rej[3],
                        0x0100u, V5F_EKF_MAG_K_MAX);
        if (st == 0u) {
            s_gate_bits |= V5F_EKF_GB_MAG;
            s_mag_used = 1u;
            s_mag_dqx = s_dx[IX_Q + 0] * RAD2DEG;
            s_mag_dqy = s_dx[IX_Q + 1] * RAD2DEG;
            s_mag_dqz = s_dx[IX_Q + 2] * RAD2DEG;
        }
    }
}
''')

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

# ---------------- 1. proc_ekf.c：整函数替换 M7 ----------------
e = load(EKF)
a = e.find('static void ekf_m7_mag(const volatile v5f_hold_t *h')
assert a > 0, '找不到 M7 起点'
b = e.find('/* M1 水平位置（2 维，按 V5F_EKF_POS_PERIOD_S 降频）+ M2 高度（1 维） */', a)
assert b > a, '找不到 M7 终点锚点'
old = e[a:b]
assert old.count('static void ekf_m7_mag') == 1
assert old.rstrip().endswith('}')
print('M7 原函数 %d 字节 -> 新函数 %d 字节' % (len(old.encode(ENC)), len(NEW_M7.encode(ENC))))
e = e[:a] + NEW_M7 + '\n' + e[b:]
e = sub1(e, 'static uint32_t s_mag_cnt_upd;        /* ★VER=73 上一次真正施加磁观测时的 ist 样本号 */',
         'static uint32_t s_mag_cnt_upd;        /* ★VER=73 上一次真正施加磁观测时的 ist 样本号 */',
         'counter-keep')
n_e = save(EKF, e, 'v76')

# ---------------- 2. v5f_tune.h：删两个门 + VER ----------------
t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        73u', '#define V5F_FW_VER        76u', 'VER')
t = sub1(t,
    '#define V5F_EKF_MAG_BH_MIN       0.12f  /* 磁偏航增益上限（tau 约 1.4 s 的牵引）*/',
    '/* ★VER=76 已删除 V5F_EKF_MAG_BH_MIN：该"几何退化门"的实际触发条件是\n'
    ' * 重力法向量与磁北重合（即到了真磁极），本地磁倾 64.3 度、水平分量恒 0.4333，\n'
    ' * 根本不可能发生；保留它只会在倾角误差大时误触发（实测 |v| 掉到 0.13~0.21\n'
    ' * 的那些帧正是倾角/投影面误差大，不是磁场退化）。 */',
    'BH_MIN-del')
t = sub1(t, '#define V5F_EKF_MAG_DEAD_DEG      12.0f',
    '/* ★VER=76 已删除 V5F_EKF_MAG_DEAD_DEG：用户要求磁环时刻工作。\n'
    ' * 旧值 12 度本来是 VER=65 为"小误差不追"加的，与几何退化无关。 */',
    'DEAD-del')
n_t = save(TUNE, t, 'v76')

# ---------------- 3. 自检 ----------------
t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        76u' in t2
assert 'V5F_EKF_MAG_BH_MIN' not in t2 and 'V5F_EKF_MAG_DEAD_DEG' not in t2
assert 'V5F_EKF_MAG_BH_MIN' not in e2 and 'V5F_EKF_MAG_DEAD_DEG' not in e2
ob, cb = e2.count('{'), e2.count('}')
oc, cc = e2.count('/*'), e2.count('*/')
op, cp = e2.count('('), e2.count(')')
print()
print('proc_ekf.c  { } %d/%d 差 %+d   /* */ %d/%d 差 %+d   ( ) %d/%d 差 %+d' %
      (ob, cb, ob-cb, oc, cc, oc-cc, op, cp, op-cp))
assert ob == cb and oc == cc
print('v5f_tune.h %d 字节 ; proc_ekf.c %d 字节' % (n_t, n_e))
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
print()
print('M7 自检：')
print('  ekf_update(R, 1u, ...) 出现 %d 次 (应 1)' % blk.count('ekf_update(R, 1u, r,'))
print('  H_zero(1u) 出现 %d 次 (应 1)' % blk.count('H_zero(1u)'))
print('  s_H[0][8] 出现 %d 次 (应 1)' % blk.count('s_H[0][8]'))
print('  旧的 2 维 H 列 s_H[0][7]/s_H[1][6] 残留 %d 处 (应 0)' %
      (blk.count('s_H[0][7]') + blk.count('s_H[1][6]')))
print('  raw_f_mps2(实测重力) 出现 %d 次 (应 1)' % blk.count('raw_f_mps2'))
print('  mask 0x0100u 出现 %d 次 (应 1)' % blk.count('0x0100u'))
assert blk.count('ekf_update(R, 1u, r,') == 1 and blk.count('H_zero(1u)') == 1
assert blk.count('s_H[0][7]') == 0 and blk.count('s_H[1][6]') == 0
print()
print('PASS: VER=76 已写入')
