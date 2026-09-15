# -*- coding: utf-8 -*-
"""
VER=78：把磁力计改成"位于重力法向量平面里的罗盘类数据源"，并把罗盘量上报

基线 = VER=75（树里现状）。按用户选择：换观测 + 罗盘量加入上报；
VER=75 的门/常量一律保留（DEAD=0、BH_MIN=0.28、R_MAX=150、Q_YAW_MIN、YAW_P_MIN）。

为什么必须换观测：
  现在 M7 吃的是导航系水平二维残差 (Bn_x-b0x, Bn_y-b0y)，其雅可比
        dth_x     dth_y     dth_z
  Bn_x   0      -0.9012   -0.4296
  Bn_y +0.9012     0      -0.0568
  倾角灵敏度 1.2745 / 偏航 0.4333 = 2.94 倍 —— 因为 b0z=-0.9012，陡峭下倾的磁场
  一遇倾斜就把巨大垂直分量倒进水平面。所以它本质是"倾角观测"，不是罗盘；
  再配 mask=0x0100（禁止修倾角），倾角失配经 P[8][7] 全泄放到偏航。

新观测（罗盘量）：
  a_up = 加计实测重力方向(机体系)              <- 它就是"重力法向量"
  mh   = mf - (mf.a_up)a_up                    机体系里落在法平面内的磁场
  xv   = x^ - (x^.a_up)a_up                    机体 x 轴在同一平面内的投影
  thm  = 绕 a_up 从 mh 转到 xv 的方位角        <- 罗盘读数(纯测量)
  thp  = psi_hat - atan2(b0y,b0x)              <- 预测航向(只用状态偏航)
  r    = wrap_pi(thm - thp)      H = [0..0,1]  倾角列按构造恒为 0
新增上报 4 列：
  144 cmp_thm 罗盘实测航向(度)   145 cmp_thp 预测航向(度)
  146 cmp_amn 比力模长(g,1.0=加计即重力)  147 cmp_mhn 法平面内磁场模长(0~1)
  -> 用 144/145 的差与 146/147 就能在日志里判定"罗盘量何时可信"。
JF_CH_NUM 144 -> 148。
"""
import os, shutil

ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
HDR  = os.path.join(ROOT, r'V5F\User\inc\SPI_rx.h')
SRC  = os.path.join(ROOT, r'V5F\User\src\SPI_rx.c')

_SUB = {'\u2202': 'd', '\u2261': '==', '\u00e2': 'a_up', '\u2192': '->',
        '\u00b7': '.', '\u00b1': '+/-', '\u00d7': 'x', '\u2264': '<=',
        '\u2265': '>=', '\u2248': '~', '\u0302': ''}
def san(s):
    for k, v in _SUB.items():
        s = s.replace(k, v)
    s.encode(ENC)
    return s

def load(p):
    with open(p, 'rb') as f: return f.read().decode(ENC)
def save(p, s, tag):
    d = s.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f: f.write(d)
    with open(p, 'rb') as f: assert f.read().decode(ENC) == s, '回读失败 ' + p
    os.utime(p, None)
    return len(d)
def sub1(s, a, b, what):
    n = s.count(a); assert n == 1, '[%s] 命中 %d 次' % (what, n)
    return s.replace(a, b, 1)

NEW_M7 = san('''static void ekf_m7_mag(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float R[1], r[1], rg[2], Rt[3][3], Bn[3], mf[3], fb[3], ab[3], mh[3], xv[3], crs3[3];
    float fhb2, ci, b0x, b0y, sig2;
    float an, dpar, mhn, xvn, thm, thp, psih, azi0;
    uint8_t st;
    uint32_t i;

    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;
    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    /* ★VER=72 量纲守卫：mag.f 由驱动归一化，|f| 恒为 1；实测上报里有 0.35% 的帧
     * |f| 达 1948~3430，一次就会把四元数打废。按模长直接拒绝。 */
    {
        float f2 = mf[0]*mf[0] + mf[1]*mf[1] + mf[2]*mf[2];
        if (f2 < 0.25f || f2 > 2.25f) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
            return;
        }
    }
    /* ★VER=45 机体系水平占比（诊断） */
    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];
    s_mag_fhb = sqrtf(fhb2);
    /* ★VER=45 补样本陈旧：v(t) = Exp(-dth) v(t_s) */
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
    /* 模型磁场水平分量（常量） */
    ci = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);
    b0x = ci * sinf(V5F_MAG_DECL_RAD);
    b0y = ci * cosf(V5F_MAG_DECL_RAD);
    sig2 = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;

    /* ---- 几何诊断（导航系水平二维残差，定义不变，供日志与整角对齐用） ---- */
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);
    s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);
    s_mag_vx = Bn[0]; s_mag_vy = Bn[1];
    s_mag_v0x = b0x; s_mag_v0y = b0y;
    rg[0] = Bn[0] - b0x;
    rg[1] = Bn[1] - b0y;
    s_mag_rx = rg[0];
    s_mag_ry = rg[1];
    /* ★VER=47/74 死点（VER=75 保留）：|v| 过小说明姿态/投影面已不可信 */
    if (s_mag_bh < V5F_EKF_MAG_BH_MIN) {
        s_mag_r = 0.0f;
        s_mag_rx = 0.0f; s_mag_ry = 0.0f;
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }

    /* ---- ★VER=78 罗盘化观测：位于**重力法向量平面**里的方位角 ----
     * 旧观测是导航系水平二维残差，雅可比显示倾角灵敏度(1.2745)是偏航(0.4333)的
     * 2.94 倍（b0z=-0.9012 使倾斜把垂直分量倒进水平面），本质是倾角观测；
     * 再配 mask=0x0100（禁止修倾角）就使倾角失配经 P[8][7] 泄放进偏航。
     * 现在只取一个方位角：法向 = 加计实测重力方向 a_up（测量量、不在状态里），
     * 观测矩阵 H 只有偏航项，倾角列按构造恒为 0 -> 磁拿不到倾角的信息，
     * 倾角也不再干涉偏航。 */
    raw_f_mps2(h, fb);
    an = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);
    if (an < 1e-6f) return;
    ab[0] = fb[0]/an; ab[1] = fb[1]/an; ab[2] = fb[2]/an;      /* a_up：重力法向 */
    s_mag_cmp_amn = an / V5F_EKF_G_MPS2;                        /* 上报：比力模长(g) */
    dpar = mf[0]*ab[0] + mf[1]*ab[1] + mf[2]*ab[2];
    mh[0] = mf[0] - dpar*ab[0];
    mh[1] = mf[1] - dpar*ab[1];
    mh[2] = mf[2] - dpar*ab[2];                 /* 法平面内的磁场（指向磁北） */
    xv[0] = 1.0f - ab[0]*ab[0];
    xv[1] =      - ab[0]*ab[1];
    xv[2] =      - ab[0]*ab[2];                 /* 机体 x 轴在同一平面内的投影 */
    mhn = sqrtf(mh[0]*mh[0] + mh[1]*mh[1] + mh[2]*mh[2]);
    xvn = sqrtf(xv[0]*xv[0] + xv[1]*xv[1] + xv[2]*xv[2]);
    s_mag_cmp_mhn = mhn;                        /* 上报：法平面内磁场模长 */
    if (mhn < 1e-3f || xvn < 1e-3f) {           /* 数值保护 */
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    crs3[0] = ab[1]*mh[2] - ab[2]*mh[1];
    crs3[1] = ab[2]*mh[0] - ab[0]*mh[2];
    crs3[2] = ab[0]*mh[1] - ab[1]*mh[0];        /* a_up x mh */
    thm = atan2f((crs3[0]*xv[0] + crs3[1]*xv[1] + crs3[2]*xv[2]) / (mhn*xvn),
                 (mh[0]*xv[0] + mh[1]*xv[1] + mh[2]*xv[2]) / (mhn*xvn));
    {
        float qw = s_x[IX_Q], qx = s_x[IX_Q+1], qy = s_x[IX_Q+2], qz = s_x[IX_Q+3];
        psih = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
        s_mag_yawpre = psih * RAD2DEG;
    }
    azi0 = atan2f(b0y, b0x);
    thp  = psih - azi0;
    r[0] = wrap_pi(thm - thp);
    s_mag_r = fabsf(r[0]) * RAD2DEG;            /* 新息 = 航向误差（度） */
    s_mag_cmp_thm = thm * RAD2DEG;              /* 上报：罗盘实测航向 */
    s_mag_cmp_thp = thp * RAD2DEG;              /* 上报：预测航向 */

    /* H 只有偏航项：倾角列按构造恒为 0 */
    H_zero(1u);
    s_H[0][8] = 1.0f;

    /* ★VER=62 开机窗内一次精确角整角修正（用几何残差 rg，与新息无关） */
    if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {
        float n2 = b0x * b0x + b0y * b0y;
        float dpsi = 0.0f, e0, e1, h2;
        float qb[4], dq[4], qt[4];
        uint32_t i2;
        e0 = sqrtf(rg[0] * rg[0] + rg[1] * rg[1]);
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
            e1 = sqrtf(rx2 * rx2 + ry2 * ry2);
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
    /* ★VER=65/75 位移死区（VER=75 的常量是 0.0f，此行当前不生效，保留以便调） */
    if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {
        return;
    }
    if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
    } else {
        /* ★VER=73 只在磁样本真的更新时施加一次 */
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

# ---------- 1) proc_ekf.c ----------
e = load(EKF)
a = e.find('static void ekf_m7_mag(const volatile v5f_hold_t *h')
assert a > 0
b = e.find('/* M1 水平位置（2 维，按 V5F_EKF_POS_PERIOD_S 降频）+ M2 高度（1 维） */', a)
assert b > a
old = e[a:b]
assert old.count('static void ekf_m7_mag') == 1 and old.rstrip().endswith('}')
print('M7: %d -> %d 字节' % (len(old.encode(ENC)), len(NEW_M7.encode(ENC))))
e = e[:a] + NEW_M7 + '\n' + e[b:]

e = sub1(e, 'static float    s_mag_dqx, s_mag_dqy, s_mag_dqz;',
         'static float    s_mag_dqx, s_mag_dqy, s_mag_dqz;\n'
         'static float    s_mag_cmp_thm, s_mag_cmp_thp;   /* ★VER=78 罗盘：实测/预测航向(度) */\n'
         'static float    s_mag_cmp_amn, s_mag_cmp_mhn;   /* ★VER=78 罗盘：比力模长(g) / 法平面内磁场模长 */',
         'statics')
e = sub1(e, '       h->ekf.mag_dqz = s_mag_dqz;',
         '       h->ekf.mag_dqz = s_mag_dqz;\n'
         '       h->ekf.mag_cmp_thm = s_mag_cmp_thm;\n'
         '       h->ekf.mag_cmp_thp = s_mag_cmp_thp;\n'
         '       h->ekf.mag_cmp_amn = s_mag_cmp_amn;\n'
         '       h->ekf.mag_cmp_mhn = s_mag_cmp_mhn;',
         'publish')
n_e = save(EKF, e, 'v78')

# ---------- 2) SPI_rx.h ----------
h = load(HDR)
h = sub1(h, '        float    tilt_prz;   /* VER=50 */',
         '        float    tilt_prz;   /* VER=50 */\n'
         '        float    mag_cmp_thm;   /* ★VER=78 罗盘：重力法平面内实测航向(度) */\n'
         '        float    mag_cmp_thp;   /* ★VER=78 罗盘：同一平面内预测航向(度) */\n'
         '        float    mag_cmp_amn;   /* ★VER=78 罗盘：比力模长(g)，1.0=加计即重力 */\n'
         '        float    mag_cmp_mhn;   /* ★VER=78 罗盘：法平面内磁场模长(0~1) */',
         'struct')
n_h = save(HDR, h, 'v78')

# ---------- 3) SPI_rx.c ----------
c = load(SRC)
c = sub1(c, '#define JF_CH_NUM     144u', '#define JF_CH_NUM     148u', 'nch')
c = sub1(c, '        ch[c++] = g_v5f_hold.ekf.tilt_prz;   /* VER=50 */',
         '        ch[c++] = g_v5f_hold.ekf.tilt_prz;   /* VER=50 */\n'
         '        ch[c++] = g_v5f_hold.ekf.mag_cmp_thm;   /* ★VER=78 罗盘：实测航向(度) */\n'
         '        ch[c++] = g_v5f_hold.ekf.mag_cmp_thp;   /* ★VER=78 罗盘：预测航向(度) */\n'
         '        ch[c++] = g_v5f_hold.ekf.mag_cmp_amn;   /* ★VER=78 罗盘：比力模长(g) */\n'
         '        ch[c++] = g_v5f_hold.ekf.mag_cmp_mhn;   /* ★VER=78 罗盘：法平面内磁场模长 */',
         'report')
n_c = save(SRC, c, 'v78')

# ---------- 4) v5f_tune.h ----------
t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        75u', '#define V5F_FW_VER        78u', 'VER')
n_t = save(TUNE, t, 'v78')

# ---------- 自检 ----------
t2, e2, h2, c2 = load(TUNE), load(EKF), load(HDR), load(SRC)
print()
print('v5f_tune.h %d B ; proc_ekf.c %d B ; SPI_rx.h %d B ; SPI_rx.c %d B' % (n_t, n_e, n_h, n_c))
assert '#define V5F_FW_VER        78u' in t2
assert '#define JF_CH_NUM     148u' in c2
# 逐符点数 ch[c++]
import re
nch = len(re.findall(r'ch\[c\+\+\]', c2))
# for 循环里的 ch[c++] 只算一次, 手算补偿: 逐个统计需要人工; 这里只报数量参考
print('SPI_rx.c 里 ch[c++] 出现 %d 处（含 for 循环，每处可能写多列）' % nch)
for nm, s, a_, b_ in (('proc_ekf.c', e2, '{', '}'), ('SPI_rx.c', c2, '{', '}'),
                      ('SPI_rx.h', h2, '{', '}'), ('v5f_tune.h', t2, '{', '}')):
    x, y = s.count(a_), s.count(b_)
    z, w = s.count('/*'), s.count('*/')
    print('  %-12s { } %d/%d   /* */ %d/%d' % (nm, x, y, z, w))
    assert x == y and z == w
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
for nm, ok in (('观测 1 维', blk.count('ekf_update(R, 1u, r,') == 1),
               ('H 只有偏航项', blk.count('s_H[0][8] = 1.0f') == 1 and 's_H[0][7]' not in blk),
               ('罗盘量已算', all(k in blk for k in ('s_mag_cmp_thm', 's_mag_cmp_thp', 's_mag_cmp_amn', 's_mag_cmp_mhn'))),
               ('BH_MIN 保留', 'V5F_EKF_MAG_BH_MIN' in blk),
               ('样本门保留', 's_mag_cnt_upd' in blk),
               ('snap 保留', 'dpsi' in blk)):
    print('  [%s] %s' % ('OK' if ok else '!!', nm)); assert ok, nm
print()
print('PASS: VER=78 已写入')
