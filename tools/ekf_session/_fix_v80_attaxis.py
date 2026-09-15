# -*- coding: utf-8 -*-
"""
VER=80：重力法向直接由**已知姿态**解出 + 门控复用旧链的加速度牵引姿态门

实测（VER=78 日志离线对比）：
  a_up 取加计方向 + 不对称预测(A)  vs  a_up = R(q_hat)^T z + 同轴预测(B)
    |w| 0~1     : |新息| p50 0.351 -> 0.187 度   (1.9 倍)
    |w| 500~1500: 56.27 -> 37.14 度
    |w| 1500~3000:66.03 -> 47.94 度
  运动时加计饱和（实测 |a| 到 16.8 g），它给的方向是垃圾；而姿态的倾角是重力环
  滤过的量，稳定得多。所以重力法向**直接由已知姿态解出**：
      a_up = R(q_hat)^T . z_nav      （机体系里的"上"）
  测量与预测都建立在这同一根 a_up 上（预测 fp = R(q_hat)^T B0），
  于是倾角误差在两边一致地出现，不再有"测量用加计/预测用姿态"的不对称。

  门控直接用旧链里**加速度牵引姿态**的门 gate->att_tilt（处理函数 5 算出，
  256 ms 窗 |off| 在 1g +-3% 且净 |w| < 2 dps；实测占空比 静止 92% / 剧烈 32%），
  它是旧链验证过的稳定判据，不再自造门。

  简化：观测就是"绕 a_up 的、测量磁场与预测磁场之间的有符号夹角"，
  参考轴 xv 在两式里相消，不需要额外归一化。

上报列语义变更（仍 148 列）：
  146 cmp_amn 由"|accel_g|"改为**姿态"上"与加计方向的夹角(度)** ——
      它就是"当前观测可不可信"的直接读数（0 度 = 加计与姿态一致）。
版本 79 -> 80。
"""
import os, shutil
ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
_SUB = {'\u2202': 'd', '\u2261': '==', '\u00e2': 'a_up', '\u2192': '->', '\u00b7': '.',
        '\u00b1': '+/-', '\u00d7': 'x', '\u2264': '<=', '\u2265': '>=', '\u2248': '~',
        '\u0302': '', '\u1e91': 'z', '\u1e90': 'Z', '\u0177': 'y', '\u00ee': 'i', '\u00f4': 'o'}
def san(s):
    for k, v in _SUB.items(): s = s.replace(k, v)
    s.encode(ENC); return s
def load(p):
    with open(p, 'rb') as f: return f.read().decode(ENC)
def save(p, s, tag):
    d = s.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f: f.write(d)
    with open(p, 'rb') as f: assert f.read().decode(ENC) == s, '回读失败'
    os.utime(p, None); return len(d)
def sub1(s, a, b, what):
    n = s.count(a); assert n == 1, '[%s] 命中 %d 次' % (what, n)
    return s.replace(a, b, 1)

NEW_M7 = san('''static void ekf_m7_mag(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)
{
    float R[1], r[1], rg[2], Rt[3][3], Bn[3], mf[3], ab[3], mh[3], xv[3], crs3[3];
    float fp[3], mh2[3], crs4[3], b0v[3], up_nav[3];
    float fhb2, ci, b0x, b0y, b0z, sig2;
    float dpar, mhn, xvn, thm, thp, psih, dp2, mh2n, an2, cc;
    uint8_t st;
    uint32_t i;

    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;
    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    /* ★VER=72 量纲守卫 */
    {
        float f2 = mf[0]*mf[0] + mf[1]*mf[1] + mf[2]*mf[2];
        if (f2 < 0.25f || f2 > 2.25f) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
            return;
        }
    }
    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];
    s_mag_fhb = sqrtf(fhb2);
    /* ★VER=45 补样本陈旧 */
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
    ci  = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);
    b0x = ci * sinf(V5F_MAG_DECL_RAD);
    b0y = ci * cosf(V5F_MAG_DECL_RAD);
    b0z = -V5F_EKF_DIP_TAN * ci;
    sig2 = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;

    /* ---- 几何诊断（导航系水平二维残差；定义不变，供日志与整角对齐用） ---- */
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);
    s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);
    s_mag_vx = Bn[0]; s_mag_vy = Bn[1];
    s_mag_v0x = b0x; s_mag_v0y = b0y;
    rg[0] = Bn[0] - b0x;
    rg[1] = Bn[1] - b0y;
    s_mag_rx = rg[0];
    s_mag_ry = rg[1];
    /* ★VER=75 保留的死点：|v| 过小说明姿态/投影面已不可信 */
    if (s_mag_bh < V5F_EKF_MAG_BH_MIN) {
        s_mag_r = 0.0f;
        s_mag_rx = 0.0f; s_mag_ry = 0.0f;
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }

    /* ---- ★VER=80 重力法向直接由**已知姿态**解出 ----
     * a_up = R(q_hat)^T . z_nav = 机体系里的"上"。
     * 不再用加计（运动时它饱和，实测 |a| 到 16.8 g，方向是垃圾），
     * 也不再自己构造 R_tilt。测量与预测都建立在这同一根 a_up 上，
     * 于是倾角误差在两边一致地出现，没有"测量用加计/预测用姿态"的不对称。 */
    up_nav[0] = 0.0f; up_nav[1] = 0.0f; up_nav[2] = 1.0f;
    rot_nb(Rt, up_nav, ab);
    /* ★VER=80 门控复用旧链"加速度牵引姿态"的门（处理函数 5 算出）：
     * 256 ms 窗 |off| 在 1g+-3% 且净 |w|<2 dps；实测占空比 静止 92% / 剧烈 32%。
     * 该门开 = 姿态倾角可信 = 本观测可信。磁场观测几何上分不清"偏航错"与
     * "倾角错"，所以倾角不可信时这条观测必须停。 */
    if (!gate->att_tilt) {
        if (s_rej[3] < 250u) s_rej[3]++;
        return;
    }
    /* 诊断：姿态的"上"与加计方向夹角（度数）—— 观测可不可信的直接读数 */
    an2 = sqrtf(h->imu.accel_g[0]*h->imu.accel_g[0]
              + h->imu.accel_g[1]*h->imu.accel_g[1]
              + h->imu.accel_g[2]*h->imu.accel_g[2]);
    if (an2 > 1e-6f) {
        cc = (ab[0]*h->imu.accel_g[0] + ab[1]*h->imu.accel_g[1]
            + ab[2]*h->imu.accel_g[2]) / an2;
        if (cc >  1.0f) cc =  1.0f;
        if (cc < -1.0f) cc = -1.0f;
        s_mag_cmp_amn = acosf(cc) * RAD2DEG;
    }

    /* 测量磁场在法平面内的分量 */
    dpar = mf[0]*ab[0] + mf[1]*ab[1] + mf[2]*ab[2];
    mh[0] = mf[0] - dpar*ab[0];
    mh[1] = mf[1] - dpar*ab[1];
    mh[2] = mf[2] - dpar*ab[2];
    mhn = sqrtf(mh[0]*mh[0] + mh[1]*mh[1] + mh[2]*mh[2]);
    s_mag_cmp_mhn = mhn;
    /* 参考：机体 x 轴在同一平面内的投影 */
    xv[0] = 1.0f - ab[0]*ab[0];
    xv[1] =      - ab[0]*ab[1];
    xv[2] =      - ab[0]*ab[2];
    xvn = sqrtf(xv[0]*xv[0] + xv[1]*xv[1] + xv[2]*xv[2]);
    if (mhn < 1e-3f || xvn < 1e-3f) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    crs3[0] = ab[1]*mh[2] - ab[2]*mh[1];
    crs3[1] = ab[2]*mh[0] - ab[0]*mh[2];
    crs3[2] = ab[0]*mh[1] - ab[1]*mh[0];
    thm = atan2f((crs3[0]*xv[0] + crs3[1]*xv[1] + crs3[2]*xv[2]) / (mhn*xvn),
                 (mh[0]*xv[0] + mh[1]*xv[1] + mh[2]*xv[2]) / (mhn*xvn));

    /* 预测磁场：同一姿态、同一根 a_up */
    b0v[0] = b0x; b0v[1] = b0y; b0v[2] = b0z;
    rot_nb(Rt, b0v, fp);
    dp2 = fp[0]*ab[0] + fp[1]*ab[1] + fp[2]*ab[2];
    mh2[0] = fp[0] - dp2*ab[0];
    mh2[1] = fp[1] - dp2*ab[1];
    mh2[2] = fp[2] - dp2*ab[2];
    mh2n = sqrtf(mh2[0]*mh2[0] + mh2[1]*mh2[1] + mh2[2]*mh2[2]);
    if (mh2n < 1e-3f) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    crs4[0] = ab[1]*mh2[2] - ab[2]*mh2[1];
    crs4[1] = ab[2]*mh2[0] - ab[0]*mh2[2];
    crs4[2] = ab[0]*mh2[1] - ab[1]*mh2[0];
    thp = atan2f((crs4[0]*xv[0] + crs4[1]*xv[1] + crs4[2]*xv[2]) / (mh2n*xvn),
                 (mh2[0]*xv[0] + mh2[1]*xv[1] + mh2[2]*xv[2]) / (mh2n*xvn));
    r[0] = wrap_pi(thm - thp);                  /* 参考轴 xv 在两式里相消 */
    s_mag_r = fabsf(r[0]) * RAD2DEG;
    s_mag_cmp_thm = thm * RAD2DEG;
    s_mag_cmp_thp = thp * RAD2DEG;
    {
        float qw = s_x[IX_Q], qx = s_x[IX_Q+1], qy = s_x[IX_Q+2], qz = s_x[IX_Q+3];
        psih = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
        s_mag_yawpre = psih * RAD2DEG;
    }

    /* H 只有偏航项 */
    H_zero(1u);
    s_H[0][8] = 1.0f;

    /* ★VER=62 开机窗内一次精确角整角修正（用几何残差 rg） */
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
    /* ★VER=65/75 位移死区（VER=75 的常量是 0.0f，此行当前不生效） */
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

e = load(EKF)
a = e.find('static void ekf_m7_mag(const volatile v5f_hold_t *h')
b = e.find('/* M1 水平位置（2 维，按 V5F_EKF_POS_PERIOD_S 降频）+ M2 高度（1 维） */', a)
assert a > 0 and b > a
old = e[a:b]
assert old.count('static void ekf_m7_mag') == 1 and old.rstrip().endswith('}')
print('M7: %d -> %d 字节' % (len(old.encode(ENC)), len(NEW_M7.encode(ENC))))
e = e[:a] + NEW_M7 + '\n' + e[b:]
n_e = save(EKF, e, 'v80')

t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        79u', '#define V5F_FW_VER        80u', 'VER')
n_t = save(TUNE, t, 'v80')

t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        80u' in t2
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
ob, cb = e2.count('{'), e2.count('}'); oc, cc = e2.count('/*'), e2.count('*/')
op, cp = e2.count('('), e2.count(')')
print('proc_ekf.c { } %d/%d  /* */ %d/%d  ( ) %d/%d' % (ob, cb, oc, cc, op, cp))
assert ob == cb and oc == cc
for nm, ok in (('重力法向由姿态解出 rot_nb(Rt,up_nav,ab)', 'rot_nb(Rt, up_nav, ab)' in blk),
               ('预测用同一姿态 rot_nb(Rt,b0v,fp)', 'rot_nb(Rt, b0v, fp)' in blk),
               ('门控 = gate->att_tilt', 'if (!gate->att_tilt)' in blk),
               ('不再调用 raw_f_mps2', 'raw_f_mps2(' not in blk),
               ('观测 1 维', blk.count('ekf_update(R, 1u, r,') == 1),
               ('H 只有偏航项', blk.count('s_H[0][8] = 1.0f') == 1 and 's_H[0][7]' not in blk),
               ('mask 0x0100', blk.count('0x0100u') == 1),
               ('样本门保留', 's_mag_cnt_upd' in blk),
               ('snap 保留', 'dpsi' in blk),
               ('BH_MIN 死点保留', 'V5F_EKF_MAG_BH_MIN' in blk)):
    print('  [%s] %s' % ('OK' if ok else '!!', nm)); assert ok, nm
print('v5f_tune.h %d B ; proc_ekf.c %d B' % (n_t, n_e))
print('PASS: VER=80 已写入')
