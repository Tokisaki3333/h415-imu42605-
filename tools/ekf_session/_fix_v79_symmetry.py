# -*- coding: utf-8 -*-
"""
VER=79：修 ①预测侧不对称 ②重力方向改用固件已算好的数据

实测 VER=78（28.48 s / 228620 帧 / 148 列）：
  · 姿态推出的"上" 与 加计方向的夹角 < 1 度时  |新息| p50 0.275 度 p90 0.87 度
    夹角 10~30 度时                            |新息| p50 31.6  度 p90 94 度
    夹角 30~90 度时                            |新息| p50 48.6  度 p90 152 度
  · 静止时 accel_g 中位 (0.0025,0.0095,1.0004), 模长 1.0005 -> 方向本身正确
  · 但 raw_f_mps2 算出的模长中位 1.034, 比 accel_g 大 3.4%（离线标定与在线零偏重复扣）

改动 1：预测侧也用**同一个实测重力轴**构造模型磁场
  旧: thp = psi_hat - atan2(b0y,b0x)     <- 隐含假设"姿态倾角 == 加计倾角"
  新: R_pred = R_tilt(a_up) . R_z(psi_hat)     (R_tilt 把 ẑ_nav 转到 a_up)
      f_pred = R_pred^T B0 = R_z(-psi_hat) . (R_tilt^T B0)
      thp    = 用与 thm **完全相同**的公式（同轴 a_up、同参考 xv）算 f_pred 的方位角
  => 倾角误差既不进测量也不进预测（两者都只用测量得到的 a_up），
     偏航误差仍 1:1 进入；dr/d(dth_z) = +1 不变。

改动 2：重力方向改用固件已经算好的 imu.accel_g（单位 g），
      不再用 raw_f_mps2 自己重跑离线标定；有效性判据用已有的
      V5F_EKF_TILT_AMAG_TOL（与 M6 的门同口径）+ imu.acc_valid。
      s_mag_cmp_amn 现在是 |accel_g|（g）。

版本 78 -> 79。
"""
import os, shutil
ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
_SUB = {'\u2202': 'd', '\u2261': '==', '\u00e2': 'a_up', '\u2192': '->', '\u00b7': '.',
        '\u00b1': '+/-', '\u00d7': 'x', '\u2264': '<=', '\u2265': '>=', '\u2248': '~', '\u0302': '',
        '\u1e91': 'z', '\u1e90': 'Z', '\u0177': 'y', '\u00ee': 'i', '\u00f4': 'o'}
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

e = load(EKF)

# ---- (1) 声明 ----
e = sub1(e,
    '    float R[1], r[1], rg[2], Rt[3][3], Bn[3], mf[3], fb[3], ab[3], mh[3], xv[3], crs3[3];\n'
    '    float fhb2, ci, b0x, b0y, sig2;\n'
    '    float an, dpar, mhn, xvn, thm, thp, psih, azi0;\n',
    '    float R[1], r[1], rg[2], Rt[3][3], Bn[3], mf[3], fb[3], ab[3], mh[3], xv[3], crs3[3];\n'
    '    float fp[3], mh2[3], crs4[3];\n'
    '    float fhb2, ci, b0x, b0y, sig2;\n'
    '    float an, dpar, mhn, xvn, thm, thp, psih;\n'
    '    float kx, ky, sn, cs, ux, uy, uz, cw, sw, dp2, mh2n;\n',
    'decl')

# ---- (2) 重力方向来源 + 有效性门 ----
e = sub1(e,
    '    raw_f_mps2(h, fb);\n'
    '    an = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);\n'
    '    if (an < 1e-6f) return;\n'
    '    ab[0] = fb[0]/an; ab[1] = fb[1]/an; ab[2] = fb[2]/an;      /* a_up：重力法向 */\n'
    '    s_mag_cmp_amn = an / V5F_EKF_G_MPS2;                        /* 上报：比力模长(g) */\n',
    san('''    /* ★VER=79 重力方向：直接用固件**已经算好**的比力 imu.accel_g（单位 g），
     * 不再用 raw_f_mps2 自己重跑一遍离线标定 —— 实测那样出来的模长中位 1.034，
     * 比 accel_g 的 1.0005 大 3.4%（离线标定与在线牵引的零偏被重复扣了一次）。
     * 有效性判据沿用已有的 V5F_EKF_TILT_AMAG_TOL（与 M6 重力门的口径一致）：
     * 加计此刻必须真的在测重力，否则投影轴没有意义。 */
    for (i = 0u; i < 3u; i++) fb[i] = h->imu.accel_g[i];
    an = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);
    if (an < 1e-6f) return;
    s_mag_cmp_amn = an;                        /* 上报：|accel_g|（g） */
    if (!h->imu.acc_valid || fabsf(an*an - 1.0f) >= V5F_EKF_TILT_AMAG_TOL) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    ab[0] = fb[0]/an; ab[1] = fb[1]/an; ab[2] = fb[2]/an;      /* a_up：实测重力法向 */
'''),
    'gravity')

# ---- (3) 预测侧改成同一根轴构造 ----
e = sub1(e,
    '    azi0 = atan2f(b0y, b0x);\n'
    '    thp  = psih - azi0;\n'
    '    r[0] = wrap_pi(thm - thp);\n',
    san('''    /* ★VER=79 预测侧也用**同一根实测重力轴**构造，消除不对称：
     *   旧写法 thp = psi_hat - atan2(b0y,b0x) 隐含"姿态倾角 == 加计倾角"，
     *   运动时两者差几度到几十度，全灌进新息（实测夹角<1度时 |新息| 0.28 度，
     *   夹角 30~90 度时 48.6 度），环路就以 k_cap 上限（|r|=88 度时约 850 度/秒）
     *   把偏航往错方向拽 = "巨大快速漂移 + 磁不稳定"。
     *   新写法：R_pred = R_tilt(a_up) . R_z(psi_hat)，R_tilt 把 ẑ_nav 转到 a_up；
     *           f_pred = R_pred^T B0 = R_z(-psi_hat) . (R_tilt^T B0)
     *   两处都只用测量得到的 a_up，倾角误差便既不进测量也不进预测；
     *   偏航仍 1:1 进入，d(r)/d(dth_z) = +1 不变。 */
    kx = -ab[1]; ky = ab[0];                   /* ẑ_nav x a_up 的 x,y 分量（轴, kz=0） */
    sn = sqrtf(kx*kx + ky*ky);                 /* = sin(夹角) */
    cs = ab[2];                                /* = cos(夹角) = ẑ.a_up */
    if (sn > 1e-6f) {
        float kxn = kx/sn, kyn = ky/sn;
        float kd = kxn*b0x + kyn*b0y;                     /* k.B0 */
        float cx = kyn*b0z;                               /* k x B0 */
        float cy = -kxn*b0z;
        float cz = kxn*b0y - kyn*b0x;
        ux = b0x*cs + cx*sn + kxn*kd*(1.0f - cs);
        uy = b0y*cs + cy*sn + kyn*kd*(1.0f - cs);
        uz = b0z*cs + cz*sn;
    } else if (cs > 0.0f) {
        ux = b0x; uy = b0y; uz = b0z;          /* a_up 与 ẑ 同向 */
    } else {
        ux = b0x; uy = -b0y; uz = -b0z;        /* a_up 与 ẑ 反向：绕机体 x 转 180 度 */
    }
    cw = cosf(psih); sw = sinf(psih);
    fp[0] =  cw*ux + sw*uy;                    /* R_z(-psi_hat) . u */
    fp[1] = -sw*ux + cw*uy;
    fp[2] =  uz;
    /* 预测航向：与 thm 完全相同的一套公式（同轴 a_up、同参考 xv） */
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
    r[0] = wrap_pi(thm - thp);
'''),
    'predict')
n_e = save(EKF, e, 'v79')

t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        78u', '#define V5F_FW_VER        79u', 'VER')
n_t = save(TUNE, t, 'v79')

# ---- 自检 ----
t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        79u' in t2
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
ob, cb = e2.count('{'), e2.count('}')
oc, cc = e2.count('/*'), e2.count('*/')
print('proc_ekf.c { } %d/%d   /* */ %d/%d' % (ob, cb, oc, cc))
assert ob == cb and oc == cc
for nm, ok in (('预测已用同一根轴(R_tilt)', 'kx = -ab[1]' in blk),
               ('f_pred 构造', 'fp[0] =  cw*ux + sw*uy' in blk),
               ('旧 azi0 写法已消失', 'azi0' not in blk),
               ('重力源改为 imu.accel_g', 'h->imu.accel_g[i]' in blk),
               ('不再用 raw_f_mps2', 'raw_f_mps2' not in blk),
               ('有效性门用已有容差常量', 'V5F_EKF_TILT_AMAG_TOL' in blk),
               ('观测仍 1 维', blk.count('ekf_update(R, 1u, r,') == 1),
               ('H 只有偏航项', blk.count('s_H[0][8] = 1.0f') == 1),
               ('mask 0x0100', blk.count('0x0100u') == 1),
               ('样本门保留', 's_mag_cnt_upd' in blk),
               ('snap 保留', 'dpsi' in blk),
               ('BH_MIN 保留', 'V5F_EKF_MAG_BH_MIN' in blk)):
    print('  [%s] %s' % ('OK' if ok else '!!', nm)); assert ok, nm
print('v5f_tune.h %d B ; proc_ekf.c %d B' % (n_t, n_e))
print('PASS: VER=79 已写入')
