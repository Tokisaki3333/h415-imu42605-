# -*- coding: utf-8 -*-
"""VER=103: 地磁改"真牵引"（2 维向量量测）+ 取消运动中禁磁。

v5f_tune.h
  V5F_FW_VER 102u -> 103u
  + V5F_EKF_MAG_MODE 1u        0=旧标量牵引(回退) / 1=真牵引(向量量测)
  + V5F_EKF_MAG_VEC_SIG_DEG 0.9f   量测 1sigma（本机实测 0.5/1.3 deg）
  + V5F_EKF_MAG_VEC_K_MAX 0.10f    单次更新 K 上限（步长 = K*|r|）
  V5F_EKF_MAG_GT_HOLD_S 标注为已弃用（代码里那条 hold 被删）

proc_ekf.c (ekf_m7_mag)
  1) 新增机体系残差：b̂ = R(q)ᵀ·b̂_n，r = (I − b̂b̂ᵀ)(m̂ − b̂)，H = −e_iᵀ[b̂]×，e1,e2 ⊥ b̂
     ⇒ H·b̂ ≡ 0 结构成立（绕磁力线无观测），不再用 R 放大去压投影耦合
  2) 删除 "倾角参考失效 > V5F_EKF_MAG_GT_HOLD_S 就 hold"（运动中禁磁）
  3) 删除 s_mag_tilt_bad_s（声明 + 累计块），它只为那条 hold 存在
  合法性只剩：外门 gate->ekf_mag_yaw（= |mag_norm−1| < V5F_MAG_ERR_LIM，只看模）
              + 残差上限 R_MAX + 同一磁样本不重复更新

用法: python tools/ekf_session/patch_v103_magvec.py
回退: V5F/User/{inc/v5f_tune.h,src/proc_ekf.c}.bak_v103_magvec
"""
import hashlib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B          # 备份唯一合法去处：<repo>/bak_src/...

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
BAK = '.bak_v103_magvec'


def g(s):
    return s.encode('gbk')


def patch(path, pairs):
    src = open(path, 'rb').read()
    if not B.exists(path, BAK):
        B.save(path, BAK)
    out = src
    for old, new, cnt in pairs:
        n = out.count(old)
        if n != cnt:
            print('FAIL %s: anchor x%d (want %d): %r' % (os.path.basename(path), n, cnt, old[:60]))
            return None
        out = out.replace(old, new)
    open(path, 'wb').write(out)
    print('patched %-12s %d -> %d B  (backup %s)' % (os.path.basename(path), len(src), len(out), BAK))
    return out


# ============================ v5f_tune.h ============================
tune_new = g(
    '/* ---- VER=103 地磁观测：真牵引（向量量测）+ 取消运动中禁磁 ------------------\n'
    ' * V5F_EKF_MAG_MODE:\n'
    ' *   0u = 旧标量牵引（把实测/预测场都投影到 ⊥a_up 平面后比航向角，H 只有 dx[8]=1）\n'
    ' *   1u = 真牵引：正牌 2 维量测更新\n'
    ' *        b^_b = R(q)^T b^_n                      预测的机体系磁场方向\n'
    ' *        r    = (I - b^ b^T)(m^_b - b^_b)        残差落在 ⊥b^ 平面内，不做姿态投影\n'
    ' *        H    = -e_i^T [b^]x,  e1,e2 ⊥ b^\n'
    ' *        => H·b^ ≡ 0 是**结构**性质：绕磁力线无观测，不再靠 R 放大人为压\n'
    ' * 合法性只看模：|mag_norm - 1| < V5F_MAG_ERR_LIM（外门 gate->ekf_mag_yaw 已是这个条件）。\n'
    ' * VER=103 起删除"运动中禁磁"（原 V5F_EKF_MAG_GT_HOLD_S 那条 hold）：那条是为\n'
    ' * "姿态误差 -> 重力法平面投影误差 -> 地磁误差"设的，是**投影式**观测的产物；\n'
    ' * 真牵引在机体系做差没有这个耦合，所以只要模合法就实时牵引。\n'
    ' * σ 依据本机实测：标定后地磁方向误差 p50 0.5 / p90 1.3 deg（见 docs/mag360_recording_protocol.md 6） */\n'
    '#define V5F_EKF_MAG_MODE          1u\n'
    '#define V5F_EKF_MAG_VEC_SIG_DEG   0.9f\n'
    '#define V5F_EKF_MAG_VEC_K_MAX     0.10f\n'
)

tune = patch(TUNE, [
    (b'#define V5F_FW_VER        102u', b'#define V5F_FW_VER        103u', 1),
    (g('#define V5F_EKF_MAG_GT_HOLD_S     0.30f'),
     g('#define V5F_EKF_MAG_GT_HOLD_S     0.30f   /* VER=103 已弃用：那条运动中禁磁的 hold 删了 */\n'
       + tune_new.decode('gbk')), 1),
])
if tune is None:
    sys.exit(1)

# ============================ proc_ekf.c ============================
# 1) 局部变量声明
decl_old = g('    float fp[3], mh2[3], crs4[3], b0v[3], up_nav[3];\n')
decl_new = decl_old + g('    float bb[3], e1[3], e2[3], r2v[2], RRv[4];   /* VER=103 真牵引用 */\n')

# 2) 删除 s_mag_tilt_bad_s 声明
decl2_old = g('static float    s_mag_tilt_bad_s;   /*')
decl2_new = g('/* static float s_mag_tilt_bad_s;  VER=103 删除：只服务于"运动中禁磁"那条 hold */\n/* static float    s_mag_tilt_bad_s;   /*')

# 3) 删除 tilt_bad 累计块
acc_old = g('''    {
        float amg2 = h->imu.accel_g[0]*h->imu.accel_g[0]
                   + h->imu.accel_g[1]*h->imu.accel_g[1]
                   + h->imu.accel_g[2]*h->imu.accel_g[2];
        uint8_t asat = 0u, kk;
        for (kk = 0u; kk < 3u; kk++) {
            int32_t av = (int32_t)h->imu.accel_lsb[kk];
            if (av >= 32000 || av <= -32000) asat = 1u;
        }
        if (gate->ekf_tilt
            || !(h->imu.acc_valid && !asat && (fabsf(amg2 - 1.0f) < V5F_EKF_TILT_AMAG_TOL))) {
            s_mag_tilt_bad_s = 0.0f;
        } else if (s_mag_tilt_bad_s < 10.0f) {
            s_mag_tilt_bad_s += dt;
        }
    }
''')
acc_new = g('''    /* VER=103: 原 s_mag_tilt_bad_s 累计块已删除（它只服务于"运动中禁磁"那条 hold）。 */
''')

# 4) 机体系残差（真牵引）计算块，插在共用门之前
mode1_compute = g('''    /* ---- VER=103 真牵引：机体系残差（不做任何姿态投影）-------------------- */
#if (V5F_EKF_MAG_MODE != 0u)
    {
        float n1, c1, sg2, d0, d1, d2, pr0, pr1, pr2, Sx[3][3];
        uint32_t ax2;
        bb[0] = Rt[0][0]*b0x + Rt[1][0]*b0y + Rt[2][0]*b0z;
        bb[1] = Rt[0][1]*b0x + Rt[1][1]*b0y + Rt[2][1]*b0z;
        bb[2] = Rt[0][2]*b0x + Rt[1][2]*b0y + Rt[2][2]*b0z;
        n1 = sqrtf(bb[0]*bb[0] + bb[1]*bb[1] + bb[2]*bb[2]);
        if (n1 < 1e-6f) return;
        bb[0] /= n1; bb[1] /= n1; bb[2] /= n1;
        c1 = mf[0]*bb[0] + mf[1]*bb[1] + mf[2]*bb[2];
        d0 = mf[0] - bb[0]; d1 = mf[1] - bb[1]; d2 = mf[2] - bb[2];
        pr0 = d0 - c1*bb[0]; pr1 = d1 - c1*bb[1]; pr2 = d2 - c1*bb[2];
        /* ⊥b^ 平面正交基：e1 = normalize(z^ x b^)，e2 = b^ x e1 */
        e1[0] = -bb[1]; e1[1] = bb[0]; e1[2] = 0.0f;
        n1 = sqrtf(e1[0]*e1[0] + e1[1]*e1[1]);
        if (n1 < 1e-4f) { e1[0] = 1.0f; e1[1] = 0.0f; e1[2] = 0.0f; }
        else { e1[0] /= n1; e1[1] /= n1; e1[2] /= n1; }
        e2[0] = bb[1]*e1[2] - bb[2]*e1[1];
        e2[1] = bb[2]*e1[0] - bb[0]*e1[2];
        e2[2] = bb[0]*e1[1] - bb[1]*e1[0];
        r2v[0] = e1[0]*pr0 + e1[1]*pr1 + e1[2]*pr2;
        r2v[1] = e2[0]*pr0 + e2[1]*pr1 + e2[2]*pr2;
        s_mag_r = sqrtf(r2v[0]*r2v[0] + r2v[1]*r2v[1]) * RAD2DEG;
        s_mag_rx = pr0; s_mag_ry = pr1;
        /* H 行 = -e_i^T [b^]x（作用在旋转矢量 dx[6..8] 上） */
        H_zero(2u);
        Sx[0][0] = 0.0f;   Sx[0][1] = -bb[2]; Sx[0][2] =  bb[1];
        Sx[1][0] = bb[2];  Sx[1][1] = 0.0f;   Sx[1][2] = -bb[0];
        Sx[2][0] = -bb[1]; Sx[2][1] = bb[0];  Sx[2][2] = 0.0f;
        for (ax2 = 0u; ax2 < 3u; ax2++) {
            s_H[0][IX_Q + ax2] = -(e1[0]*Sx[0][ax2] + e1[1]*Sx[1][ax2] + e1[2]*Sx[2][ax2]);
            s_H[1][IX_Q + ax2] = -(e2[0]*Sx[0][ax2] + e2[1]*Sx[1][ax2] + e2[2]*Sx[2][ax2]);
        }
        sg2 = V5F_EKF_MAG_VEC_SIG_DEG * DEG2RAD;
        sg2 = sg2 * sg2;
        RRv[0] = sg2; RRv[1] = 0.0f; RRv[2] = 0.0f; RRv[3] = sg2;
        s_mag_rs = 1.0f;                 /* 真牵引不再用 R 放大去压投影耦合 */
    }
#endif

''')

# 5) 尾巴：共用门 + 两种模式的更新（删掉运动禁磁）
tail_old = g('''    R[0] = sig2;
    /* ''')
i0 = None
src_now = open(EKF, 'rb').read()
p0 = src_now.find(g('    R[0] = sig2;'))
assert p0 > 0, 'R[0] = sig2 未找到'
p1 = src_now.find(g('\n}\n'), p0)
assert p1 > 0, 'M7 函数结束未找到'
tail_old_full = src_now[p0:p1]

tail_new = g('''    /* ---- VER=103 共用门：残差上限 + 同一磁样本不重复更新 -------------------
     * **不再有"运动中禁磁"**：合法性由外门给（gate->ekf_mag_yaw = |mag_norm-1| < MAG_ERR_LIM，只看模）；
     * 这里只剩残差太大(>R_MAX)拒一次、以及同一个磁样本不重复更新（磁 ~190Hz vs 帧 335Hz）。 */
    if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
    {
        uint32_t icm = g_shm ? g_shm->ist.hdr.cnt : 0u;
        if (icm == s_mag_cnt_upd) {
            s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
            return;
        }
        s_mag_cnt_upd = icm;
    }

#if (V5F_EKF_MAG_MODE == 0u)
    /* ---------------- 模式 0：旧标量牵引（保留回退） ---------------- */
    R[0] = sig2;
    if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) return;
    st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                    0x0100u, V5F_EKF_MAG_K_MAX);
    if (st == 0u) {
        s_gate_bits |= V5F_EKF_GB_MAG;
        s_mag_used = 1u;
        s_mag_dqx = s_dx[IX_Q + 0] * RAD2DEG;
        s_mag_dqy = s_dx[IX_Q + 1] * RAD2DEG;
        s_mag_dqz = s_dx[IX_Q + 2] * RAD2DEG;
    }
#else
    /* ---------------- 模式 1：真牵引（2 维向量量测） ---------------- */
    st = ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                    0x01C0u, V5F_EKF_MAG_VEC_K_MAX);
    if (st == 0u) {
        s_gate_bits |= V5F_EKF_GB_MAG;
        s_mag_used = 1u;
        s_mag_dqx = s_dx[IX_Q + 0] * RAD2DEG;
        s_mag_dqy = s_dx[IX_Q + 1] * RAD2DEG;
        s_mag_dqz = s_dx[IX_Q + 2] * RAD2DEG;
    } else {
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
    }
#endif
''')

ekf = patch(EKF, [
    (decl_old, decl_new, 1),
    (decl2_old, decl2_new, 1),
    (acc_old, acc_new, 1),
    (g('    R[0] = sig2;'), mode1_compute + g('    R[0] = sig2;'), 1),
    (tail_old_full, tail_new, 1),
])
if ekf is None:
    sys.exit(1)

# ============================ 复核 ============================
txt = open(EKF, 'rb').read().decode('gbk', errors='replace')
tun = open(TUNE, 'rb').read().decode('gbk', errors='replace')
checks = [
    ('tune VER=103', '#define V5F_FW_VER        103u' in tun),
    ('tune MODE', '#define V5F_EKF_MAG_MODE          1u' in tun),
    ('ekf 无 s_mag_tilt_bad_s 使用', 's_mag_tilt_bad_s' not in txt.replace('/* static float    s_mag_tilt_bad_s', '')),
    ('ekf 无运动禁磁 hold', 'V5F_EKF_MAG_GT_HOLD_S' not in txt),
    ('ekf 有向量残差', 'r = (I - b^ b^T)(m^_b - b^_b)' in txt or 'r2v[0] = e1[0]*pr0' in txt),
    ('ekf 有 H 行', 's_H[0][IX_Q + ax2]' in txt),
    ('ekf inj_mask 0x01C0', '0x01C0u' in txt),
    ('uk 括号平衡', txt.count('{') - txt.count('}') == 0 and txt.count('(') - txt.count(')') == 0),
]
for n, ok in checks:
    print('  %-28s %s' % (n, 'OK' if ok else 'FAIL'))
print('VERIFY', 'OK' if all(o for _, o in checks) else 'FAIL')
print('md5 proc_ekf.c', hashlib.md5(open(EKF, 'rb').read()).hexdigest()[:8],
      ' v5f_tune.h', hashlib.md5(open(TUNE, 'rb').read()).hexdigest()[:8])
sys.exit(0 if all(o for _, o in checks) else 1)
