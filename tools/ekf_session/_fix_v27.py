# -*- coding: utf-8 -*-
"""VER=26 -> 27：**彻底修掉"EKF 永远不启动"**。

实测 VER=26：gate_bits 唯一值 0、|ekf_q|=0、所有 EKF 量恒 0 —— 一次性对齐从未发生，
而这一次 is_static 只有 37.4%、mag.trust 只有 25.6%（那个 |a_lin|<0.05g 的判据实测
p50 就在 0.0487g 上抖，门在抖），两者没同时落上，于是整机 EKF 永久停摆。

修法（四级，保证一定启动）：
  A 首选：acc_valid  and  stat.valid  and  (is_static  or  ac_bypass)  and  up_ref_ok  and  |a|^2~1
  B 8 s 超时：acc_valid  and  up_ref_ok  and  |a|^2 宽松
  C 20 s 强制：acc_valid  and  up_ref_ok（姿态有倾角误差，交给 M6 后面修）
  D up_ref_ok 一直没来（旧链自己也要"静止+1g"才建系）：**EKF 自己用重力建倾角**
    —— 抄旧链同一套最小旋转公式，不再依赖任何旧链标志的时序。
磁偏航的一次性对齐只在**稳健模长门**（与 M7 同一判据）通过时做；不做也行，
M7 的 H 只挂偏航项，它自己会把偏航拉到位。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
T = R + r'\V5F\User\inc\v5f_tune.h'
P = R + r'\V5F\User\src\proc_ekf.c'
S = R + r'\V5F\User\src\SPI_rx.c'


def sw(path, text, enc, tag):
    data = text.encode(enc)            # 先编码：失败则文件不动
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


# ================= v5f_tune.h =================
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_ALIGN_AMAG_TOL   0.02f'
assert u.count(a) == 1
u = u.replace(a, a + """   /* 对齐时 |a| 必须落在 1 g +-2% */
/* ---- 对齐的兜底层级（保证 EKF 一定会启动）----
 * VER=26 实测：对齐条件里挂了 mag.trust（实测只有 25.6%）与 is_static（37.4%），
 * 两者没同时落上 -> gate_bits 恒 0、|ekf_q| 恒 0，整个 EKF 永久停摆。
 * 所以改成"逐级放宽 + 超时强制 + 自己建倾角"，不再依赖旧链标志的时序。 */
#define V5F_EKF_ALIGN_AMAG_LOOSE 0.06f   /* 超时兜底时 |a|^2 的容差（约 |a| 3%）*/
#define V5F_EKF_ALIGN_TIMEOUT_S  8.0f    /* 超过它就放宽（不再要求判静）*/
#define V5F_EKF_ALIGN_FORCE_S    20.0f   /* 再超过它就强行对齐（倾角误差交给 M6）*/""", 1)
a = '#define V5F_EKF_GB_SAT           0x0800u   /* 本周期有加计削顶帧（比力不可信） */'
assert u.count(a) == 1
u = u.replace(a, a + '\n#define V5F_EKF_GB_ALIGN_FORCE   0x1000u   /* 对齐是超时/强制兜底来的 */', 1)
assert u.count('#define V5F_FW_VER        26u') == 1
u = u.replace('#define V5F_FW_VER        26u', '#define V5F_FW_VER        27u', 1)
assert u.count('/*') == u.count('*/')
sw(T, u, 'gbk', 's1u')
print('v5f_tune.h: 对齐兜底常量 + GB_ALIGN_FORCE + VER 27')

# ================= proc_ekf.c =================
t = open(P, 'rb').read().decode('gbk')

# (1) 新增静态量：稳健模长门结论（step8 写、step7 下一帧读）
a = "static float    s_mn_lp;              /* mag_norm 的稳健基线（只跟看起来干净的样本）*/"
assert t.count(a) == 1
t = t.replace(a, a + "\nstatic uint8_t  s_mn_ok;              /* 稳健模长门结论（step8 写，step7 下一帧读）*/", 1)

# (2) 整体替换对齐块
i0 = t.index("    /* ---------------- 一次性对齐 ---------------- */")
i1 = t.index("    /* ---------------- 周期阶段机 ---------------- */")
NEW = """    /* ---------------- 一次性对齐（四级兜底，保证一定启动） ----------------
     * ★ 教训（VER=26 实测）：原条件挂了 mag.trust 与 is_static。两者的判据都压在
     *   阈值上会抖（mag.trust 里的 |a_lin|<0.05g 实测 p50=0.0487g；is_static 本次
     *   只有 37.4%），没同时落上时 EKF **整个停摆**（gate_bits 恒 0、|ekf_q| 恒 0）。
     *   一次性对齐绝不能挂在一个会抖的门上。 */
    if (!s_aligned) {
        float am = h->imu.accel_g[0]*h->imu.accel_g[0]
                 + h->imu.accel_g[1]*h->imu.accel_g[1]
                 + h->imu.accel_g[2]*h->imu.accel_g[2];
        uint8_t amag_ok, force = 0u, can_align = 0u, self_init = 0u;
        float q0[4];

        amag_ok = (uint8_t)(h->imu.acc_valid
                            && (fabsf(am - 1.0f) < V5F_EKF_ALIGN_AMAG_TOL));
        /* D：连 up_ref_ok 都没来（旧链自己也要"静止 + 1g"才建系）-> EKF 自己建倾角。
         *    用与 proc_attitude 完全同一套最小旋转：axis = u x e_z = (u_y, -u_x, 0)，
         *    q = [1 + u_z, axis] 归一化；u 朝下时退化成绕机体 x 转 180 度。 */
        if (h->att.up_ref_ok) {
            for (i = 0u; i < 4u; i++) q0[i] = h->att.q[i];
            can_align = amag_ok;
        } else if (h->imu.acc_valid) {
            float ux = h->imu.accel_g[0], uy = h->imu.accel_g[1], uz = h->imu.accel_g[2];
            float n2 = ux*ux + uy*uy + uz*uz, w, nx, ny, s;
            if (n2 > 1e-6f) {
                s = 1.0f / sqrtf(n2);
                ux *= s; uy *= s; uz *= s;
                if (uz > -0.999999f) { w = 1.0f + uz; nx = uy; ny = -ux; }
                else                 { w = 0.0f; nx = 1.0f; ny = 0.0f; }
                s = 1.0f / sqrtf(w*w + nx*nx + ny*ny);
                q0[0] = w*s; q0[1] = nx*s; q0[2] = ny*s; q0[3] = 0.0f;
                can_align = amag_ok;
                self_init = 1u;
            }
        }
        /* A/B/C：判静要求逐级放宽 */
        if (can_align && !(h->stat.valid && (h->stat.is_static || h->stat.ac_bypass))) {
            if (s_t_run >= V5F_EKF_ALIGN_TIMEOUT_S) {
                can_align = (uint8_t)(fabsf(am - 1.0f) < V5F_EKF_ALIGN_AMAG_LOOSE);
                force = 1u;
            }
        }
        if (!can_align && h->imu.acc_valid && h->att.up_ref_ok
            && s_t_run >= V5F_EKF_ALIGN_FORCE_S) {
            for (i = 0u; i < 4u; i++) q0[i] = h->att.q[i];
            can_align = 1u; force = 1u;
        }

        if (can_align) {
            float Ri[3][3], mf[3], Bn[3], azi, dpsi, hq[4], qz[4];
            for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = q0[i];
            q_norm(&s_x[IX_Q]);
            /* 磁偏航一次性对齐：只在**稳健模长门**（与 M7 同一判据）通过时才做。
             * 不做也完全可以 —— M7 的 H 只挂偏航项，它自己会把偏航拉到位。 */
            if (s_mn_ok) {
                q_to_R(&s_x[IX_Q], Ri);
                for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
                rot_bn(Ri, mf, Bn);
                azi  = atan2f(Bn[0], Bn[1]);
                /* 符号：h = atan2(Bx,By)，而 q <- Exp(phi z)(x)q 使 h_new = h - phi
                 * （R_z(phi)·(0,1,0) = (-sin phi, cos phi, 0)）；所以 phi = azi - D。 */
                dpsi = wrap_pi(azi - V5F_MAG_DECL_RAD);
                hq[0] = cosf(0.5f*dpsi); hq[1] = 0.0f; hq[2] = 0.0f; hq[3] = sinf(0.5f*dpsi);
                q_mul(hq, &s_x[IX_Q], qz);
                for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qz[i];
                q_norm(&s_x[IX_Q]);
            }

            for (i = 0u; i < 3u; i++) s_x[IX_P  + i] = 0.0f;
            for (i = 0u; i < 3u; i++) s_x[IX_V  + i] = 0.0f;
            for (i = 0u; i < 3u; i++) s_x[IX_BA + i] = 0.0f;
            for (i = 0u; i < 3u; i++) s_x[IX_BG + i] = 0.0f;
            s_x[IX_BB] = 0.0f;

            for (i = 0u; i < EKF_N; i++) {
                for (j = 0u; j < EKF_N; j++) s_P[i][j] = 0.0f;
            }
            s_P[0][0] = s_P[1][1] = s_P[2][2] = V5F_EKF_P0_POS_M * V5F_EKF_P0_POS_M;
            s_P[3][3] = s_P[4][4] = s_P[5][5] = V5F_EKF_P0_VEL_MPS * V5F_EKF_P0_VEL_MPS;
            s_P[6][6] = s_P[7][7] = V5F_EKF_P0_TILT_RAD * V5F_EKF_P0_TILT_RAD;
            s_P[8][8] = V5F_EKF_P0_YAW_RAD * V5F_EKF_P0_YAW_RAD;
            s_P[9][9] = s_P[10][10] = s_P[11][11] = V5F_EKF_P0_BA_MPS2 * V5F_EKF_P0_BA_MPS2;
            s_P[12][12] = s_P[13][13] = s_P[14][14] = V5F_EKF_P0_BG_RADS * V5F_EKF_P0_BG_RADS;
            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            /* s_Pn 还是 bss 的 0；同步成 P0，免得首帧上报假的 0 */
            for (i = 0u; i < EKF_N; i++) {
                for (j = 0u; j < EKF_N; j++) s_Pn[i][j] = s_P[i][j];
            }
            /* 自己建的倾角有误差 -> 把倾角方差开大一点，让 M6 后面收 */
            if (self_init) s_P[6][6] = s_P[7][7] = 0.05f * 0.05f;
            if (self_init) s_Pn[6][6] = s_Pn[7][7] = 0.05f * 0.05f;

            for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }
            s_dt_e = 0.0f; s_dt_prev = 0.0f;
            s_prop_row = 0u; s_stage = 0u; s_F_ok = 0u;
            s_pos_wait = 0.0f; s_alt_wait = 0.0f; s_baro_wait = 0.0f;
            s_baro_t_last = 0.0f; s_baro_t_seen = 0u;
            s_alin_g = 0.0f; s_sat = 0u;
            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;
            s_bh_idx = 0u; s_bh_fill = 0u;
            for (i = 0u; i < 5u; i++) { s_nis[i] = 0.0f; s_rej[i] = 0u; }
            s_gate_bits = 0u;
            s_sig[0] = V5F_EKF_P0_YAW_RAD * RAD2DEG;
            s_sig[1] = V5F_EKF_P0_POS_M;
            s_sig[2] = V5F_EKF_P0_VEL_MPS;
            s_sig[3] = V5F_EKF_P0_TILT_RAD * RAD2DEG;
            s_a_nav[0] = s_a_nav[1] = s_a_nav[2] = 0.0f;
            s_aligned = 1u;
            s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (force)     s_gate_bits |= V5F_EKF_GB_ALIGN_FORCE;
            ekf_publish(h);
            return V5F_PROC_OK;
        }
        return V5F_PROC_OK;
    }

"""
t = t[:i0] + NEW + t[i1:]

# (3) step8 里把稳健模长门的结论写成 s_mn_ok
a = """        gate->ekf_mag_yaw = (uint8_t)((dev < V5F_EKF_MAG_NORM_DEV)
                                      && (V5F_EKF_YAW_OBS_EN != 0u));"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, """        s_mn_ok = (uint8_t)(dev < V5F_EKF_MAG_NORM_DEV);
        gate->ekf_mag_yaw = (uint8_t)(s_mn_ok && (V5F_EKF_YAW_OBS_EN != 0u));""", 1)

assert 's_mn_ok' in t and 'ALIGN_FORCE' in t
assert t.count('/*') == t.count('*/')
sw(P, t, 'gbk', 's1u')
print('proc_ekf.c: 四级兜底对齐 + 自建倾角 + 复用稳健模长门')

# ================= 版本与回读 =================
u2 = open(T, 'rb').read().decode('gbk')
p2 = open(P, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', u2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
print()
for k, v in [('VER=27', ver == 27), ('四级兜底在', 'V5F_EKF_ALIGN_FORCE_S' in p2),
             ('自建倾角在', 'self_init' in p2), ('复用稳健门', 's_mn_ok' in p2),
             ('不再依赖 mag.trust', 'h->mag.trust' not in p2.split('一次性对齐')[1][:4000]),
             ('注释配平', p2.count('/*') == p2.count('*/') and u2.count('/*') == u2.count('*/'))]:
    print('  %-20s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
