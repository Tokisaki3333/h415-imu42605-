# -*- coding: utf-8 -*-
"""VER=24 -> 25：① 加 P_zz/P_bb 两列（115 列）② 气压改**差分为观测**（高通）。

②的理由：实测桌面等高段 press_avg(W=32) 段内 sigma=0.81~1.57 Pa(7~13 cm)，
但段间慢漂 ±4 Pa（60 s 峰-峰 73 cm）—— 20 cm 抬高只有 2.4 Pa，被慢漂淹没。
差值观测 z = h_baro(t) - h_baro(t-4s)、预测 = p_z(t) - p_z(t-4s)：
  * 慢漂在差值里抵消；4 s 内的漂移只有几 cm
  * H 只挂 δp_z，**不含 b_baro** -> 绝对基准只由 GPS 高度 M2 提供（没有 GPS 时
    b_baro 不可观测，就让它待着）
  * R = 2sigma_baro^2 + 老估计的方差（老样本当成已知量，把它的不确定度算进 R）
"""
import re
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
H = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\SPI_rx.h'
S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

# ---------------- v5f_tune.h ----------------
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_BARO_PERIOD_S    0.164f'
assert u.count(a) == 1
u = u.replace(a, a + """
/* 气压改为**差值观测**（高通）：只负责补两次绝对基准之间的瞬态。
 * 实测等高段 press_avg(W=32) 段内 sigma=0.81~1.57 Pa(7~13 cm)，但段间慢漂 ±4 Pa
 * （60 s 峰-峰 73 cm），而 20 cm 抬高只有 2.4 Pa —— 绝对观测会被慢漂淹掉。
 * 差值窗口 = 尝试次数 N / 6.1 Hz，取 24 -> 约 3.9 s。 */
#define V5F_EKF_BARO_HP_N        24u
#define V5F_EKF_BARO_OLD_VAR_M2  0.09f   /* 老 p_z 估计的方差容差（0.3 m）^2，算进 R */""", 1)
assert u.count('#define V5F_FW_VER        24u') == 1
u = u.replace('#define V5F_FW_VER        24u', '#define V5F_FW_VER        25u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1r')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: BARO_HP_N + OLD_VAR + VER 25')

# ---------------- SPI_rx.h ----------------
h = open(H, 'rb').read().decode('gbk')
a = "    float            b_baro;          /* 气压高度零偏 m；气压高度 = p_z + b_baro */"
assert h.count(a) == 1
h = h.replace(a, a + """
    float            pzz;             /* P[2][2]：垂直位置的方差 m^2（诊断气压观测的分配）*/
    float            pbb;             /* P[15][15]：b_baro 的方差 m^2（同上）*/""", 1)
assert h.count('/*') == h.count('*/')
shutil.copy2(H, H + '.bak_s1r')
open(H, 'wb').write(h.encode('gbk'))
print('SPI_rx.h: pzz/pbb 字段')

# ---------------- proc_ekf.c ----------------
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


sub("static uint8_t  s_mn_seen;",
    """static uint8_t  s_mn_seen;
/* 气压差值观测的环形缓冲：存 (h_baro, p_z) 的历史，取最老的一笔做差分 */
static float    s_bh_ring[V5F_EKF_BARO_HP_N];
static float    s_bp_ring[V5F_EKF_BARO_HP_N];
static uint8_t  s_bh_idx;
static uint8_t  s_bh_fill;""", 'ring')

sub("""static void ekf_m3_baro(const volatile v5f_proc_gate_t *gate)
{
    float R[1], r[1];
    uint8_t st;

    if (!gate->ekf_baro || !s_baro_new) return;
    H_zero(1u);
    s_H[0][2]  = 1.0f;         /* d/dp_z */
    s_H[0][15] = 1.0f;         /* d/db_baro（气压高度 = p_z + b_baro） */
    R[0] = V5F_EKF_BARO_SIG_M * V5F_EKF_BARO_SIG_M;
    r[0] = s_baro_h - (s_x[IX_P + 2] + s_x[IX_BB]);
    st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);
    if (st == 0u) s_gate_bits |= V5F_EKF_GB_BARO;
}""",
    """static void ekf_m3_baro(const volatile v5f_proc_gate_t *gate)
{
    float R[1], r[1], bnow, pnow, bold, pold;
    uint8_t st;

    if (!gate->ekf_baro || !s_baro_new) return;
    bnow = s_baro_h;
    pnow = s_x[IX_P + 2];
    if (s_bh_fill >= (uint8_t)V5F_EKF_BARO_HP_N) {
        /* 差值观测：z = 气压高度在窗口内的变化，预测 = p_z 在窗口内的变化。
         * 慢漂在差值里抵消；H 只挂 δp_z（**不含 b_baro**），所以绝对基准只由
         * GPS 高度 M2 提供 —— 这正是"气压只补瞬态"的意思。 */
        bold = s_bh_ring[s_bh_idx];
        pold = s_bp_ring[s_bh_idx];
        H_zero(1u);
        s_H[0][2] = 1.0f;
        R[0] = 2.0f * V5F_EKF_BARO_SIG_M * V5F_EKF_BARO_SIG_M + V5F_EKF_BARO_OLD_VAR_M2;
        r[0] = (bnow - bold) - (pnow - pold);
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_BARO;
    }
    /* 入环（无论本次是否更新，环都要走，保证窗口恒定） */
    s_bh_ring[s_bh_idx] = bnow;
    s_bp_ring[s_bh_idx] = pnow;
    s_bh_idx = (uint8_t)((s_bh_idx + 1u) % (uint8_t)V5F_EKF_BARO_HP_N);
    if (s_bh_fill < (uint8_t)V5F_EKF_BARO_HP_N) s_bh_fill++;
}""", 'm3')

sub("    h->ekf.b_baro = s_x[IX_BB];",
    """    h->ekf.b_baro = s_x[IX_BB];
    h->ekf.pzz = s_Pn[2][2];              /* 诊断：垂直位置方差 */
    h->ekf.pbb = s_Pn[15][15];            /* 诊断：b_baro 方差 */""", 'pub')
sub("""            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;""",
    """            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;
            s_bh_idx = 0u; s_bh_fill = 0u;""", 'clr')

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1r')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 气压差值观测 + pzz/pbb')

# ---------------- SPI_rx.c：+2 列 ----------------
s = open(S, 'rb').read().decode('gbk')
s = re.sub(r'#define JF_CH_NUM\s+113u', '#define JF_CH_NUM     115u', s, count=1)
s = re.sub(r'/\* 458 = 帧头2\+帧长2\+载荷452\+帧尾2 \*/',
           '/* 466 = 帧头2+帧长2+载荷460+帧尾2 */', s, count=1)
a = "    for (i = 0u; i < 5u; i++) ch[c++] = g_v5f_hold.ekf.nis[i];\n"
assert s.count(a) == 1, s.count(a)
s = s.replace(a, a + """    ch[c++] = g_v5f_hold.ekf.pzz;                  /* 垂直位置方差 m^2（诊断）*/
    ch[c++] = g_v5f_hold.ekf.pbb;                  /* b_baro 方差 m^2（诊断）*/
""", 1)
assert s.count('/*') == s.count('*/')
shutil.copy2(S, S + '.bak_s1r')
open(S, 'wb').write(s.encode('gbk'))
print('SPI_rx.c: 115 列 + pzz/pbb 上报')

# ---------------- 工具表 ----------------
CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
c = re.sub(r'assert n == 113,', 'assert n == 115,', c, count=1)
open(CP, 'w', encoding='utf-8', newline='\n').write(c)
JL = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()
a = 'CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113}'
assert j.count(a) == 1
j = j.replace(a, """# ---- VER=25：115 列（末尾追加 ekf_pzz / ekf_pbb 两个诊断列）----
CH_115 = dict(CH_113)
CH_115.update({'ekf_pzz': 113, 'ekf_pbb': 114})

CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113,
             115: CH_115}""", 1)
j = j.replace('CH = CH_113', 'CH = CH_115', 1)
j = j.replace("""    if nch == 113:
        return CH_113""", """    if nch == 113:
        return CH_113
    if nch == 115:
        return CH_115""", 1)
open(JL, 'w', encoding='utf-8', newline='\n').write(j)
print('count_cols/jf_load: 115 列')
chk = open(P, 'rb').read().decode('gbk')
for k, v in [('差值观测', 'bnow - bold' in chk), ('H 不含 b_baro', 's_H[0][15]' not in chk),
             ('环形缓冲', 's_bh_ring' in chk)]:
    print('  %-16s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((25 << 16) | (115 << 8) | 1 | 2 | 4))
