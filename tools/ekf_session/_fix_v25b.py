# -*- coding: utf-8 -*-
"""恢复 v5f_tune.h 并重做 VER=25。
★ 教训（这次真的踩了）：`open(T,'wb').write(u.encode('gbk'))` 里 open 会**先截断**，
  然后才求值 encode —— 编码一抛异常，文件就变成 0 字节。
  正确顺序：先在内存里 encode（可能抛异常，文件没动）-> 备份 -> 再 open+write。
  本脚本所有写入都遵守这个顺序，并且在写之前统一做一次 GBK 可编码检查。"""
import os
import re
import shutil

INC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc'
T = INC + r'\v5f_tune.h'
H = INC + r'\SPI_rx.h'
P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'


def safe_write(path, text, enc, tag):
    data = text.encode(enc)            # 先编码：失败则文件不动
    assert all(ord(c) < 128 or True for c in text)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


# ---------- 1) 恢复 ----------
bak = T + '.bak_s1r'
t = open(bak, 'rb').read().decode('gbk')
assert '#define V5F_FW_VER        24u' in t and 'BARO_HP_N' not in t
safe_write(T, t, 'gbk', 's1s_restore')
print('已恢复 v5f_tune.h (%d B, VER=24)' % len(t))

# ---------- 2) v5f_tune.h 加常量 ----------
a = '#define V5F_EKF_BARO_PERIOD_S    0.164f'
assert t.count(a) == 1
t = t.replace(a, a + """
/* 气压改为**差值观测**（高通）：只负责补两次绝对基准之间的瞬态。
 * 实测等高段 press_avg(W=32) 段内 sigma=0.81~1.57 Pa(7~13 cm)，但段间慢漂 ±4 Pa
 * （60 s 峰-峰 73 cm），而 20 cm 抬高只有 2.4 Pa —— 绝对观测会被慢漂淹掉。
 * 差值窗口 = 尝试次数 N / 6.1 Hz，取 24 -> 约 3.9 s。 */
#define V5F_EKF_BARO_HP_N        24u
#define V5F_EKF_BARO_OLD_VAR_M2  0.09f   /* 老 p_z 估计的方差容差 (0.3 m)^2，算进 R */""", 1)
assert t.count('#define V5F_FW_VER        24u') == 1
t = t.replace('#define V5F_FW_VER        24u', '#define V5F_FW_VER        25u', 1)
assert t.count('/*') == t.count('*/')
safe_write(T, t, 'gbk', 's1s')
print('v5f_tune.h: BARO_HP_N + VER 25  (+%d B)' % (len(t) - len(open(bak, 'rb').read().decode('gbk'))))

# ---------- 3) SPI_rx.h ----------
h = open(H, 'rb').read().decode('gbk')
a = "    float            b_baro;          /* 气压高度零偏 m；气压高度 = p_z + b_baro */"
assert h.count(a) == 1
h = h.replace(a, a + """
    float            pzz;             /* P[2][2]：垂直位置的方差 m^2（诊断气压观测的分配）*/
    float            pbb;             /* P[15][15]：b_baro 的方差 m^2（同上）*/""", 1)
assert h.count('/*') == h.count('*/')
safe_write(H, h, 'gbk', 's1s')
print('SPI_rx.h: pzz/pbb')

# ---------- 4) proc_ekf.c ----------
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


sub("static uint8_t  s_mn_seen;",
    """static uint8_t  s_mn_seen;
/* 气压差值观测的环形缓冲：存 (h_baro, p_z) 历史，取最老的一笔做差分 */
static float    s_bh_ring[V5F_EKF_BARO_HP_N];
static float    s_bp_ring[V5F_EKF_BARO_HP_N];
static uint8_t  s_bh_idx;
static uint8_t  s_bh_fill;""", 'ring')

sub("""    float R[1], r[1];
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
    """    float R[1], r[1], bnow, pnow, bold, pold;
    uint8_t st;

    if (!gate->ekf_baro || !s_baro_new) return;
    bnow = s_baro_h;
    pnow = s_x[IX_P + 2];
    if (s_bh_fill >= (uint8_t)V5F_EKF_BARO_HP_N) {
        /* 差值观测：z = 气压高度在窗口内的变化，预测 = p_z 在窗口内的变化。
         * 慢漂在差值里抵消；H 只挂 dp_z（**不含 b_baro**），所以绝对基准只由
         * GPS 高度 M2 提供 —— 这正是"气压只补瞬态"的意思。 */
        bold = s_bh_ring[s_bh_idx];
        pold = s_bp_ring[s_bh_idx];
        H_zero(1u);
        s_H[0][2] = 1.0f;
        R[0] = 2.0f * V5F_EKF_BARO_SIG_M * V5F_EKF_BARO_SIG_M
             + V5F_EKF_BARO_OLD_VAR_M2;
        r[0] = (bnow - bold) - (pnow - pold);
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_BARO;
    }
    /* 入环（无论本次是否更新都要走，保证窗口长度恒定） */
    s_bh_ring[s_bh_idx] = bnow;
    s_bp_ring[s_bh_idx] = pnow;
    s_bh_idx = (uint8_t)((s_bh_idx + 1u) % (uint8_t)V5F_EKF_BARO_HP_N);
    if (s_bh_fill < (uint8_t)V5F_EKF_BARO_HP_N) s_bh_fill++;
}""", 'm3')

sub("    h->ekf.b_baro = s_x[IX_BB];",
    """    h->ekf.b_baro = s_x[IX_BB];
    h->ekf.pzz = s_Pn[2][2];              /* 诊断：垂直位置方差 */
    h->ekf.pbb = s_Pn[15][15];            /* 诊断：b_baro 方差 */""", 'pub')
sub("            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;",
    """            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;
            s_bh_idx = 0u; s_bh_fill = 0u;""", 'clr')
assert t.count('/*') == t.count('*/')
safe_write(P, t, 'gbk', 's1s')
print('proc_ekf.c: 差值观测 + pzz/pbb')

# ---------- 5) SPI_rx.c ----------
s = open(S, 'rb').read().decode('gbk')
s = re.sub(r'#define JF_CH_NUM\s+113u', '#define JF_CH_NUM     115u', s, count=1)
s = re.sub(r'/\* 458 = 帧头2\+帧长2\+载荷452\+帧尾2 \*/',
           '/* 466 = 帧头2+帧长2+载荷460+帧尾2 */', s, count=1)
a = "    for (i = 0u; i < 5u; i++) ch[c++] = g_v5f_hold.ekf.nis[i];\n"
assert s.count(a) == 1
s = s.replace(a, a + """    ch[c++] = g_v5f_hold.ekf.pzz;                  /* 垂直位置方差 m^2（诊断）*/
    ch[c++] = g_v5f_hold.ekf.pbb;                  /* b_baro 方差 m^2（诊断）*/
""", 1)
assert s.count('/*') == s.count('*/')
safe_write(S, s, 'gbk', 's1s')
print('SPI_rx.c: 115 列 + pzz/pbb')

# ---------- 6) 工具表 ----------
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

# ---------- 7) 回读校验 ----------
t2 = open(T, 'rb').read().decode('gbk')
p2 = open(P, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
print()
for k, v in [('v5f_tune.h 非空', len(t2) > 40000),
             ('VER=25', ver == 25), ('NCH=115', nch == 115),
             ('差值观测在', 'bnow - bold' in p2),
             ('H 不含 b_baro', 's_H[0][15]' not in p2),
             ('环形缓冲', 's_bh_ring' in p2),
             ('上报 pzz/pbb', 'ekf.pzz' in s2)]:
    print('  %-18s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
