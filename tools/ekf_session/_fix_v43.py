# -*- coding: utf-8 -*-
"""加 5 列 CDC 诊断，专门回答"地磁为什么没有牵引"：
  117 mag_gate   第 8 步算出的**外部门**状态（1 = 稳健模长门通过）
  118 mag_bh     |B 水平分量| / |B|（死点判据量；正常 0.59，<0.30 被死点门丢）
  119 mag_r_deg  M7 的新息 r（度）—— 恒 0 就说明观测根本没形成
  120 mag_used   1 = 本周期 M7 真的执行了更新
  121 p_yy       P[8][8]（偏航方差）—— 与 R 一比就能估出 K
★ 有这 5 列，"门没开 / 死点门丢 / 新息为 0 / 增益为 0" 四种情况就能一次分开。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
H = R + r'\V5F\User\inc\SPI_rx.h'
S = R + r'\V5F\User\src\SPI_rx.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


# ---------- SPI_rx.h ----------
h = open(H, 'rb').read().decode('gbk')
a = "    float            p_p_bb;          /* s_P[15][15]（同上）*/"
assert h.count(a) == 1
h = h.replace(a, a + """
    /* ↓ 磁偏航（M7）专项诊断：回答"地磁为什么没有牵引" */
    volatile uint8_t mag_gate;        /* 第 8 步算出的外部门状态（1 = 稳健模长门通过）*/
    uint8_t          mag_used;        /* 1 = 本周期 M7 真的执行了更新 */
    uint8_t          _rsv_m7[2];
    float            mag_bh;          /* |B 水平分量| / |B|（死点判据量，正常 0.59）*/
    float            mag_r_deg;       /* M7 新息（度）：恒 0 说明观测没形成 */
    float            p_yy;            /* P[8][8]：偏航方差，与 R 一比可估 K */""", 1)
assert h.count('/*') == h.count('*/')
sw(H, h, 'gbk', 's2n')
print('SPI_rx.h: 5 个诊断字段')

# ---------- proc_ekf.c ----------
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)
    print('  ok:', nm)


sub("static uint8_t  s_chi2_soft;",
    "static uint8_t  s_chi2_soft;\n"
    "static uint8_t  s_mag_gate;           /* 第 8 步的外部门结论（供上报）*/\n"
    "static uint8_t  s_mag_used;           /* 本周期 M7 是否真的更新了 */\n"
    "static float    s_mag_bh;             /* Bh/|B| 死点判据量 */\n"
    "static float    s_mag_r;              /* M7 新息（度）*/", 'vars')

# step8：记录门结论
sub("""        s_mn_ok = (uint8_t)(dev < V5F_EKF_MAG_NORM_DEV);
        gate->ekf_mag_yaw = (uint8_t)(s_mn_ok && (V5F_EKF_YAW_OBS_EN != 0u));""",
    """        s_mn_ok = (uint8_t)(dev < V5F_EKF_MAG_NORM_DEV);
        gate->ekf_mag_yaw = (uint8_t)(s_mn_ok && (V5F_EKF_YAW_OBS_EN != 0u));
        s_mag_gate = gate->ekf_mag_yaw;      /* 上报：门到底开没开 */""", 'gate')

# M7：记录 bh、r、used
sub("""    if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {""",
    """    s_mag_r = r[0] * RAD2DEG;
    if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {""", 'r')
sub("""        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                        0x0100u, V5F_EKF_MAG_K_MAX);
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;""",
    """        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                        0x0100u, V5F_EKF_MAG_K_MAX);
        if (st == 0u) { s_gate_bits |= V5F_EKF_GB_MAG; s_mag_used = 1u; }""", 'used')

# bh 记录（死点门那里）
sub("""        float bh2 = Bn[0]*Bn[0] + Bn[1]*Bn[1];""",
    """        float bh2 = Bn[0]*Bn[0] + Bn[1]*Bn[1];
        s_mag_bh = sqrtf(bh2);          /* |B| = 1（mag_f 是单位矢量）*/""", 'bh')

# 周期开始清 used
sub("""            s_chi2_soft = 0u;""",
    """            s_chi2_soft = 0u;
            s_mag_used = 0u;""", 'clr')

# publish
sub("""    h->ekf.p_p_bb = s_P[15][15];""",
    """    h->ekf.p_p_bb = s_P[15][15];
    h->ekf.mag_gate   = s_mag_gate;
    h->ekf.mag_used   = s_mag_used;
    h->ekf.mag_bh     = s_mag_bh;
    h->ekf.mag_r_deg  = s_mag_r;
    h->ekf.p_yy       = s_Pn[8][8];""", 'pub')

assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2n')
print('proc_ekf.c: 诊断量已采集')

# ---------- SPI_rx.c：+5 列 ----------
s = open(S, 'rb').read().decode('gbk')
s = re.sub(r'#define JF_CH_NUM\s+117u', '#define JF_CH_NUM     122u', s, count=1)
s = re.sub(r'/\* 474 = 帧头2\+帧长2\+载荷468\+帧尾2 \*/',
           '/* 494 = 帧头2+帧长2+载荷488+帧尾2 */', s, count=1)
a = "    ch[c++] = g_v5f_hold.ekf.p_p_bb;               /* s_P[15][15]（P0 自检）*/\n"
assert s.count(a) == 1
s = s.replace(a, a + """    /* 磁偏航（M7）专项诊断：门/死点/新息/增益一次看全 */
    ch[c++] = (float)g_v5f_hold.ekf.mag_gate;
    ch[c++] = g_v5f_hold.ekf.mag_bh;
    ch[c++] = g_v5f_hold.ekf.mag_r_deg;
    ch[c++] = (float)g_v5f_hold.ekf.mag_used;
    ch[c++] = g_v5f_hold.ekf.p_yy;
""", 1)
assert s.count('/*') == s.count('*/')
sw(S, s, 'gbk', 's2n')
print('SPI_rx.c: 122 列')

# ---------- 版本 + 表 ----------
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        42u') == 1
u = u.replace('#define V5F_FW_VER        42u', '#define V5F_FW_VER        43u', 1)
sw(T, u, 'gbk', 's2n')

CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
c = re.sub(r'assert n == 117,', 'assert n == 122,', c, count=1)
open(CP, 'w', encoding='utf-8', newline='\n').write(c)

JL = R + r'\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()
a = "CH_117.update({'ekf_p_pzz': 115, 'ekf_p_pbb': 116})"
assert j.count(a) == 1
j = j.replace(a, a + """

# ---- VER=43：122 列（末尾追加 5 个磁偏航专项诊断列）----
CH_122 = dict(CH_117)
CH_122.update({'ekf_mag_gate': 117, 'ekf_mag_bh': 118, 'ekf_mag_r_deg': 119,
               'ekf_mag_used': 120, 'ekf_p_yy': 121})""", 1)
j = j.replace('117: CH_117}', '117: CH_117, 122: CH_122}', 1)
j = j.replace('CH = CH_117', 'CH = CH_122', 1)
j = j.replace("""    if nch == 117:
        return CH_117""", """    if nch == 117:
        return CH_117
    if nch == 122:
        return CH_122""", 1)
open(JL, 'w', encoding='utf-8', newline='\n').write(j)

c2 = open(S, 'rb').read().decode('gbk')
p2 = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', c2).group(1))
print()
for k, v in [('VER=43', ver == 43), ('NCH=122', nch == 122),
             ('5 个采集点', all(x in p2 for x in ['s_mag_gate =', 's_mag_bh = sqrtf',
                                                  's_mag_r = r[0]', 's_mag_used = 1u',
                                                  'h->ekf.p_yy'])),
             ('上报 5 列', c2.count('ekf.mag_') == 3 and 'ekf.p_yy' in c2),
             ('注释配平', p2.count('/*') == p2.count('*/') and c2.count('/*') == c2.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
