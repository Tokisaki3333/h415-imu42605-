# -*- coding: utf-8 -*-
"""VER=25 -> 26：三条一起做
 1) s_baro_new 只在**真的尝试了观测**时才清（原来每次 stage 19 都清 -> 气压被饿到 0.34%）
 2) 加两列 P0 自检：把 s_P[2][2]/s_P[15][15] 与 s_Pn 的两个一起上报 ->
    一眼看出 P0 到底有没有落进 s_P、传播有没有把它搬到 s_Pn
 3) 依 2 的结论修 P_bb=0 的问题
写入一律"先 encode 再开文件"（上一个脚本把 v5f_tune.h 清零的教训）。
"""
import os
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
T = R + r'\V5F\User\inc\v5f_tune.h'
H = R + r'\V5F\User\inc\SPI_rx.h'
P = R + r'\V5F\User\src\proc_ekf.c'
S = R + r'\V5F\User\src\SPI_rx.c'


def sw(path, text, enc, tag):
    data = text.encode(enc)            # 先编码：失败则文件不动
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


# ---------- 1) s_baro_new 清零位置 ----------
t = open(P, 'rb').read().decode('gbk')
a = """        case 19u:
            /* 气压按设计降到 6.1 Hz：press_avg 的噪声相关时间就是 W=32 后的 165 ms，
             * 原来每个周期（最多 187 Hz）都喂是 58 倍冗余 —— 那会让 R=(0.13 m)^2 名不副实，
             * 也让它 0.19 的 NIS 完全失去意义。降频后相邻观测才近似独立。 */
            if (s_prop_ok && s_baro_wait >= V5F_EKF_BARO_PERIOD_S) {
                ekf_m3_baro(gate);
                s_baro_wait = 0.0f;
            }
            if (s_prop_ok) ekf_m7_mag(h, gate);
            s_baro_new = 0u;                   /* 号志在被消费的这一帧才清 */
            break;"""
assert t.count(a) == 1, t.count(a)
b = """        case 19u:
            /* 气压按设计降到 6.1 Hz（press_avg 的噪声相关时间就是 W=32 后的 165 ms）。
             * ★ 号志必须**只在真的尝试了观测时**才清。原来无条件在每次 stage 19 清，
             *   而 M3 只在抽取门允许时才执行 -> 绝大多数周期把新样本白清了，
             *   真正轮到 M3 时号志早已是 0。实测 baro 门只剩 0.34%（应约 1.75%），
             *   而 40 cm 抬高在气压里明确可见（h_baro 5 s 差分 +0.477 m）、
             *   p_z 却只动了 +0.111 m —— 观测被饿死，不是观测不准。 */
            if (s_prop_ok && s_baro_wait >= V5F_EKF_BARO_PERIOD_S) {
                s_baro_wait = 0.0f;
                s_baro_new = 0u;               /* 消费掉 */
                ekf_m3_baro(gate);
            }
            if (s_prop_ok) ekf_m7_mag(h, gate);
            break;"""
t = t.replace(a, b, 1)

# ---------- 2) P0 / s_P 自检列 ----------
a = """    h->ekf.pzz = s_Pn[2][2];              /* 诊断：垂直位置方差 */
    h->ekf.pbb = s_Pn[15][15];            /* 诊断：b_baro 方差 */"""
assert t.count(a) == 1
t = t.replace(a, """    /* 诊断四件套：s_Pn 是"传播/更新后的工作协方差"，s_P 是"上周期末的"。对齐那一步
     * 只写了 s_P（s_Pn 还是 bss 的 0），所以两者一起报就能判定 P0 有没有落地、
     * 传播有没有把它搬过去。 */
    h->ekf.pzz    = s_Pn[2][2];
    h->ekf.pbb    = s_Pn[15][15];
    h->ekf.p_p_zz = s_P[2][2];
    h->ekf.p_p_bb = s_P[15][15];""", 1)

# 对齐那一帧：把 s_Pn 也写成 P0，避免首帧报 0 造成误判
a = """            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;"""
assert t.count(a) == 1
t = t.replace(a, a + """
            /* 对齐时 s_Pn 还是 bss 的 0；把它也置成 P0，免得首帧上报出一个假的 0 */
            for (i = 0u; i < EKF_N; i++) {
                for (j = 0u; j < EKF_N; j++) s_Pn[i][j] = s_P[i][j];
            }""", 1)
assert t.count('/*') == t.count('*/')
sw(P, t, 'gbk', 's1t')
print('proc_ekf.c: 号志清零位置 + P0 自检 + 对齐时同步 s_Pn')

# ---------- 3) 字段 ----------
h = open(H, 'rb').read().decode('gbk')
a = """    float            pbb;             /* P[15][15]：b_baro 的方差 m^2（同上）*/"""
assert h.count(a) == 1
h = h.replace(a, a + """
    float            p_p_zz;          /* s_P[2][2]（上周期末）—— 与 pzz 一起看可判定
                                       * P0 是否落地、传播是否搬运 */
    float            p_p_bb;          /* s_P[15][15]（同上）*/""", 1)
assert h.count('/*') == h.count('*/')
sw(H, h, 'gbk', 's1t')
print('SPI_rx.h: p_p_zz/p_p_bb')

# ---------- 4) 上报 +2 列 -> 117 ----------
s = open(S, 'rb').read().decode('gbk')
s = re.sub(r'#define JF_CH_NUM\s+115u', '#define JF_CH_NUM     117u', s, count=1)
s = re.sub(r'/\* 466 = 帧头2\+帧长2\+载荷460\+帧尾2 \*/',
           '/* 474 = 帧头2+帧长2+载荷468+帧尾2 */', s, count=1)
a = "    ch[c++] = g_v5f_hold.ekf.pbb;                  /* b_baro 方差 m^2（诊断）*/\n"
assert s.count(a) == 1
s = s.replace(a, a + """    ch[c++] = g_v5f_hold.ekf.p_p_zz;               /* s_P[2][2]（P0 自检）*/
    ch[c++] = g_v5f_hold.ekf.p_p_bb;               /* s_P[15][15]（P0 自检）*/
""", 1)
assert s.count('/*') == s.count('*/')
sw(S, s, 'gbk', 's1t')
print('SPI_rx.c: 117 列')

# ---------- 5) 版本 + 工具表 ----------
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        25u') == 1
u = u.replace('#define V5F_FW_VER        25u', '#define V5F_FW_VER        26u', 1)
sw(T, u, 'gbk', 's1t')
CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
c = re.sub(r'assert n == 115,', 'assert n == 117,', c, count=1)
open(CP, 'w', encoding='utf-8', newline='\n').write(c)
JL = R + r'\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()
a = "CH_115.update({'ekf_pzz': 113, 'ekf_pbb': 114})"
assert j.count(a) == 1
j = j.replace(a, """CH_115.update({'ekf_pzz': 113, 'ekf_pbb': 114})

# ---- VER=26：117 列（再追加 ekf_p_pzz / ekf_p_pbb 两个 P0 自检列）----
CH_117 = dict(CH_115)
CH_117.update({'ekf_p_pzz': 115, 'ekf_p_pbb': 116})""", 1)
j = j.replace('115: CH_115}', '115: CH_115, 117: CH_117}', 1)
j = j.replace('CH = CH_115', 'CH = CH_117', 1)
j = j.replace("""    if nch == 115:
        return CH_115""", """    if nch == 115:
        return CH_115
    if nch == 117:
        return CH_117""", 1)
open(JL, 'w', encoding='utf-8', newline='\n').write(j)
print('count_cols/jf_load: 117 列')

# ---------- 6) 回读 ----------
t2 = open(P, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
u2 = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', u2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
print()
for k, v in [('VER=26', ver == 26), ('NCH=117', nch == 117),
             ('号志在抽取分支内清', 's_baro_new = 0u;               /* 消费掉 */' in t2),
             ('P0 自检上传', 'ekf.p_p_zz' in s2),
             ('对齐同步 s_Pn', 's_Pn[i][j] = s_P[i][j]' in t2),
             ('注释配平', t2.count('/*') == t2.count('*/') and s2.count('/*') == s2.count('*/'))]:
    print('  %-20s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
