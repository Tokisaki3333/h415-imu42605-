# -*- coding: utf-8 -*-
"""VER=27 -> 28：没有 GPS 高度时把 b_baro **彻底冻结**。

实测 VER=27：对齐正常（gate_bits 已对齐 100%、兜底 0%、P0 自检首帧 1/25/1/25 全对），
20 cm 台阶 p_z 3 s 差分 +0.204 m ✓；但 b_baro 漂到 **-116.2 m** 且 pbb 被压到 0。
机理：差分观测的 H 虽然不含 b_baro，但 ekf_update 里 K[15] = Pn[15][2]*Si ——
一旦 P[15][2] 被传播/更新弄成非零，b_baro 就被拖走，而 P 的更新又加深这个相关，
形成正反馈。b_baro 跑飞之后协方差被带坏，偏航环跟着失效
（EKF 总转角 -156.9 vs 旧链 +0.88，sigma_yaw 卡在 P0 的 3 度）。

用户的要求本来就是"气压只负责两次绝对基准之间的瞬态"，绝对基准由 GPS 高度 M2 给。
所以没有 M2 时 b_baro 应当**完全惰性**：每次差分更新后把它的值还原、
把 P 的第 15 行/列清零（对角保留 P0），让它既不被动也不动人。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
T = R + r'\V5F\User\inc\v5f_tune.h'
P = R + r'\V5F\User\src\proc_ekf.c'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
a = """        r[0] = (bnow - bold) - (pnow - pold);
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_BARO;
    }"""
assert t.count(a) == 1, t.count(a)
b = """        r[0] = (bnow - bold) - (pnow - pold);
        {
            float bb_save = s_x[IX_BB];       /* 更新前先存 b_baro */
            uint32_t k2;
            st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);
            /* ★ 没有 GPS 高度时 b_baro 必须**完全惰性**。
             *   差分观测的 H 不含 b_baro，但 ekf_update 里 K[15] = Pn[15][2]*Si：
             *   只要 P[15][2] 被传播/更新弄成非零，b_baro 就会被拖走，而 P 的更新
             *   又加深这个相关 -> 正反馈。实测 VER=27 漂到 -116.2 m，pbb 被压到 0，
             *   协方差被带坏后偏航环一起失效（总转角 -156.9 vs 旧链 +0.88）。
             *   "气压只补瞬态"就意味着：没有绝对基准时它既不被动、也不动人。 */
            s_x[IX_BB] = bb_save;
            for (k2 = 0u; k2 < EKF_N; k2++) {
                s_Pn[15][k2] = 0.0f;
                s_Pn[k2][15] = 0.0f;
            }
            s_Pn[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            for (k2 = 0u; k2 < EKF_N; k2++) {
                s_P[15][k2] = 0.0f;
                s_P[k2][15] = 0.0f;
            }
            s_P[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
        }
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_BARO;
    }"""
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/')
sw(P, t, 'gbk', 's1v')
print('proc_ekf.c: b_baro 冻结')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        27u') == 1
u = u.replace('#define V5F_FW_VER        27u', '#define V5F_FW_VER        28u', 1)
sw(T, u, 'gbk', 's1v')

p2 = open(P, 'rb').read().decode('gbk')
u2 = open(T, 'rb').read().decode('gbk')
S = R + r'\V5F\User\src\SPI_rx.c'
s2 = open(S, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', u2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
for k, v in [('VER=28', ver == 28), ('冻结代码在', 'bb_save' in p2),
             ('花括号平衡', p2.count('{') == p2.count('}')),
             ('注释配平', p2.count('/*') == p2.count('*/') and u2.count('/*') == u2.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
