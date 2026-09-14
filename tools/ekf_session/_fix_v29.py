# -*- coding: utf-8 -*-
"""VER=28 -> 29：修"清号志先于调用"这个抄写错误（唯一确定的 bug）。

`s_baro_new = 0u;` 被写在 `ekf_m3_baro(gate);` **之前**，而 M3 第一行就是
`if (!gate->ekf_baro || !s_baro_new) return;` -> M3 每次都在第一行 return，
所以 baro 门恒 0.00%、差分观测从未生效、我那段 b_baro 冻结代码也从未执行。
把两行调换即可。

另外加一道防御：没有 ENU 原点（即没有 GPS 高度 M2）时，每个 EKF 周期把 b_baro
压回 0 并清掉它的协方差行/列 —— 绝对基准不该由一个没有观测者的状态承担。
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
a = """            if (s_prop_ok && s_baro_wait >= V5F_EKF_BARO_PERIOD_S) {
                s_baro_wait = 0.0f;
                s_baro_new = 0u;               /* 消费掉 */
                ekf_m3_baro(gate);
            }"""
assert t.count(a) == 1, t.count(a)
b = """            if (s_prop_ok && s_baro_wait >= V5F_EKF_BARO_PERIOD_S) {
                s_baro_wait = 0.0f;
                /* ★ 顺序不能反：ekf_m3_baro 第一行就是
                 *   `if (!gate->ekf_baro || !s_baro_new) return;`
                 *   先清号志再调用 = 每次都在第一行 return。
                 *   VER=26~28 实测 baro 门恒 0.00%、差分观测从未生效，
                 *   就是这两行写反了。 */
                ekf_m3_baro(gate);
                s_baro_new = 0u;               /* 调用之后再消费 */
            }"""
t = t.replace(a, b, 1)

# 防御：没有 ENU 原点时 b_baro 每周期压回 0（没有观测者的状态不该承担绝对基准）
a = """            if (s_aligned)   s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;"""
assert t.count(a) == 1
t = t.replace(a, """            /* 没有 ENU 原点（= 没有 GPS 高度 M2）时，b_baro 没有任何观测者。
             * 差分观测的 H 不含它，但它仍可能被协方差交叉项间接拖动。既然没人
             * 观测，就每个周期把它压回 0 并清掉它的协方差行/列，绝不让它偷偷长成
             * 一个假的绝对基准（VER=27 实测曾漂到 -116 m）。 */
            if (!s_origin_ok) {
                uint32_t k3;
                s_x[IX_BB] = 0.0f;
                for (k3 = 0u; k3 < EKF_N; k3++) {
                    s_P[k3][15] = 0.0f;
                    s_P[15][k3] = 0.0f;
                    s_Pn[k3][15] = 0.0f;
                    s_Pn[15][k3] = 0.0f;
                }
                s_P[15][15]  = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
                s_Pn[15][15] = V5F_EKF_P0_BARO_M * V5F_EKF_P0_BARO_M;
            }
            if (s_aligned)   s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;""", 1)
assert t.count('/*') == t.count('*/')
assert t.count('{') == t.count('}')
sw(P, t, 'gbk', 's1w')
print('proc_ekf.c: 号志顺序已调换 + 无原点时 b_baro 每周期归零')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        28u') == 1
u = u.replace('#define V5F_FW_VER        28u', '#define V5F_FW_VER        29u', 1)
sw(T, u, 'gbk', 's1w')

p2 = open(P, 'rb').read().decode('gbk')
S = R + r'\V5F\User\src\SPI_rx.c'
s2 = open(S, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', u).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
i = p2.index('ekf_m3_baro(gate);')
j = p2.index('s_baro_new = 0u;               /* 调用之后再消费 */')
for k, v in [('VER=29', ver == 29), ('调用在清号志之前', i < j),
             ('b_baro 防御在', 'if (!s_origin_ok) {' in p2),
             ('花括号平衡', p2.count('{') == p2.count('}')),
             ('注释配平', p2.count('/*') == p2.count('*/'))]:
    print('  %-20s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
