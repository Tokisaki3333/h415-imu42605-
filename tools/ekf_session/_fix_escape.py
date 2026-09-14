# -*- coding: utf-8 -*-
"""VER=12 -> 13：
  1) chi2 逃脱阀 —— 同一道观测连续被剔够 N 次就用放大 R 软拉一把。
     没有它，任何一道观测只要初始新息 >3sigma 就永久锁死（实测两回：
     气压 b_baro 差 13.5 m 时 nis 恒 180；磁偏航差 80 度时 nis 恒 1.3e4）。
  2) 偏置状态钳位 —— bg/ba 没有任何直接观测，可以漂到物理上不可能的值；
     bg 现在参与积分，漂了就真去转姿态（实测 -40.7 dps）。
  3) 磁偏航单独放宽 chi2 门限：它是绝对航向，不是野值多发道。
"""
import shutil

T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'

J = '''
/* ---- chi2 的逃脱阀（没有它，任何一道观测都会被永久锁在门外）----
 * 实测两次死锁：气压 b_baro 差 13.5 m 时 nis 恒 180、磁偏航差 80 度时 nis 恒 1.3e4。
 * 机理：新息 > 3sigma 时 chi2 剔除；而剔除不改 P，S 不会变大，nis 就一直超限。
 * 做法：连续被剔够 ESCAPE 次后，用放大 ESC_R 倍的 R 放行一次（K 与 P 修正同比例
 *   缩小，等于只信这一笔的 1/ESC_R）。它把状态往观测方向软推，新息随之变小，
 *   正常剔除规则随即恢复。既保留了"挡单帧野值"，又不会把整条环关死。 */
#define V5F_EKF_CHI2_ESCAPE     8u
#define V5F_EKF_CHI2_ESC_R      9.0f
#define V5F_EKF_NIS_MAX_MAG     100.0f  /* 磁偏航单独放宽：绝对航向，不是野值多发道 */

/* ---- 零偏残差钳位（离线标定常量之上的残差）----
 * ★ bg/ba 没有任何直接观测，只靠与新息的弱相关被间接推；不钳位就能漂到
 *   物理上不可能的值（实测 bg_z = -40.7 dps 且恒定）。而 bg 现在**参与积分**，
 *   漂了就会真去转姿态 —— 这是"偏置状态必须钳位"的实证。 */
#define V5F_EKF_BG_LIM_DPS      10.0f
#define V5F_EKF_BG_LIM_RADS     (V5F_EKF_BG_LIM_DPS * 0.017453292f)
#define V5F_EKF_BA_LIM_MPS2     0.05f   /* 约 5 mg */

'''

t = open(T, 'rb').read().decode('gbk')
a = '/* ---- gate_bits 位定义（上报列 104 的原始值）---- */'
assert t.count(a) == 1
t = t.replace(a, J.lstrip('\n') + a, 1)
assert t.count('#define V5F_FW_VER        12u') == 1
t = t.replace('#define V5F_FW_VER        12u', '#define V5F_FW_VER        13u', 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(T, T + '.bak_s1e')
open(T, 'wb').write(t.encode('gbk'))
print('v5f_tune.h: 逃脱阀 + 钳位常量 + VER 13')

t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


# 1) 拒绝计数器
sub("static uint8_t  s_prop_ok;            /* 本周期的协方差传播是否已完成（第一周期没有） */",
    "static uint8_t  s_prop_ok;            /* 本周期的协方差传播是否已完成（第一周期没有） */\n"
    "static uint8_t  s_rej[5];             /* 各道观测连续被 chi2 剔除的次数（逃脱阀用）：\n"
    "                                       * 0=重力/倾斜 1=速度 2=气压 3=磁偏航 4=位置 */",
    'rej')

# 2) ekf_update 签名 + 逃脱逻辑
sub("""static uint8_t ekf_update(const float *R, uint8_t m, const float *r,
                          float nis_max, float *nis_out)""",
    """static uint8_t ekf_update(const float *R, uint8_t m, const float *r,
                          float nis_max, float *nis_out, uint8_t *rej)""", 'sig')

sub("""    if (nis_out) *nis_out = nis;
    if (nis > nis_max) return 1u;""",
    """    if (nis_out) *nis_out = nis;
    if (nis > nis_max) {
        if (rej && *rej < 250u) (*rej)++;
        if (!rej || *rej < V5F_EKF_CHI2_ESCAPE) return 1u;
        /* 逃脱：连续被剔够多次 -> 用放大 ESC_R 倍的 R 放行一次。
         * 等价于把 Si 缩小 ESC_R 倍（K = PHt·Si，P 修正 = K·S·K' 同比例缩小）。 */
        for (i = 0u; i < m; i++) {
            for (j = 0u; j < m; j++) Si[i][j] *= (1.0f / V5F_EKF_CHI2_ESC_R);
        }
    }
    if (rej) *rej = 0u;""", 'escape')

# 3) 偏置钳位
sub("""    for (i = 0u; i < 3u; i++) s_x[IX_BA + i] += dx[IX_BA + i];
    for (i = 0u; i < 3u; i++) s_x[IX_BG + i] += dx[IX_BG + i];""",
    """    for (i = 0u; i < 3u; i++) s_x[IX_BA + i] += dx[IX_BA + i];
    for (i = 0u; i < 3u; i++) s_x[IX_BG + i] += dx[IX_BG + i];
    /* 钳位：这两个状态没有直接观测，只被弱相关间接推；不钳就能漂到不可能的值
     * （实测 bg_z = -40.7 dps），而 bg 参与积分 -> 会真去转姿态。 */
    for (i = 0u; i < 3u; i++) {
        if (s_x[IX_BA + i] >  V5F_EKF_BA_LIM_MPS2) s_x[IX_BA + i] =  V5F_EKF_BA_LIM_MPS2;
        if (s_x[IX_BA + i] < -V5F_EKF_BA_LIM_MPS2) s_x[IX_BA + i] = -V5F_EKF_BA_LIM_MPS2;
        if (s_x[IX_BG + i] >  V5F_EKF_BG_LIM_RADS) s_x[IX_BG + i] =  V5F_EKF_BG_LIM_RADS;
        if (s_x[IX_BG + i] < -V5F_EKF_BG_LIM_RADS) s_x[IX_BG + i] = -V5F_EKF_BG_LIM_RADS;
    }""", 'clamp')

# 4) 七处调用加拒绝计数器；磁偏航换门限
sub("st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3]);",
    "st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0]);", 'c6')
sub("st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[1]);",
    "st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[1], &s_rej[1]);", 'c5')
sub("st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2]);",
    "st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[2], &s_rej[2]);", 'c3')
sub("st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, &s_nis[4]);",
    "st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3]);", 'c7')
sub("st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[0]);",
    "st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[0], &s_rej[4]);", 'c1')
sub("st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL);",
    "st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL, &s_rej[4]);", 'c2')
sub("st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[1]);",
    "st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[1], &s_rej[1]);", 'c4')

# 5) 对齐时清拒绝计数
sub("            for (i = 0u; i < 5u; i++) s_nis[i] = 0.0f;",
    "            for (i = 0u; i < 5u; i++) { s_nis[i] = 0.0f; s_rej[i] = 0u; }", 'rejclr')

assert t.count('/*') == t.count('*/'), '注释不配平'
assert t.count('ekf_update(R,') == 7
shutil.copy2(P, P + '.bak_s1e')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 逃脱阀 + 钳位 + 7 处调用已改')
print()
print('fw_tag 期望 = %d  (VER=13, 112 列)' % ((13 << 16) | (112 << 8) | 1 | 2 | 4))
