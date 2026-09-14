# -*- coding: utf-8 -*-
"""完成 VER=40 的 proc_ekf.c 部分（M7 锚点缩进是 8 空格，上一版用了 12 空格 -> 失败）。
按"行范围"替换 M7 的限幅块，不再依赖精确空白。"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)
    print('  ok:', nm)


if 'float k_cap)' not in t:
    sub("""                          float nis_max, float *nis_out, uint8_t *rej,
                          uint16_t inj_mask)""",
        """                          float nis_max, float *nis_out, uint8_t *rej,
                          uint16_t inj_mask, float k_cap)""", 'sig')
    sub("""    /* dx = K r */
    for (i = 0u; i < EKF_N; i++) {""",
        """    /* ★ 增益上限 k_cap：K 的每一项取绝对值上限。关键：dx 与下面的 P 修正用的是
     * **同一个 K**，所以状态与协方差始终一致 —— 这正是上次"限幅"犯的错
     * （只回滚状态、不回滚 P，形成正反馈，把闪现变成了恒速漂移）。 */
    if (k_cap > 0.0f) {
        for (i = 0u; i < EKF_N; i++) {
            for (j = 0u; j < m; j++) {
                if (s_K[i][j] >  k_cap) s_K[i][j] =  k_cap;
                if (s_K[i][j] < -k_cap) s_K[i][j] = -k_cap;
            }
        }
    }
    /* dx = K r */
    for (i = 0u; i < EKF_N; i++) {""", 'kcap')
    for old, new, nm in [
        ("&s_rej[0], 0x7FC0u);", "&s_rej[0], 0x7FC0u, V5F_EKF_TILT_K_MAX);", 'M6'),
        ("&s_rej[1], 0x0E38u);", "&s_rej[1], 0x0E38u, 0.05f);", 'M5'),
        ("&s_rej[2], 0x0024u);", "&s_rej[2], 0x0024u, 0.02f);", 'M3'),
        ("&s_rej[4], 0x0E3Fu);", "&s_rej[4], 0x0E3Fu, 0.05f);", 'M1'),
        ("&s_rej[4], 0x8024u);", "&s_rej[4], 0x8024u, 0.05f);", 'M2'),
        ("&s_rej[1], 0x0E18u);", "&s_rej[1], 0x0E18u, 0.05f);", 'M4'),
    ]:
        sub(old, new, nm)

# ---- M7：按行范围替换限幅块 ----
L = t.split('\n')
i0 = next(k for k, l in enumerate(L) if 'r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);' in l)
i1 = next(k for k, l in enumerate(L) if 's_gate_bits |= V5F_EKF_GB_MAG;' in l)
assert i1 > i0, (i0, i1)
NEW = """    /* ★ 硬新息门（物理约束，不是调参）：|r| 超过 45 度就整帧丢弃。
     *   实测板子静止时出现过单周期 169 度的跳变（旧链同帧 0.0000 度）——
     *   2.87 ms 转 169 度需要 59000 度/s，物理上不可能，必然是坏样本。
     *   模长门抓不住（坏样本模长仍约 1），软 chi2 也抓不住（整流项抬高 P 使 S 变大）。
     *   用硬门 + 增益上限；不再用"限幅状态"那种做法（只回滚状态不回滚 P，
     *   会把闪现变成每周期 17 度/s 的恒速漂移）。 */
    if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
    } else {
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                        0x0100u, V5F_EKF_MAG_K_MAX);
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;
    }"""
L = L[:i0 + 1] + NEW.split('\n') + L[i1 + 1:]
t = '\n'.join(L)
print('  ok: M7 硬门 + 增益上限（行 %d..%d）' % (i0, i1))

# 去掉不再使用的 q_yaw
if t.count('q_yaw(') == 1:
    t = re.sub(r'/\* 四元数的导航系偏航角（与上报里的定义一致） \*/\n'
               r'static float q_yaw\(const float \*q\)\n\{\n'
               r'[^\n]*\n\}\n\n', '', t)
    print('  ok: 去掉 q_yaw')

assert 'V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD' not in t, '状态限幅残留'
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2k')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=40', ver == 40),
             ('增益上限在', 'if (s_K[i][j] >  k_cap)' in c),
             ('M7 硬新息门', 'fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG' in c),
             ('M7 用 k_cap', '0x0100u, V5F_EKF_MAG_K_MAX' in c),
             ('状态限幅已去', 'V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD' not in c),
             ('六处 k_cap', c.count(', 0.05f);') + c.count('0x7FC0u, V5F_EKF_TILT_K_MAX') + c.count('0x0024u, 0.02f') == 6),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-16s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
