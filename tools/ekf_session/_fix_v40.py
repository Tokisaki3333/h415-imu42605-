# -*- coding: utf-8 -*-
"""VER=39 -> 40：给观测更新加**物理约束**——增益上限 + 磁偏航的硬新息门。

实测（VER=35 日志，板子静止）：EKF 单帧跳变最大 169.47 度，同帧旧链只动 0.0000 度，
15 次最严重跳变**全部**有 mag 门开着。
  2.87 ms 内转 169 度需要 59000 度/s —— 物理上不可能，陀螺同帧说 0 度。
链条：坏磁样本（模长仍约 1，模长门抓不住）+ Q_tt 整流项抬高 P[8][8] 使 S 变大
      -> 软 chi2 不降权 -> K 约等于 1 -> 姿态整个 snap 到坏方位。
VER=36/37 的限幅只是把它从"闪现"变成"每周期 17 度/s 的恒速漂移"（同一个根因）。

修法（都是物理约束，不是调参）：
 1) **增益上限** k_cap：K 的每一项不超过 k_cap，且 dx 与 P 的修正**用同一个 K**，
    所以状态与协方差始终一致（这正是我上次限幅的错误：只回滚状态).
 2) **磁偏航硬新息门**：|r| > 45 度直接整帧丢弃（不是软加权）。
    45 度/2.87 ms = 15000 度/s，超过它必然是坏样本。
"""
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


# ---------------- 常量 ----------------
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_NIS_MAX_MAG     100.0f'
assert u.count(a) == 1
u = u.replace(a, a + """
/* ---- 观测更新的**物理约束**（不是调参）----
 * 实测 VER=35 板子静止时 EKF 单帧跳变 169 度（旧链同帧 0.0000 度），
 * 15 次最严重跳变全部伴随 mag 门开启。机理：坏磁样本（模长仍约 1，模长门抓不住）
 * + 整流项抬高 P[8][8] 使 S 变大 -> 软 chi2 不降权 -> K 约等于 1 -> 整体 snap。
 * 2.87 ms 转 169 度 = 59000 度/s，物理上不可能。 */
#define V5F_EKF_MAG_R_MAX_DEG    45.0f   /* 磁偏航硬新息门：|r| 超它整帧丢弃 */
#define V5F_EKF_MAG_K_MAX        0.002f  /* 磁偏航增益上限（tau 约 1.4 s 的牵引）*/
#define V5F_EKF_TILT_K_MAX       0.010f  /* 重力增益上限（比磁快，它是强观测）*/""", 1)
assert u.count('#define V5F_FW_VER        39u') == 1
u = u.replace('#define V5F_FW_VER        39u', '#define V5F_FW_VER        40u', 1)
assert u.count('/*') == u.count('*/')
sw(T, u, 'gbk', 's2j')
print('v5f_tune.h: 物理约束常量 + VER 40')

# ---------------- proc_ekf.c ----------------
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)
    print('  ok:', nm)


# 1) 签名加 k_cap
sub("""                          float nis_max, float *nis_out, uint8_t *rej,
                          uint16_t inj_mask)""",
    """                          float nis_max, float *nis_out, uint8_t *rej,
                          uint16_t inj_mask, float k_cap)""", 'sig')

# 2) 掩码之后、dx 之前，加 K 的绝对值上限（dx 与 P 共用同一个 K -> 一致）
sub("""    /* dx = K r */
    for (i = 0u; i < EKF_N; i++) {""",
    """    /* ★ 增益上限 k_cap：K 的每一项取绝对值上限。
     * 关键：dx 与下面的 P 修正**用的是同一个 K**，所以状态与协方差始终一致 ——
     * 这正是上次"限幅"犯的错（只回滚状态、不回滚 P，形成正反馈）。
     * 它是一条物理约束：一个观测不该在一个周期内把状态推超过 k_cap*r。 */
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

# 3) 七个调用点加 k_cap
for old, new, nm in [
    ("&s_rej[0], 0x7FC0u);", "&s_rej[0], 0x7FC0u, V5F_EKF_TILT_K_MAX);", 'M6'),
    ("&s_rej[1], 0x0E38u);", "&s_rej[1], 0x0E38u, 0.05f);", 'M5'),
    ("&s_rej[2], 0x0024u);", "&s_rej[2], 0x0024u, 0.02f);", 'M3'),
    ("&s_rej[4], 0x0E3Fu);", "&s_rej[4], 0x0E3Fu, 0.05f);", 'M1'),
    ("&s_rej[4], 0x8024u);", "&s_rej[4], 0x8024u, 0.05f);", 'M2'),
    ("&s_rej[1], 0x0E18u);", "&s_rej[1], 0x0E18u, 0.05f);", 'M4'),
]:
    sub(old, new, nm)

# 4) M7：硬新息门 + k_cap
sub("""            float y0 = q_yaw(&s_x[IX_Q]);
            float y1, dlt, lim = V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD;
            st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3], 0x0100u);
            y1 = q_yaw(&s_x[IX_Q]);
            dlt = wrap_pi(y1 - y0);
            if (dlt > lim || dlt < -lim) {
                float corr = dlt - ((dlt > 0.0f) ? lim : -lim);
                float hq[4], qt[4];
                uint32_t n2;
                hq[0] = cosf(-0.5f * corr); hq[1] = 0.0f; hq[2] = 0.0f;
                hq[3] = sinf(-0.5f * corr);
                q_mul(hq, &s_x[IX_Q], qt);
                for (n2 = 0u; n2 < 4u; n2++) s_x[IX_Q + n2] = qt[n2];
                q_norm(&s_x[IX_Q]);
            }
        }
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;""",
    """            /* ★ 硬新息门（物理约束，不是调参）：|r| > 45 度直接整帧丢弃。
             *   实测板子静止时出现过 169 度的单周期跳变（旧链同帧 0.0000 度）——
             *   2.87 ms 转 169 度需要 59000 度/s，物理上不可能，必然是坏样本。
             *   模长门抓不住它（模长仍约 1），软 chi2 也抓不住（整流项抬高 P 使 S 变大）。
             *   用硬门 + 增益上限，而不是像上次那样"限幅状态"（那只回滚状态不回滚 P,
             *   会把闪现变成恒速漂移）。 */
            if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {
                if (s_rej[3] < 250u) s_rej[3]++;
                s_gate_bits |= V5F_EKF_GB_CHI2;
            } else {
                st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                                0x0100u, V5F_EKF_MAG_K_MAX);
                if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;
            }
        }""", 'm7')

# 5) 掩码补丁顺带删掉不再使用的 q_yaw（若其它地方没用到）
if t.count('q_yaw(') == 1:
    t = t.replace("""/* 四元数的导航系偏航角（与上报里的定义一致） */
static float q_yaw(const float *q)
{
    return atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
}

""", '')
    print('  ok: 去掉不再使用的 q_yaw')

assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
assert 'V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD' not in t, '状态限幅残留'
sw(P, t, 'gbk', 's2j')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=40', ver == 40),
             ('增益上限在', 'if (s_K[i][j] >  k_cap)' in c),
             ('M7 硬新息门', 'fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG' in c),
             ('M7 用 k_cap', '0x0100u, V5F_EKF_MAG_K_MAX' in c),
             ('7 处调用带 k_cap', c.count('k_cap') >= 3 and c.count('0x7FC0u, V5F_EKF_TILT_K_MAX') == 1),
             ('状态限幅已去', 'V5F_EKF_YAW_STEP_MAX_DEG * DEG2RAD' not in c),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-16s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
