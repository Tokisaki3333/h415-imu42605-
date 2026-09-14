# -*- coding: utf-8 -*-
"""VER=44 -> 45：把地磁硬新息门的"毛刺"与"框架偏置"分开。

--- 实测（VER=44 日志，150409 帧 / 18.73 s）---
  EKF 姿态 与 旧链姿态 夹角 p50 8.35 度      -> 姿态本身没问题
  r = wrap(D - az) : 旧链 162.7 度 / EKF 168.9 度，且 6 段漂移 160.9 -> 171.6
  chi2_rej 位 置位 100%；mag_yaw 位 从未置位；mag_used = 0%
  -> 45 度绝对硬新息门每周期都在拒，地磁对偏航**一次都没修正过**。

--- 为什么"感觉在牵引却牵不到磁北"---
门只丢了绝对值大的新息，没有区分它的**性质**：
  * 坏样本/毛刺：与上一周期残差差异巨大（实测静止时出现过 169 度单帧跳变）-> 必须丢；
  * 框架偏置：偏航与磁北差一个**常量**角（本机 160~172 度）-> 大且**持续**，
    这正是唯一能把四元数坐标牵到磁北的信息，却被当成毛刺永久拒掉了。
后果：偏航只能靠陀螺自由漂移，靠旧链的地磁环间接获得"相对"阻尼
（所以连续旋转误差收敛、但坐标永远不在磁北）。

--- 修法 ---
判据同时看**幅值**和**与上一周期的差**：
    |r| > 45 度 且 |r - r_prev| > SPIKE  -> 瞬时跳变，丢；
    否则                                  -> 交给 ekf_update（仍受 MAG_K_MAX 限幅）。
框架偏置被接受后每周期拉 K*|r|，约几十个周期收敛到 0；
收敛过程中 r 每周期只变 K*|r| << SPIKE，所以不会被自己重新判成毛刺。
无新增上报通道（JF_CH_NUM 仍 122）：收敛证据直接看 119 列 mag_r_deg 由 168 -> ~0。

**风险提示**：若这台机器的地磁水平轴是"镜像"(极性反)而非"绕 Z 转 180 度"，
则拉 r->0 会收敛到与真磁北差 180 度的框架。镜像是**真旋转无法区分**的，
必须在源头（磁矢量符号）解决。验证办法见下发的判据。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)                 # 先编码：GBK 失败时不毁文件
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
n = 0


def sub(a, b, nm):
    global t, n
    c = t.count(a)
    assert c == 1, '锚点[%s] 匹配 %d 次（应为 1）' % (nm, c)
    t = t.replace(a, b, 1)
    n += 1
    print('  ok  %s' % nm)


# ---- 1. 新增两个状态 ----
sub("static uint8_t  s_q_ms_ok;",
    "static uint8_t  s_q_ms_ok;\n"
    "static float    s_mag_r_prev;   /* 上一周期的磁残差（用于区分毛刺/框架偏置）*/\n"
    "static uint8_t  s_mag_r_seed;   /* 首次调用只播种，不判定 */\n"
    "static uint8_t  s_mag_spike;    /* 被判为瞬时跳变而丢弃的计数（诊断）*/",
    '1-新增状态')

# ---- 2. 硬新息门 -> 毛刺/偏置判别 ----
old = """    if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
    } else {
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                        0x0100u, V5F_EKF_MAG_K_MAX);
        if (st == 0u) { s_gate_bits |= V5F_EKF_GB_MAG; s_mag_used = 1u; }
    }"""
assert t.count(old) == 1, '硬新息门锚点匹配 %d 次' % t.count(old)
new = """    /* ★VER=45 大新息必须按**性质**分开，否则偏航永远牵不到磁北：
     *   瞬时跳变（坏样本）：与上一周期残差差异巨大 -> 丢；
     *   持续大残差（框架偏置）：偏航与磁北差一个常量角 -> **必须积分掉**。
     * 实测 VER=44：r 长期 160~172 度、chi2_rej 置位 100%、mag_used 恒 0，
     * 就是被原来那道 45 度绝对门永久锁死 —— 表现为"看着在牵引、却牵不到磁北"
     * （只剩陀螺自由漂移 + 旧链地磁环给的相对阻尼）。
     * 接受框架偏置后单步仍由 MAG_K_MAX 限幅，几十个周期收敛；
     * 收敛中 r 每周期只变 K*|r| << SPIKE，不会被自己重新判成毛刺。 */
    {
        float dr;
        if (!s_mag_r_seed) { s_mag_r_seed = 1u; s_mag_r_prev = r[0]; }
        dr = fabsf(wrap_pi(r[0] - s_mag_r_prev));
        s_mag_r_prev = r[0];
        if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD &&
            dr > V5F_EKF_MAG_SPIKE_DEG * DEG2RAD) {
            if (s_mag_spike < 250u) s_mag_spike++;
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
        } else {
            st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],
                            0x0100u, V5F_EKF_MAG_K_MAX);
            if (st == 0u) { s_gate_bits |= V5F_EKF_GB_MAG; s_mag_used = 1u; }
        }
    }"""
t = t.replace(old, new, 1)
n += 1
print('  ok  2-毛刺/偏置判别')
dump(P, t, 'gbk', 's2p')

# ---- 3. tune：新增 SPIKE 阈值 + VER 45 ----
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_R_MAX_DEG[^\r\n]*$', u, re.M)
assert m, '未找到 V5F_EKF_MAG_R_MAX_DEG'
ins = (u[m.end():m.end()] +
       '\n/* ★VER=45 地磁"瞬时跳变"判据：仅当 |r|>MAG_R_MAX_DEG 且与上一周期残差\n'
       ' * 之差超过本值时，才判为坏样本丢弃。持续的框架偏置（大而稳定）不丢，\n'
       ' * 否则偏航永远无法被牵引到磁北（VER=44 实测据此锁死）。20 度。 */\n'
       '#define V5F_EKF_MAG_SPIKE_DEG     20.0f')
u = u[:m.end()] + ins + u[m.end():]
assert u.count('V5F_EKF_MAG_SPIKE_DEG') == 2, u.count('V5F_EKF_MAG_SPIKE_DEG')
assert u.count('#define V5F_FW_VER        44u') == 1
u = u.replace('#define V5F_FW_VER        44u', '#define V5F_FW_VER        45u', 1)
dump(T, u, 'gbk', 's2p')

# ---- 4. 校验 ----
c = open(P, 'rb').read().decode('gbk')
h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPIx.c'.replace('SPIx', 'SPI_rx'), 'rb')
                    .read().decode('gbk')).group(1))
ck = [('VER==45', ver == 45), ('编辑数==2', n),
      ('判别在新', 'dr > V5F_EKF_MAG_SPIKE_DEG * DEG2RAD' in c),
      ('旧绝对门已去', 'if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {' not in c),
      ('MAG_R_MAX 仍用于判别', 'V5F_EKF_MAG_R_MAX_DEG * DEG2RAD &&' in c),
      ('播种在', 'if (!s_mag_r_seed)' in c),
      ('tune 定义在', h.count('V5F_EKF_MAG_SPIKE_DEG') == 2),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/')),
      ('括号差仍 -4', c.count('(') - c.count(')') == -4)]
h.encode('gbk'); c.encode('gbk')
for k, v in ck:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=45, %d ch, 无新增通道)' % ((ver << 16) | (nch << 8) | 7, nch))
