# -*- coding: utf-8 -*-
"""VER=43 -> 44：地磁观测改用「磁样本那一刻的姿态」旋转（去掉自积分去陈旧）。

--- 实测（VER=43，19.2 s，122 通道）---
  mag_gate  开 99.79%              <- 门不是问题
  mag_used  执行 69.89%
  mag_bh    p50 0.4957 min 0.3008  <- 水平分量占比，正常应恒为 cos(dip)=0.59，却随运动变
  mag_r_deg 非零 100%，p50 0.768deg，p90 59.9deg，max 72.0deg   <- ★ 新息经常错 60 度
  p_yy      p50 5.8e-2 (R=7.6e-5)  <- K 被 MAG_K_MAX=0.05 顶死
  EKF 磁残差 +0.001 -> +54.451 deg；总转角 EKF -5351.88 vs 旧链 -5396.12

--- 根因：又一处「相加」当成「复合」---
VER=38 我把姿态增量改成逐帧复合（三轴同时转的 coning 修正），但**同一处的磁陈旧补偿漏了**：

    s_mag_dth[i] += w[i] * dt;                 /* 机体轴角相加 */
    dq = Exp(-s_mag_dth);  q_rot_vec(dq, mf)   /* 用一个错的 dq 去转磁矢量 */

连续快转时 IST8310 样本陈旧可达 60~190 度（187 Hz，2000+ dps），此时「相加的轴角」与
「复合的旋转」方向完全不同 -> 去陈旧把 f 转到错的方向 -> 方位角错 60 度 -> 偏航被拽走 54 度。
K=0.05 在 349 Hz 下 tau=57 ms，本来绝对追得上 279 deg/s；追不上说明**测量是错的**，不是增益不够。

--- 修法：直接消掉这一步（数学上恒等）---
陈旧补偿 + 当前姿态旋转两步合起来：
    mf   = R(q_now)^T R(q_s) f_s          (去陈旧)
    Bn   = R(q_now) mf  =  R(q_now) R(q_now)^T R(q_s) f_s  =  R(q_s) f_s
**q_now 精确抵消**。所以整个「去陈旧 + R(q_now)」在数学上就等于「用磁样本那一帧的姿态去转」。
于是根本不需要积分、也不需要复合——直接

    q_s = 磁样本边沿那一帧的 EKF 姿态（在 preintegrate 里存）

    q_to_R(q_s, Rt);  rot_bn(Rt, f_s, Bn);        /* B = R(q_s) f_s */

既精确又省掉一段代码。**这同时给出可验证的预测**：
  * mag_bh 应稳定收敛到 cos(dip) 附近（约 0.55~0.59）且不再随运动大幅波动；
  * mag_r 的大角度尾巴（p90 59.9deg）应大幅收缩；
  * EKF 磁残差不应再漂到几十度，且与旧链的总转角差应显著变小。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)                 # 先编码：GBK 失败时不会毁掉文件
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
n_edits = 0


def sub(a, b, nm):
    global t, n_edits
    n = t.count(a)
    assert n == 1, '锚点[%s] 匹配 %d 次（应为 1）' % (nm, n)
    t = t.replace(a, b, 1)
    n_edits += 1
    print('  ok  %s' % nm)


# ---- 1. 新增「磁样本时刻的姿态」----
sub("static uint32_t s_ist_last;",
    "static uint32_t s_ist_last;\n"
    "static float    s_q_ms[4];   /* ★VER=44：磁样本那一帧的 EKF 姿态（地磁观测的旋转基准）*/\n"
    "static uint8_t  s_q_ms_ok;",
    '1-新增 s_q_ms')

# ---- 2. IST 样本边沿：记录姿态 ----
sub("""        if (ic != s_ist_last) {
            s_ist_last = ic;
            s_mag_dth[0] = 0.0f; s_mag_dth[1] = 0.0f; s_mag_dth[2] = 0.0f;
        }""",
    """        if (ic != s_ist_last) {
            uint32_t nq;
            s_ist_last = ic;
            s_mag_dth[0] = 0.0f; s_mag_dth[1] = 0.0f; s_mag_dth[2] = 0.0f;
            /* ★VER=44：记下磁样本这一帧的姿态，地磁观测用它把 f 转到导航系 */
            for (nq = 0u; nq < 4u; nq++) s_q_ms[nq] = s_x[IX_Q + nq];
            s_q_ms_ok = 1u;
        }""",
    '2-样本边沿记姿态')

# ---- 3. M7：删掉自积分去陈旧，改用 q_s ----
old_m7 = """    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
"""
f0 = t.index('static void ekf_m7_mag')      # 只在 M7 函数体内定位（该行全文出现 2 次）
i0 = t.index(old_m7, f0)
assert t.count(old_m7, f0, t.index('static void ', f0 + 10)) == 1
i1 = t.index("    q_to_R(&s_x[IX_Q], Rt);", i0)
assert i1 > i0
old_blk = t[i0:i1 + len("    q_to_R(&s_x[IX_Q], Rt);")]
assert 's_mag_dth' in old_blk and 'q_rot_vec(dq, mf, tmp)' in old_blk
new_blk = """    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];
    /* ★VER=44 地磁观测的姿态基准 = **磁样本那一帧**的姿态（q_s），不是当前姿态。
     * 旧的写法是「s_mag_dth 自积分 -> Exp(-dth) 转 f -> 再乘 R(q_now)」，
     * 而这两步在数学上恒等于 R(q_s) f_s（q_now 精确抵消）：
     *     Bn = R(q_now) R(q_now)^T R(q_s) f_s = R(q_s) f_s
     * 旧写法的错误不在抵消，而在 **s_mag_dth 是"轴角相加"而不是"旋转复合"**：
     * 连续快转时样本陈旧可达 60~190 度（187 Hz / 2000+ dps），此时两者方向完全不同，
     * 于是把 f 转到错方向 -> 方位角错 60 度（实测 mag_r p90 59.9 / max 72.0）
     * -> 偏航被拖走 54 度（实测残差 +0.001 -> +54.451）。VER=38 修过姿态增量的同一类错，
     * 这里漏了。现在直接用 q_s，既精确又不再需要任何积分。
     * 注：s_mag_dth 自 VER=44 起仅作残留诊断，**不要再用于补偿**。 */
    if (s_q_ms_ok) {
        q_to_R(s_q_ms, Rt);
    } else {
        q_to_R(&s_x[IX_Q], Rt);
    }"""
t = t[:i0] + new_blk + t[i1 + len("    q_to_R(&s_x[IX_Q], Rt);"):]
n_edits += 1
print('  ok  3-M7 改用 q_s')

# ---- 4. 保持 mf 的 const 语义不变：mf 之后仍被使用（B 计算），无需改动 ----
assert t.count('rot_bn(Rt, mf, Bn)') == 1, 'B 计算处应仍用 mf'

dump(P, t, 'gbk', 's2p')

# ---- 5. VER 44 ----
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        43u') == 1
u = u.replace('#define V5F_FW_VER        43u', '#define V5F_FW_VER        44u', 1)
dump(T, u, 'gbk', 's2p')

# ---- 6. 校验 ----
c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u',
                    open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
checks = [
    ('VER == 44', ver == 44),
    ('编辑数 == 3', n_edits == 3),
    ('存姿态', 's_q_ms[nq] = s_x[IX_Q + nq]' in c),
    ('M7 用 q_s', 'q_to_R(s_q_ms, Rt);' in c),
    ('M7 不再用 dth', 'q_rot_vec(dq, mf, tmp)' not in c),
    ('回退分支在', 'q_to_R(&s_x[IX_Q], Rt);' in c),
    ('B 计算仍在', 'rot_bn(Rt, mf, Bn)' in c),
    ('{ } 平衡', c.count('{') == c.count('}')),
    ('/* */ 平衡', c.count('/*') == c.count('*/')),
    ('括号平衡', c.count('(') == c.count(')')),
    ('无非GBK字符', True),
]
c.encode('gbk')
print()
for k, v in checks:
    print('  %-16s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print()
print('fw_tag 期望 = %d   (VER=44, %d ch, EKF+MAGCAL+DETAC)' %
      ((ver << 16) | (nch << 8) | 1 | 2 | 4, nch))
