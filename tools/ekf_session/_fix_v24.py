# -*- coding: utf-8 -*-
"""VER=23 -> 24：死区关掉，改让 ba 吸收重力泄漏。

实测（VER=23）：死区在运动段的门限 td = 0.15~0.28 m/s^2（sigma_tilt 0.9~1.1 度），
比静止时的 0.036 大 4~5 倍 —— 因为 td 正比于 sigma_tilt，而运动让 sigma_tilt 涨了。
于是它"削掉一切低频"：一段 |a_lin|=0.129（门限的 3.4 倍）的真实运动仍被吃掉 86%。
这不是"选择性挡漏"，是幅值取舍，而且是最坏的那种（越不确定越激进）。

正确做法：泄漏是**姿态误差造成的、对给定姿态恒定的**比选力误差，它的物理归宿
本来就是加计零偏状态 ba。机制已在，但被我锁死：
  V5F_EKF_SIG_BA_RW = 0（S0.3 测的是**器件零偏**的随机游走，低于门限）
  P0_BA = 0.02 m/s^2 (2 mg)，而泄漏可达 0.1~0.2 m/s^2，是它的 5~10 倍
  -> 滤波器根本推不动 ba。
现在给 ba 一个合适的随机游走，让它能把泄漏吸进去；a_nav 变干净且**不做幅值取舍**，
真实运动全部保留。sigma_v 的诚实性仍由 Q_vv 那项（重力经倾角泄漏）负责。
"""
import re
import shutil

T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
u = open(T, 'rb').read().decode('gbk')

a = '#define V5F_EKF_SIG_BA_RW        0.0f    /* 加计零偏随机游走：实测低于门限（S0.3） */'
assert u.count(a) == 1
u = u.replace(a, """/* ★ 这条不是"器件零偏"的随机游走，而是给**重力经倾角误差的泄漏**留的去处。
 *   S0.3 测出器件零偏的游走低于门限（所以不能拿它当器件模型），但泄漏本身是
 *   一段"对给定姿态恒定"的比力误差，物理归宿就是 ba。原来取 0 且 P0_BA 只有
 *   2 mg，而泄漏可达 0.1~0.2 m/s^2（= sigma_tilt 0.6~1.1 度 x g），滤波器推不动 ba，
 *   于是泄漏全跑到速度里去了（VER=23 实测：平移段速度被死区削掉 33~86%）。
 *   0.01 m/s^2/sqrt(s)：10 s 长到 0.032、100 s 长到 0.1 —— 正好覆盖泄漏量级。 */
#define V5F_EKF_SIG_BA_RW        0.01f""", 1)

a = '#define V5F_EKF_VDEAD_K          1.0f'
assert u.count(a) == 1
u = u.replace(a, """/* 死区默认**关闭**：实测它是"削掉一切低频"（一段 |a_lin|=0.129、门限 0.038 的
 * 真实运动仍被吃掉 86%），而且门限正比于 sigma_tilt -> 运动让 sigma_tilt 涨 4~5 倍，
 * 死区跟着放大，正好在最需要保留真实运动时最激进。泄漏交给 ba 去吸（见 SIG_BA_RW）。
 * 留成可在线改的标量，需要时可以拿来做 A/B 对照。 */
#define V5F_EKF_VDEAD_K          0.0f""", 1)

assert u.count('#define V5F_FW_VER        23u') == 1
u = u.replace('#define V5F_FW_VER        23u', '#define V5F_FW_VER        24u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1q')
open(T, 'wb').write(u.encode('gbk'))

chk = open(T, 'rb').read().decode('gbk')
for k, v in [('VDEAD_K = 0', re.search(r'#define V5F_EKF_VDEAD_K\s+([\d.]+)f', chk).group(1) == '0.0'),
             ('SIG_BA_RW = 0.01', re.search(r'#define V5F_EKF_SIG_BA_RW\s+([\d.]+)f', chk).group(1) == '0.01'),
             ('注释配平', chk.count('/*') == chk.count('*/'))]:
    print('  %-20s %s' % (k, v))
    assert v, k
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c',
                         'rb').read().decode('gbk')).group(1))
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', chk).group(1))
print()
print('VER=%d NCH=%d  fw_tag = %d' % (ver, nch, (ver << 16) | (nch << 8) | 1 | 2 | 4))
