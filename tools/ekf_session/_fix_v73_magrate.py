# -*- coding: utf-8 -*-
"""
VER=73：地磁慢环两处结构修正（基于 VER=72 录制 44.04 s / 353625 帧的实测）

改动 1 —— M7 只在**磁样本真的更新**时施加一次卡尔曼更新
  实测：ist_cnt 变化沿 8537 个 / 44.04 s = 193.8 Hz（磁物理率）；
        EKF 阶段机 23 帧一周期 = 351.7 Hz，而 M7 每个周期都被调用
        -> 同一个磁样本被重复施加 1.81 次。
  这不是"降频"，是把假的高速率改回真实观测率，不引入额外延迟
  （陈旧量仍由既有的 s_mag_dth 机体转动补偿处理）。
  门用 g_shm->ist.hdr.cnt（与 1243 行清 s_mag_dth 用的是同一个计数器）。
  被门挡住的周期把 dqx/dqy/dqz 显式清零，日志语义唯一：
  "dqz == 0" = 本周期没有施加修正（死区 / 无新样本），不再是陈旧值。

改动 2 —— 偏航过程噪声下限 V5F_EKF_Q_YAW_MIN
  实测（VER=72）：p_yy = s_Pn[8][8] 从 1.06 单调塌到 ~0（10.5 s -> 18.5 s，
  时间常数 ~2.8 s），sigma_yaw 随之为 0.000，环路增益饿死 ->
  20~28 s 残差 67->106 度而单步施加只剩 0.03~0.12 度，直到 28~33 s
  运动把 Q 补回来才重新抓住。
  机理：k_cap 限幅下 S ≈ |H|^2 P，所以每次削减 ∝ P 且与 R 无关
        => 指数塌陷；Q 比它小 3~4 个数量级，托不住。
        2x2 精确式（H=[-b0y,b0x], R=(0.7 度)^2, k_cap=0.05）算得
        drain/P = 5.9e-4 /样本，与实测时间常数一致。
  取值：Q_yaw = 1.0e-5 rad^2/样本 -> P[8][8] 稳在 0.0122 = (7.2 度)^2。
        即"滤波器对偏航的不确定度永不小于磁力计自身的精度量级" —— 这不是
        拟合，是项目自己写下的物理量（v5f_tune.h 里"磁角典型误差可达 10 度"
        就是 12 度死区的依据）。sigma_yaw 从此是个诚实读数（~7 度），不再是 0。
  效果：增益恒在 k_cap 上限，|r|=0.7 时修正速率 0.05*0.7*193.8 = 388 度/秒
        （100 度误差约 0.3 s 收敛）；15 度误差时约 62 度/秒。
"""
import os, shutil

ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')

def load(p):
    with open(p, 'rb') as f:
        return f.read().decode(ENC)

def save(p, text, tag):
    data = text.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f:
        f.write(data)
    with open(p, 'rb') as f:
        assert f.read().decode(ENC) == text, '回读校验失败 ' + p
    return len(data)

def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '[%s] 锚点命中 %d 次（必须恰好 1 次）' % (what, n)
    return t.replace(old, new, 1)

# ---------------- 1. v5f_tune.h ----------------
t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        72u', '#define V5F_FW_VER        73u', 'VER')
t = sub1(t,
    '#define V5F_EKF_Q_FLOOR          1.0e-12f',
    '#define V5F_EKF_Q_FLOOR          1.0e-12f\n'
    '/* ★VER=73 偏航过程噪声下限（rad^2 / EKF 周期）。必须单独给：M7 的 k_cap 限幅\n'
    ' * 下 S 约等于 |H|^2 P，于是每次更新对 P[8][8] 的削减正比于 P 本身（与 R 无关）\n'
    ' * = 指数塌陷；普通 Q 比它小 3~4 个数量级，托不住。\n'
    ' * 实测 VER=72：p_yy 从 1.06 塌到 ~0、sigma_yaw 读到 0.000、环路增益饿死，\n'
    ' * 残差涨到 106 度而单步只剩 0.03~0.12 度。\n'
    ' * 2x2 精确式算得 drain/P = 5.9e-4 /样本；本值让 P[8][8] 稳在 0.0122\n'
    ' * = (7.2 度)^2，即偏航不确定度永不小于磁力计自身精度量级。 */\n'
    '#define V5F_EKF_Q_YAW_MIN        1.0e-5f',
    'Q_YAW_MIN')
n1 = save(TUNE, t, 'v73')

# ---------------- 2. proc_ekf.c ----------------
e = load(EKF)
assert 's_mag_cnt_upd' not in e

# 2a) 新增静态计数器
e = sub1(e,
    'static uint32_t s_ist_last;',
    'static uint32_t s_ist_last;\n'
    'static uint32_t s_mag_cnt_upd;        /* ★VER=73 上一次真正施加磁观测时的 ist 样本号 */',
    'counter')

# 2b) 偏航 Q 下限
e = sub1(e,
    '            else if (row == 15u)               qd += V5F_EKF_SIG_BARO_RW * V5F_EKF_SIG_BARO_RW * dtq;\n'
    '            s += qd;',
    '            else if (row == 15u)               qd += V5F_EKF_SIG_BARO_RW * V5F_EKF_SIG_BARO_RW * dtq;\n'
    '            /* ★VER=73 偏航方差下限：不加这一项，M7 的 k_cap 限幅更新会把\n'
    '             * P[8][8] 指数抽干（削减 ∝ P）-> 偏航增益饿死 -> 环路失效。\n'
    '             * 见 v5f_tune.h V5F_EKF_Q_YAW_MIN 的实测数据与取值依据。 */\n'
    '            if (row == 8u) qd += V5F_EKF_Q_YAW_MIN;\n'
    '            s += qd;',
    'Qfloor')

# 2c) M7 只在磁样本更新时施加一次
e = sub1(e,
    '        } else {\n'
    '            /* ★VER=56 慢环符号翻转：实测开机 mag_r=1.16 度（已在磁北）',
    '        } else {\n'
    '            /* ★VER=73 只在**磁样本真的更新**时施加一次卡尔曼更新。\n'
    '             * 实测 ist_cnt 变化沿 8537 个 / 44.04 s = 193.8 Hz（磁物理率），\n'
    '             * 而 M7 每 23 帧的 EKF 周期被调用一次 = 351.7 Hz -> 同一个样本\n'
    '             * 被重复施加 1.81 次（观测被当成独立样本，等效增益与协方差削减\n'
    '             * 都放大 1.81 倍）。这不是降频：是把假的高速改回真实观测率，\n'
    '             * 陈旧量仍由既有的 s_mag_dth 机体转动补偿处理，无额外延迟。\n'
    '             * 被挡住的周期把三个 dq 显式清零，使日志语义唯一：\n'
    '             * dqz == 0 就是"本周期没有施加修正"（死区 / 无新样本），\n'
    '             * 不再是一个陈旧值。 */\n'
    '            {\n'
    '                uint32_t icm = g_shm ? g_shm->ist.hdr.cnt : 0u;\n'
    '                if (icm == s_mag_cnt_upd) {\n'
    '                    s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;\n'
    '                    return;\n'
    '                }\n'
    '                s_mag_cnt_upd = icm;\n'
    '            }\n'
    '            /* ★VER=56 慢环符号翻转：实测开机 mag_r=1.16 度（已在磁北）',
    'gate')
n2 = save(EKF, e, 'v73')

# ---------------- 3. 自检 ----------------
t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        73u' in t2
assert '#define V5F_EKF_Q_YAW_MIN        1.0e-5f' in t2
for name, s in (('proc_ekf.c', e2),):
    ob, cb = s.count('{'), s.count('}')
    oc, cc = s.count('/*'), s.count('*/')
    op, cp = s.count('('), s.count(')')
    print('%-11s { } %d/%d 差 %+d   /* */ %d/%d 差 %+d   ( ) %d/%d 差 %+d'
          % (name, ob, cb, ob-cb, oc, cc, oc-cc, op, cp, op-cp))
    assert ob == cb and oc == cc
print()
print('v5f_tune.h %d 字节 ; proc_ekf.c %d 字节' % (n1, n2))
print()
print('--- 关键行回读 ---')
for ln in e2.split('\n'):
    if ('s_mag_cnt_upd' in ln or 'Q_YAW_MIN' in ln or 'icm == ' in ln
        or '2u, rk, V5F_EKF_NIS_MAX_MAG_NOSW' in ln or 'if (row == 8u)' in ln):
        print('   ' + ln.strip())
print()
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
assert blk.count('ekf_update(R, 2u, r,') == 0, '仍有用 r 做新息的地方'
assert blk.count('ekf_update(R, 2u, rk,') == 1, 'rk 新息调用数不是 1'
print('M7 结构自检: rk 新息调用 1 处、旧 r 调用 0 处、样本门 1 处 (%d)'
      % blk.count('s_mag_cnt_upd'))
print()
print('PASS: VER=73 已写入')
