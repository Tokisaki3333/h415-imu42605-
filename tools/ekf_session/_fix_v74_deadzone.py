# -*- coding: utf-8 -*-
"""
VER=74：把两个被实测证伪的常量改对

改动 1 —— 死区 V5F_EKF_MAG_DEAD_DEG 12 -> 3 度
  12 度的依据是注释里"磁角典型误差可达 10 度"，但 VER=73 录制（44 s，
  v0 校验字滤掉 4.95% 污染帧）实测：
    * 开机静止段（|w|=0.2 dps，14 s）：mag_r 仅 2.39 -> 2.69 度，
      **真实偏航误差 |psi| 只有 0.43 -> 1.27 度**；
    * 剧烈运动后回到静止（36~44 s）：|psi| = 10.36 度（max 11.53），
      而 mag_r = 10.84~10.95 —— 恰好卡在 12 度死区下面，
      **环路拒绝把它拉回去**，这个 10.4 度纯粹是死区留下的残渣。
    * 死区占用统计：12 度 -> 环路休眠 70.5%、可更新仅 23.6%；
      3 度 -> 休眠 36.1%、可更新 58.1%。
  即 12 度比磁力计的真实误差大约 10 倍。取 3.0 度：
    * 它略高于 mag_r 的**模长失配底**（|v| 与 |v0| 不等造成的偏移，
      实测干净静止约 2.4 度）；低于它就不会去追模长误差；
    * 高于它即修正，于是静止末态从 10.4 度收到 ~2 度；
    * 环路从"休眠->集中爆发"变成近似均匀的持续小步牵引。
  单步幅度仍由 k_cap=0.05 限幅：3 度误差时 0.14 度/更新（193.8 Hz = 28 度/秒，
  平滑不过冲）；100 度误差时 2.15 度/更新 = 417 度/秒（快速拉回）。

改动 2 —— 硬新息上限 V5F_EKF_MAG_R_MAX_DEG 150 -> 179 度
  实测：mag_r > 150 的帧 19460 个（占 5.59%），其中 mag_used==1 的有 **0 个**
  —— 残差一超 150 度就把整条观测丢掉，环路什么都不做。
  VER=73 录制里 30 s 处残差正是**挂在 175~176 度不动**（|w|=145 dps，
  既不是饱和也不是测量坏），直到运动衰减、残差落回 150 以下才以 ~40 度/秒 拉回。
  这个门是 VER=70 之前为"符号反了、大残差会发散"设的保护；VER=72 符号已修正，
  大残差现在是**可修的**，拒绝它恰恰是反的。
  安全性由三层保留：|mag.f| 模长守卫、BH_MIN 死点、k_cap(单步 <= 2.5 度)。
"""
import os, shutil

ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')

def load(p):
    with open(p, 'rb') as f:
        return f.read().decode(ENC)

def save(p, text, tag):
    data = text.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f:
        f.write(data)
    with open(p, 'rb') as f:
        assert f.read().decode(ENC) == text
    return len(data)

def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '[%s] 锚点命中 %d 次' % (what, n)
    return t.replace(old, new, 1)

t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        73u', '#define V5F_FW_VER        74u', 'VER')
t = sub1(t,
    '#define V5F_EKF_MAG_R_MAX_DEG    150.0f   /* 磁偏航硬新息门：|r| 超它整帧丢弃 */',
    '/* ★VER=74 150 -> 179：实测 mag_r>150 的帧 19460 个(5.59%)，其中 mag_used==1 的\n'
    ' * 有 0 个 —— 残差一超 150 度环路就整条放弃，VER=73 录制 30 s 处残差正是挂在\n'
    ' * 175~176 度不动，直到运动衰减才拉回。这个门是符号修正(VER=72)之前的保护，\n'
    ' * 现在大残差是可修的，拒绝它恰恰反了。安全性由 |mag.f| 模长守卫 + BH_MIN 死点\n'
    ' * + k_cap(单步<=2.5 度) 三层保留。 */\n'
    '#define V5F_EKF_MAG_R_MAX_DEG    179.0f   /* 磁偏航硬新息门：|r| 超它整帧丢弃 */',
    'R_MAX')
t = sub1(t,
    '#define V5F_EKF_MAG_DEAD_DEG      12.0f',
    '/* ★VER=74 12 -> 3 度。12 度的依据是"磁角典型误差可达 10 度"，但 VER=73 录制实测：\n'
    ' *   开机静止 14 s（|w|=0.2 dps）mag_r 仅 2.39~2.69 度、**真实 |psi| 只有 0.43~1.27 度**；\n'
    ' *   剧烈运动后回到静止 36~44 s，|psi|=10.36 度而 mag_r=10.84~10.95 —— 恰好卡在\n'
    ' *   12 度死区下面，环路拒绝拉回，这 10.4 度纯粹是死区留下的残渣。\n'
    ' *   死区占用：12 度 -> 休眠 70.5%/可更新 23.6% ; 3 度 -> 休眠 36.1%/可更新 58.1%。\n'
    ' * 取 3.0 度：略高于 mag_r 的模长失配底(实测干净静止约 2.4 度，低于它就会去追\n'
    ' * 模长误差)，高于它即修正 -> 静止末态由 10.4 度收到约 2 度，且牵引趋于均匀小步。\n'
    ' * 单步幅度仍由 k_cap 限幅：3 度误差 0.14 度/更新(193.8 Hz=28 度/秒，不过冲)；\n'
    ' * 100 度误差 2.15 度/更新 = 417 度/秒。 */\n'
    '#define V5F_EKF_MAG_DEAD_DEG      3.0f',
    'DEAD')
n = save(TUNE, t, 'v74')

t2 = load(TUNE)
assert '#define V5F_FW_VER        74u' in t2
assert '#define V5F_EKF_MAG_DEAD_DEG      3.0f' in t2
assert '#define V5F_EKF_MAG_R_MAX_DEG    179.0f' in t2
print('v5f_tune.h %d 字节' % n)
for ln in t2.split('\n'):
    if ('V5F_FW_VER' in ln or 'MAG_DEAD_DEG' in ln or 'MAG_R_MAX_DEG' in ln):
        print('   ' + ln.strip())
print()
print('PASS: VER=74 已写入')
