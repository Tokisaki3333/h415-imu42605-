# -*- coding: utf-8 -*-
"""
VER=72：修正 M7 地磁慢环的**新息符号**（唯一根因）

实测（VER=70 录制 14.33 s，144 列，fw_tag 4624391）：
  * 2.695~2.703 s 残差 |psi| 从 9.4 度单调涨到 93 度，而 mag_dqz 恰好等于
    k_cap 上限（0.05*|r| ≈ 2.2 度）；
  * 同窗口 psi 均值 +121.5 度、dqz 均值 +1.515 度，**同号占比 100%**（n=919）；
  * 全段 |dqz|>0.02 时 corr(psi, dqz) = +0.383；
  * v 是世界系磁场（机体转动时应恒定），其方位角只能被施加的修正改变：
    d(az of v) 与 dqz 同号占 95.9% -> psi_new = psi + dqz。
    要收敛必须 dqz = -psi；实测 dqz = +psi => 正反馈，稳定点是 180 度**假点**。
  * 代码比对：M1/M2/M4/M6 全是 r = z - h(x)，仅 M7 写成 r = h(x) - z。

改动：只翻转**喂给卡尔曼**的那一份新息（新增 rk[]），几何残差 r 与
      s_mag_rx/ry、mag_r 的定义全部不变（仍是 v-v0），日志语义不变，
      整角对齐的 psi 也不受影响（它只用 crs/dt2，与 r 的符号无关）。
另加 mag.f 模长守卫：上报里实测有 0.35% 的帧 |f| 达 1948~3430（成簇、
随时间线性增长），而 M7 的 R_MAX 判据是尺度无关的（|r|/|v| 对 |f| 同比缩放）
挡不住它；一次 |f|=1954 的新息经 k_cap 会给 dx[8]≈97 rad，注入后四元数直接废掉。
"""
import os, shutil, sys

ENC  = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')

def load(p):
    with open(p, 'rb') as f:
        return f.read().decode(ENC)

def save(p, text, tag):
    data = text.encode(ENC)                 # 先编码：失败就一个字都不动
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f:
        f.write(data)
    with open(p, 'rb') as f:                # 回读校验
        back = f.read().decode(ENC)
    assert back == text, '回读校验失败 ' + p
    return len(data)

def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '[%s] 锚点命中 %d 次（必须恰好 1 次）' % (what, n)
    return t.replace(old, new, 1)

# ---------------- 1. v5f_tune.h: VER 71 -> 72 ----------------
t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        71u', '#define V5F_FW_VER        72u', 'VER')
n1 = save(TUNE, t, 'v72')

# ---------------- 2. proc_ekf.c ----------------
e = load(EKF)
assert e.count('rk[') == 0, 'rk 标识符已被占用'

# 2a) 局部数组
e = sub1(e,
    '    float R[4], r[2], Rt[3][3], Bn[3], mf[3], fhb2;',
    '    float R[4], r[2], rk[2], Rt[3][3], Bn[3], mf[3], fhb2;',
    'decl')

# 2b) 模长守卫
e = sub1(e,
    '    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;\n'
    '    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];\n',
    '    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;\n'
    '    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];\n'
    '    /* ★VER=72 量纲守卫：mag.f 由驱动归一化，正常帧 |f| 恒为 1.0000（实测）。\n'
    '     * 上报里实测有 0.35% 的帧 |f| 达 1948~3430（成簇、随时间线性增长，疑似\n'
    '     * 与计数器别名/半写），而 M7 的 R_MAX 判据是**尺度无关**的（s_mag_r = |r|/|v|\n'
    '     * 对 |f| 同比缩放，永远 <= ~115 度）根本挡不住：一次 |f|=1954 的新息经\n'
    '     * k_cap(0.05) 会给 dx[8]≈97 rad，注入后四元数直接废掉。按模长直接拒绝。*/\n'
    '    {\n'
    '        float f2 = mf[0]*mf[0] + mf[1]*mf[1] + mf[2]*mf[2];\n'
    '        if (f2 < 0.25f || f2 > 2.25f) {\n'
    '            if (s_rej[3] < 250u) s_rej[3]++;\n'
    '            s_gate_bits |= V5F_EKF_GB_CHI2;\n'
    '            return;\n'
    '        }\n'
    '    }\n',
    'guard')

# 2c) 计算卡尔曼新息 rk = z - h(x)，几何残差 r 保持 v-v0 不变
e = sub1(e,
    '        s_mag_rx = r[0];\n'
    '        s_mag_ry = r[1];\n',
    '        s_mag_rx = r[0];\n'
    '        s_mag_ry = r[1];\n'
    '        /* ★VER=72 ★★★ 本条是整条地磁链的**根因修正** ★★★\n'
    '         * 标准卡尔曼是 r = z - h(x)；本块原先写成 r = h(x) - z，于是 dx = K*r\n'
    '         * 方向相反，施加后误差**翻倍** = 正反馈，稳定点是 180 度**假点**。\n'
    '         * 代码比对：M1 位置 / M2 高度 / M4 速度 / M6 倾斜 全部是 z - h(x)，\n'
    '         * 七个观测块里只有 M7 是 h - z。\n'
    '         * 实测证据（VER=70 录制 14.33 s，144 列）：\n'
    '         *   2.695~2.703 s 残差 |psi| 由 9.4 度单调涨到 93 度，而 mag_dqz 恰好\n'
    '         *   等于 k_cap 上限（0.05*|r| 约 2.2 度）；同窗口 psi 均值 +121.5 度、\n'
    '         *   dqz 均值 +1.515 度，**同号占比 100%**（n=919）；全段\n'
    '         *   corr(psi,dqz) = +0.383。v 是世界系磁场（机体转动时应恒定），其方位\n'
    '         *   角只能被施加的修正改变：d(az of v) 与 dqz 同号占 95.9%\n'
    '         *   -> psi_new = psi + dqz；要收敛必须 dqz = -psi，实测 dqz = +psi。\n'
    '         * 这里只翻转喂给卡尔曼的那一份；上面的几何残差 r（= v - v0）以及\n'
    '         * s_mag_r / s_mag_rx / s_mag_ry / 死区 全部保持原定义，日志语义不变，\n'
    '         * 整角对齐用的 psi = atan2(crs,dt2) 与 r 的符号无关，也不受影响。*/\n'
    '        rk[0] = b0x - Bn[0];\n'
    '        rk[1] = b0y - Bn[1];\n',
    'rk')

# 2d) 喂给卡尔曼的量换成 rk
e = sub1(e,
    '                        ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_MAG_NOSW, &s_nis[4], &s_rej[3],\n'
    '                            0x0100u, V5F_EKF_MAG_K_MAX);\n',
    '                        ekf_update(R, 2u, rk, V5F_EKF_NIS_MAX_MAG_NOSW, &s_nis[4], &s_rej[3],\n'
    '                            0x0100u, V5F_EKF_MAG_K_MAX);\n',
    'call')

n2 = save(EKF, e, 'v72')

# ---------------- 3. 结构自检 ----------------
def bal(s, a, b):
    return s.count(a), s.count(b)

t2 = load(TUNE); e2 = load(EKF)
assert '#define V5F_FW_VER        72u' in t2
print('v5f_tune.h  %d 字节' % n1)
print('proc_ekf.c  %d 字节' % n2)
print()
print('--- 结构自检 ---')
for name, s in (('proc_ekf.c', e2),):
    ob, cb = bal(s, '{', '}')
    oc, cc = bal(s, '/*', '*/')
    op, cp = bal(s, '(', ')')
    print('%-11s { } %d/%d 差 %+d    /* */ %d/%d 差 %+d    ( ) %d/%d 差 %+d (括号差 -4 为中文注释里的历史遗留)'
          % (name, ob, cb, ob-cb, oc, cc, oc-cc, op, cp, op-cp))
    assert ob == cb, '{ } 不平衡'
    assert oc == cc, '/* */ 不平衡'
print()
print('--- 关键行回读 ---')
for ln in e2.split('\n'):
    if ('rk[0] = b0x' in ln or 'rk[1] = b0y' in ln or 'float R[4], r[2], rk[2]' in ln
        or '2u, rk, V5F_EKF_NIS_MAX_MAG_NOSW' in ln or '0x0100u, V5F_EKF_MAG_K_MAX' in ln
        or 'f2 < 0.25f' in ln):
        print('   ' + ln.strip())
print()
i = e2.find('static void ekf_m7_mag')
j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
print('M7 函数体: %d 字节; 其中 rk 出现 %d 次, r 出现(作为新息给卡尔曼) %d 次'
      % (len(blk.encode(ENC)), blk.count('rk'), blk.count('ekf_update(R, 2u, r,')))
assert blk.count('ekf_update(R, 2u, r,') == 0, '仍有一处用 r 做新息'
print()
print('PASS: VER=72 已写入')
