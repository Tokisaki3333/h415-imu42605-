# -*- coding: utf-8 -*-
r"""VER=122 = VER=118（最后一个好用版本）+ ①恢复 VER=119 陀螺精度标定
                            + ②地磁修正速度用 **EKF 标准数学** 实现（= 2 dps 上限）。

用户口径：
  * VER=118 是最后一个好用版本；119/120/121 作废回退（docs/mag_calibration_log.md §8.6）。
  * "恢复119的精度标定"。
  * "卡严地磁修正速度限制为先前讨论值"，且"地磁修正速度以 ekf 标准数学实现"
    => **不许再用"给 K 打钳位"这类非标准手法**；修正速度必须由 R、Q、K=P H'/(H P H'+R) 自然给出。

① 精度标定（v119）
   v5f_proc.h：per-axis 陀螺 LSB 用 v119 实测值
     X 16.2753 -> 16.2608（-890 ppm）、Y 16.4793 -> 16.3992（-4860 ppm）、Z 16.4235 -> 16.4431（+1200 ppm）
   交叉轴 KXY/KXZ/KYZ 值不变。tune：GYRO_KS 5e-3 -> 2e-3（v119 配套；只作用于倾角行）。

② 地磁修正速度 = EKF 标准数学（本版核心）

   VER=118 的偏航修正速度其实**不是**由物理决定的，而是被两个人造量顶上去的：
     * `V5F_EKF_Q_YAW_MIN = 1.0e-5`（每 epoch 直接加进 P[8][8]）≈ 5.0e-3 rad²/s 谱密度
       —— 比陀螺真实随机游走大 17 万倍；P[8][8] 因此被顶大，K 变大，修正被"强制加快"。
     * `V5F_EKF_YAW_P_MIN = (SIG_DEG·DEG2RAD)² = (2°)²` —— P[8][8] 的地板就是"地磁自身误差"，
       于是 K = P/(P+R) ≥ 0.5，地磁恒定满权重。
   结果：VER=118 静止时 τ ≈ 0.04 s（1° 偏差约 9.5 dps 的修正速度）。

   本版把偏航行的 Q 与地板都按**标准记账**：
     Q_yaw = ARW² + (KS_YAW·|w|·DEG2RAD)² ，并整体不超过 Q_MAX
       ARW    = 0.58 deg/rt-h（S3 实测随机游走，accel_position_design.md 7）
       KS_YAW = 1000 ppm（用户口径：满量程 1000 ppm 漂移）；原式漏 DEG2RAD，等效放大 3283 倍
       Q_MAX  = R/((h/dt_e)²·dt_e) <=> **闭环时间常数 τ ≥ τ_min = 1 s**
                （R=(2°)², h=cos(dip)=0.5831, dt_e=1/501.4 s -> Q_MAX = 7.14e-06 rad²/s）
     R_yaw  = (2°)²（地磁自身误差，保持 VER=118 的诚实取值，不动）
     Q_YAW_MIN 1.0e-5 -> 1.0e-12（纯数值下限）；YAW_P_MIN (2°)² -> (0.01°)²（纯数值地板）

   于是修正速度完全由 K = P h/(h²P+R) 决定，稳态 τ = dt·sqrt(R/(Q·dt))/h：
       静止        Q=2.85e-08  τ=15.9 s   （1° 偏差 -> 0.063 dps，远低于上限）
       20 dps      Q=1.50e-07  τ= 6.9 s
       50 dps      Q=7.90e-07  τ= 3.0 s
       >=153 dps   Q 被 Q_MAX 卡住  τ= 1.0 s
   速率上限核对（用户"先前讨论值"= 1000 ppm x FS(2000 dps) = **2 dps**）：
       τ ≥ 1 s  =>  修正速度 = 误差/τ ≤ 2 dps 只要误差 ≤ 2° = 地磁自身误差 σ_mag。
       即"地磁只在陀螺误差已经涨到地磁自身误差那个量级时，才以刚好能抵消满量程漂移的速度牵引"。
       满量程漂移(2 dps)下的稳态滞后 = 漂移率×τ = 2 dps × 1 s = 2° = σ_mag（自洽）。

   **不加任何 K 钳位/速率钳位**：全部走 R、Q、K 的标准公式。

改文件：V5F/User/inc/v5f_proc.h、V5F/User/inc/v5f_tune.h、V5F/User/src/proc_ekf.c
用法: python tools/ekf_session/patch_v122_magrate.py
回退: Copy-Item bak_src\V5F\User\inc\v5f_tune.h.bak_v118_LASTGOOD V5F\User\inc\v5f_tune.h -Force
      Copy-Item bak_src\V5F\User\inc\v5f_proc.h.bak_v118_LASTGOOD V5F\User\inc\v5f_proc.h -Force
      Copy-Item bak_src\V5F\User\src\proc_ekf.c.bak_v118_LASTGOOD V5F\User\src\proc_ekf.c -Force
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
PROC = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_proc.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v122'

NEW_TUNE = (
    '/* VER=122 偏航行的"角随机游走"基底（rad/rt-s）。\n'
    ' * S3 实测陀螺随机游走 N_eff = 0.58 deg/rt-h（docs/accel_position_design.md 7）\n'
    ' *   = 0.009667 deg/rt-s -> (0.009667*DEG2RAD)^2 = 2.851e-08 rad^2/s。\n'
    ' * **不要**再用 V5F_EKF_SIG_G_DPS(0.1224 dps)：那是 8 kHz 单样本 RMS，当谱密度用\n'
    ' *   等效 7.34 deg/rt-h（大 12.6 倍，Q 大 336 倍）-> P[8][8] 被顶大 -> 地磁修正被强制加快。 */\n'
    '#define V5F_EKF_Q_YAW_ARW       (0.58f / 60.0f * 0.017453292f)\n'
    '/* VER=122 偏航行的"转动标度"系数（无量纲）：用户口径 = 满量程 1000 ppm。\n'
    ' * ICM-42605 量程 +-2000 dps（16.4 LSB/dps，v5f_tune.h 141 行）。\n'
    ' * 只作用于偏航行；倾角行仍用 V5F_EKF_GYRO_KS(2e-3，v119 口径)。 */\n'
    '#define V5F_EKF_GYRO_KS_YAW     1.0e-3f\n'
    '/* VER=122 地磁修正速度的**等效上限**（偏航行 Q 的上限，rad^2/s）。\n'
    ' * "地磁修正速度以 EKF 标准数学实现"：不钳 K，而是把 Q 限制住，让标准增益\n'
    ' *   K = P h/(h^2 P + R)、tau = dt*sqrt(R/(Q*dt))/h 自然给出速度上限。\n'
    ' * 推导：要求 tau >= tau_min = 1 s（用户"先前讨论值"= 1000 ppm x FS(2000 dps) = 2 dps，\n'
    ' *   即"误差 <= 地磁自身误差 2 deg 时修正速度 <= 2 dps"）：\n'
    ' *     Q_MAX = R_yaw / ((h/dt_e)^2 * dt_e)\n'
    ' *           = (2*DEG2RAD)^2 / ((0.5831/0.001995)^2 * 0.001995)\n'
    ' *           = 1.2185e-03 / 170.45 = 7.14e-06 rad^2/s\n'
    ' *   （h = cos(dip) = 0.5831 = e1 方向对偏航的灵敏度，dt_e = 1/501.4 s）\n'
    ' * 满量程漂移(2 dps)下的稳态滞后 = 漂移率*tau = 2 dps * 1 s = 2 deg = 地磁自身误差（自洽）。 */\n'
    '#define V5F_EKF_Q_YAW_MAX       7.14e-6f\n')

NEW_QBLOCK = (
    '        sg2 = sq;\n'
    '        /* ---- VER=122 偏航行(row 8)的 Q 按 EKF 标准记账（用户：地磁修正速度用 ekf 标准数学实现）----\n'
    '         * Q_yaw = ARW^2 + (KS_YAW*|w|*DEG2RAD)^2，并整体 <= V5F_EKF_Q_YAW_MAX。\n'
    '         *   1) ARW 用 S3 实测随机游走 V5F_EKF_Q_YAW_ARW；原式把 8 kHz 单样本 RMS\n'
    '         *      (0.1224 dps) 当谱密度用，等效 7.34 deg/rt-h（Q 大 336 倍）；\n'
    '         *   2) 转动项用 1000 ppm 口径 V5F_EKF_GYRO_KS_YAW 并补上 DEG2RAD\n'
    '         *      （sq 其它项都是 rad 量纲；原式漏掉 = 放大 1/DEG2RAD^2 = 3283 倍）；\n'
    '         *   3) Q_MAX 等价于"闭环 tau >= 1 s"，即修正速度 <= 2 dps（误差 <= 2 deg 时）；\n'
    '         *   4) 倾角行(row 6/7)保持 VER=118/119 原状，不动已验收的通道。 */\n'
    '        {\n'
    '            float kry = V5F_EKF_GYRO_KS_YAW * s_wmag * DEG2RAD;\n'
    '            float ark = V5F_EKF_Q_YAW_ARW;\n'
    '            sq_yaw = ark * ark + kry * kry;\n'
    '            if (sq_yaw > V5F_EKF_Q_YAW_MAX) sq_yaw = V5F_EKF_Q_YAW_MAX;\n'
    '        }')

ANCH_DECL = '    float sa2, sg2, qd, dtq = s_dt_prev;'
ANCH_SG2 = '        sg2 = sq;'
ANCH_ROW = '            else if (row >= 6u  && row <= 8u)  qd += sg2 * dtq;'


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+122u', tune):
        print('已是 VER=122（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+118u', tune), '基线不是 VER=118'
        assert re.search(r'#define V5F_EKF_GYRO_KS\s+5\.0e-3f', tune), 'KS 不是 5e-3'
        assert re.search(r'#define V5F_EKF_Q_YAW_MIN\s+1\.0e-5f', tune), 'Q_YAW_MIN 不是 1e-5'
        for k in ('V5F_EKF_Q_YAW_ARW', 'V5F_EKF_Q_YAW_MAX', 'V5F_EKF_GYRO_KS_YAW'):
            assert k not in tune
        assert 'sq_yaw' not in ekf and 's_mag_yaw_lim' not in ekf
        for a in (ANCH_DECL, ANCH_SG2, ANCH_ROW):
            assert ekf.count(a) == 1, ('锚点 x%d: %r' % (ekf.count(a), a[:50]))
        NEW_TUNE.encode('gbk')

        for f in (TUNE, PROC, EKF):
            B.save(f, TAG)

        # ---- ① 恢复 VER=119 精度标定 ----
        B.restore(PROC, '.bak_v120')      # 该备份里是 v119 的 v5f_proc.h

        # ---- ② proc_ekf.c：偏航 Q 标准记账（不钳 K）----
        ekf = ekf.replace(ANCH_DECL, '    float sa2, sg2, sq_yaw = 0.0f, qd, dtq = s_dt_prev;', 1)
        ekf = ekf.replace(ANCH_SG2, NEW_QBLOCK, 1)
        ekf = ekf.replace(ANCH_ROW,
                          '            else if (row >= 6u  && row <= 8u)  qd += ((row == 8u) ? sq_yaw : sg2) * dtq;',
                          1)
        open(EKF, 'wb').write(ekf.encode('gbk'))

        # ---- tune ----
        m = re.search(r'#define V5F_EKF_MAG_VEC_SIG_TILT_DEG[^\n]*\n', tune)
        assert m, 'SIG_TILT 锚点'
        tune = tune[:m.end()] + NEW_TUNE + tune[m.end():]
        tune, nk = re.subn(r'#define V5F_EKF_GYRO_KS\s+5\.0e-3f[^\n]*',
                           '#define V5F_EKF_GYRO_KS          2.0e-3f  /* VER=122: 恢复 v119 口径'
                           '（固定标度修掉常数项后剩速率相关 ~2000ppm）；\n'
                           '                                                  * 用户口径是满量程'
                           ' 1000 ppm，偏航行已单列 V5F_EKF_GYRO_KS_YAW */', tune, count=1)
        assert nk == 1, 'KS 锚点'
        tune, nq = re.subn(r'#define V5F_EKF_Q_YAW_MIN\s+1\.0e-5f',
                           '#define V5F_EKF_Q_YAW_MIN        1.0e-12f /* VER=122: 降为纯数值下限；'
                           '随机游走改由 V5F_EKF_Q_YAW_ARW 承载 */', tune, count=1)
        assert nq == 1, 'Q_YAW_MIN 锚点'
        tune, np_ = re.subn(
            r'#define V5F_EKF_YAW_P_MIN\s*\\\n\s*\(\(V5F_EKF_MAG_VEC_SIG_DEG \* 0\.017453292f\) \* '
            r'\(V5F_EKF_MAG_VEC_SIG_DEG \* 0\.017453292f\)\)',
            '#define V5F_EKF_YAW_P_MIN        ((0.01f * 0.017453292f) * (0.01f * 0.017453292f))  '
            '/* VER=122: 原为地磁 sigma^2=(2deg)^2，使 K>=0.5（地磁恒定满权重）；改为纯数值地板 */',
            tune, count=1)
        assert np_ == 1, 'YAW_P_MIN 锚点'
        tune, n = re.subn(r'#define V5F_FW_VER\s+118u', '#define V5F_FW_VER        122u', tune, count=1)
        assert n == 1
        open(TUNE, 'wb').write(tune.encode('gbk'))
        print('patched: v5f_proc.h(v119 per-axis LSB) + tune(+Q_YAW_ARW,+GYRO_KS_YAW,+Q_YAW_MAX,'
              ' Q_YAW_MIN=1e-12, YAW_P_MIN=0.01deg, GYRO_KS=2e-3, VER=122u)')
        print('         proc_ekf.c(偏航 Q 标准记账，无任何 K 钳位)')

    # ---------------- verify ----------------
    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    proc = open(PROC, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+122u', tune)
    assert re.search(r'#define V5F_EKF_GYRO_KS\s+2\.0e-3f', tune)
    assert re.search(r'#define V5F_EKF_GYRO_KS_YAW\s+1\.0e-3f', tune)
    assert re.search(r'#define V5F_EKF_Q_YAW_ARW\s+\(0\.58f / 60\.0f \* 0\.017453292f\)', tune)
    assert re.search(r'#define V5F_EKF_Q_YAW_MAX\s+7\.14e-6f', tune)
    assert re.search(r'#define V5F_EKF_Q_YAW_MIN\s+1\.0e-12f', tune)
    assert re.search(r'#define V5F_EKF_YAW_P_MIN\s+\(\(0\.01f \* 0\.017453292f\)', tune)
    assert re.search(r'#define V5F_GYRO_LSB_PER_DPS_X\s+16\.2608f', proc)
    assert re.search(r'#define V5F_GYRO_LSB_PER_DPS_Y\s+16\.3992f', proc)
    assert re.search(r'#define V5F_GYRO_LSB_PER_DPS_Z\s+16\.4431f', proc)
    assert 'sq_yaw = ark * ark + kry * kry;' in ekf
    assert '((row == 8u) ? sq_yaw : sg2) * dtq' in ekf
    # 无任何 K 钳位/速率钳位残留
    for k in ('s_mag_yaw_lim', 'scy', 's_krow_scale', 's_kscale', 'V5F_EKF_MAG_YAW_RATE'):
        assert k not in ekf and k not in tune, ('残留 %s' % k)
    for k in ('sy2', 'V5F_EKF_MAG_GATE'):
        assert k not in ekf and k not in tune, ('残留 %s' % k)
    assert 'float ss = V5F_EKF_GYRO_KS * s_wmag;' in ekf and 'RRv[0] = sg2 * sg2;' in ekf
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'), '括号不平衡'
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_GYRO_KS[^\n]*',
                r'#define V5F_EKF_GYRO_KS_YAW[^\n]*', r'#define V5F_EKF_Q_YAW_ARW[^\n]*',
                r'#define V5F_EKF_Q_YAW_MAX[^\n]*', r'#define V5F_EKF_Q_YAW_MIN[^\n]*',
                r'#define V5F_EKF_YAW_P_MIN[^\n]*', r'#define V5F_EKF_MAG_VEC_SIG_DEG[^\n]*'):
        print('  |', re.search(pat, tune).group()[:116])
    print('---- v5f_proc.h ----')
    for pat in (r'#define V5F_GYRO_LSB_PER_DPS_[XYZ][^\n]*',):
        for mm in re.findall(pat, proc):
            print('  |', mm[:96])
    print('---- proc_ekf.c 偏航 Q 标准记账 ----')
    prop = ekf.split('static void ekf_prop_row')[1]
    k = prop.find('VER=122 偏航行(row 8)')
    for s in prop[k - 60:k + 1300].split('\n')[:22]:
        print('  |', s[:116])
    print()
    for mac in ('V5F_FW_VER', 'V5F_EKF_GYRO_KS', 'V5F_EKF_GYRO_KS_YAW', 'V5F_EKF_Q_YAW_ARW',
                'V5F_EKF_Q_YAW_MAX', 'V5F_EKF_Q_YAW_MIN', 'V5F_EKF_YAW_P_MIN'):
        n = len(re.findall(r'#define\s+%s\b' % mac, tune))
        print('LINT %-28s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= n == 1
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v118_LASTGOOD V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\inc\\v5f_proc.h.bak_v118_LASTGOOD V5F\\User\\inc\\v5f_proc.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v118_LASTGOOD V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
