# -*- coding: utf-8 -*-
r"""VER=125：把地磁"牵引速度"锁到设计值（唯一改动，幅度门不动）。

实测（新录像 R:\imu_20260922_195513.bin，设备静止 + 外部磁干扰）：
    p_yy(列121) p50 = 2.11e-03  sigma_yaw(列104) p50 = 2.63 deg
    -> K = P/(P+R) ~ 0.63，地磁按满权重硬拽，实测有效 tau ~ 2 s
    -> 30 s 内 EKF 偏航被拽 -62.5°（陀螺同期只积 -0.2°）
设计值应为：P88 = 4.51e-07、sigma_yaw = 0.039 deg、静态 tau = 15.85 s。

改两处，全部是标准 EKF 数学：
 ① `V5F_EKF_YAW_P_MIN`：`(0.01deg)^2` -> **1.0e-12**（纯数值地板；不再用"地磁的大 sigma^2"
    当前偏航 P 的地板 —— 实测 P88 恰好卡在旧地板 (2deg)^2 = 1.2185e-03 附近）。
 ② **牵引速度上限（与 P/R 记账无关，一劳永逸）**：每次地磁更新前把偏航行的 R 抬到
       R_yaw >= P88 * (1/K_max - 1),  K_max = dt_mag / tau_design
    即"声明地磁更吵"来限增益（标准手法），保证闭环 tau 恒 >= V5F_MAG_YAW_TAU_MIN_S = 15.85 s。
    两个入口（A 投影 / B 矢量）共用同一地板 s_mag_ryaw_min。

效果（就用这段录像算）：τ=15.85 s 把干扰的**快分量**滤掉，净漂移从 -62.5° 降到
    -8.6°（= 磁航向自身的慢分量），最大偏移从 63° 降到几度。
验收：静置录像 `p_yy` ≈ 4.5e-7、`sigma_yaw` ≈ 0.04°（若仍 2e-3 / 2.6° 说明这份固件没带上本改动）；
      同段静止+干扰录像 EKF 偏航净漂 <= ~9°、最大 <= ~5°。

用法: python tools/ekf_session/patch_v125_yaw_tau.py
回退: Copy-Item bak_src\V5F\User\inc\v5f_tune.h.bak_v125 V5F\User\inc\v5f_tune.h -Force
      Copy-Item bak_src\V5F\User\src\proc_ekf.c.bak_v125 V5F\User\src\proc_ekf.c -Force
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v125'

NEW_TUNE = (
    '/* VER=125 地磁偏航"牵引速度"下限（闭环时间常数，s）。\n'
    ' * 实测教训：刷进去的固件里 P[8][8] 卡在旧地板 (2 deg)^2 附近\n'
    ' *   -> K = P/(P+R) ~ 0.63 -> 地磁按满权重硬拽 -> 30 s 被拽 -62 deg。\n'
    ' * 本版把牵引速度**按构造**锁死：每次地磁更新前抬偏航行的 R，使\n'
    ' *   K <= K_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S\n'
    ' * 即 R_yaw >= P88*(1/K_max - 1)（"声明地磁更吵"来限增益，标准 EKF 手法），\n'
    ' * 与 P/R 记账是否精确无关 => 闭环 tau 恒 >= 本值。 */\n'
    '#define V5F_MAG_YAW_TAU_MIN_S    15.85f\n'
    '/* 地磁更新间隔（IST 采样 ~190 Hz；VER=122 设计 tau=15.85 s 就是按它算的）*/\n'
    '#define V5F_MAG_EPOCH_DT_S       0.00526f\n')

NEW_STATIC = ('static float    s_mag_ryaw_min;        /* VER=125 偏航 R 地板（牵引速度上限用）*/\n')


def _w(path, text):
    b = text.encode('gbk')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


ANCH_YPM = ('#define V5F_EKF_YAW_P_MIN        ((0.01f * 0.017453292f) * (0.01f * 0.017453292f))'
            '  /* VER=122: 原为地磁 sigma^2=(2deg)^2，使 K>=0.5（地磁恒定满权重）；改为纯数值地板 */')
ANCH_R1 = '    R1[0] = (sig_h * DEG2RAD) * (sig_h * DEG2RAD) + dl * dl;'
ANCH_MODE = '    if (s_grav_err > (uint32_t)(V5F_EKF_MAG_FF_THR_DEG * 1.0e6f)) {'
ANCH_STATIC = 'static float    s_mag_mode;'


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+125u', tune):
        print('已是 VER=125（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+124u', tune), '基线不是 VER=124'
        assert tune.count(ANCH_YPM) == 1, 'YAW_P_MIN 锚点'
        for a in (ANCH_R1, ANCH_MODE, ANCH_STATIC):
            assert ekf.count(a) == 1, ('锚点 x%d: %r' % (ekf.count(a), a[:50]))
        assert 's_mag_ryaw_min' not in ekf
        NEW_TUNE.encode('gbk')

        B.save(TUNE, TAG)
        B.save(EKF, TAG)

        # ---- ① YAW_P_MIN -> 纯数值地板 ----
        tune = tune.replace(ANCH_YPM,
            '#define V5F_EKF_YAW_P_MIN        1.0e-12f  /* VER=125: 纯数值地板；'
            '原 (2deg)^2 会把 P88 钉在大值上 -> K~0.63 -> 地磁满权重硬拽 */', 1)
        # ---- 新增两个常量 ----
        m = re.search(r'#define V5F_EKF_MAG_VEC_SIG_TILT_DEG[^\n]*\n', tune)
        assert m, 'SIG_TILT 锚点'
        tune = tune[:m.end()] + NEW_TUNE + tune[m.end():]
        tune, n = re.subn(r'#define V5F_FW_VER\s+124u', '#define V5F_FW_VER        125u', tune, count=1)
        assert n == 1
        _w(TUNE, tune)

        # ---- ② 静态量 + 入口 A 的 R 地板 + 更新前的 R 抬高 ----
        ekf = ekf.replace(ANCH_STATIC, NEW_STATIC + ANCH_STATIC, 1)
        ekf = ekf.replace(ANCH_R1, ANCH_R1 +
            '\n    /* VER=125 牵引速度上限：抬 R 使 K <= K_max（地板由 ekf_m7_mag 每次算） */'
            '\n    if (R1[0] < s_mag_ryaw_min) R1[0] = s_mag_ryaw_min;', 1)
        ekf = ekf.replace(ANCH_MODE,
            '    /* ---- VER=125 磁牵引速度上限：K = P/(P+R) <= K_max = dt_mag/tau_design -------\n'
            '     * 抬 R = 声明地磁更吵（标准 EKF 限增益），保证闭环 tau 恒 >= V5F_MAG_YAW_TAU_MIN_S，\n'
            '     * 与 P88 记账是否精确无关：R_min = P88*(1/K_max - 1)。两个入口共用同一地板。 */\n'
            '    {\n'
            '        float k_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S;\n'
            '        if (k_max < 1.0f && k_max > 1.0e-9f) {\n'
            '            float rmin = s_P[8][8] * (1.0f / k_max - 1.0f);\n'
            '            s_mag_ryaw_min = rmin;\n'
            '            if (RRv[0] < rmin) RRv[0] = rmin;      /* 入口 B 用 RRv[0] */\n'
            '        } else {\n'
            '            s_mag_ryaw_min = 0.0f;\n'
            '        }\n'
            '    }\n' + ANCH_MODE, 1)
        _w(EKF, ekf)
        print('patched: YAW_P_MIN=1e-12 + 牵引速度上限(K<=dt/tau, 地板抬 R)')

    # ---------------- verify ----------------
    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+125u', tune)
    assert re.search(r'#define V5F_EKF_YAW_P_MIN\s+1\.0e-12f', tune)
    assert re.search(r'#define V5F_MAG_YAW_TAU_MIN_S\s+15\.85f', tune)
    assert re.search(r'#define V5F_MAG_EPOCH_DT_S\s+0\.00526f', tune)
    assert 'V5F_EKF_Q_YAW_MIN        1.0e-12f' in tune
    assert ekf.count('static float    s_mag_ryaw_min;') == 1
    assert ekf.count('if (R1[0] < s_mag_ryaw_min) R1[0] = s_mag_ryaw_min;') == 1
    assert ekf.count('float rmin = s_P[8][8] * (1.0f / k_max - 1.0f);') == 1
    assert ekf.count('float k_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S;') == 1
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'), '括号不平衡'
    # 算一下设计点
    import math
    P, R, h, dtm, tau = 4.51e-7, 1.2185e-3, 0.5831, 5.26e-3, 15.85
    k = P / (P + R)
    print()
    print('  设计点核对: P88=%.3e R=%.4e -> K=%.3e -> tau=dtm/K=%.2f s (设计 %.2f s)'
          % (P, R, k, dtm / k, tau))
    print('  实测点核对: P88=2.11e-3  -> R_min=%.3f -> K=%.3e -> tau=%.2f s'
          % (2.11e-3 * (1 / (dtm / tau) - 1), 2.11e-3 / (2.11e-3 + 2.11e-3 * (1 / (dtm / tau) - 1)),
             dtm / (2.11e-3 / (2.11e-3 + 2.11e-3 * (1 / (dtm / tau) - 1)))))
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v125 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v125 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
