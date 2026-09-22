# -*- coding: utf-8 -*-
r"""VER=128：地磁偏航牵引速度上限改为**跟随设计曲线**（不是常数）

!!!! 状态：**未落盘 / 已按用户指示回滚（2026-09-22）** !!!!
     落盘版本仍是 VER=127（常数 K_max）；门控 VER=126/127 保持不动。
     本脚本仅作为"试验律可重放"的记录；回滚脚本 tools/ekf_session/rollback_v128.py。
     详见 docs/mag_calibration_log.md §8.13。

用户问题："你先前修改的地磁牵引速度，在极限运动中依然能有效牵引？"
答案：**不能**。VER=125 的 R 地板把 K 钉成 dt_mag/tau_min（常数，与 |w| 无关），
而陀螺自身的偏航残差率按设计口径 = KS_YAW*|w| 随转速线性上升：
    |w| = 2000 dps -> 残差 2 dps，tau 仍 15.85 s -> 稳态滞后 = 2*15.85 = 31.7 deg
实测录像 R:\imu_20260922_201451.bin（|w| p50 337 / p90 2010 / max 2524 dps）：
    A 现版常数地板：整体滞后 p50 5.3 / p90 31.9 / max 40.0 deg；|w|>500 段 p50 28.3 deg
    C VER=128 曲线：整体滞后 p50 0.34 / p90 2.01 / max 2.53 deg；|w|>500 段 p50 1.8 deg
    （C 律注入的地磁高频噪声 sigma_mag*sqrt(K/2) = 0.103 deg，可忽略）
脚本：tools/ekf_session/magpull_extreme.py

改动（仍用"抬 R"这一 EKF 标准手法，不夹 K）：
    Q(|w|) = min(q0^2 + (KS_YAW*|w|*DEG2RAD)^2, V5F_EKF_Q_YAW_MAX)   [与传播同一口径]
    tau_min(|w|) = V5F_MAG_YAW_TAU_MIN_S * sqrt(q0^2 / Q(|w|))
    K_max = V5F_MAG_EPOCH_DT_S / tau_min(|w|)
静止点**与 VER=125 完全一致**（K=3.32e-4, tau=15.85 s），故台面/小转动行为不变；
|w| 满量程时 K_max 升到 5.26e-3（tau = 1.0 s，受 V5F_EKF_Q_YAW_MAX 的 1 s 设计底线约束）。
地板此时只在 P88 被抬高（失效后重开等）时起作用，不再压制转速带来的合理加速。

用法: python tools/ekf_session/patch_v128_yaw_speedup.py
回退: bak_src\V5F\User\{inc\v5f_tune.h,src\proc_ekf.c}.bak_v128
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v128'

# tune 中 VER=125 "牵引速度上限" 注释块的唯一前缀（unicode 转义，避免脚本自身编码歧义）
ANCH_T = '/* VER=125 \u5730\u78c1\u504f\u822a"\u7275\u5f15\u901f\u5ea6"\u4e0b'

ANCHOR_KM = '        float k_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S;'
NEW_KM = (
    '        /* VER=128: K_max 不是常数，而跟随**设计曲线**（v5f_tune.h 同节注释）：\n'
    '         *   Q(|w|) = min(q0^2 + (KS_YAW*|w|*DEG2RAD)^2, V5F_EKF_Q_YAW_MAX)  [与传播同口径]\n'
    '         *   tau_min(|w|) = V5F_MAG_YAW_TAU_MIN_S * sqrt(q0^2 / Q(|w|))\n'
    '         *   K_max = V5F_MAG_EPOCH_DT_S / tau_min(|w|)\n'
    '         * 静止点与 VER=125 完全一致（3.32e-4 / 15.85 s）；|w| 满量程 5.26e-3（tau 1.0 s）。\n'
    '         * 实测 201451（|w| p90 2010 dps）稳态滞后 31.9 deg -> 2.0 deg。 */\n'
    '        float kry = V5F_EKF_GYRO_KS_YAW * s_wmag * DEG2RAD;\n'
    '        float qd = V5F_EKF_Q_YAW_ARW * V5F_EKF_Q_YAW_ARW + kry * kry;\n'
    '        if (qd > V5F_EKF_Q_YAW_MAX) qd = V5F_EKF_Q_YAW_MAX;\n'
    '        float k_max = V5F_MAG_EPOCH_DT_S\n'
    '                    * sqrtf(qd / (V5F_EKF_Q_YAW_ARW * V5F_EKF_Q_YAW_ARW))\n'
    '                    / V5F_MAG_YAW_TAU_MIN_S;\n'
    '        if (k_max > 1.0f) k_max = 1.0f;\n')

NEW_NOTE = (
    '/* VER=128 上式的 K_max 改为**跟随设计曲线**（原为常数，见 proc_ekf.c 对应块）：\n'
    ' * 陀螺偏航残差率按设计口径 = KS_YAW*|w| 随转速上升，常数地板会让极限运动下\n'
    ' * 地磁牵引被冻结在 15.85 s -> 稳态滞后 = 残差率*tau = 31.9 deg（实测 201451）。\n'
    ' * 现 tau_min(|w|) = tau0*sqrt(q0^2/Q(|w|))，Q 与传播同一口径（含 Q_YAW_MAX）。\n'
    ' * 静止点不变（15.85 s）；|w|=2000 dps 时 1.0 s -> 滞后 2.0 deg，注入噪声 0.10 deg。\n'
    ' * 实测（R:\\imu_20260922_201451.bin, |w| p50 337/p90 2010/max 2524 dps）：\n'
    ' *   常数地板 滞后 p50 5.3/p90 31.9/max 40.0 deg；曲线 0.34/2.01/2.53 deg。 */\n'
    '#define V5F_MAG_YAW_TAU_MIN_S    15.85f\n')


def _w(path, text):
    b = text.encode('gbk')          # 先编码，后打开（防截断）
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    # 允许重复执行：先回到 bak_v128（首次运行时它就是补丁前状态）
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    if (re.search(r'#define V5F_FW_VER\s+128u', tune)
            and not re.search(r'#define V5F_MAG_YAW_TAU_MIN_S', tune)):
        print('检测到半成品 VER=128（TAU_MIN 定义被注释块替换吞掉），先从 bak_v128 回退')
        B.restore(TUNE, TAG)
        B.restore(EKF, TAG)
        tune = open(TUNE, 'rb').read().decode('gbk')
        ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+128u', tune):
        print('已是 VER=128（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+127u', tune), '基线不是 VER=127'
        assert ekf.count(ANCHOR_KM) == 1, 'k_max 锚点不唯一'
        assert ekf.count('float kry = V5F_EKF_GYRO_KS_YAW * s_wmag * DEG2RAD;') == 1, 'kry 锚点'

        B.save(TUNE, TAG)
        B.save(EKF, TAG)

        tune, n = re.subn(r'#define V5F_FW_VER\s+127u', '#define V5F_FW_VER        128u',
                          tune, count=1)
        assert n == 1
        # 锚点用中文前缀（脚本为 UTF-8，tune 已按 GBK 解码，str 层可直接匹配）
        i = tune.index(ANCH_T)
        j = tune.index('\n', tune.index('#define V5F_MAG_YAW_TAU_MIN_S', i)) + 1
        assert tune.count(ANCH_T) == 1, 'TAU_MIN 注释块锚点不唯一'
        tune = tune[:i] + NEW_NOTE + tune[j:]
        _w(TUNE, tune)

        ekf = ekf.replace(ANCHOR_KM, NEW_KM.rstrip('\n'), 1)
        _w(EKF, ekf)
        print('patched: K_max -> 设计曲线（静止点不变，|w| 越大越快）')

    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    ok = True
    ok &= bool(re.search(r'#define V5F_FW_VER\s+128u', tune))
    ok &= bool(re.search(r'#define V5F_MAG_YAW_TAU_MIN_S\s+15\.85f', tune))
    ok &= bool(re.search(r'#define V5F_EKF_Q_YAW_MAX\s+7\.14e-6f', tune))
    ok &= bool(re.search(r'#define V5F_EKF_GYRO_KS_YAW\s+1\.0e-3f', tune))
    ok &= (ekf.count(ANCHOR_KM) == 0)
    ok &= (ekf.count('V5F_MAG_YAW_TAU_MIN_S;') == 1)
    ok &= (ekf.count('float kry = V5F_EKF_GYRO_KS_YAW * s_wmag * DEG2RAD;') == 2)
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    ok &= (e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'))
    ok &= bool(re.search(r'#define V5F_MAG_ERR_LIM\s+0\.10f', tune))
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_MAG_YAW_TAU_MIN_S[^\n]*',
                r'#define V5F_MAG_EPOCH_DT_S[^\n]*', r'#define V5F_EKF_GYRO_KS_YAW[^\n]*',
                r'#define V5F_EKF_Q_YAW_MAX[^\n]*'):
        print('  |', re.search(pat, tune).group()[:110])
    print('---- proc_ekf.c ----')
    i = ekf.index('V5F_MAG_YAW_TAU_MIN_S;')
    print(ekf[i - 620:i + 40].replace('\r', ''))
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v128 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v128 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
