# -*- coding: utf-8 -*-
r"""VER=130：**静止状态**下把地磁偏航牵引放开到 tau ≈ 1.5 s

用户口径（2026-09-22）："现在一致性门良好，请将静止状态下地磁牵引速度放开到 tau≈1.5 s。"
（前一句背景："进入静止状态后，地磁对姿态角依然有运动中积攒的残差，此时其被迫慢慢修正。"）

机理（全部走 EKF 标准数学：调 R / 调 P 地板，不夹 K）：
  静止判定：|w| < V5F_MAG_DIST_WMAX_DPS(5 dps，与一致性门同一判据) 且磁模幅度门 s_mn_ok 通过
  取本帧实际 R_yaw（入口 A 的 R1[0] = sig_h^2 + (tanI*sig_tilt)^2，入口 B 的 RRv[0] = sg2^2），
  把**偏航 P88 地板**抬到   P_min = R_yaw * K/(1-K)，  K = V5F_MAG_EPOCH_DT_S / V5F_MAG_STATIC_TAU_S
  => 闭环 K = P/(P+R) = K 恒成立 => tau 精确 = 1.5 s，**与 R 大小无关**（倾角不确定导致 R 变大时依然 1.5 s）。
  运动时地板回到 V5F_EKF_YAW_P_MIN（纯数值下限），牵引由自然增益决定（|w| 大时 tau≈0.95 s）。

为什么必须抬 P 而不是只压 R：K = P/(P+R)，静止时 P88≈4.5e-7、R≈(2deg)^2 -> K≈3.7e-4 -> tau≈14 s。
只压 R 无法把 K 抬到 3.5e-3；抬 P 同时把 NIS = nu^2/(P+R) 拉低，避免"运动残差被判成野值而拒收"
（V5F_EKF_NIS_MAX_MAG=16.27 -> R=4deg 时可接受创新约 16 deg）。

副作用（预期）：静止 sigma_yaw 由 0.04 deg 变为 sqrt(P_min)≈0.2 deg（这是"诚实的不确定度"）；
调试帧 col121 p_yy 相应变大；门控参数、入口选择、幅度门、k_cap 全部不动。

用法: python tools/ekf_session/patch_v130_static_fast.py
回退: bak_src\V5F\User\{inc\v5f_tune.h,src\proc_ekf.c}.bak_v130
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v130'

A_STATIC = 'static float    s_mag_rs;'
A_HELPER = 'static uint8_t ekf_mag_entry_plane(float thm, float thp)'
A_PLANE_R = '    R1[0] = (sig_h * DEG2RAD) * (sig_h * DEG2RAD) + dl * dl;'
A_VEC_RET = '    return ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],'
A_M7_TOP = ('    s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;\n'
            '    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;\n')
A_FLOOR = '            if (i == 8u && j == 8u && a < V5F_EKF_YAW_P_MIN) a = V5F_EKF_YAW_P_MIN;'
A_RESET = '            s_mag_dist = 0u; s_md_hold = 0u; s_md_n = 0u;'
A_TUNE_INS = '#define V5F_MAG_CLIP_LSB'

NEW_STATICS = (A_STATIC + '\n'
               '/* VER=130 静止态偏航牵引：s_yaw_p_min 为**运行时** P88 地板（默认纯数值下限），\n'
               ' * s_yaw_fast 为"本帧静止且磁可信"标志（在 ekf_m7_mag 开头判定）。 */\n'
               'static float    s_yaw_p_min = V5F_EKF_YAW_P_MIN;\n'
               'static uint8_t  s_yaw_fast;\n')

NEW_HELPER = (
    '/* VER=130 静止态牵引速度：由本帧实际 R_yaw 反解 P88 地板，使闭环 K = dt/tau 恒成立。\n'
    ' * 只在静止（s_yaw_fast）时生效；否则回到 V5F_EKF_YAW_P_MIN（= VER=129 行为）。 */\n'
    'static void ekf_yaw_pmin_static(float r_yaw)\n'
    '{\n'
    '    float k = V5F_MAG_EPOCH_DT_S / V5F_MAG_STATIC_TAU_S;\n'
    '    if (V5F_MAG_STATIC_TAU_EN != 0u && s_yaw_fast != 0u && k > 0.0f && k < 1.0f) {\n'
    '        float pf = r_yaw * (k / (1.0f - k));\n'
    '        s_yaw_p_min = (pf > V5F_EKF_YAW_P_MIN) ? pf : V5F_EKF_YAW_P_MIN;\n'
    '    } else {\n'
    '        s_yaw_p_min = V5F_EKF_YAW_P_MIN;\n'
    '    }\n'
    '}\n\n')

NEW_M7_TOP = (
    '    s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;\n'
    '    /* VER=130 静止态牵引判定：默认关闭（含门关时的早退路径） */\n'
    '    s_yaw_p_min = V5F_EKF_YAW_P_MIN;\n'
    '    {\n'
    '        float wm = sqrtf(h->imu.gyro_dps[0] * h->imu.gyro_dps[0]\n'
    '                       + h->imu.gyro_dps[1] * h->imu.gyro_dps[1]\n'
    '                       + h->imu.gyro_dps[2] * h->imu.gyro_dps[2]);\n'
    '        s_yaw_fast = (uint8_t)((wm < V5F_MAG_DIST_WMAX_DPS) && (s_mn_ok != 0u));\n'
    '    }\n'
    '    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;\n')

NEW_RESET = (A_RESET + '\n'
             '            s_yaw_p_min = V5F_EKF_YAW_P_MIN; s_yaw_fast = 0u;   /* VER=130 */')

NOTE_T = (
    '/* VER=130 静止态地磁牵引放开到 tau≈1.5 s（用户：一致性门良好，静止后要快速消掉\n'
    ' * 运动中积攒的偏航残差）。机制：静止（|w| < V5F_MAG_DIST_WMAX_DPS 且磁模幅度门\n'
    ' * 通过）时把偏航 P88 地板抬到 P_min = R_yaw*K/(1-K)，K = V5F_MAG_EPOCH_DT_S/tau，\n'
    ' * 于是 K = P/(P+R) = K 恒成立 -> tau 精确 1.5 s（与 R 无关）；运动时地板回到\n'
    ' * V5F_EKF_YAW_P_MIN（纯数值下限），牵引由自然增益决定（|w| 大时 tau≈0.95 s）。\n'
    ' * 必要性：只压 R 无法把 K 从 3.7e-4 抬到 3.5e-3；抬 P 同时把 NIS 拉低，避免运动\n'
    ' * 残差被 chi2(V5F_EKF_NIS_MAX_MAG=16.27) 判成野值。副作用：静止 sigma_yaw 由\n'
    ' * 0.04 deg -> sqrt(P_min)≈0.2 deg（诚实不确定度）。 */\n'
    '#define V5F_MAG_STATIC_TAU_S     1.5f\n'
    '#define V5F_MAG_STATIC_TAU_EN    1u\n')


def _w(path, text):
    b = text.encode('gbk')          # 先编码后打开（防截断）
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+130u', tune):
        print('已是 VER=130（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+129u', tune), '基线不是 VER=129'
        for a in (A_STATIC, A_HELPER, A_PLANE_R, A_VEC_RET, A_M7_TOP, A_FLOOR, A_RESET):
            assert ekf.count(a) == 1, '锚点不唯一: %s' % a[:60]
        assert tune.count(A_TUNE_INS) == 1, 'tune 锚点'

        B.save(TUNE, TAG)
        B.save(EKF, TAG)

        # ---- tune ----
        tune, n = re.subn(r'#define V5F_FW_VER\s+129u', '#define V5F_FW_VER        130u',
                          tune, count=1)
        assert n == 1
        i = tune.index(A_TUNE_INS)
        tune = tune[:i] + NOTE_T + tune[i:]
        _w(TUNE, tune)

        # ---- proc_ekf.c ----
        ekf = ekf.replace(A_STATIC, NEW_STATICS.rstrip('\n'), 1)
        ekf = ekf.replace(A_HELPER, NEW_HELPER + A_HELPER, 1)
        ekf = ekf.replace(A_PLANE_R,
                          A_PLANE_R + '\n    ekf_yaw_pmin_static(R1[0]);   /* VER=130 */', 1)
        ekf = ekf.replace(A_VEC_RET,
                          '    ekf_yaw_pmin_static(RRv[0]);   /* VER=130 */\n' + A_VEC_RET, 1)
        ekf = ekf.replace(A_M7_TOP, NEW_M7_TOP, 1)
        ekf = ekf.replace(A_FLOOR,
                          '            if (i == 8u && j == 8u && a < s_yaw_p_min) a = s_yaw_p_min;'
                          '   /* VER=130 */', 1)
        ekf = ekf.replace(A_RESET, NEW_RESET, 1)
        _w(EKF, ekf)
        print('patched: 静止态 P88 地板 -> tau = V5F_MAG_STATIC_TAU_S（运动态不变）')

    # ---------------- verify ----------------
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    ok = True
    ok &= bool(re.search(r'#define V5F_FW_VER\s+130u', tune))
    ok &= bool(re.search(r'#define V5F_MAG_STATIC_TAU_S\s+1\.5f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_STATIC_TAU_EN\s+1u', tune))
    ok &= bool(re.search(r'#define V5F_EKF_YAW_P_MIN\s+1\.0e-12f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_DIST_WMAX_DPS\s+5\.0f', tune))
    # 代码改动点
    ok &= (ekf.count('static void ekf_yaw_pmin_static(float r_yaw)') == 1)
    ok &= (ekf.count('ekf_yaw_pmin_static(R1[0]);') == 1)
    ok &= (ekf.count('ekf_yaw_pmin_static(RRv[0]);') == 1)
    ok &= (ekf.count('s_yaw_fast = (uint8_t)((wm < V5F_MAG_DIST_WMAX_DPS) && (s_mn_ok != 0u));') == 1)
    ok &= (ekf.count('if (i == 8u && j == 8u && a < s_yaw_p_min) a = s_yaw_p_min;') == 1)
    ok &= (ekf.count('a < V5F_EKF_YAW_P_MIN') == 0)
    ok &= (ekf.count('s_yaw_p_min = V5F_EKF_YAW_P_MIN; s_yaw_fast = 0u;') == 1)
    # 不该动的东西
    ok &= (ekf.count('float k_max = V5F_MAG_EPOCH_DT_S') == 0)
    ok &= (ekf.count('s_mag_ryaw_min') == 0)
    ok &= bool(re.search(r'#define V5F_MAG_DIST_THR_DPS\s+20\.0f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_DIST_KS\s+0\.0f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_DIST_HOLD_N\s+4011u', tune))
    ok &= bool(re.search(r'#define V5F_MAG_ERR_LIM\s+0\.10f', tune))
    ok &= bool(re.search(r'#define V5F_EKF_MAG_VEC_K_MAX\s+0\.10f', tune))
    ok &= (ekf.count('ekf_mag_entry_vec(r2v, RRv)') == 1)
    ok &= (ekf.count('ekf_mag_entry_plane(thm, thp)') == 1)
    ok &= (ekf.count('if (!clipped && wm < V5F_MAG_DIST_WMAX_DPS && rate > thr)') == 1)
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    ok &= (e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'))
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_MAG_STATIC_TAU_[A-Z_]*[^\n]*',
                r'#define V5F_EKF_YAW_P_MIN[^\n]*', r'#define V5F_MAG_DIST_WMAX_DPS[^\n]*'):
        for m in re.finditer(pat, tune):
            print('  |', m.group()[:112])
    print('---- 代码 ----')
    for a in ('static void ekf_yaw_pmin_static(float r_yaw)',
              '    ekf_yaw_pmin_static(R1[0]);',
              '    ekf_yaw_pmin_static(RRv[0]);',
              '        s_yaw_fast = (uint8_t)((wm < V5F_MAG_DIST_WMAX_DPS) && (s_mn_ok != 0u));',
              'if (i == 8u && j == 8u && a < s_yaw_p_min) a = s_yaw_p_min;'):
        print('  %-72s x%d' % (a[:72], ekf.count(a)))
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v130 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v130 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
