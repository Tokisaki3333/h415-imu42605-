# -*- coding: utf-8 -*-
r"""VER=121（最终 v3）：地磁偏航**闸门** + 姿态 Q/R 记账修正。

用户口径（五句，必须同时满足）：
  1) 小转动时地磁的**固定误差**绝不许进姿态（此时纯陀螺误差远小于地磁自身误差）；
  2) "在陀螺仪 R 足够小的数十甚至数百秒内都不应当让地磁快速牵引姿态"；
  3) "重力参与牵引的时间内应当正确缩小陀螺仪 R"；
  4) 只有陀螺**自己的**不确定度涨到超过地磁自身误差时，才让地磁接管；
  5) 牵引速度尽量慢，够抵消陀螺漂移率即可。

**被否掉的方案（记录，别再走）**：
  * τ=600 s  : 用 1.23 deg/h 当漂移率 -> 1000 ppm 口径下 300 dps 滞后 5.8 deg，跟不上；
  * R=0.2077 : 持续比例牵引，静置 tau=207 s / 20 dps tau=90 s，地磁固定误差(2~4.4 deg)
               会按这个时间常数一路灌进偏航（用户实测抓到）。τ 只决定"多快"，不决定"会不会"。
  根因：把地磁当**绝对航向观测**的持续环路，对地磁固定偏差的直流增益恒为 1。

**做法**：
 (1) 偏航 R 用诚实值 sigma_mag^2=(2 deg)^2；另加**闸门 w**（读 EKF 自己的 P[8][8]）：
        P[8][8] <= (2 deg)^2            -> w = 0   本次地磁更新整块不执行（K=0、P 不动）
        (2 deg)^2 < P < (4 deg)^2       -> w 线性 0->1
        P[8][8] >= (4 deg)^2            -> w = 1   满权重 KF 牵引
     P[8][8] = 陀螺自己的偏航累积不确定度（重力观测不到偏航，所以重力帮不上它，
     只能由 ARW + (1000ppm*|w|) 累积）=> 闸门时机只由陀螺自己决定。
     到达闸门：静止 ~35 min（含整流项）/ 20 dps ~6 min / 300 dps 44 s / 2000 dps 1.0 s
     => 常规 10 min 飞行内**地磁全程不参与**，它的 2~4.4 deg 固定误差进不来。
 (2) 姿态三行 Q 统一按"实测随机游走"记账（"重力参与时正确缩小陀螺 R"）：
        Q_att = ARW^2 + (KS*|w|*DEG2RAD)^2 + (0.85*|a_lin|*DEG2RAD)^2
        ARW = 0.58 deg/rt-h（S3 实测，accel_position_design.md 7）
        KS  = 1000 ppm（用户口径；原 2e-3 且漏了 DEG2RAD，等效放大 3283 倍）
        删掉把 8 kHz 单样本 RMS(0.1224 dps) 当谱密度的 SIG_G_RADS 项（Q 大 336 倍）
     -> 重力牵引时 P[6][6]/P[7][7] 收缩到"正确"量级（静置 sigma_tilt ~0.02 deg，
        机动 5 s@300dps -> ~0.68 deg 而不是原来的 77 deg），
        而速度泄漏 lambda = 5*sigma_tilt 自动跟着变正确。
        偏航行 P[8][8] 仍只由陀螺自己累积（重力不可观测偏航）。

改动文件：V5F/User/inc/v5f_tune.h、V5F/User/src/proc_ekf.c
用法: python tools/ekf_session/patch_v121_magpull.py
回退: Copy-Item bak_src\V5F\User\inc\v5f_tune.h.bak_v121 V5F\User\inc\v5f_tune.h -Force
      Copy-Item bak_src\V5F\User\src\proc_ekf.c.bak_v121 V5F\User\src\proc_ekf.c -Force
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v121'

NEW_TUNE = (
    '/* VER=121 姿态三行(roll/pitch/yaw)的"角随机游走"基底（rad/rt-s）。\n'
    ' * 依据：S3 实测陀螺随机游走 N_eff = 0.58 deg/rt-h\n'
    ' *   （docs/accel_position_design.md 7）= 0.009667 deg/rt-s\n'
    ' *   -> (0.009667*DEG2RAD)^2 = 2.851e-08 rad^2/s。\n'
    ' * 注意 **不要** 再用 V5F_EKF_SIG_G_DPS(0.1224 dps)：那是 8 kHz 的**单样本 RMS**，\n'
    ' *   当谱密度用会等效成 7.34 deg/rt-h（比实测大 12.6 倍，Q 大 336 倍）\n'
    ' *   -> 姿态被外部观测（重力/地磁）过度牵引。 */\n'
    '#define V5F_EKF_Q_ATT_ARW       (0.58f / 60.0f * 0.017453292f)\n'
    '/* VER=121 地磁偏航"闸门"（用户：小转动时地磁的固定误差不许进姿态）。\n'
    ' * 只有 **EKF 自己的偏航不确定度 P[8][8]**（= 陀螺自己的累积，重力帮不上它）\n'
    ' * 涨到超过地磁自身误差，才允许地磁参与本次更新：\n'
    ' *   P <= P_LO -> w = 0（本次地磁更新整块不执行：K=0、P 不动、不计 chi2）\n'
    ' *   P >= P_HI -> w = 1（满权重 KF 牵引）；中间线性。\n'
    ' * 2 deg = S3 实测地磁偏航残差 p50 1.7~2.15 deg；\n'
    ' * 4 deg = 其系统项量级（会话间硬磁中心差 2.33 uT => 4.4 deg）。\n'
    ' * 到达闸门：静止 ~35 min / 20 dps ~6 min / 50 dps ~15 s / 300 dps 44 s\n'
    ' *   / 2000 dps 1.0 s => 常规 10 min 飞行内**地磁全程不参与**。 */\n'
    '#define V5F_EKF_MAG_GATE_LO_DEG   2.0f\n'
    '#define V5F_EKF_MAG_GATE_HI_DEG   4.0f\n')

NEW_BLOCK = [
    '        /* ---- VER=121 地磁偏航"闸门"（用户：小转动时地磁固定误差不许进姿态）----------',
    '         * 任何"把地磁当绝对航向观测"的持续牵引，对地磁固定偏差的直流增益都是 1，',
    '         * 最终都把偏航拉到地磁（含其 2~4.4 deg 偏差）上 —— tau 只决定多快，不决定会不会。',
    '         * 所以这里不是调 tau，而是加**闸门 w**，读 EKF 自己的偏航不确定度 P[8][8]：',
    '         *   P[8][8] <= (2 deg)^2          -> w = 0（本次地磁更新整块不执行）',
    '         *   (2 deg)^2 < P < (4 deg)^2     -> w 线性 0->1',
    '         *   P[8][8] >= (4 deg)^2          -> w = 1（满权重 KF 牵引）',
    '         * P[8][8] 只由陀螺自己累积：重力观测不到偏航（M6 的 H 第 3 列恒 0），',
    '         * 所以"陀螺 R 足够小的数十~数百秒内"闸门必为 0 -> 地磁一个字节都进不来。',
    '         * 到达闸门：静止 ~35 min（含整流项）/ 300 dps 44 s / 2000 dps 1.0 s。',
    '         * R 本身用**诚实值** sigma_mag^2（不再人为放大）。 */',
    '        {',
    '            float sy  = V5F_EKF_MAG_VEC_SIG_DEG * DEG2RAD;',
    '            float plo = (V5F_EKF_MAG_GATE_LO_DEG * DEG2RAD)',
    '                      * (V5F_EKF_MAG_GATE_LO_DEG * DEG2RAD);',
    '            float phi = (V5F_EKF_MAG_GATE_HI_DEG * DEG2RAD)',
    '                      * (V5F_EKF_MAG_GATE_HI_DEG * DEG2RAD);',
    '            float py  = s_P[8][8];',
    '            RRv[0] = sy * sy;',
    '            RRv[1] = 0.0f; RRv[2] = 0.0f;',
    '            if      (py <= plo) s_kscale = 0.0f;',
    '            else if (py >= phi) s_kscale = 1.0f;',
    '            else                s_kscale = (py - plo) / (phi - plo);',
    '            s_mag_rs = s_kscale;   /* VER=121 上报：偏航地磁闸门（0=未参与）*/',
    '        }',
    '        {   /* 倾斜行：固定 R = SIG_TILT^2（VER=118 起不再按 conf 抬 R） */',
    '            float st = V5F_EKF_MAG_VEC_SIG_TILT_DEG;',
    '            RRv[3] = (st * DEG2RAD) * (st * DEG2RAD);',
    '            (void)conf;',
    '        }',
    '        if (s_kscale <= 0.0f) return;   /* 闸门关：不更新、不动 P、不计 chi2 */',
]

NEW_STATICS = (
    '/* VER=121 地磁闸门：本次更新 K 的整体缩放（1.0 = 不缩放）。\n'
    ' * 由 ekf_m7_mag 在调用 ekf_update 前写入、调用后复位；只作用于那一次更新。\n'
    ' * 缩放 K 而不是只缩 dx -> Joseph 形式的 P 更新同步变小，协方差保持一致。 */\n'
    'static float s_kscale = 1.0f;\n'
)

ANCH_KUPD = ('static uint8_t ekf_update(const float *R, uint8_t m, const float *r,\n'
             '                          float nis_max, float *nis_out, uint8_t *rej,\n'
             '                          uint16_t inj_mask, float k_cap)\n')
ANCH_KCAP = ('    if (k_cap > 0.0f) {\n'
             '        for (i = 0u; i < EKF_N; i++) {\n'
             '            for (j = 0u; j < m; j++) {\n'
             '                if (s_K[i][j] >  k_cap) s_K[i][j] =  k_cap;\n'
             '                if (s_K[i][j] < -k_cap) s_K[i][j] = -k_cap;\n'
             '            }\n'
             '        }\n'
             '    }\n')
ANCH_CALL = ('    st = ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],\n'
             '                    0x01C0u, V5F_EKF_MAG_VEC_K_MAX);\n')
# VER=120 的 Q 块（要整段替换成"诚实记账"版）
ANCH_QOLD = ('    {\n'
             '        float sr = V5F_EKF_RECT_DPS_PER_G * s_alin_g * DEG2RAD;\n'
             '        float ss = V5F_EKF_GYRO_KS * s_wmag;')


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    txt = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+121u', tune) and 'V5F_EKF_MAG_GATE_LO_DEG' in tune:
        print('已是 VER=121 最终版（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+120u', tune), 'tune FW_VER 不是 120u'
        assert re.search(r'#define V5F_EKF_GYRO_KS\s+2\.0e-3f', tune), 'KS 不是 2e-3'
        for mac in ('V5F_EKF_MAG_YAW_R', 'V5F_EKF_Q_ATT_ARW', 'V5F_EKF_MAG_GATE_LO_DEG'):
            assert mac not in tune, mac
        NEW_TUNE.encode('gbk')

        L = txt.split('\n')
        assert L[904] == '        sg2 = V5F_EKF_MAG_VEC_SIG_DEG * DEG2RAD;', L[904]
        assert L[905].startswith('        RRv[0] = sg2 * sg2;'), L[905]
        assert L[925] == '        }' and L[926] == '    }', (L[925], L[926])
        assert L[827] == '        float n1, c1, sg2, ddip, conf, Sx[3][3];', repr(L[827])
        for a in (ANCH_KUPD, ANCH_KCAP, ANCH_CALL, ANCH_QOLD):
            assert txt.count(a) == 1, ('锚点 x%d: %r' % (txt.count(a), a[:60]))

        B.save(EKF, TAG)
        B.save(TUNE, TAG)

        # ---- R 段（闸门）+ 声明清理 ----
        L[827] = '        float n1, c1, ddip, conf, Sx[3][3];'
        L[904:926] = NEW_BLOCK
        txt = '\n'.join(L)

        # ---- 姿态 Q 记账（三行统一，含重力行）----
        QOLD = ('    {\n'
                '        float sr = V5F_EKF_RECT_DPS_PER_G * s_alin_g * DEG2RAD;\n'
                '        float ss = V5F_EKF_GYRO_KS * s_wmag;')
        QNEW = ('    {\n'
                '        /* ---- VER=121 姿态三行 Q 统一"诚实记账"（用户：重力参与时正确缩小陀螺 R）----\n'
                '         * Q_att = ARW^2 + (KS*|w|*DEG2RAD)^2 + (0.85*|a_lin|*DEG2RAD)^2\n'
                '         *   1) 基底换成 S3 实测随机游走 V5F_EKF_Q_ATT_ARW（原式把 8 kHz 单样本\n'
                '         *      RMS 0.1224 dps 当谱密度用，等效 7.34 deg/rt-h，Q 大 336 倍）；\n'
                '         *   2) 转动项用 1000 ppm(KS) 并补 DEG2RAD：sq 的其它项都是 rad 量纲，\n'
                '         *      原式 ss = k_s*|w| 是 dps，直接平方等于放大 1/DEG2RAD^2 = 3283 倍\n'
                '         *      （20 dps 时 Q=1.6e-3 -> 姿态被重力/地磁过度牵引）；\n'
                '         *   3) 重力牵引(M6)靠 KF 正常收缩 P[6][6]/P[7][7]，速度泄漏\n'
                '         *      lambda = 5*sigma_tilt 随之回到正确量级（原来 sigma_tilt 被抬到 77 deg）。\n'
                '         *   4) 偏航行 P[8][8] 只由陀螺自己累积（重力 H 第 3 列恒 0，观测不到偏航）\n'
                '         *      —— 地磁闸门读的就是它。 */\n'
                '        float sr = V5F_EKF_RECT_DPS_PER_G * s_alin_g * DEG2RAD;\n'
                '        float ss = V5F_EKF_GYRO_KS * s_wmag * DEG2RAD;\n'
                '        float arw = V5F_EKF_Q_ATT_ARW;')
        txt = txt.replace(QOLD, QNEW, 1)
        txt = txt.replace('        float sq = V5F_EKF_SIG_G_RADS * V5F_EKF_SIG_G_RADS + sr * sr + ss * ss;',
                          '        float sq = arw * arw + sr * sr + ss * ss;', 1)
        # QOLD 只锚到 "s_wmag;"，原 ss 行尾的旧注释会留在 arw 行上 -> 换掉
        txt, na = re.subn(r'(float arw = V5F_EKF_Q_ATT_ARW;)\s*/\*[^\n]*\*/',
                          r'\1   /* VER=121: S3 实测随机游走基底（原 SIG_G_RADS 项删）*/', txt, count=1)
        assert na == 1, 'arw 行注释'

        # ---- ekf_update 钩子 ----
        txt = txt.replace(ANCH_KUPD, NEW_STATICS + ANCH_KUPD, 1)
        txt = txt.replace(ANCH_KCAP, ANCH_KCAP +
                          '    /* ---- VER=121 地磁闸门：整块缩放本次更新的 K（dx 与 P 同步）---- */\n'
                          '    if (s_kscale < 1.0f) {\n'
                          '        for (i = 0u; i < EKF_N; i++) {\n'
                          '            for (j = 0u; j < m; j++) s_K[i][j] *= s_kscale;\n'
                          '        }\n'
                          '    }\n', 1)
        txt = txt.replace(ANCH_CALL, ANCH_CALL +
                          '    s_kscale = 1.0f;   /* VER=121 钩子只作用于本次地磁更新 */\n', 1)
        open(EKF, 'wb').write(txt.encode('gbk'))

        # ---- tune ----
        m = re.search(r'#define V5F_EKF_MAG_VEC_SIG_TILT_DEG[^\n]*\n', tune)
        assert m, 'SIG_TILT 锚点'
        tune = tune[:m.end()] + NEW_TUNE + tune[m.end():]
        tune, nk = re.subn(r'#define V5F_EKF_GYRO_KS\s+2\.0e-3f[^\n]*',
                           '#define V5F_EKF_GYRO_KS          1.0e-3f  /* VER=121: 用户口径 —— '
                           '满量程 1000 ppm（原 2e-3） */', tune, count=1)
        assert nk == 1, 'KS 锚点'
        tune, nq = re.subn(
            r'#define V5F_EKF_Q_YAW_MIN\s+1\.0e-8f[^\n]*',
            '#define V5F_EKF_Q_YAW_MIN        1.0e-12f /* VER=121: 降为纯数值下限；'
            '随机游走改由 V5F_EKF_Q_ATT_ARW 承载 */', tune, count=1)
        assert nq == 1, 'Q_YAW_MIN 锚点'
        tune, n = re.subn(r'#define V5F_FW_VER\s+120u', '#define V5F_FW_VER        121u', tune, count=1)
        assert n == 1
        open(TUNE, 'wb').write(tune.encode('gbk'))
        print('patched: tune(+Q_ATT_ARW,+MAG_GATE_LO/HI_DEG, KS 2e-3->1e-3, Q_YAW_MIN=1e-12, VER=121u)')
        print('         proc_ekf.c(姿态 Q 诚实记账 + 地磁闸门 s_kscale)')

    # ---------------- verify ----------------
    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    txt = open(EKF, 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+121u', tune)
    assert re.search(r'#define V5F_EKF_Q_ATT_ARW\s+\(0\.58f / 60\.0f \* 0\.017453292f\)', tune)
    assert re.search(r'#define V5F_EKF_GYRO_KS\s+1\.0e-3f', tune)
    assert re.search(r'#define V5F_EKF_MAG_GATE_LO_DEG\s+2\.0f', tune)
    assert re.search(r'#define V5F_EKF_MAG_GATE_HI_DEG\s+4\.0f', tune)
    assert re.search(r'#define V5F_EKF_Q_YAW_MIN\s+1\.0e-12f', tune)
    assert 'V5F_EKF_MAG_YAW_R' not in tune and 'V5F_EKF_MAG_YAW_R' not in txt
    assert txt.count('static float s_kscale = 1.0f;') == 1
    assert txt.count('s_K[i][j] *= s_kscale;') == 1
    assert txt.count('s_kscale = (py - plo) / (phi - plo);') == 1
    assert txt.count('s_kscale = 1.0f;   /* VER=121') == 1
    assert 'V5F_EKF_SIG_G_RADS * V5F_EKF_SIG_G_RADS + sr * sr' not in txt
    assert 'float ss = V5F_EKF_GYRO_KS * s_wmag * DEG2RAD;' in txt
    assert 'float sq = arw * arw + sr * sr + ss * ss;' in txt
    assert 'float arw = V5F_EKF_Q_ATT_ARW;' in txt
    mag = txt.split('static void ekf_m7_mag(')[1].split('static void ekf_m1m2_gps')[0]
    assert 'sg2' not in mag and 'sy2' not in txt and 'RRv[0] = sy * sy;' in mag
    upd = txt.split('static uint8_t ekf_update(')[1].split('static void ekf_m6_tilt')[0]
    assert upd.count('s_kscale') == 2
    e2 = re.sub(r'/\*.*?\*/', '', txt, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'), '括号不平衡'
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_Q_ATT_ARW[^\n]*',
                r'#define V5F_EKF_GYRO_KS[^\n]*', r'#define V5F_EKF_MAG_GATE_LO_DEG[^\n]*',
                r'#define V5F_EKF_MAG_GATE_HI_DEG[^\n]*', r'#define V5F_EKF_Q_YAW_MIN[^\n]*'):
        print('  |', re.search(pat, tune).group()[:118])
    print('---- proc_ekf.c R 段（闸门）----')
    j = mag.find("VER=121 地磁偏航")
    for s in mag[j - 9:j + 2300].split('\n')[:28]:
        print('  |', s[:118])
    print('---- proc_ekf.c Q 段（诚实记账）----')
    prop = txt.split('static void ekf_prop_row')[1]
    k = prop.find('VER=121 姿态三行')
    for s in prop[k - 40:k + 1500].split('\n')[:22]:
        print('  |', s[:118])
    print('---- ekf_update 钩子 ----')
    k = upd.find('VER=121 地磁闸门')
    for s in upd[k - 30:k + 300].split('\n'):
        print('  |', s[:118])
    print()
    for mac in ('V5F_FW_VER', 'V5F_EKF_Q_ATT_ARW', 'V5F_EKF_GYRO_KS',
                'V5F_EKF_MAG_GATE_LO_DEG', 'V5F_EKF_MAG_GATE_HI_DEG', 'V5F_EKF_Q_YAW_MIN'):
        n = len(re.findall(r'#define\s+%s\b' % mac, tune))
        print('LINT %-28s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= n == 1
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v121 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v121 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
