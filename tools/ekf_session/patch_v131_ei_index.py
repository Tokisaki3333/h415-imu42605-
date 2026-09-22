# -*- coding: utf-8 -*-
r"""VER=131：统一 EKF **误差状态索引**（地基修复，先于新增状态）

发现（记录在 docs/ekf_index_audit.md）：固件里并存两套索引空间，且被混用：
   存储空间 s_x[17]  : p(0-2) v(3-5) **q(6-9, 四元数 4 维)** ba(10-12) bg(13-15) bb(16)
   误差空间 dx[16]   : p(0-2) v(3-5) **dθ(6-8, 旋转矢量 3 维)** ba(9-11)  bg(12-14)  bb(15)
`P0 初始化 / Q 行 / H / publish` 全部用**误差空间**（9-11, 12-14, 15）✓，
但 `s_F` 的雅可比与 `ekf_inject` 用了 IX_*（= 存储空间）：
    s_F[EI_V+i][IX_BA+j] = F[3..5][10..12]   -> 应为 [9..11]
    s_F[EI_Q+i][IX_BG+j] = F[6..8][13..15]   -> 应为 [12..14]（F[.][15] 把 bb 串进了姿态）
    ekf_inject: dx[10..12] -> ba（应为 dx[9..11]），dx[13..15] -> bg（应为 dx[12..14]），
                dx[15] 同时被当作 bg_z 与 bb ✗
后果（与实测吻合）：`ekf.ba` 恒为 0、`ekf.bg` ≈ 0 ⇒ **陀螺零偏从未被补偿**。

本补丁只做一件事：引入 EI_* 误差状态索引宏，并把 s_F / ekf_inject / Q 行 / s_dx 读取
全部改用 EI_*。**状态数不变（16）**。

用法: python tools/ekf_session/patch_v131_ei_index.py
回退: bak_src\V5F\User\{src\proc_ekf.c,inc\v5f_tune.h}.bak_v131
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

EKF = os.path.join(B.REPO, 'V5F', 'User', 'src', 'proc_ekf.c')
TUNE = os.path.join(B.REPO, 'V5F', 'User', 'inc', 'v5f_tune.h')
TAG = '.bak_v131'

EI_BLOCK = (
    '/* ---- 误差状态索引 EI_*（**与存储索引 IX_* 不是同一套！**）-----------------\n'
    ' * 存储 s_x[EKF_NX=17]：p(0-2) v(3-5) q(6-9,**四元数 4 维**) ba(10-12) bg(13-15) bb(16)\n'
    ' * 误差 dx[EKF_N =16] ：p(0-2) v(3-5) dth(6-8,**旋转矢量 3 维**) ba(9-11) bg(12-14) bb(15)\n'
    ' * 四元数多占一个存储槽，故 ba 起的**误差索引 = 存储索引 - 1**。\n'
    ' * 铁律：s_P / s_Pn / s_F / s_G / s_H / s_K / s_dx **一律用 EI_**；只有 s_x 用 IX_*。\n'
    ' * （VER=131 之前 s_F 与 ekf_inject 误用 IX_*，导致 ba/bg 估计失效）*/\n'
    '#define EI_P         0u\n'
    '#define EI_V         3u\n'
    '#define EI_Q         6u     /* 旋转矢量 3 维（不是四元数） */\n'
    '#define EI_BA        9u\n'
    '#define EI_BG       12u\n'
    '#define EI_BB       15u\n')

REPL = [
    ('    for (i = 0u; i < 3u; i++) s_F[IX_P + i][IX_V + i] = s_dt_e;',
     '    for (i = 0u; i < 3u; i++) s_F[EI_P + i][EI_V + i] = s_dt_e;   /* VER=131 */'),
    ('            s_F[IX_V + i][IX_Q  + j] = -s_Rf[i][j] * s_dt_e;\n'
     '            s_F[IX_V + i][IX_BA + j] = -s_Rn[i][j] * s_dt_e;\n'
     '            s_F[IX_Q + i][IX_BG + j] = -s_Rn[i][j] * s_dt_e;',
     '            /* VER=131 修复：原来用 IX_*（存储索引）写 F，ba/bg 错位一格，\n'
     '             * 且 F[.][15] 把气压实偏 bb 串进了姿态。现一律用 EI_*。 */\n'
     '            s_F[EI_V + i][EI_Q  + j] = -s_Rf[i][j] * s_dt_e;\n'
     '            s_F[EI_V + i][EI_BA + j] = -s_Rn[i][j] * s_dt_e;\n'
     '            s_F[EI_Q + i][EI_BG + j] = -s_Rn[i][j] * s_dt_e;'),
    ('    for (i = 0u; i < 3u; i++) s_x[IX_P  + i] += dx[IX_P  + i];\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_V  + i] += dx[IX_V  + i];\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_BA + i] += dx[IX_BA + i];\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_BG + i] += dx[IX_BG + i];',
     '    /* VER=131 修复：dx 是**误差状态**，索引用 EI_*（原来误用 IX_*，\n'
     '     * ba/bg 各错一格，dx[15] 同时被当成 bg_z 与 bb）。 */\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_P  + i] += dx[EI_P  + i];\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_V  + i] += dx[EI_V  + i];\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_BA + i] += dx[EI_BA + i];\n'
     '    for (i = 0u; i < 3u; i++) s_x[IX_BG + i] += dx[EI_BG + i];'),
    ('    s_x[IX_BB] += dx[15];', '    s_x[IX_BB] += dx[EI_BB];'),
    ('    dq[1] = 0.5f * dx[6];\n    dq[2] = 0.5f * dx[7];\n    dq[3] = 0.5f * dx[8];',
     '    dq[1] = 0.5f * dx[EI_Q + 0u];\n    dq[2] = 0.5f * dx[EI_Q + 1u];\n'
     '    dq[3] = 0.5f * dx[EI_Q + 2u];'),
    ('            if      (row >= 3u  && row <= 5u)  qd += sa2 * dtq;\n'
     '            else if (row >= 6u  && row <= 8u)  qd += ((row == 8u) ? sq_yaw : sg2) * dtq;\n'
     '            else if (row >= 9u  && row <= 11u) qd += V5F_EKF_SIG_BA_RW * V5F_EKF_SIG_BA_RW * dtq;\n'
     '            else if (row >= 12u && row <= 14u) qd += V5F_EKF_SIG_BG_RW * V5F_EKF_SIG_BG_RW * dtq;\n'
     '            else if (row == 15u)               qd += V5F_EKF_SIG_BARO_RW * V5F_EKF_SIG_BARO_RW * dtq;',
     '            if      (row >= EI_V  && row <= EI_V  + 2u) qd += sa2 * dtq;\n'
     '            else if (row >= EI_Q  && row <= EI_Q  + 2u) qd += ((row == EI_Q + 2u) ? sq_yaw : sg2) * dtq;\n'
     '            else if (row >= EI_BA && row <= EI_BA + 2u) qd += V5F_EKF_SIG_BA_RW * V5F_EKF_SIG_BA_RW * dtq;\n'
     '            else if (row >= EI_BG && row <= EI_BG + 2u) qd += V5F_EKF_SIG_BG_RW * V5F_EKF_SIG_BG_RW * dtq;\n'
     '            else if (row == EI_BB)                      qd += V5F_EKF_SIG_BARO_RW * V5F_EKF_SIG_BARO_RW * dtq;'),
    ('            if (row == 8u) qd += V5F_EKF_Q_YAW_MIN;',
     '            if (row == EI_Q + 2u) qd += V5F_EKF_Q_YAW_MIN;'),
    ('            if (i == 8u && j == 8u && a < s_yaw_p_min) a = s_yaw_p_min;   /* VER=130 */',
     '            if (i == EI_Q + 2u && j == EI_Q + 2u && a < s_yaw_p_min) a = s_yaw_p_min;   /* VER=130 */'),
    ('        s_tilt_dqx = s_dx[IX_Q + 0] * RAD2DEG;\n'
     '        s_tilt_dqy = s_dx[IX_Q + 1] * RAD2DEG;\n'
     '        s_tilt_dqz = s_dx[IX_Q + 2] * RAD2DEG;',
     '        s_tilt_dqx = s_dx[EI_Q + 0] * RAD2DEG;\n'
     '        s_tilt_dqy = s_dx[EI_Q + 1] * RAD2DEG;\n'
     '        s_tilt_dqz = s_dx[EI_Q + 2] * RAD2DEG;'),
    ('        s_mag_dqx = s_dx[IX_Q + 0] * RAD2DEG;\n'
     '        s_mag_dqy = s_dx[IX_Q + 1] * RAD2DEG;\n'
     '        s_mag_dqz = s_dx[IX_Q + 2] * RAD2DEG;',
     '        s_mag_dqx = s_dx[EI_Q + 0] * RAD2DEG;\n'
     '        s_mag_dqy = s_dx[EI_Q + 1] * RAD2DEG;\n'
     '        s_mag_dqz = s_dx[EI_Q + 2] * RAD2DEG;'),
    ('    s_H[0][8] = 1.0f;', '    s_H[0][EI_Q + 2u] = 1.0f;'),
]


def _w(path, text):
    b = text.encode('gbk')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    ekf = open(EKF, 'rb').read().decode('gbk')
    tune = open(TUNE, 'rb').read().decode('gbk')
    if 'EI_BA' in ekf:
        print('已是 VER=131：只校验')
    else:
        assert re.search(r'#define IX_BB\s+16u', ekf), '基线不对'
        B.save(EKF, TAG)
        B.save(TUNE, TAG)
        anchor = '#define EKF_NX       17u\n'
        assert ekf.count(anchor) == 1, 'IX 块锚点'
        ekf = ekf.replace(anchor, anchor + EI_BLOCK, 1)
        for old, new in REPL:
            n_hit = ekf.count(old)
            expect = 2 if old == '    s_H[0][8] = 1.0f;' else 1
            assert n_hit == expect, '锚点命中 %d != %d: %s' % (n_hit, expect,
                                                             old.split('\n')[0][:70])
            ekf = ekf.replace(old, new)
        _w(EKF, ekf)
        tune, n = re.subn(r'#define V5F_FW_VER\s+130u',
                          '#define V5F_FW_VER        131u', tune, count=1)
        assert n == 1, 'VER 替换'
        _w(TUNE, tune)

    ekf = open(EKF, 'rb').read().decode('gbk')
    tune = open(TUNE, 'rb').read().decode('gbk')
    body = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    ok = True
    ok &= bool(re.search(r'#define V5F_FW_VER\s+131u', tune))
    for m in ('EI_P\\s+0u', 'EI_V\\s+3u', 'EI_Q\\s+6u', 'EI_BA\\s+9u', 'EI_BG\\s+12u', 'EI_BB\\s+15u'):
        ok &= bool(re.search('#define ' + m, ekf))
    ok &= (body.count('dx[IX_') == 0 and body.count('s_dx[IX_') == 0)
    ok &= (not re.search(r's_F\[[^\]]*\]\[IX_(BA|BG|BB)', body))
    ok &= (body.count('s_F[EI_BA + j]') == 1 and body.count('s_F[EI_BG + j]') == 1)
    ok &= (body.count('dx[EI_BA + i]') == 1 and body.count('dx[EI_BG + i]') == 1)
    ok &= (body.count('dx[EI_BB]') == 1)
    ok &= (body.count('s_x[IX_BA + i]') == 1 and body.count('s_x[IX_BG + i]') == 1)
    ok &= (body.count('s_H[0][EI_Q + 2u] = 1.0f;') == 2)
    ok &= (body.count('{') == body.count('}') and body.count('(') == body.count(')'))
    print()
    print('EKF_N =', re.search(r'#define EKF_N\s+(\d+)u', ekf).group(1),
          ' EKF_NX =', re.search(r'#define EKF_NX\s+(\d+)u', ekf).group(1))
    print('F ba/bg 列：', re.findall(r's_F\[EI_[A-Z]+ \+ [ij]\]\[EI_[A-Z]+ \+ [ij]\] = -s_R[fn]', ekf))
    print('inject：', re.findall(r'\+ dx\[EI_(?:BA|BG|BB)[^]]*\]', ekf))
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v131 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
