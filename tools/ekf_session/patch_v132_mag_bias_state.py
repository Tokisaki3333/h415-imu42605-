# -*- coding: utf-8 -*-
r"""VER=132：偏航通道增广「地磁偏置状态」（落实 docs/ekf_yaw_timescale.md 的 M3 结构）

数学：状态 x = [ψ(误差态 6-8 的第 3 个), b_g(12-14), b_m(新 EI_MB=16)]
  传播  F[EI_MB][EI_MB] = 1（导航系常值偏置，无动力学）
        Q[EI_MB] = 2·σ_bm²/T_c = V5F_EKF_BM_Q_RADS2   （GM 过程，等号关系，不是 max/min）
  观测  y_mag = ψ + b_m + n_m
        入口A（平面投影，静态/小重力误差）：H[0][EI_Q+2] = 1, H[0][EI_MB] = 1,
              R = 白噪² + (tanI·σ_tilt)²  —— 白噪用实测 V5F_EKF_MAG_WHITE_DEG(0.35°)
        入口B（矢量式）：H[0][EI_MB] = H[0][EI_Q+2]（同为导航系竖直轴偏置），R 保持原样
  限幅/复位：|b_m| ≤ V5F_EKF_BM_LIM_DEG；复位/对齐时 b_m = 0，P = (BM_P0)²

为什么这样能"陀螺短期主导 + 地磁长期修正"：新息按方差比分配
  K_ψ = P_ψ/(P_ψ+P_bm+R)、K_bm = P_bm/(P_ψ+P_bm+R)；P_bm=(2°)² ≫ P_ψ ⇒ 新息几乎记到 b_m，
  姿态不被地磁偏置直接污染（PC 仿真 sim_yaw_tau.py：1s 增量误差回到陀螺物理底 0.0090°，
  2° 偏置阶跃残余 0.029°），且**不需要任何 τ 旋钮**。

可验证性：`g_v5f_ekf_mb_deg` 全局（不改 SHM 结构体布局）+ 验收帧加第 6 路 float。

用法: python tools/ekf_session/patch_v132_mag_bias_state.py
回退: bak_src\V5F\User\{src\proc_ekf.c,inc\v5f_tune.h,src\SPI_rx.c}.bak_v132
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

EKF = os.path.join(B.REPO, 'V5F', 'User', 'src', 'proc_ekf.c')
TUNE = os.path.join(B.REPO, 'V5F', 'User', 'inc', 'v5f_tune.h')
SPI = os.path.join(B.REPO, 'V5F', 'User', 'src', 'SPI_rx.c')
TAG = '.bak_v132'

NOTE_T = (
    '/* VER=132 偏航通道增广「地磁偏置状态」b_m（落实 docs/ekf_yaw_timescale.md 的 M3 结构，\n'
    ' * 取代"调 τ"）：观测 y = psi + b_m + n_m，H 同时挂 psi 与 b_m；新息按方差比分配。\n'
    ' * 白噪只用实测值（静止创新 sigma≈0.33 deg），偏置由 GM 过程 Q = 2 sigma_bm^2/T_c 承载。 */\n'
    '#define V5F_EKF_MAG_WHITE_DEG   0.35f\n'
    '#define V5F_EKF_BM_P0_DEG       2.0f\n'
    '#define V5F_EKF_BM_TC_S         60.0f\n'
    '#define V5F_EKF_BM_LIM_DEG      20.0f\n'
    '/* Q_bm = 2*(2deg)^2/60 s = 4.062e-5 rad^2/s（单位 rad^2/s，写死避免头文件依赖） */\n'
    '#define V5F_EKF_BM_Q_RADS2      4.062e-5f\n')


def _w(path, text):
    b = text.encode('gbk')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    ekf = open(EKF, 'rb').read().decode('gbk')
    tune = open(TUNE, 'rb').read().decode('gbk')
    spi = open(SPI, 'rb').read().decode('gbk')
    if 'EI_MB' in ekf:
        print('已是 VER=132：只校验')
    else:
        assert re.search(r'#define EKF_N\s+17u', open(EKF, 'rb').read().decode('gbk')) or \
               re.search(r'#define EKF_N\s+16u', ekf), '基线不对'
        B.save(EKF, TAG)
        B.save(TUNE, TAG)
        B.save(SPI, TAG)

        # ---------- tune ----------
        tune, n = re.subn(r'#define V5F_FW_VER\s+131u', '#define V5F_FW_VER        132u',
                          tune, count=1)
        assert n == 1
        k = tune.index('#define V5F_MAG_CLIP_LSB')
        tune = tune[:k] + NOTE_T + tune[k:]
        _w(TUNE, tune)

        # ---------- proc_ekf.c ----------
        r = [
            ('#define EKF_N        16u', '#define EKF_N        17u    /* VER=132: +地磁偏航偏置 */'),
            ('#define EKF_NX       17u', '#define EKF_NX       18u    /* VER=132 */'),
        ]
        for old, new in r:
            assert ekf.count(old) == 1, old
            ekf = ekf.replace(old, new, 1)

        ekf, n = re.subn(r'(#define IX_BB\s+16u[^\n]*\n)',
                         r'\1#define IX_MB       17u    /* VER=132 地磁偏航偏置（存储槽） */\n',
                         ekf, count=1)
        assert n == 1, 'IX_MB'
        ekf, n = re.subn(r'(#define EI_BB\s+15u\n)',
                         r'\1#define EI_MB       16u    /* VER=132 地磁偏航偏置（误差态） */\n',
                         ekf, count=1)
        assert n == 1, 'EI_MB'

        # 全局遥测量（不改 SHM 结构体布局）
        old = 'static uint8_t  s_mag_dist;'
        assert ekf.count(old) == 1
        ekf = ekf.replace(old, old + '\n/* VER=132 供验收帧读取的地磁偏航偏置（deg）；用全局量避免改 v5f_ekf_t 布局 */\n'
                                     'volatile float   g_v5f_ekf_mb_deg;', 1)

        # 复位：b_m = 0（只在**上电/自检初始化**块；ENU 原点复位那里不动，与偏置无关）
        old = ('            s_x[IX_BB] = 0.0f;\n'
               '\n'
               '            for (i = 0u; i < EKF_N; i++) {')
        assert ekf.count(old) == 1, 'init IX_BB 锚点'
        ekf = ekf.replace(old, '            s_x[IX_BB] = 0.0f;\n'
                               '            s_x[IX_MB] = 0.0f;   /* VER=132 复位地磁偏置 */\n'
                               '\n'
                               '            for (i = 0u; i < EKF_N; i++) {', 1)

        # P0（用后续注释行做唯一性锚点：初始化块里的那一处）
        ekf, n = re.subn(
            r'(            s_P\[15\]\[15\] = V5F_EKF_P0_BARO_M \* V5F_EKF_P0_BARO_M;\n)'
            r'(            /\* s_Pn)',
            r'\1            s_P[EI_MB][EI_MB] = (V5F_EKF_BM_P0_DEG * DEG2RAD)\n'
            r'                              * (V5F_EKF_BM_P0_DEG * DEG2RAD);   /* VER=132 */\n\2',
            ekf, count=1)
        assert n == 1, 'P0 MB'

        # Q 行
        old = ('            else if (row == EI_BB)                      qd += V5F_EKF_SIG_BARO_RW '
               '* V5F_EKF_SIG_BARO_RW * dtq;')
        assert ekf.count(old) == 1, 'Q 行锚点'
        ekf = ekf.replace(old, old + '\n            else if (row == EI_MB)                      qd += '
                                     'V5F_EKF_BM_Q_RADS2 * dtq;   /* VER=132 GM 过程 */', 1)

        # 入口A：白噪 + 偏置耦合
        old = '    sig_h = V5F_EKF_MAG_VEC_SIG_DEG;'
        assert ekf.count(old) == 1, 'sig_h'
        ekf = ekf.replace(old, '    sig_h = V5F_EKF_MAG_WHITE_DEG;   /* VER=132: 只用白噪，偏置交给 b_m */', 1)

        # 两处"纯偏航观测"的 H（入口A 与 m7_mag 的平面路径）
        old = '    s_H[0][EI_Q + 2u] = 1.0f;'
        assert ekf.count(old) == 2, 'H yaw x2'
        ekf = ekf.replace(old, old + '\n    s_H[0][EI_MB] = 1.0f;   /* VER=132 偏置与偏航同系数 */')

        # 入口B（矢量式）的 H：偏置与偏航同一系数
        old = ('            s_H[1][IX_Q + ax2] = e2[0]*w0 + e2[1]*w1 + e2[2]*w2;\n'
               '        }')
        assert ekf.count(old) == 1, 'vec H'
        ekf = ekf.replace(old, '            s_H[1][IX_Q + ax2] = e2[0]*w0 + e2[1]*w1 + e2[2]*w2;\n'
                               '        }\n'
                               '        s_H[0][EI_MB] = s_H[0][EI_Q + 2u];   /* VER=132 导航竖直轴偏置 */', 1)

        # inject：写回 + 限幅
        old = '    s_x[IX_BB] += dx[EI_BB];'
        assert ekf.count(old) == 1, 'inject BB'
        ekf = ekf.replace(old, old + '\n'
                               '    s_x[IX_MB] += dx[EI_MB];   /* VER=132 */\n'
                               '    if (s_x[IX_MB] >  V5F_EKF_BM_LIM_DEG * DEG2RAD) '
                               's_x[IX_MB] =  V5F_EKF_BM_LIM_DEG * DEG2RAD;\n'
                               '    if (s_x[IX_MB] < -V5F_EKF_BM_LIM_DEG * DEG2RAD) '
                               's_x[IX_MB] = -V5F_EKF_BM_LIM_DEG * DEG2RAD;', 1)

        # 发布
        old = '    h->ekf.mag_mode    = s_mag_mode;   /* VER=124 */'
        assert ekf.count(old) == 1, 'publish'
        ekf = ekf.replace(old, old + '\n    g_v5f_ekf_mb_deg = s_x[IX_MB] * RAD2DEG;   /* VER=132 */', 1)
        _w(EKF, ekf)

        # ---------- SPI_rx.c：验收帧 5 -> 6 路 float（+b_m） ----------
        old = '        static uint8_t qbuf[24];'
        assert spi.count(old) == 1, 'qbuf'
        spi = spi.replace(old, '        static uint8_t qbuf[28];        /* VER=132: 6 float + 4 B 尾 */', 1)
        old = '        qbuf[20] = 0x00u; qbuf[21] = 0x00u; qbuf[22] = 0x80u; qbuf[23] = 0x7Fu;\n' \
              '        (void)hid_up_enqueue(qbuf, 24u);'
        assert spi.count(old) == 1, 'accept tail'
        spi = spi.replace(old,
                          '        {   /* VER=132 第 6 路：地磁偏航偏置 b_m（deg） */\n'
                          '            float bm = g_v5f_ekf_mb_deg;\n'
                          '            memcpy(qbuf + 20u, &bm, 4u);\n'
                          '        }\n'
                          '        qbuf[24] = 0x00u; qbuf[25] = 0x00u; qbuf[26] = 0x80u; qbuf[27] = 0x7Fu;\n'
                          '        (void)hid_up_enqueue(qbuf, 28u);', 1)
        old = '#if (V5F_CDC_QUAT_ONLY != 0u)'
        # 声明 extern
        assert spi.count('extern volatile float g_v5f_ekf_mb_deg;') == 0
        spi = spi.replace(old, 'extern volatile float g_v5f_ekf_mb_deg;   /* VER=132 */\n\n' + old, 1)
        _w(SPI, spi)
        print('patched: 偏航增广 b_m 状态 + 验收帧第 6 路遥测')

    # ---------------- verify ----------------
    ekf = open(EKF, 'rb').read().decode('gbk')
    tune = open(TUNE, 'rb').read().decode('gbk')
    spi = open(SPI, 'rb').read().decode('gbk')
    body = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    ok = True
    ok &= bool(re.search(r'#define V5F_FW_VER\s+132u', tune))
    for m in ('V5F_EKF_MAG_WHITE_DEG\\s+0\\.35f', 'V5F_EKF_BM_P0_DEG\\s+2\\.0f',
              'V5F_EKF_BM_TC_S\\s+60\\.0f', 'V5F_EKF_BM_LIM_DEG\\s+20\\.0f',
              'V5F_EKF_BM_Q_RADS2\\s+4\\.062e-5f'):
        ok &= bool(re.search('#define ' + m, tune))
    ok &= bool(re.search(r'#define EKF_N\s+17u', ekf))
    ok &= bool(re.search(r'#define EKF_NX\s+18u', ekf))
    ok &= bool(re.search(r'#define IX_MB\s+17u', ekf))
    ok &= bool(re.search(r'#define EI_MB\s+16u', ekf))
    ok &= (ekf.count('s_H[0][EI_MB] = 1.0f;') == 2)
    ok &= (ekf.count('s_H[0][EI_MB] = s_H[0][EI_Q + 2u];') == 1)
    ok &= (ekf.count('s_x[IX_MB] += dx[EI_MB];') == 1)
    ok &= (ekf.count('s_x[IX_MB] = 0.0f;') == 1)
    ok &= (ekf.count('s_P[EI_MB][EI_MB]') == 1)
    ok &= (ekf.count('V5F_EKF_BM_Q_RADS2 * dtq') == 1)
    ok &= (ekf.count('V5F_EKF_BM_LIM_DEG') == 2)
    ok &= (ekf.count('sig_h = V5F_EKF_MAG_WHITE_DEG;') == 1)
    ok &= (ekf.count('g_v5f_ekf_mb_deg = s_x[IX_MB] * RAD2DEG;') == 1)
    ok &= (body.count('{') == body.count('}'))
    ok &= (spi.count('static uint8_t qbuf[28];') == 1)
    ok &= (spi.count('hid_up_enqueue(qbuf, 28u);') == 1)
    ok &= (spi.count('memcpy(qbuf + 20u, &bm, 4u);') == 1)
    ok &= (spi.count('extern volatile float g_v5f_ekf_mb_deg;') == 1)
    print()
    print('EKF_N =', re.search(r'#define EKF_N\s+(\d+)u', ekf).group(1),
          ' EKF_NX =', re.search(r'#define EKF_NX\s+(\d+)u', ekf).group(1),
          ' EI_MB =', re.search(r'#define EI_MB\s+(\d+)u', ekf).group(1),
          ' IX_MB =', re.search(r'#define IX_MB\s+(\d+)u', ekf).group(1))
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: bak_src\\V5F\\User\\{src\\proc_ekf.c,inc\\v5f_tune.h,src\\SPI_rx.c}.bak_v132')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
