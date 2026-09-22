# -*- coding: utf-8 -*-
r"""VER=123 = VER=122 + 地磁→EKF 的**两个入口**（互斥）+ 重力矢量误差估计累加器。

用户设计口径：
  * 两个入口，互斥，一次只有一个使能，另一个不修正：
      入口 A（旧）：把地磁矢量**投影到以重力为法向量的平面** —— VER=103 之前这是
                    地磁唯一的融合方式（原文在 bak_src/.../proc_ekf.c.bak_v103_magvec）
      入口 B（今）：矢量方式（e1 偏航 + e2 倾斜，二维）
  * 内部维护一个数：无有效重力牵引时按**当前速度**累加（不是固定 +1），
    有重力牵引时比例衰减；用两个实测系数拟合：运动误差 1000 ppm、零漂 2 deg/h。
  * 高于阈值才允许矢量方式；低于阈值（重力矢量误差很低）走投影入口，只修正姿态角。
  * 防溢出：u32，饱和加，到顶停在最大值不回绕。
  * 帧基 = **EKF 观测帧**；门控判断用两观测帧之间的 **DRDY 帧降采样**（多数表决）。

累加器（= 重力矢量/倾角误差估计，u32 定点，单位 1e-6 deg）：
  每 EKF 观测帧（348.8 Hz = 23 DRDY 帧/周期，8021.9/23）：
    多数 DRDY 帧有重力牵引 -> s_grav_err -= s_grav_err >> V5F_EKF_MAG_FF_SHIFT
        SHIFT=7 => x(1-1/128)/帧 => tau = 128 帧 = 0.367 s
        （与重力通道自己的倾角收敛 tau = dt/(K*h) ≈ 0.36 s 一致）
    否则                   -> inc = (1000ppm*|w|_dps + 2deg/h) * dt * 1e6  [u deg]
        分辨率：纯零漂 (2/3600)*2.867ms = 1.59 u deg/帧（定点不截断）；100 dps 时 288 u deg/帧
        饱和加：s_grav_err <= MAX-a 才加，否则 = MAX（0xFFFFFFFF = 4295 deg），不回绕
  阈值 THR = 2.0 deg = 地磁自身误差量级（S3 实测 p50 1.7~2.15 deg）：
    连续无重力牵引到达 2 deg 的用时 t = 2 /(1000ppm*|w| + 2deg/h)：
        静止 1.0 h / 20 dps 97 s / 100 dps 19.9 s / 300 dps 6.6 s / 1000 dps 2.0 s
    => 静止与小运动永远停在入口 A；持续机动几秒~几十秒后自动切入口 B。
  不加回差：累加器不由 EKF 的 P 反馈（只由 |w| 与重力门驱动），单帧增量 (~300 u deg)
    对阈值 (2e6 u deg) 是 1/7000，穿越缓慢单调，不存在"更新->P 变->判据翻"的极限环。

入口 A（原文复用，符号已被当年实机验证）：
    r[0] = wrap_pi(thm - thp);          /* 测量 - 预测（平面内航向角） */
    H_zero(1u); s_H[0][8] = 1.0f;       /* H 只有偏航行 */
    R[0] = sigma_h^2 + (tanI*sigma_tilt)^2;   /* sigma_tilt 上限 5 deg；重力有效时退化为 sigma_h^2 */
    ekf_update(R, 1u, r, ..., 0x0100u, ...);  /* 注入掩码只开偏航状态 = 只修正姿态角 */
入口 B：ekf_update(RRv, 2u, r2v, ..., 0x01C0u, ...)  （VER=122 现状，一字未改）

改动文件：V5F/User/inc/v5f_tune.h、V5F/User/src/proc_ekf.c、V5F/User/src/SPI_rx.c（仅注释）
用法: python tools/ekf_session/patch_v123_mag_two_entries.py
回退: Copy-Item bak_src\V5F\User\inc\v5f_tune.h.bak_v122 V5F\User\inc\v5f_tune.h -Force
      Copy-Item bak_src\V5F\User\src\proc_ekf.c.bak_v122 V5F\User\src\proc_ekf.c -Force
      （或 §8.6.1 三条回到 VER=118）
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
SPI = os.path.join(ROOT, 'V5F', 'User', 'src', 'SPI_rx.c')
TAG = '.bak_v123'

NEW_TUNE = (
    '/* VER=123 地磁→EKF 两个入口（互斥）用的"重力矢量误差估计"累加器。\n'
    ' * 每 EKF 观测帧（348.8 Hz = 23 DRDY 帧/周期 = 8021.9/23）更新一次；u32 定点，\n'
    ' * 单位 1e-6 deg（微度）-> u32 满量程 4294.97 deg。\n'
    ' *  该窗口内多数 DRDY 帧有重力牵引 -> 重力正在修倾角 -> 比例衰减（位移实现）：\n'
    ' *    SHIFT=7 => x(1-1/128)/帧 => tau = 128 帧 = 0.367 s\n'
    ' *    （与重力通道自己的倾角收敛 tau = dt/(K*h) ≈ 0.36 s 一致）\n'
    ' *  否则 -> 累加陀螺自己的误差率（两个实测系数）：1000 ppm*|w| + 2 deg/h。\n'
    ' *    分辨率够：纯零漂 = (2/3600)*2.867ms = 1.59 u deg/帧（定点不截断到 0）；\n'
    ' *              100 dps 时 = 288 u deg/帧。\n'
    ' *  阈值 THR = 2 deg = 地磁自身误差量级 => 超过它说明"重力矢量误差已经不低"，\n'
    ' *    才允许地磁用矢量方式；低于它只用地磁修姿态角（投影到重力法平面）。\n'
    ' *  防溢出：饱和加（MAX-a 比较后才加），到顶停在 0xFFFFFFFF 不回绕；\n'
    ' *          衰减用位移，无乘法、无中间量。\n'
    ' * 到达 2 deg 的用时 t = 2/(1000ppm*|w| + 2deg/h)：\n'
    ' *   静止 1.0 h / 20 dps 97 s / 100 dps 19.9 s / 300 dps 6.6 s / 1000 dps 2.0 s。 */\n'
    '#define V5F_EKF_GYRO_DRIFT_DPS         (2.0f / 3600.0f)  /* 零漂 2 deg/h（实测口径） */\n'
    '#define V5F_EKF_MAG_FF_MAX             0xFFFFFFFFu       /* u32 顶 = 4294.97 deg（微度） */\n'
    '#define V5F_EKF_MAG_FF_SHIFT           7u                /* 衰减 x(1-2^-7)，tau=128 帧=0.367 s */\n'
    '#define V5F_EKF_MAG_FF_THR_DEG         2.0f              /* 切矢量方式的阈值 = 2 deg */\n'
    '#define V5F_EKF_MAG_PLANE_TILT_CAP_DEG 5.0f              /* 入口A 的 R 里倾角项钳位 */\n')

NEW_STATICS = (
    '/* VER=123 地磁两入口的切换判据：重力矢量（倾角）误差估计，u32 定点，单位 1e-6 deg */\n'
    'static uint32_t s_grav_err;\n'
    'static float    s_ff_t;                /* 上次累加器更新时刻（s_t_run） */\n'
    'static uint32_t s_ff_n, s_ff_ok;       /* 两观测帧之间的 DRDY 帧数 / 其中有重力牵引的帧数 */\n'
)

NEW_ENTRIES = (
    '/* ===========================================================================\n'
    ' * VER=123 地磁→EKF 的**两个入口**（互斥：每个观测帧只调其中一个）\n'
    ' *   入口 A：重力法向量平面投影 -> 只修正姿态角（偏航）\n'
    ' *   入口 B：矢量方式（e1 偏航 + e2 倾斜）\n'
    ' * 判据 s_grav_err（重力矢量误差估计，单位 1e-6 deg）见 ekf_m7_mag 末尾。\n'
    ' * ========================================================================= */\n'
    '\n'
    '/* ---- 入口 A：把地磁矢量投影到以重力为法向量的平面，只用平面内的航向角残差 ------\n'
    ' * 原文取自 VER=103 之前的实现（bak_src/V5F/User/src/proc_ekf.c.bak_v103_magvec），\n'
    ' * 那一版里这是**地磁唯一的融合方式**、经过实机验证 => 残差/H/掩码直接复用：\n'
    ' *   r = wrap_pi(thm - thp)   测量 - 预测（xm 参考轴下的平面内角）\n'
    ' *   H 只有偏航行（s_H[0][8] = 1）\n'
    ' *   注入掩码 0x0100：只开偏航状态，roll/pitch 的 K 被清零 = 只修正姿态角\n'
    ' *   R = sigma_h^2 + (tanI*sigma_tilt)^2：倾角不确定度会污染平面内航向；\n'
    ' *     重力有效时 sigma_tilt 极小，R 自动退化到 sigma_h^2。 */\n'
    'static uint8_t ekf_mag_entry_plane(float thm, float thp)\n'
    '{\n'
    '    float R1[1], r1[1], sig_h, sig_t, dl;\n'
    '\n'
    '    r1[0] = wrap_pi(thm - thp);\n'
    '    sig_h = V5F_EKF_MAG_VEC_SIG_DEG;\n'
    '    sig_t = sqrtf(s_P[6][6] + s_P[7][7]) * RAD2DEG;\n'
    '    if (sig_t > V5F_EKF_MAG_PLANE_TILT_CAP_DEG) sig_t = V5F_EKF_MAG_PLANE_TILT_CAP_DEG;\n'
    '    dl = V5F_EKF_DIP_TAN * (sig_t * DEG2RAD);\n'
    '    R1[0] = (sig_h * DEG2RAD) * (sig_h * DEG2RAD) + dl * dl;\n'
    '\n'
    '    H_zero(1u);\n'
    '    s_H[0][8] = 1.0f;\n'
    '    return ekf_update(R1, 1u, r1, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],\n'
    '                      0x0100u, V5F_EKF_MAG_VEC_K_MAX);\n'
    '}\n'
    '\n'
    '/* ---- 入口 B：矢量方式（VER=103 起的现实现，一字未改）：e1 偏航 + e2 倾斜 ---------- */\n'
    'static uint8_t ekf_mag_entry_vec(const float *r2v, const float *RRv)\n'
    '{\n'
    '    return ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],\n'
    '                      0x01C0u, V5F_EKF_MAG_VEC_K_MAX);\n'
    '}\n'
    '\n'
)

ANCH_STATIC = 'static uint32_t s_mag_cnt_upd;'
ANCH_GRAVOK = '    s_grav_ok = gate->ekf_tilt;'
ANCH_SMAGRS = '            s_mag_rs = 1.0f;'
ANCH_CALL = ('    st = ekf_update(RRv, 2u, r2v, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],\n'
             '                    0x01C0u, V5F_EKF_MAG_VEC_K_MAX);\n')
ANCH_CASE16 = '            s_gate_bits = 0u;'      # 行首锚（该行有两处：对齐块 + case16）
ANCH_ALIGN = '            s_bh_idx = 0u; s_bh_fill = 0u;'
ANCH_MAGFN = 'static void ekf_m7_mag(const volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate)'

NEW_CASE16_LINES = [
    '            /* ---- VER=123 重力矢量误差估计（每 EKF 观测帧一次）----------------------',
    '             * 门控判断用两观测帧之间的 DRDY 帧降采样：多数有重力牵引 -> 算"有牵引"。',
    '             * 有牵引：重力正在修倾角 -> 比例衰减（位移，无乘法/无溢出）；',
    '             * 无牵引：累加陀螺自己的误差率（1000 ppm*|w| + 2 deg/h）。',
    '             * 饱和加：到 u32 顶停在最大值，不回绕。 */',
    '            {',
    '                float dtw = s_t_run - s_ff_t;',
    '                s_ff_t = s_t_run;',
    '                if (s_ff_n > 0u) {',
    '                    if (s_ff_ok * 2u >= s_ff_n) {',
    '                        s_grav_err -= (s_grav_err >> V5F_EKF_MAG_FF_SHIFT);',
    '                    } else {',
    '                        float inc = (V5F_EKF_GYRO_KS_YAW * s_wmag * RAD2DEG',
    '                                     + V5F_EKF_GYRO_DRIFT_DPS) * dtw * 1.0e6f;',
    '                        uint32_t a = (inc > 0.0f) ? (uint32_t)inc : 0u;',
    '                        if (s_grav_err <= (V5F_EKF_MAG_FF_MAX - a)) {',
    '                            s_grav_err += a;',
    '                        } else {',
    '                            s_grav_err = V5F_EKF_MAG_FF_MAX;   /* 饱和，不回绕 */',
    '                        }',
    '                    }',
    '                    s_ff_n = 0u; s_ff_ok = 0u;',
    '                }',
    '            }',
]

NEW_MODE = (
    '    /* ---- VER=123 两个入口互斥：由"重力矢量误差估计"决定用哪一个 ------------------\n'
    '     * s_grav_err <= THR(2 deg)：重力矢量误差很低 -> 入口 A（投影到重力法平面，\n'
    '     *                            只修正姿态角，roll/pitch 完全不参与）\n'
    '     * s_grav_err >  THR       ：重力长期没牵引、倾角只能靠陀螺 -> 入口 B（矢量方式）\n'
    '     * 阈值比较用微度整数；不加回差（判据不含 EKF 的 P 反馈，不存在极限环）。 */\n'
    '    if (s_grav_err > (uint32_t)(V5F_EKF_MAG_FF_THR_DEG * 1.0e6f)) {\n'
    '        st = ekf_mag_entry_vec(r2v, RRv);      /* 入口 B：矢量方式 */\n'
    '    } else {\n'
    '        st = ekf_mag_entry_plane(thm, thp);    /* 入口 A：重力法平面投影，只修正姿态角 */\n'
    '    }\n')


def _w(path, text):
    """先编码再打开写：否则 encode 抛异常时文件已被 open('wb') 截断成 0 字节（踩过一次）。"""
    b = text.encode('gbk')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+123u', tune) and 'ekf_mag_entry_plane' in ekf:
        print('已是 VER=123（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+122u', tune), '基线不是 VER=122'
        assert 'ekf_mag_entry_plane' not in ekf and 's_grav_err' not in ekf
        for a in (ANCH_STATIC, ANCH_GRAVOK, ANCH_SMAGRS, ANCH_CALL, ANCH_ALIGN, ANCH_MAGFN):
            assert ekf.count(a) == 1, ('锚点 x%d: %r' % (ekf.count(a), a[:60]))
        _ci = [i for i, l in enumerate(ekf.split('\n')) if l.startswith(ANCH_CASE16)]
        assert len(_ci) == 2, ('case16 锚点 x%d' % len(_ci))
        # 新文本必须是 GBK 可编码的（源文件是 GBK；U+2212 之类的破折号会炸）
        for _nm, _s in (('NEW_TUNE', NEW_TUNE), ('NEW_STATICS', NEW_STATICS),
                        ('NEW_ENTRIES', NEW_ENTRIES),
                        ('NEW_CASE16', '\n'.join(NEW_CASE16_LINES))):
            try:
                _s.encode('gbk')
            except UnicodeEncodeError as _ex:
                raise SystemExit('%s 含非 GBK 字符 %r (U+%04X)'
                                 % (_nm, _s[_ex.start:_ex.end], ord(_s[_ex.start])))

        for f in (TUNE, EKF, SPI):
            B.save(f, TAG)

        # ---- proc_ekf.c ----
        ekf = ekf.replace(ANCH_STATIC, NEW_STATICS + ANCH_STATIC, 1)
        ekf = ekf.replace(ANCH_MAGFN, NEW_ENTRIES + ANCH_MAGFN, 1)
        ekf = ekf.replace(ANCH_SMAGRS,
            '            s_mag_rs = (float)s_grav_err * 1.0e-6f;   /* VER=123 上报：'
            '重力矢量误差估计 deg（0=重力有效）*/', 1)
        ekf = ekf.replace(ANCH_CALL,
            '    /* ---- VER=123 两个入口互斥：由"重力矢量误差估计"决定用哪一个 ------------------\n'
            '     * s_grav_err <= THR(2 deg)：重力矢量误差很低 -> 入口 A（投影到重力法平面，\n'
            '     *                            只修正姿态角，roll/pitch 完全不参与）\n'
            '     * s_grav_err >  THR       ：重力长期没牵引、倾角只能靠陀螺 -> 入口 B（矢量方式）\n'
            '     * 阈值比较用微度整数；不加回差（判据不含 EKF 的 P 反馈，不存在极限环）。 */\n'
            '    if (s_grav_err > (uint32_t)(V5F_EKF_MAG_FF_THR_DEG * 1.0e6f)) {\n'
            '        st = ekf_mag_entry_vec(r2v, RRv);      /* 入口 B：矢量方式 */\n'
            '    } else {\n'
            '        st = ekf_mag_entry_plane(thm, thp);    /* 入口 A：重力法平面投影，只修正姿态角 */\n'
            '    }\n', 1)
        # case16 的累加器块：按行插入（那行注释是中文，避免整行锚点编码坑）
        _L = ekf.split('\n')
        _ci = [i for i, l in enumerate(_L) if l.startswith(ANCH_CASE16)]
        assert len(_ci) == 2, ('case16 锚点 x%d' % len(_ci))
        _L[_ci[-1]:_ci[-1]] = NEW_CASE16_LINES
        ekf = '\n'.join(_L)
        ekf = ekf.replace(ANCH_ALIGN, ANCH_ALIGN +
            '\n            s_grav_err = 0u; s_ff_n = 0u; s_ff_ok = 0u; s_ff_t = s_t_run;'
            '   /* VER=123 切换判据复位 */', 1)
        # DRDY 帧计数（两观测帧之间的降采样源）
        ekf = ekf.replace(ANCH_GRAVOK, ANCH_GRAVOK +
            '\n    /* VER=123 地磁两入口切换：观测帧之间的 DRDY 帧计数（门控降采样源） */\n'
            '    s_ff_n++;\n'
            '    if (gate->ekf_tilt) s_ff_ok++;', 1)
        _w(EKF, ekf)

        # ---- tune ----
        m = re.search(r'#define V5F_EKF_MAG_VEC_SIG_TILT_DEG[^\n]*\n', tune)
        assert m, 'SIG_TILT 锚点'
        tune = tune[:m.end()] + NEW_TUNE + tune[m.end():]
        tune, n = re.subn(r'#define V5F_FW_VER\s+122u', '#define V5F_FW_VER        123u', tune, count=1)
        assert n == 1
        _w(TUNE, tune)

        # ---- SPI_rx.c：第 761 列注释（只改注释）----
        spi = open(SPI, 'rb').read().decode('gbk')
        if 'g_v5f_hold.ekf.mag_rs' in spi:
            spi = re.sub(r'(ch\[c\+\+\] = g_v5f_hold\.ekf\.mag_rs;)\s*/\*[^\n]*\*/',
                         r'\1           /* VER=123 上报：重力矢量误差估计(deg)，<2 用投影入口 */',
                         spi, count=1)
            _w(SPI, spi)
        print('patched: tune(+GYRO_DRIFT_DPS,+MAG_FF_MAX/SHIFT/THR_DEG,+PLANE_TILT_CAP,VER=123u)')
        print('         proc_ekf.c(两个入口 + 重力矢量误差估计累加器)  SPI_rx.c(仅注释)')

    # ---------------- verify ----------------
    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+123u', tune)
    assert re.search(r'#define V5F_EKF_GYRO_DRIFT_DPS\s+\(2\.0f / 3600\.0f\)', tune)
    assert re.search(r'#define V5F_EKF_MAG_FF_MAX\s+0xFFFFFFFFu', tune)
    assert re.search(r'#define V5F_EKF_MAG_FF_SHIFT\s+7u', tune)
    assert re.search(r'#define V5F_EKF_MAG_FF_THR_DEG\s+2\.0f', tune)
    assert re.search(r'#define V5F_EKF_MAG_PLANE_TILT_CAP_DEG\s+5\.0f', tune)
    # 两个入口函数
    assert ekf.count('static uint8_t ekf_mag_entry_plane(float thm, float thp)') == 1
    assert ekf.count('static uint8_t ekf_mag_entry_vec(const float *r2v, const float *RRv)') == 1
    assert ekf.count('return ekf_update(R1, 1u, r1, V5F_EKF_NIS_MAX_MAG') == 1
    assert ekf.count('0x0100u, V5F_EKF_MAG_VEC_K_MAX);') == 1     # 入口A 只开偏航状态
    assert ekf.count('0x01C0u, V5F_EKF_MAG_VEC_K_MAX);') == 1     # 入口B 三姿态行
    assert ekf.count('r1[0] = wrap_pi(thm - thp);') == 1
    # s_H[0][8]=1 两处：入口A + ekf_m7_mag 里 VER=103 之前遗留的（现已死代码）诊断块
    assert ekf.count('s_H[0][8] = 1.0f;') == 2
    # 互斥调用
    assert ekf.count('st = ekf_mag_entry_vec(r2v, RRv);') == 1
    assert ekf.count('st = ekf_mag_entry_plane(thm, thp);') == 1
    # 原内联的 2 维调用只剩入口函数里那一处
    assert ekf.count('ekf_update(RRv, 2u, r2v') == 1
    assert 'ekf_mag_entry_vec(const float *r2v, const float *RRv)\n{\n    return ekf_update(RRv, 2u, r2v' in ekf
    # 累加器
    assert ekf.count('static uint32_t s_grav_err;') == 1
    assert ekf.count('s_ff_n++;') == 1 and ekf.count('if (gate->ekf_tilt) s_ff_ok++;') == 1
    assert ekf.count('s_grav_err -= (s_grav_err >> V5F_EKF_MAG_FF_SHIFT);') == 1
    assert ekf.count('V5F_EKF_GYRO_KS_YAW * s_wmag * RAD2DEG') == 1
    assert ekf.count('s_grav_err <= (V5F_EKF_MAG_FF_MAX - a)') == 1
    assert ekf.count('s_grav_err = V5F_EKF_MAG_FF_MAX;') == 1
    assert ekf.count('if (s_grav_err > (uint32_t)(V5F_EKF_MAG_FF_THR_DEG * 1.0e6f))') == 1
    assert ekf.count('s_grav_err = 0u; s_ff_n = 0u; s_ff_ok = 0u; s_ff_t = s_t_run;') == 1
    # 入口A/B 都必须在 ekf_m7_mag 之前定义
    assert ekf.find('ekf_mag_entry_plane(float') < ekf.find('static void ekf_m7_mag(')
    assert ekf.find('static uint8_t ekf_update(') < ekf.find('ekf_mag_entry_plane(float')
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'), '括号不平衡'
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_GYRO_DRIFT_DPS[^\n]*',
                r'#define V5F_EKF_MAG_FF_MAX[^\n]*', r'#define V5F_EKF_MAG_FF_SHIFT[^\n]*',
                r'#define V5F_EKF_MAG_FF_THR_DEG[^\n]*',
                r'#define V5F_EKF_MAG_PLANE_TILT_CAP_DEG[^\n]*'):
        print('  |', re.search(pat, tune).group()[:110])
    print('---- 入口 A / 入口 B ----')
    i = ekf.find('static uint8_t ekf_mag_entry_plane(float')
    for s in ekf[i - 90:i + 1550].split('\n'):
        print('  |', s[:114])
    print('---- 互斥调用 + 累加器 ----')
    k = ekf.find('VER=123 两个入口互斥')
    for s in ekf[k - 60:k + 700].split('\n'):
        print('  |', s[:114])
    k = ekf.find('VER=123 重力矢量误差估计（每 EKF 观测帧一次）')
    for s in ekf[k - 30:k + 1500].split('\n'):
        print('  |', s[:114])
    print()
    for mac in ('V5F_FW_VER', 'V5F_EKF_GYRO_DRIFT_DPS', 'V5F_EKF_MAG_FF_MAX',
                'V5F_EKF_MAG_FF_SHIFT', 'V5F_EKF_MAG_FF_THR_DEG',
                'V5F_EKF_MAG_PLANE_TILT_CAP_DEG'):
        n = len(re.findall(r'#define\s+%s\b' % mac, tune))
        print('LINT %-32s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= n == 1
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v122 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v122 V5F\\User\\src\\proc_ekf.c -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\SPI_rx.c.bak_v122 V5F\\User\\src\\SPI_rx.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
