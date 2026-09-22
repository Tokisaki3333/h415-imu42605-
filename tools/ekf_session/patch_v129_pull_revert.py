# -*- coding: utf-8 -*-
r"""VER=129：把地磁偏航牵引**回滚到 VER=125 降低之前**（去掉常数 R 地板）

用户口径（2026-09-22）："回滚牵引速度 …… 现在牵引还不是期望的那个旧版本的快速牵引，
应该在上次降低前。"

"上次降低" = VER=125 给偏航牵引加的**常数 R 地板**：
    K_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S   （常数 3.32e-4，与状态无关）
    R_yaw >= P88*(1/K_max - 1)
它把增益钉死在"静止工作点"，于是 P88 变大（上电初值、失效后重开、剧烈运动 Q 增长）
时地磁**也不能快拉**，只能按 15.85 s 慢爬 —— 实测极限段稳态滞后 28~40 deg
（docs/mag_calibration_log.md §8.13）。

本补丁只删这三处（`git diff bak_v125` 里全部牵引相关差异）：
  1) proc_ekf.c 的静态量 `s_mag_ryaw_min`
  2) `ekf_mag_entry_plane()` 里的 `if (R1[0] < s_mag_ryaw_min) ...`
  3) `ekf_m7_mag()` 里生成该地板的整个 `{ ... }` 块
  4) v5f_tune.h 的 `V5F_MAG_YAW_TAU_MIN_S`（删除；`V5F_MAG_EPOCH_DT_S` 保留作设计记录）
**不动**：VER=126/127 运动一致性门、VER=123 双入口 + 重力失效累加器、VER=122 的
Q_yaw 设计（ARW + KS_YAW*|w|，Q_YAW_MAX）、`V5F_EKF_MAG_VEC_K_MAX=0.10`、
`V5F_EKF_YAW_P_MIN`（1e-12 与 v124 的 (0.01deg)^2=3e-8 都远低于自然 P88≈4.5e-7，无影响）。

回滚后的牵引速度 = **自然 EKF 增益** K = P88/(P88+R)（K 上限仍由 0.10 的 k_cap 把关）：
  - 静止：P88≈4.5e-7 -> K≈3.5e-4（tau≈15 s，与设计一致，未变快）
  - 高转速：Q 增长 -> K≈5.5e-3（tau≈0.95 s）
  - P88 变大（上电 / 失效后重开 / 大机动后）：K 按 P/(P+R) 上升到 ≤0.10（tau≥53 ms）**快速牵引**
这正是"旧版本的快速牵引"。

用法: python tools/ekf_session/patch_v129_pull_revert.py
回退: bak_src\V5F\User\{inc\v5f_tune.h,src\proc_ekf.c}.bak_v129
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v129'

# tune 中 VER=125 "牵引速度上限" 注释块 + 宏定义（中文前缀锚点，unique）
ANCH_T = '/* VER=125 \u5730\u78c1\u504f\u822a"\u7275\u5f15\u901f\u5ea6"\u4e0b'

NOTE_T = (
    '/* VER=129：**删除** VER=125 的常数牵引速度上限，回到"降低前"的自然 EKF 增益\n'
    ' * K = P88/(P88+R)（用户口径：牵引应是旧版本的快速牵引）。VER=125 曾用\n'
    ' *   K <= K_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S (3.32e-4 常数)\n'
    ' * 把增益钉在静止工作点，导致 P88 变大时地磁也不能快拉：实测极限运动\n'
    ' * （R:\\imu_20260922_201451.bin, |w| p90 2010 dps）偏航稳态滞后 28~40 deg。\n'
    ' * 现由 EKF 自身增益决定，K 上限只由 V5F_EKF_MAG_VEC_K_MAX=0.10 把关；\n'
    ' * 运动一致性门（VER=126/127）与双入口（VER=123）不变。\n'
    ' * 静止点不变：P88≈4.5e-7 -> K≈3.5e-4（tau≈15 s）；P88 变大时按 P/(P+R) 快速牵引。 */\n')

STATIC_LINE = re.compile(r'[ \t]*static float[ \t]+s_mag_ryaw_min;[^\n]*\n')
PLANE_IF = re.compile(r'[ \t]*/\* VER=125[^\n]*\n[ \t]*if \(R1\[0\] < s_mag_ryaw_min\)[^\n]*\n')


def _w(path, text):
    b = text.encode('gbk')          # 先编码后打开（防截断）
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+129u', tune):
        print('已是 VER=129（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+127u', tune), '基线不是 VER=127'
        assert tune.count(ANCH_T) == 1, 'TAU_MIN 注释块锚点'
        assert len(STATIC_LINE.findall(ekf)) == 1, 's_mag_ryaw_min 静态量锚点'
        assert len(PLANE_IF.findall(ekf)) == 1, 'entry_plane R 地板锚点'
        # 地板块：从 VER=125 注释头到块尾（index 定位，避免正则跨块误吞）
        i = ekf.index('    /* ---- VER=125')
        j = ekf.index('    if (s_grav_err > (uint32_t)', i)
        blk = ekf[i:j]
        assert 's_mag_ryaw_min' in blk and 'k_max' in blk and blk.count('{') == blk.count('}')
        assert blk.count('}') == 3, '地板块条目数异常: %d' % blk.count('}')

        B.save(TUNE, TAG)
        B.save(EKF, TAG)

        # ---- tune：去掉 TAU_MIN 注释块 + 宏，替换为 VER=129 说明；版本号 +2
        k = tune.index(ANCH_T)
        m = tune.index('\n', tune.index('#define V5F_MAG_YAW_TAU_MIN_S', k)) + 1
        tune = tune[:k] + NOTE_T + tune[m:]
        tune, n = re.subn(r'#define V5F_FW_VER\s+127u', '#define V5F_FW_VER        129u',
                          tune, count=1)
        assert n == 1, 'VER 替换'
        assert tune.count('#define V5F_MAG_EPOCH_DT_S       0.00526f') == 1
        tune = tune.replace(
            '#define V5F_MAG_EPOCH_DT_S       0.00526f',
            '#define V5F_MAG_EPOCH_DT_S       0.00526f  /* VER=129 起仅作设计口径记录，'
            '代码不再用（原 R 地板已删） */', 1)
        _w(TUNE, tune)

        # ---- proc_ekf.c：删静态量 / entry_plane 抬 R / 地板块
        ekf2, n1 = STATIC_LINE.subn('', ekf, count=1)
        ekf2, n2 = PLANE_IF.subn('', ekf2, count=1)
        i = ekf2.index('    /* ---- VER=125')
        j = ekf2.index('    if (s_grav_err > (uint32_t)', i)
        ekf2 = ekf2[:i] + ekf2[j:]
        assert (n1, n2) == (1, 1)
        assert 's_mag_ryaw_min' not in ekf2, '仍有 s_mag_ryaw_min 残留'
        _w(EKF, ekf2)
        print('patched: 牵引回到自然 EKF 增益（常数 R 地板已删）')

    # ---------------- verify ----------------
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    ok = True
    ok &= bool(re.search(r'#define V5F_FW_VER\s+129u', tune))
    ok &= (not re.search(r'#define\s+V5F_MAG_YAW_TAU_MIN_S', tune))
    ok &= ('V5F_MAG_YAW_TAU_MIN_S' not in ekf)
    ok &= ('s_mag_ryaw_min' not in ekf)
    ok &= (ekf.count('        float k_max') == 0)
    # 门控 / 双入口 / 设计 Q 全部保留
    ok &= bool(re.search(r'#define V5F_MAG_DIST_THR_DPS\s+20\.0f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_DIST_KS\s+0\.0f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_DIST_WMAX_DPS\s+5\.0f', tune))
    ok &= bool(re.search(r'#define V5F_MAG_DIST_HOLD_N\s+4011u', tune))
    ok &= bool(re.search(r'#define V5F_MAG_ERR_LIM\s+0\.10f', tune))
    ok &= bool(re.search(r'#define V5F_EKF_YAW_OBS_EN\s+1u', tune))
    ok &= bool(re.search(r'#define V5F_EKF_MAG_VEC_K_MAX\s+0\.10f', tune))
    ok &= bool(re.search(r'#define V5F_EKF_Q_YAW_MAX\s+7\.14e-6f', tune))
    ok &= bool(re.search(r'#define V5F_EKF_GYRO_KS_YAW\s+1\.0e-3f', tune))
    ok &= (ekf.count('if (!clipped && wm < V5F_MAG_DIST_WMAX_DPS && rate > thr)') == 1)
    ok &= (ekf.count('ekf_mag_entry_plane(thm, thp)') == 1)
    ok &= (ekf.count('ekf_mag_entry_vec(r2v, RRv)') == 1)
    ok &= (ekf.count('0x0100u, V5F_EKF_MAG_VEC_K_MAX') == 1)
    ok &= (ekf.count('0x01C0u, V5F_EKF_MAG_VEC_K_MAX') == 1)
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    ok &= (e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'))
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_MAG_VEC_K_MAX[^\n]*',
                r'#define V5F_EKF_YAW_P_MIN[^\n]*', r'#define V5F_EKF_Q_YAW_MAX[^\n]*',
                r'#define V5F_MAG_EPOCH_DT_S[^\n]*', r'#define V5F_MAG_DIST_[A-Z_]*[^\n]*'):
        for mm in re.finditer(pat, tune):
            print('  |', mm.group()[:118])
    print('---- proc_ekf.c：入口选择处上下文 ----')
    i = ekf.index('    if (s_grav_err > (uint32_t)')
    print(''.join(c if ord(c) < 128 else '.' for c in ekf[i - 260:i + 200]))
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v129 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v129 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
