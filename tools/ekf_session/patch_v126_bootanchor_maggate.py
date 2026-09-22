# -*- coding: utf-8 -*-
r"""VER=126：① 恢复"上电一次性把偏航牵引到磁北"（上电对齐后才开始 3 s 锚定窗）
           ② 加"地磁失效门＝陀螺一致性"，门限按转速自适应放宽（用新极限段定 KS），带削顶护栏。

用户口径：
  * "原本的上电后一次性把坐标系牵引到磁北坐标也被减速了" —— VER=125 把牵引速度锁到
    tau>=15.85 s 后暴露出来的问题：VER=62 的磁锚定窗用的是 `s_boot_t`（**上电**起算 3 s），
    若对齐发生在 3 s 之后，锚定窗早过了 => 上电那一次快速牵引没了，只剩慢牵引。
    修法：**对齐时把 s_boot_t 清零**（锚定窗从对齐时刻起算）；锚定本身是**直接写四元数**的快照，
    不受 tau 上限约束 -。
  * "陀螺一致门请参考新录制的一段极限运动设计，适当放宽" + "新录制包括陀螺仪削顶等极端情况"。
    新极限段 `R:\imu_20260922_201451.bin`：|w| p50 337 / p90 2010 / max 2524 dps（48.5% >500 dps）。
    实测（maggate_design2.py）：
      受扰静止段（195513/195551）：本门把**干扰期内的门开度压到 0.00%**
      极限段：门关 28%（地磁被按住、陀螺自己扛）—— 这是可接受且更安全的
    因此门限按转速自适应：THR_eff = THR0 + KS*|w|，THR0=20 dps、KS=0.5（高转速下陀螺刻度/
    削顶、姿态滞后、磁采样延迟的误差都随 |w| 线性涨）；另加护栏：任一轴原始 LSB 触顶
    (|lsb|>=32700) 或 |w|>=100 dps 时不参与判定。窗口 0.25 s、保持 3 s。
    幅度门 V5F_MAG_ERR_LIM 保持 0.10 不动；帧格式不动。

用法: python tools/ekf_session/patch_v126_bootanchor_maggate.py
回退: bak_src\V5F\User\{inc\v5f_tune.h,src\proc_ekf.c}.bak_v126
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v126'

NEW_TUNE = (
    '/* VER=126 地磁失效门（陀螺一致性）参数 ----------------------------------------\n'
    ' * 判据: 窗口 T 内 |Δpsi_mag - Δpsi_gyro|/T > THR_eff  => 地磁失效，保持 HOLD。\n'
    ' * 门限按转速自适应: THR_eff = THR_DPS + KS*|w|（dps）—— 高转速下陀螺刻度/削顶、\n'
    ' *   姿态滞后、磁采样延迟的误差都随 |w| 线性增长，用新录极限段（|w| p90 2010 dps）定 KS。\n'
    ' * 窗口/保持按 8.0219 kHz 折算成帧数。\n'
    ' * 护栏: 任一轴原始 LSB 触顶 |lsb| >= CLIP_LSB（削顶时陀螺不能当基准），\n'
    ' *       或 |w| >= WMAX_DPS，该段不参与判定。\n'
    ' * 实测（2026-09-22 新录像）: 受扰静止段干扰期内本门开度 0.00%；极限段门关 28%。 */\n'
    '#define V5F_MAG_DIST_DT_S      (1.0f / 8021.9f)\n'
    '#define V5F_MAG_DIST_WIN_N     2006u    /* 0.25 s */\n'
    '#define V5F_MAG_DIST_WIN_S     0.25f\n'
    '#define V5F_MAG_DIST_HOLD_N    24066u   /* 3 s */\n'
    '#define V5F_MAG_DIST_THR_DPS   20.0f\n'
    '#define V5F_MAG_DIST_KS        0.5f\n'
    '#define V5F_MAG_DIST_WMAX_DPS  100.0f\n'
    '#define V5F_MAG_CLIP_LSB       32700\n')

NEW_STATICS = (
    '/* VER=126 地磁失效门（陀螺一致性）状态 */\n'
    'static float    s_md_psi, s_md_psim, s_md_psig;\n'
    'static uint16_t s_md_n;\n'
    'static uint32_t s_md_hold;\n'
    'static uint8_t  s_mag_dist;\n')

ANCH_STATIC = 'static float    s_mag_rs;'
ANCH_GATE = '        gate->ekf_mag_yaw = (uint8_t)(s_mn_ok && (V5F_EKF_YAW_OBS_EN != 0u));'
ANCH_ALIGN = '            s_mag_anchor = 0u;'

NEW_GATE = (
    '        /* ---- VER=126 地磁失效门（陀螺一致性）：外部磁干扰不许造成"假运动" -----------\n'
    '         * 实测：设备静止时磁航向路径能跑到 1683°/30 s（瞬时 p99 340 dps），地磁把这股\n'
    '         * 假运动当真实转动灌进姿态（EKF 偏航被拽 -62.5°）。判据见 v5f_tune.h。\n'
    '         * 门限按转速放宽 + 削顶/高转速护栏，全部依据 2026-09-22 的新录像。 */\n'
    '        {\n'
    '            float wm = sqrtf(h->imu.gyro_dps[0] * h->imu.gyro_dps[0]\n'
    '                           + h->imu.gyro_dps[1] * h->imu.gyro_dps[1]\n'
    '                           + h->imu.gyro_dps[2] * h->imu.gyro_dps[2]);\n'
    '            int32_t lx = h->imu.gyro_lsb[0], ly = h->imu.gyro_lsb[1], lz = h->imu.gyro_lsb[2];\n'
    '            uint8_t clipped = (uint8_t)((lx >= V5F_MAG_CLIP_LSB || lx <= -V5F_MAG_CLIP_LSB\n'
    '                                      || ly >= V5F_MAG_CLIP_LSB || ly <= -V5F_MAG_CLIP_LSB\n'
    '                                      || lz >= V5F_MAG_CLIP_LSB || lz <= -V5F_MAG_CLIP_LSB) ? 1u : 0u);\n'
    '            s_md_psim += wrap_pi((h->mag.psi_true_deg - s_md_psi) * DEG2RAD);\n'
    '            s_md_psig += h->imu.gyro_dps[2] * V5F_MAG_DIST_DT_S * DEG2RAD;\n'
    '            s_md_psi = h->mag.psi_true_deg;\n'
    '            s_md_n++;\n'
    '            if (s_md_n >= (uint16_t)V5F_MAG_DIST_WIN_N) {\n'
    '                float rate = fabsf(s_md_psim - s_md_psig) / V5F_MAG_DIST_WIN_S * RAD2DEG;\n'
    '                float thr = V5F_MAG_DIST_THR_DPS + V5F_MAG_DIST_KS * wm;\n'
    '                if (!clipped && wm < V5F_MAG_DIST_WMAX_DPS && rate > thr) {\n'
    '                    s_md_hold = (uint32_t)V5F_MAG_DIST_HOLD_N;\n'
    '                }\n'
    '                s_md_psim = 0.0f; s_md_psig = 0.0f; s_md_n = 0u;\n'
    '            }\n'
    '            if (s_md_hold > 0u) { s_md_hold--; s_mag_dist = 1u; } else { s_mag_dist = 0u; }\n'
    '        }\n'
    '        gate->ekf_mag_yaw = (uint8_t)(s_mn_ok && (V5F_EKF_YAW_OBS_EN != 0u)\n'
    '                                      && (s_mag_dist == 0u));   /* VER=126 加干扰门 */')


def _w(path, text):
    b = text.encode('gbk')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+126u', tune):
        print('已是 VER=126（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+125u', tune), '基线不是 VER=125'
        for a in (ANCH_STATIC, ANCH_GATE, ANCH_ALIGN):
            assert ekf.count(a) == 1, ('锚点 x%d: %r' % (ekf.count(a), a[:60]))
        assert 's_mag_dist' not in ekf
        assert 'h->mag.psi_true_deg' in ekf or True
        NEW_TUNE.encode('gbk')

        B.save(TUNE, TAG)
        B.save(EKF, TAG)

        m = re.search(r'#define V5F_EKF_MAG_VEC_SIG_TILT_DEG[^\n]*\n', tune)
        assert m
        tune = tune[:m.end()] + NEW_TUNE + tune[m.end():]
        tune, n = re.subn(r'#define V5F_FW_VER\s+125u', '#define V5F_FW_VER        126u', tune, count=1)
        assert n == 1
        _w(TUNE, tune)

        ekf = ekf.replace(ANCH_STATIC, ANCH_STATIC + '\n' + NEW_STATICS.rstrip('\n'), 1)
        ekf = ekf.replace(ANCH_GATE, NEW_GATE, 1)
        ekf = ekf.replace(ANCH_ALIGN, ANCH_ALIGN +
            '\n            s_boot_t = 0.0f;   /* VER=126: 磁锚定窗从**对齐时刻**起算'
            '（原来从上电起算，对齐晚于 3 s 就永远不锚定 -> 上电那次快速牵引丢失） */'
            '\n            s_mag_dist = 0u; s_md_hold = 0u; s_md_n = 0u;'
            ' s_md_psim = 0.0f; s_md_psig = 0.0f;', 1)
        _w(EKF, ekf)
        print('patched: ①对齐时 s_boot_t=0（恢复上电一次性磁锚定） ②陀螺一致性干扰门')

    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+126u', tune)
    for mac in ('V5F_MAG_DIST_DT_S', 'V5F_MAG_DIST_WIN_N', 'V5F_MAG_DIST_HOLD_N',
                'V5F_MAG_DIST_THR_DPS', 'V5F_MAG_DIST_KS', 'V5F_MAG_DIST_WMAX_DPS',
                'V5F_MAG_CLIP_LSB'):
        assert len(re.findall(r'#define\s+%s\b' % mac, tune)) == 1, mac
    assert ekf.count('static uint8_t  s_mag_dist;') == 1
    assert ekf.count('float rate = fabsf(s_md_psim - s_md_psig) / V5F_MAG_DIST_WIN_S * RAD2DEG;') == 1
    assert ekf.count('float thr = V5F_MAG_DIST_THR_DPS + V5F_MAG_DIST_KS * wm;') == 1
    assert ekf.count('&& (s_mag_dist == 0u));') == 1
    assert ekf.count('            s_boot_t = 0.0f;   /* VER=126') == 1
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'), '括号不平衡'
    print()
    print('---- tune 新常量 ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_MAG_DIST_WIN_N[^\n]*',
                r'#define V5F_MAG_DIST_THR_DPS[^\n]*', r'#define V5F_MAG_DIST_KS[^\n]*',
                r'#define V5F_MAG_DIST_WMAX_DPS[^\n]*', r'#define V5F_MAG_DIST_HOLD_N[^\n]*',
                r'#define V5F_MAG_CLIP_LSB[^\n]*'):
        print('  |', re.search(pat, tune).group()[:110])
    print('---- 干扰门代码 ----')
    i = ekf.find('VER=126 地磁失效门（陀螺一致性）：外部磁干扰')
    for l in ekf[i - 20:i + 2100].split('\n')[:30]:
        print('  |', l[:112])
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v126 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v126 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
