# -*- coding: utf-8 -*-
r"""VER=127：地磁失效门的保持时间与判定条件重定（按用户口径）
   - "无效保持时间太长了"        -> HOLD 3 s -> 0.5 s
   - "门关的时间还是太长；这段运动中基本没有真的受到干扰，基本是陀螺仪残差"
     -> 判定条件从"按转速放宽门限(THR+0.5|w|)"改成**只在接近静止时判定**：|w| < 5 dps。
        理由：剧烈运动时 e=|Δψmag-Δψgyro|/T 的大头是**陀螺自己的残差/削顶/姿态滞后**，
        不是地磁干扰（用户实测结论）；而且 VER=125 已经把地磁牵引锁到 τ>=15.85 s，
        运动中本来就不需要这道门。静止/小运动才是外部磁干扰会"造成假运动"的场景。

新录像实测（T=0.25 s, THR=20 dps）：
    |w|<5 dps guard, HOLD=0.5 s:
        极限段(201451) 门关 4.6%（原 28%）
        受扰静止段(195513/195551) **干扰期内门开 0.00% / 0.05%**（照样全挡）
    HOLD=1 s: 极限段 6.9% / 干扰期 0.00% —— 取 0.5 s 更符合"不要再关那么久"。

用法: python tools/ekf_session/patch_v127_gate_relax.py
回退: bak_src\V5F\User\{inc\v5f_tune.h,src\proc_ekf.c}.bak_v127
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v127'

NEW_NOTE = (
    '/* VER=127 判定条件收紧为"只在接近静止时判"（用户：运动中门关太久，且那些偏差基本\n'
    ' * 是陀螺自己的残差/削顶，不是地磁干扰）；保持时间 3 s -> 0.5 s。\n'
    ' * 实测（T=0.25 s、THR=20 dps、|w|<5 dps、HOLD=0.5 s）：\n'
    ' *   受扰静止段(195513/195551) 干扰期内门开 0.00% / 0.05%（照样全挡）；\n'
    ' *   极限段(201451, |w| p50 337 / p90 2010 dps) 门关 4.6%（原 28%）。 */\n')


def _w(path, text):
    b = text.encode('gbk')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')

    if re.search(r'#define V5F_FW_VER\s+127u', tune):
        print('已是 VER=127（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+126u', tune), '基线不是 VER=126'
        assert re.search(r'#define V5F_MAG_DIST_WMAX_DPS\s+100\.0f', tune)
        assert re.search(r'#define V5F_MAG_DIST_HOLD_N\s+24066u', tune)
        assert ekf.count('        float thr = V5F_MAG_DIST_THR_DPS + V5F_MAG_DIST_KS * wm;') == 1
        assert ekf.count('if (!clipped && wm < V5F_MAG_DIST_WMAX_DPS && rate > thr)') == 1

        B.save(TUNE, TAG)
        B.save(EKF, TAG)

        tune = tune.replace('#define V5F_MAG_DIST_WMAX_DPS  100.0f',
                            '#define V5F_MAG_DIST_WMAX_DPS  5.0f     /* VER=127: 只在接近静止时判定 */', 1)
        tune = tune.replace('#define V5F_MAG_DIST_HOLD_N    24066u   /* 3 s */',
                            '#define V5F_MAG_DIST_HOLD_N    4011u    /* VER=127: 3 s -> 0.5 s */', 1)
        tune, nk = re.subn(r'#define V5F_MAG_DIST_KS\s+0\.5f[^\n]*',
                           '#define V5F_MAG_DIST_KS        0.0f     /* VER=127: 改为"只在静止判"，'
                           '不再按转速放宽（保留常量便于回退） */', tune, count=1)
        assert nk == 1, 'KS 锚点'
        # 在参数块开头插入说明
        m = re.search(r'#define V5F_MAG_DIST_DT_S[^\n]*\n', tune)
        assert m
        tune = tune[:m.start()] + NEW_NOTE + tune[m.start():]
        tune, n = re.subn(r'#define V5F_FW_VER\s+126u', '#define V5F_FW_VER        127u', tune, count=1)
        assert n == 1
        _w(TUNE, tune)

        # proc_ekf.c：注释同步（门限公式仍用 KS，KS 现在是 0）
        ekf = ekf.replace('float thr = V5F_MAG_DIST_THR_DPS + V5F_MAG_DIST_KS * wm;',
                          'float thr = V5F_MAG_DIST_THR_DPS + V5F_MAG_DIST_KS * wm;'
                          '   /* VER=127: KS=0 -> 固定 20 dps */', 1)
        _w(EKF, ekf)
        print('patched: |w| guard 100 -> 5 dps, HOLD 3 s -> 0.5 s, KS 0.5 -> 0')

    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+127u', tune)
    assert re.search(r'#define V5F_MAG_DIST_WMAX_DPS\s+5\.0f', tune)
    assert re.search(r'#define V5F_MAG_DIST_HOLD_N\s+4011u', tune)
    assert re.search(r'#define V5F_MAG_DIST_KS\s+0\.0f', tune)
    assert re.search(r'#define V5F_MAG_DIST_THR_DPS\s+20\.0f', tune)
    assert re.search(r'#define V5F_MAG_DIST_WIN_N\s+2006u', tune)
    assert re.search(r'#define V5F_MAG_CLIP_LSB\s+32700', tune)
    assert ekf.count('VER=127: KS=0') == 1
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')')
    print()
    print('---- tune ----')
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_MAG_DIST_WIN_N[^\n]*',
                r'#define V5F_MAG_DIST_THR_DPS[^\n]*', r'#define V5F_MAG_DIST_KS[^\n]*',
                r'#define V5F_MAG_DIST_WMAX_DPS[^\n]*', r'#define V5F_MAG_DIST_HOLD_N[^\n]*'):
        print('  |', re.search(pat, tune).group()[:112])
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v127 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v127 V5F\\User\\src\\proc_ekf.c -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
