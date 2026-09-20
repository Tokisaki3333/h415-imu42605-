# -*- coding: utf-8 -*-
r"""通用磁标定写回：把给定 A/C 写进 v5f_tune.h 并把 V5F_FW_VER 加 1（默认 112）。

用法:
  python tools/ekf_session/patch_magAC.py --ver 112 \
      --A "-5.67435473e-03,1.90975069e-03,-3.72241548e-05" \
          "-1.84134583e-03,-6.20937298e-03,-6.23511870e-04" \
          "-3.42886202e-05,-4.55624744e-05,5.99987496e-03" \
      --C "-3.49747510e-03,1.96358466e-02,8.78533321e-03" \
      --note "三轴集(044249/044537/044559)显式参数化拟合+重力对齐；校验集045243/045306/045327 dip ±0.26°、rs 1.1"

备份 -> bak_src/V5F/User/inc/v5f_tune.h.bak_v<ver>（绝不写源码目录）
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')


def g(s):
    return s.encode('gbk', errors='replace')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ver', type=int, required=True)
    ap.add_argument('--A', nargs=3, required=True)
    ap.add_argument('--C', required=True)
    ap.add_argument('--note', default='')
    a = ap.parse_args()
    A = [[float(x) for x in row.split(',')] for row in a.A]
    Cv = [float(x) for x in a.C.split(',')]
    assert all(len(r) == 3 for r in A) and len(Cv) == 3
    src = open(TUNE, 'rb').read()
    d0 = src.decode('gbk', errors='replace')
    cur = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', d0).group(1))
    if cur == a.ver:
        print('已是 VER=%d（只校验）' % a.ver)
    else:
        assert a.ver == cur + 1, 'ver 必须 = 当前 %d + 1' % cur
        B.save(TUNE, '.bak_v%d' % a.ver)
        am = re.search(rb'#define V5F_MAG_A_INIT\s*\{.*?\}\s*\}', src, re.S)
        assert am
        src = src[:am.start()] + g(
            '#define V5F_MAG_A_INIT  { { %+.8ef, %+.8ef, %+.8ef }, \\\n'
            '                          { %+.8ef, %+.8ef, %+.8ef }, \\\n'
            '                          { %+.8ef, %+.8ef, %+.8ef } }'
            % (A[0][0], A[0][1], A[0][2], A[1][0], A[1][1], A[1][2],
               A[2][0], A[2][1], A[2][2])) + src[am.end():]
        cm = re.search(rb'#define V5F_MAG_C_INIT\s*\{[^}]*\}', src)
        assert cm
        src = src[:cm.start()] + g('#define V5F_MAG_C_INIT  { %+.8ef, %+.8ef, %+.8ef }'
                                   % tuple(Cv)) + src[cm.end():]
        src = g('/* VER=%d 磁标定：%s\n'
                ' *  形状=显式参数化(中心+log半轴+旋转)GN，合成自测形状误差 0.058%%；\n'
                ' *  取向=重力对齐到 WMM2025 I=54.330 deg @36.23N 120.44E；\n'
                ' *  拟合集 dip ±0.11 deg、留出校验集 ±0.26 deg、mag_rs 1.1（改前 2125~2500）。\n'
                ' *  详见 docs/mag_observability_math.md 7.10~7.16 */\n' % (a.ver, a.note)) + src
        src, n = re.subn((r'#define V5F_FW_VER\s+%du' % cur).encode(), b'#define V5F_FW_VER        %du' % a.ver, src, count=1)
        assert n == 1
        open(TUNE, 'wb').write(src)
        print('patched: VER=%du + 新 A/C' % a.ver)

    d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
    ok = True
    for mac in ('V5F_FW_VER', 'V5F_MAG_A_INIT', 'V5F_MAG_C_INIT', 'V5F_EKF_DIP_TAN', 'V5F_CDC_QUAT_ONLY'):
        n = len(re.findall(r'#define\s+%s\b' % mac, d))
        ok &= (n == 1)
        print('LINT %-20s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_DIP_TAN[^\n]*', r'#define V5F_MAG_C_INIT[^\n]*'):
        print('TUNE |', re.search(pat, d).group()[:100])
    left = B.strays()
    ok &= (not left) and bool(re.search(r'#define V5F_FW_VER\s+%du' % a.ver, d))
    print('源码树残留 %d %s' % (len(left), 'OK' if not left else '!!'))
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v%d V5F\\User\\inc\\v5f_tune.h -Force' % a.ver)
    print('VERIFY', 'OK' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
