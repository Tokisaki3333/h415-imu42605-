# -*- coding: utf-8 -*-
"""VER=111: 用新簇（imu_20260921_043248.bin，完整球型）的全拟 A/C 替换磁标定。

方法（代码先自测通过）：Gauss-Newton 直接最小化 Σ(|A u + C| - 1)^2 得**形状**，
再用**重力对齐**到 WMM2025 倾角 I=54.330°（36.23N 120.44E）得**取向**。
自测：已知合成椭球可还原 |y| p1/p50/p99 = 1.00000/1.00000/1.00000。
043248 上：|y|（全帧）0.957/1.000/1.035、对齐 RMS 0.39°、dip 53.89/54.31/54.78、
固件算式 rs 1.00/1.11/1.8（改前 VER=110 在该数据上是 dip 50.61、rs 221）。

用户判定"新簇更准"：04:26/04:06 那簇（042518/040450）与新簇（043104/043248）互差 3.7°，
两簇各自内部自洽（043248 全翻滚中 dip 展宽仅 0.95°），按用户结论采用新簇。

备份：bak_src/V5F/User/inc/v5f_tune.h.bak_v111
用法: python tools/ekf_session/patch_v111_magcal3.py
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
TAG = '.bak_v111'
A_NEW = [[-2.91311037e-03, +2.48554999e-03, -4.26777693e-03],
         [+1.49220959e-03, -4.51835900e-03, -3.94441722e-03],
         [-4.56905629e-03, -3.12125787e-03, +1.01543645e-03]]
C_NEW = [-2.75604564e-02, +8.58618375e-03, +6.45757002e-03]


def g(s):
    return s.encode('gbk', errors='replace')


def main():
    src = open(TUNE, 'rb').read()
    d0 = src.decode('gbk', errors='replace')
    if re.search(r'#define V5F_FW_VER\s+111u', d0):
        print('已是 VER=111（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+110u', d0), 'FW_VER 不是 110u'
        assert re.search(r'#define V5F_EKF_DIP_TAN\s+1\.3932f', d0), 'DIP_TAN 不是 1.3932f'
        B.save(TUNE, TAG)
        am = re.search(rb'#define V5F_MAG_A_INIT\s*\{.*?\}\s*\}', src, re.S)
        assert am
        src = src[:am.start()] + g(
            '#define V5F_MAG_A_INIT  { { %+.8ef, %+.8ef, %+.8ef }, \\\n'
            '                          { %+.8ef, %+.8ef, %+.8ef }, \\\n'
            '                          { %+.8ef, %+.8ef, %+.8ef } }'
            % (A_NEW[0][0], A_NEW[0][1], A_NEW[0][2], A_NEW[1][0], A_NEW[1][1],
               A_NEW[1][2], A_NEW[2][0], A_NEW[2][1], A_NEW[2][2])) + src[am.end():]
        cm = re.search(rb'#define V5F_MAG_C_INIT\s*\{[^}]*\}', src)
        assert cm
        src = src[:cm.start()] + g('#define V5F_MAG_C_INIT  { %+.8ef, %+.8ef, %+.8ef }'
                                   % tuple(C_NEW)) + src[cm.end():]
        src = g('/* VER=111 磁标定：新簇全拟（形状+取向），源 imu_20260921_043248.bin（完整球型翻滚）。\n'
                ' *  形状 = Gauss-Newton 最小化 Σ(|A u + C| - 1)^2（合成椭球自测 |y| = 1.00000）;\n'
                ' *  取向 = 重力对齐到 WMM2025 I = 54.330 deg（36.23N 120.44E）;\n'
                ' *  043248 上 |y| 0.957/1.000/1.035、对齐 RMS 0.39 deg、dip 53.89/54.31/54.78、\n'
                ' *  固件算式 rs 1.00/1.11/1.8（VER=110 同数据仅 dip 50.61、rs 221）。\n'
                ' *  注：04:06/04:26 那一簇（040450/042518）与本簇互差 3.7 deg，用户判定新簇更准。\n'
                ' *  详见 docs/mag_observability_math.md 7.10~7.15 */\n') + src
        src, n = re.subn(rb'#define V5F_FW_VER\s+110u', b'#define V5F_FW_VER        111u', src, count=1)
        assert n == 1
        open(TUNE, 'wb').write(src)
        print('patched: VER=111u + 新簇全拟 A/C')

    d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
    ok = True
    for mac in ('V5F_FW_VER', 'V5F_MAG_A_INIT', 'V5F_MAG_C_INIT', 'V5F_EKF_DIP_TAN', 'V5F_CDC_QUAT_ONLY'):
        n = len(re.findall(r'#define\s+%s\b' % mac, d))
        print('LINT %-20s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= n == 1
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_DIP_TAN[^\n]*', r'#define V5F_MAG_C_INIT[^\n]*'):
        print('TUNE |', re.search(pat, d).group()[:100])
    print('TUNE | A ', re.search(r'#define V5F_MAG_A_INIT.*?\}\s*\}', d, re.S).group().replace('\n', ' ')[:120])
    left = B.strays()
    print('源码树残留 %d %s' % (len(left), 'OK' if not left else '!!'))
    ok &= (not left) and bool(re.search(r'#define V5F_FW_VER\s+111u', d))
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v111 V5F\\User\\inc\\v5f_tune.h -Force')
    print('VERIFY', 'OK' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
