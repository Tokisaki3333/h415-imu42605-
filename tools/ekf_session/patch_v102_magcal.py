# -*- coding: utf-8 -*-
"""VER=102: 把最新一次全球面标定的 A/C 写回 v5f_tune.h（GBK 字节级补丁）。

数据: R:\\imu_20260921_022718.bin (30106 帧 / 90 s, roll ±180°, pitch 105°, 校验和 100%)
拟合: 姿态辅助 9 参数, 参考=旧链 att.q, 软权 1/(1+(w/20dps)^2), median|y|=1
校验: 零空间=1, 半分 ‖A1-A2‖/‖A‖=2.7%, 方向残差 静态 p50 3.53->0.95°, 全场 2.78->0.48°

用法: python tools/ekf_session/patch_v102_magcal.py
回退: bak_src/V5F/User/inc/v5f_tune.h.bak_v102_mag（或 git checkout -- V5F/User/inc/v5f_tune.h）
"""
import hashlib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B          # 备份唯一合法去处：<repo>/bak_src/...

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
BAK = '.bak_v102_mag'

A = [[+7.82513172e-05, -6.10340235e-03, -7.24195460e-05],
     [-5.97676400e-03, -1.84772880e-05, -1.84887488e-05],
     [+3.76624289e-04, +7.79929442e-04, +6.05283507e-03]]
C = [+1.41276398e-02, +9.17333192e-03, -1.31003127e-04]

PROV = (
    '/* VER=102 mag calib (2026-09-21 capture R:\\imu_20260921_022718.bin, 30106 帧 / 90 s):\n'
    ' * attitude-aided 9-param fit, reference = legacy gyro attitude (per-frame),\n'
    ' * soft weight 1/(1+(w/20dps)^2); |y| normalized so median|y| = 1.\n'
    ' * 校验: 秩判据零空间=1, 半分 ‖A1-A2‖/‖A‖=2.7%, 方向残差 静态 p50 3.53->0.95 deg,\n'
    ' *       全场 p50 2.78->0.48 deg (C 的硬铁项近似减半, z 项 -> 0 是主要改善来源) */\n')


def g(s):
    return s.encode('gbk')


def main():
    src = open(TUNE, 'rb').read()
    B.save(TUNE, BAK)                     # 备份写进 bak_src/，绝不落源码目录
    before = hashlib.md5(src).hexdigest()[:8]

    i0 = src.find(b'#define V5F_MAG_A_INIT')
    i1 = src.find(b'#define V5F_MAG_C_INIT')
    if i0 < 0 or i1 < 0:
        print('FAIL: 找不到 V5F_MAG_A_INIT / V5F_MAG_C_INIT')
        return 1
    eol = src.find(b'\n', i1)
    new_block = (
        g(PROV) +
        g('#define V5F_MAG_A_INIT  { { %+.8ef, %+.8ef, %+.8ef }, \\\n' % tuple(A[0])) +
        g('                          { %+.8ef, %+.8ef, %+.8ef }, \\\n' % tuple(A[1])) +
        g('                          { %+.8ef, %+.8ef, %+.8ef } }\n' % tuple(A[2])) +
        g('#define V5F_MAG_C_INIT  { %+.8ef, %+.8ef, %+.8ef }' % tuple(C))
    )
    src = src[:i0] + new_block + src[eol:]

    src, n = re.subn(rb'#define V5F_FW_VER\s+101u', b'#define V5F_FW_VER        102u', src, count=1)
    assert n == 1, 'FW_VER 101u 未找到'

    open(TUNE, 'wb').write(src)
    print('patched v5f_tune.h  md5 %s -> %s (backup %s)' % (before, hashlib.md5(src).hexdigest()[:8], BAK))

    # 复核：用工具自己的解析器读回
    sys.path.insert(0, os.path.join(ROOT, 'tools', 'calib'))
    import mag360_cal as M
    A2, C2 = M.read_current_AC(TUNE)
    okA = A2 is not None and abs(A2 - __import__('numpy').array(A)).max() < 1e-12
    okC = C2 is not None and abs(C2 - __import__('numpy').array(C)).max() < 1e-12
    txt = open(TUNE, 'rb').read().decode('gbk', errors='replace')
    ver_ok = '#define V5F_FW_VER        102u' in txt
    for l in txt.splitlines():
        if 'V5F_FW_VER' in l or 'V5F_MAG_A_INIT' in l or 'V5F_MAG_C_INIT' in l:
            print('  |', l.strip()[:110])
    print('VERIFY A=%s C=%s VER=%s -> %s' % (okA, okC, ver_ok, 'OK' if (okA and okC and ver_ok) else 'FAIL'))
    return 0 if (okA and okC and ver_ok) else 1


if __name__ == '__main__':
    sys.exit(main())
