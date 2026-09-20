# -*- coding: utf-8 -*-
"""VER=109: 换掉台面污染的磁标定（椭球+重力对齐，来自低干扰录像 040450），并把 dip 常量改成真值。

改动（只动 v5f_tune.h）：
  V5F_MAG_A_INIT / V5F_MAG_C_INIT  <- 椭球(|A raw + C|=const) + 重力对齐(WMM I=54.330°) 的拟合结果
  V5F_EKF_DIP_TAN  1.70f -> 1.3932f (= tan(54.330°)，青岛 WMM2025)
  V5F_FW_VER       108u  -> 109u
  V5F_CDC_QUAT_ONLY 保持不变（要 162 通道调试流就保持 0u；切验收四元数流改 1u）

依据（docs/mag_observability_math.md §7.10~§7.13）：
  台面标定在干净点位方向残差差 8 倍（0.48° -> 4.12°）；实测倾角
  原始 23.86° / 现用 A/C 12.34° / 本次 3.59°（展宽 p10~p90），ΔI(p50) = -0.17°，原始 |B| -1.2%。
  换完后 conf(ddip) 应回到 ~1、mag_rs -> 1，静止那个 10.6° 的 tilt 残差才会归零。

备份：bak_src/V5F/User/inc/v5f_tune.h.bak_v109（绝不写进源码目录）
用法: python tools/ekf_session/patch_v109_magcal2.py
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B          # 备份唯一合法去处：<repo>/bak_src/...

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
TAG = '.bak_v109'

A_NEW = [[+2.87508917e-04, +7.28617565e-04, +1.01406737e-03],
         [-7.09199398e-04, +8.22262668e-04, -3.47199115e-04],
         [-8.03753696e-04, -4.44306852e-04, +5.65119343e-04]]
C_NEW = [+5.99541656e-02, +5.96288002e-01, -6.58694019e-01]
DIP_NEW = 1.3932


def g(s):
    return s.encode('gbk')


def main():
    src = open(TUNE, 'rb').read()
    d0 = src.decode('gbk', errors='replace')
    if re.search(r'#define V5F_FW_VER\s+109u', d0):
        print('已是 VER=109，无需改动（只做校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+108u', d0), 'FW_VER 不是 108u，先确认基线'
        assert re.search(r'#define V5F_EKF_DIP_TAN\s+1\.70f', d0), 'DIP_TAN 不是 1.70f'
        B.save(TUNE, TAG)

        # 1) 磁标定 A（多行 + 反斜杠续行）
        am = re.search(rb'#define V5F_MAG_A_INIT\s*\{.*?\}\s*\}', src, re.S)
        assert am, 'A_INIT 块未找到'
        a_new = g('#define V5F_MAG_A_INIT  { { %+.8ef, %+.8ef, %+.8ef }, \\\n'
                  '                          { %+.8ef, %+.8ef, %+.8ef }, \\\n'
                  '                          { %+.8ef, %+.8ef, %+.8ef } }'
                  % (A_NEW[0][0], A_NEW[0][1], A_NEW[0][2],
                     A_NEW[1][0], A_NEW[1][1], A_NEW[1][2],
                     A_NEW[2][0], A_NEW[2][1], A_NEW[2][2]))
        src = src[:am.start()] + a_new + src[am.end():]

        # 2) 磁标定 C
        cm = re.search(rb'#define V5F_MAG_C_INIT\s*\{[^}]*\}', src)
        assert cm, 'C_INIT 块未找到'
        c_new = g('#define V5F_MAG_C_INIT  { %+.8ef, %+.8ef, %+.8ef }'
                  % (C_NEW[0], C_NEW[1], C_NEW[2]))
        src = src[:cm.start()] + c_new + src[cm.end():]

        # 3) dip 常量（顺带把注释里的实测来源写清楚）
        src, n3 = re.subn(rb'#define V5F_EKF_DIP_TAN\s+[\d.]+f',
                          ('#define V5F_EKF_DIP_TAN          %gf' % DIP_NEW).encode('ascii'), src, count=1)
        assert n3 == 1, 'DIP_TAN 锚点未找到'
        prov = g('/* VER=109 磁标定 + 磁倾角真值：\n'
                 ' *  A/C 来自低干扰点位录像 imu_20260921_040450.bin（全球面翻滚，\n'
                 ' *  椭球 |A raw + C| = const + 重力对齐到 WMM2025 倾角 I = 54.330 deg @36.23N 120.44E）；\n'
                 ' *  倾角展宽 p10~p90：原始 23.86 -> 现用(台面) 12.34 -> 本版 3.59 deg，ΔI(p50) = -0.17 deg；\n'
                 ' *  原始 |B| = 51.6 uT（WMM 52.2，-1.2%）。旧标定在干净点位方向残差差 8 倍（0.48 -> 4.12 deg）。\n'
                 ' *  详见 docs/mag_observability_math.md 7.10~7.13 */\n')
        src = prov + src

        # 4) 版本号
        src, n4 = re.subn(rb'#define V5F_FW_VER\s+108u', b'#define V5F_FW_VER        109u', src, count=1)
        assert n4 == 1, 'FW_VER 锚点未找到'
        open(TUNE, 'wb').write(src)
        print('patched v5f_tune.h: VER=109u, 新 A/C, DIP_TAN=%.4f' % DIP_NEW)

    # ---------------- 校验 ----------------
    d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
    ok = True
    for mac in ('V5F_FW_VER', 'V5F_MAG_A_INIT', 'V5F_MAG_C_INIT', 'V5F_EKF_DIP_TAN', 'V5F_CDC_QUAT_ONLY'):
        n = len(re.findall(r'#define\s+%s\b' % mac, d))
        print('LINT #define %-20s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= (n == 1)
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_DIP_TAN[^\n]*',
                r'#define V5F_MAG_C_INIT[^\n]*', r'#define V5F_CDC_QUAT_ONLY[^\n]*'):
        m = re.search(pat, d)
        print('TUNE |', m.group()[:110] if m else '**MISSING**')
    am2 = re.search(r'#define V5F_MAG_A_INIT.*?\}\s*\}', d, re.S)
    if am2:
        print('TUNE |', am2.group().replace('\n', ' ')[:150])
    ok &= bool(re.search(r'#define V5F_FW_VER\s+109u', d))
    ok &= bool(re.search(r'#define V5F_EKF_DIP_TAN\s+%gf' % DIP_NEW, d))
    ok &= ('%.8e' % A_NEW[0][0]).replace('e-04', 'e-04') in d.replace('+', '+')
    left = B.strays()
    print('源码树残留变体副本: %d %s' % (len(left), 'OK' if not left else '!!'))
    ok &= not left
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v109 V5F\\User\\inc\\v5f_tune.h -Force')
    print('VERIFY', 'OK' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
