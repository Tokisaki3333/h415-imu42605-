# -*- coding: utf-8 -*-
"""VER=115: 抬磁的 R（0.9->1.5 deg）并压低"非偏航"(tilt 通道)权重：新增独立的 SIG_TILT_DEG=6.0。
   护栏：tilt 的 sigma 永不小于偏航的 sigma；上限 30 deg（防 conf 极小时无限放大）。
   同时 s_mag_rs 改为纯 (1/conf)^2（语义 = tilt 权重的膨胀倍数）。

改动文件：V5F/User/inc/v5f_tune.h（常量 + VER）、V5F/User/src/proc_ekf.c（RRv[3] 用新常量 + 护栏）
用法: python tools/ekf_session/patch_v115_magR.py
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v115'
SIG_YAW, SIG_TILT, TILT_MAX = 1.5, 6.0, 30.0


def g(s):
    return s.encode('gbk', errors='replace')


def main():
    src = open(TUNE, 'rb').read()
    d0 = src.decode('gbk', errors='replace')
    if re.search(r'#define V5F_FW_VER\s+115u', d0):
        print('已是 VER=115（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+114u', d0), 'FW_VER 不是 114u'
        assert re.search(r'#define V5F_EKF_MAG_VEC_SIG_DEG\s+0\.9f', d0), 'SIG_DEG 不是 0.9f'
        B.save(TUNE, TAG)
        # 1) 抬 R
        src, n1 = re.subn(rb'#define V5F_EKF_MAG_VEC_SIG_DEG\s+0\.9f',
                          b'#define V5F_EKF_MAG_VEC_SIG_DEG    1.5f', src, count=1)
        assert n1 == 1
        # 2) 新增 tilt 通道独立 sigma（放在 SIG_DEG 定义之后）
        m = re.search(rb'#define V5F_EKF_MAG_VEC_SIG_DEG[^\n]*\n', src)
        src = src[:m.end()] + g(
            '/* VER=115 磁"非偏航"(tilt)通道的独立 sigma：\n'
            ' *  抬磁的 R(SIG_DEG 0.9->1.5) 的同时，把 tilt 通道压低(SIG_TILT=6.0,\n'
            ' *  即比偏航弱 ~16 倍)，避免重力通道失效时由磁独自决定倾斜 -> 大残差。\n'
            ' *  护栏：tilt 的 sigma 恒 >= 偏航的 sigma，且上限 TILT_SIG_MAX。 */\n'
            '#define V5F_EKF_MAG_VEC_SIG_TILT_DEG   6.0f\n'
            '#define V5F_EKF_MAG_VEC_TILT_SIG_MAX_DEG  %.1ff\n' % TILT_MAX) + src[m.end():]
        # 3) VER
        src, n3 = re.subn(rb'#define V5F_FW_VER\s+114u', b'#define V5F_FW_VER        115u', src, count=1)
        assert n3 == 1
        open(TUNE, 'wb').write(src)
        print('patched v5f_tune.h: SIG_DEG=%.1f  SIG_TILT=%.1f (max %.1f)  VER=115u'
              % (SIG_YAW, SIG_TILT, TILT_MAX))

        # 4) proc_ekf.c：RRv[3] 用新常量 + 护栏
        e = open(EKF, 'rb').read()
        # 只改单行锚点：多行锚点会被 CRLF 坑（本文件是 CRLF）
        old2 = b'            float st = V5F_EKF_MAG_VEC_SIG_DEG / conf;'
        assert e.count(old2) == 1, 'st 锚点 x%d' % e.count(old2)
        e = e.replace(old2, b'            float st = V5F_EKF_MAG_VEC_SIG_TILT_DEG / conf;\n'
                            b'            if (st < V5F_EKF_MAG_VEC_SIG_DEG) st = V5F_EKF_MAG_VEC_SIG_DEG;\n'
                            b'            if (st > V5F_EKF_MAG_VEC_TILT_SIG_MAX_DEG) st = V5F_EKF_MAG_VEC_TILT_SIG_MAX_DEG;')
        old3 = b'            s_mag_rs = (st / V5F_EKF_MAG_VEC_SIG_DEG) * (st / V5F_EKF_MAG_VEC_SIG_DEG);'
        assert e.count(old3) == 1, 's_mag_rs 锚点 x%d' % e.count(old3)
        e = e.replace(old3, b'            s_mag_rs = (1.0f / conf) * (1.0f / conf);')
        open(EKF, 'wb').write(e)
        B.save(EKF, TAG)
        print('patched proc_ekf.c: RRv[3] <- SIG_TILT/conf + 护栏;  s_mag_rs <- (1/conf)^2')

    # ---------------- verify ----------------
    ok = True
    d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
    for mac in ('V5F_FW_VER', 'V5F_MAG_A_INIT', 'V5F_MAG_C_INIT', 'V5F_EKF_DIP_TAN',
                'V5F_EKF_MAG_VEC_SIG_DEG', 'V5F_EKF_MAG_VEC_SIG_TILT_DEG',
                'V5F_EKF_MAG_VEC_TILT_SIG_MAX_DEG', 'V5F_CDC_QUAT_ONLY'):
        n = len(re.findall(r'#define\s+%s\b' % mac, d))
        print('LINT %-32s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= n == 1
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_MAG_VEC_SIG_DEG[^\n]*',
                r'#define V5F_EKF_MAG_VEC_SIG_TILT_DEG[^\n]*',
                r'#define V5F_EKF_MAG_VEC_TILT_SIG_MAX_DEG[^\n]*'):
        print('TUNE |', re.search(pat, d).group()[:100])
    s = open(EKF, 'rb').read().decode('gbk', errors='replace')
    i = s.find('sg2 = V5F_EKF_MAG_VEC_SIG_DEG')
    print('EKF |', ' | '.join(s[i:i + 520].split('\n')[:9]))
    ok &= bool(re.search(r'#define V5F_FW_VER\s+115u', d)) and 'SIG_TILT_DEG / conf' in s
    left = B.strays()
    print('源码树残留 %d %s' % (len(left), 'OK' if not left else '!!'))
    ok &= not left
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v115 V5F\\User\\inc\\v5f_tune.h -Force')
    print('       再 Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v115 V5F\\User\\src\\proc_ekf.c -Force')
    print('VERIFY', 'OK' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
