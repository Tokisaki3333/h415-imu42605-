# -*- coding: utf-8 -*-
"""VER=110: 磁标定只修**取向**（形状沿用台面 A/C）—— 由低干扰录像 040450 的重力对齐给出。

为什么只修取向：这条录像里器件经过手/桌附近，**场强**被局部扰动（|B| 变动大），
椭球拟合会退化（实测 `|y|` 0.33~2.6、硬铁等效偏移 −181 LSB 超量程）；
而"磁场与重力的夹角（dip）"只用**方向**，鲁棒。台面 A/C 的形状是在场强稳定的数据上拟合的，
实测 `|y|` p1~p99 = 0.927~1.053（±5%）✓。所以：A_new = U·A_old，C_new = U·C_old，
U 用重力对齐到 WMM2025 倾角 I = 54.330°（36.23N 120.44E）。

验证（040450 逐帧，复刻固件 ddip/conf/mag_rs 算式）：
  改前(台面 A/C + DIP=1.70): dip_m 48.39/53.17/64.05  ddip 9.26/11.24  conf 0.02   rs 1.4/**2500**/2500
  本次(旋转修正 + DIP=1.3932): dip_m 54.03/54.39/62.27  ddip 0.21/7.94  conf 0.96  rs 1.0/**1.1**/2500
  （p90 的 2500 = 真正被扰动的帧，conf 逻辑本就该把它们压掉；改前是**中位数**就恒为 2500）
  对齐 RMS 0.27°（6496 帧，roll 360°/pitch 177°）；U = 绕 body z 26.9° + 水平 5.4°。

 U 含 26.9° 绕竖直分量 ⇒ **航向整体平移 ~27°**：上机后务必做"指北读数"复测
（改前是偏 15°；若这版接近 0，说明取向修对了；若反而更远，说明 U 的方位分量需要旋转回来）。

备份：bak_src/V5F/User/inc/v5f_tune.h.bak_v110
用法: python tools/ekf_session/patch_v110_magrot.py
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
TAG = '.bak_v110'

A_NEW = [[-2.64148796e-03, -5.49557599e-03, -4.55681111e-04],
         [-5.37283275e-03, +2.69732215e-03, -3.96620199e-04],
         [-1.57754855e-04, +6.19071340e-04, +6.02307612e-03]]
C_NEW = [+1.67202500e-02, +1.75671478e-03, +1.05098948e-03]


def g(s):
    # 注释里可能有 GBK 编不出的符号（=> / 警告 等）：replace 掉，绝不让它中断补丁
    return s.encode('gbk', errors='replace')


def main():
    src = open(TUNE, 'rb').read()
    d0 = src.decode('gbk', errors='replace')
    if re.search(r'#define V5F_FW_VER\s+110u', d0):
        print('已是 VER=110（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+109u', d0), 'FW_VER 不是 109u'
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
        src = g('/* VER=110 磁标定=只修取向：A_new = U·A_old（形状沿用 VER=102 台面标定），\n'
                ' *  U 由低干扰录像 imu_20260921_040450.bin 的**重力对齐**给出（WMM2025 I=54.330° @36.23N 120.44E），\n'
                ' *  对齐 RMS 0.27°；只修取向的原因：该录像场强被局部扰动（椭球会退化，|y| 0.33~2.6），\n'
                ' *  而 dip 只用方向、鲁棒；台面 A/C 的形状实测 |y| p1~p99 = 0.927~1.053（±5%）。\n'
                ' *  验证：dip_m 54.03/54.39/62.27（p10/50/90），ddip 0.21/7.94，rs 1.0/1.1/2500\n'
                ' *  （改前 dip 48.39/53.17/64.05、rs 中位恒 2500）。 U 含 26.9° 绕竖直 ⇒ 航向平移 ~27°，需指北复测。\n'
                ' *  详见 docs/mag_observability_math.md 7.10~7.14 */\n') + src
        src, n = re.subn(rb'#define V5F_FW_VER\s+109u', b'#define V5F_FW_VER        110u', src, count=1)
        assert n == 1
        open(TUNE, 'wb').write(src)
        print('patched: VER=110u, 旋转修正后的 A/C')

    d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
    ok = True
    for mac in ('V5F_FW_VER', 'V5F_MAG_A_INIT', 'V5F_MAG_C_INIT', 'V5F_EKF_DIP_TAN', 'V5F_CDC_QUAT_ONLY'):
        n = len(re.findall(r'#define\s+%s\b' % mac, d))
        print('LINT %-20s x%d %s' % (mac, n, 'OK' if n == 1 else '!!'))
        ok &= n == 1
    for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_EKF_DIP_TAN[^\n]*', r'#define V5F_MAG_C_INIT[^\n]*'):
        print('TUNE |', re.search(pat, d).group()[:100])
    ok &= bool(re.search(r'#define V5F_FW_VER\s+110u', d))
    left = B.strays()
    print('源码树残留 %d %s' % (len(left), 'OK' if not left else '!!'))
    ok &= not left
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v110 V5F\\User\\inc\\v5f_tune.h -Force')
    print('VERIFY', 'OK' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
