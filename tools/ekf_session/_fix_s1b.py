# -*- coding: utf-8 -*-
"""S1 修复版落地：proc_ekf.c 重写 + 报告缓冲改 static + 栈 2K->8K + VER 10->11。"""
import os
import re
import shutil
import sys

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
SRC = R + r'\V5F\User\src\SPI_rx.c'
TUNE = R + r'\V5F\User\inc\v5f_tune.h'
LD = R + r'\V5F\Ld\Link_v5f.ld'
NEW = R + r'\V5F\User\src\proc_ekf.c'
UTF = r'C:\Users\33\Documents\v2\_proc_ekf_utf8.c'


def sub(t, a, b, name):
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (name, n)
    return t.replace(a, b, 1)


def bak(p, tag):
    shutil.copy2(p, p + '.bak_' + tag)


# ---- 1) proc_ekf.c: UTF-8 -> GBK ----
u = open(UTF, 'rb').read().decode('utf-8')
assert u.count('/*') == u.count('*/'), '注释不配平 %d/%d' % (u.count('/*'), u.count('*/'))
try:
    g = u.encode('gbk')
except UnicodeEncodeError as e:
    print('GBK 编不了:', e)
    sys.exit(1)
bak(NEW, 's1b')
open(NEW, 'wb').write(g)
print('proc_ekf.c  %d 行 / %d B (GBK)' % (u.count('\n') + 1, len(g)))
for ch in ('s_stage', 's_prop_ok', 'ekf_m6_tilt', 'ekf_m1m2_gps', 'static float    s_Pn'):
    assert ch in u, ch

# ---- 2) SPI_rx.c: 报告缓冲改 static（这是 2 KB 栈里最大的一块） ----
t = open(SRC, 'rb').read().decode('gbk')
n0 = len(t)
t = sub(t, '''    float   ch[JF_CH_NUM];
    uint8_t buf[JF_FRAME_LEN];''',
        '''    /* ★ static，不是局部：本函数在 DMA1 中断里，是整条链最深的一层，
     *   而链接脚本给的栈只有 2 KB（Link_v5f.ld 的 __stack_size）—— 光这两个数组
     *   就 902 B，加上处理链（尤其 EKF 的 16x16 中间量）把栈踩穿了。
     *   实测症状：每秒一两次、单帧的 ekf 上报块被清零（|q| -> 0.003、
     *   sigma_yaw -> 185 度、press_avg -> 0），以及 0.013% 的坏帧。
     *   只有这一个中断会调用本函数（单线程），所以 static 安全、且是零代价的。 */
    static float   ch[JF_CH_NUM];
    static uint8_t buf[JF_FRAME_LEN];''', 'static')
assert len(t) > n0 and t.count('/*') == t.count('*/')
bak(SRC, 's1b')
open(SRC, 'wb').write(t.encode('gbk'))
print('SPI_rx.c   报告缓冲 -> static')

# ---- 3) 栈 2 KB -> 8 KB ----
t = open(LD, 'rb').read().decode('utf-8')
n0 = len(t)
a = '__stack_size = 2048;'
assert t.count(a) == 1
t = t.replace(a, '__stack_size = 8192;   /* 原 2048：V5F 处理链 8 级 + EKF 的 16x16 中间量\n                          * 实测把 2 KB 栈踩穿（症状是单帧上报块被清零）。\n                          * RAM 有 256K-256+384K，bss 才 23 KB，8 KB 栈毫无压力。 */', 1)
assert len(t) > n0
bak(LD, 's1b')
open(LD, 'wb').write(t.encode('utf-8'))
print('Link_v5f.ld  __stack_size 2048 -> 8192')

# ---- 4) VER 10 -> 11（刷的是哪版必须能分辨） ----
t = open(TUNE, 'rb').read().decode('gbk')
n0 = len(t)
t = sub(t, '#define V5F_FW_VER        10u', '#define V5F_FW_VER        11u', 'VER')
assert t.count('/*') == t.count('*/')
bak(TUNE, 's1b')
open(TUNE, 'wb').write(t.encode('gbk'))
print('v5f_tune.h VER 10 -> 11')

print()
print('fw_tag 期望 = %d  (VER=11, 112 列, EKF=1, MAGCAL=1, AC=1)'
      % ((11 << 16) | (112 << 8) | 1 | 2 | 4))
