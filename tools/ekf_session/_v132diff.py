# -*- coding: utf-8 -*-
r"""VER=132 变更核验：与 .bak_v132 逐行 diff（ASCII 过滤）+ 关键字原始计数。"""
import re
import difflib

pairs = [
    (r'bak_src/V5F/User/src/proc_ekf.c.bak_v132', r'V5F/User/src/proc_ekf.c'),
    (r'bak_src/V5F/User/src/SPI_rx.c.bak_v132', r'V5F/User/src/SPI_rx.c'),
    (r'bak_src/V5F/User/inc/v5f_tune.h.bak_v132', r'V5F/User/inc/v5f_tune.h'),
]


def asc(s):
    return ''.join(c if ord(c) < 128 else '.' for c in s)


for A, B in pairs:
    a = open(A, 'rb').read().decode('gbk').split('\n')
    b = open(B, 'rb').read().decode('gbk').split('\n')
    print('===== %s' % B.split('/')[-1])
    na = nd = 0
    for l in difflib.unified_diff(a, b, 'bak', 'now', n=0, lineterm=''):
        if l[:3] in ('---', '+++') or l[:2] == '@@':
            print(asc(l))
        elif l.startswith('+'):
            na += 1
            print('+ ' + asc(l[1:])[:108])
        elif l.startswith('-'):
            nd += 1
            print('- ' + asc(l[1:])[:108])
    print('   +%d / -%d' % (na, nd))
    raw = open(B, 'rb').read().decode('gbk')
    rawa = open(A, 'rb').read().decode('gbk')
    for ch in '{}()':
        if raw.count(ch) != rawa.count(ch):
            print('   %s 计数变化 %d -> %d' % (ch, rawa.count(ch), raw.count(ch)))
    print()
ekf = open(r'V5F/User/src/proc_ekf.c', 'rb').read().decode('gbk')
spi = open(r'V5F/User/src/SPI_rx.c', 'rb').read().decode('gbk')
tune = open(r'V5F/User/inc/v5f_tune.h', 'rb').read().decode('gbk')
print('关键计数：')
for k, s in (('EKF_N 17', '#define EKF_N        17u'), ('EKF_NX 18', '#define EKF_NX       18u'),
             ('IX_MB', '#define IX_MB       17u'), ('EI_MB', '#define EI_MB       16u'),
             ('H mb=1', 's_H[0][EI_MB] = 1.0f;'), ('H mb=yaw', 's_H[0][EI_MB] = s_H[0][EI_Q + 2u];'),
             ('inject mb', 's_x[IX_MB] += dx[EI_MB];'), ('reset mb', 's_x[IX_MB] = 0.0f;'),
             ('P0 mb', 's_P[EI_MB][EI_MB]'), ('Q mb', 'V5F_EKF_BM_Q_RADS2 * dtq'),
             ('white', 'sig_h = V5F_EKF_MAG_WHITE_DEG;'), ('pub', 'g_v5f_ekf_mb_deg = s_x[IX_MB]')):
    print('  %-10s x%d' % (k, ekf.count(s)))
print('  qbuf[28]   x%d | enqueue 28 x%d | bm memcpy x%d | extern x%d'
      % (spi.count('static uint8_t qbuf[28];'), spi.count('hid_up_enqueue(qbuf, 28u);'),
         spi.count('memcpy(qbuf + 20u, &bm, 4u);'),
         spi.count('extern volatile float g_v5f_ekf_mb_deg;')))
print('  VER 132    x%d' % tune.count('#define V5F_FW_VER        132u'))
