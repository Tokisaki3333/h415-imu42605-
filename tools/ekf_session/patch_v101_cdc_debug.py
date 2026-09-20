# -*- coding: utf-8 -*-
"""VER=101: CDC 输出从"验收模式"切到"调试模式"（162 通道诊断帧）+ 抽帧降频。

改动：
  V5F/User/inc/v5f_tune.h
    V5F_FW_VER        100u -> 101u
    V5F_CDC_QUAT_ONLY 1u   -> 0u   (1=验收 20B 四元数 JustFloat; 0=调试 162ch 654B)
    + V5F_CDC_DEBUG_DIV 24u        (8080/24 ≈ 337 Hz ≈ 220 kB/s)
  V5F/User/src/SPI_rx.c
    justfloat_report() 162 通道分支开头插入抽帧（在 clip 统计/组帧之前 return）

幂等：总是先从 .bak_v100（VER=100 的原始态）还原再打补丁。
用法: python tools/ekf_session/patch_v101_cdc_debug.py
"""
import hashlib
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
SPI = os.path.join(ROOT, 'V5F', 'User', 'src', 'SPI_rx.c')
MD5_TUNE_V100 = 'c0d0beda'
MD5_SPI_V100 = '055445ac'
BAKSUF = '.bak_head'          # git show HEAD:<path> 导出的 VER=100 原始态


def g(s):
    return s.encode('gbk')


def restore_v100(path, want_md5):
    bak = path + BAKSUF
    if not os.path.exists(bak):
        print('FAIL: %s missing' % bak)
        return None
    src = open(bak, 'rb').read()
    got = hashlib.md5(src).hexdigest()[:8]
    if got != want_md5:
        print('FAIL: %s md5 %s != expected %s' % (bak, got, want_md5))
        return None
    open(path, 'wb').write(src)
    open(path + '.bak_v101', 'wb').write(src)   # 回退点 = VER=100 原始态
    print('restored %-14s from %s (md5 %s)' % (os.path.basename(path), BAKSUF, got))
    return src


# ---------------- v5f_tune.h ----------------
src = restore_v100(TUNE, MD5_TUNE_V100)
if src is None:
    sys.exit(1)

new = (g('#define V5F_CDC_QUAT_ONLY 0u   /* VER=101 输出模式:\n'
         '                                 *   1u = 验收: CDC(EP2) 只报 EKF 四元数\n'
         '                                 *        (JustFloat: 4 float + 00 00 80 7F, 20B/帧)\n'
         '                                 *   0u = 调试: 旧 162 通道 JustFloat 帧(654B/帧) */\n'
         '#define V5F_CDC_DEBUG_DIV 24u  '
         '/* VER=101 调试模式抽帧: 8080/24 = 337 Hz, 654B*337 = 220 kB/s */\n'))

# 1) QUAT_ONLY 定义 + 其后可能存在的多行注释 -> 整块替换
m = re.search(rb'#define V5F_CDC_QUAT_ONLY[^\n]*\n(?:[^\n]*\n)*?[^\n]*\*/\n', src)
assert m, 'QUAT_ONLY block not found'
src = src[:m.start()] + new + src[m.end():]

# 2) FW_VER -> 101
src, n1 = re.subn(rb'#define V5F_FW_VER\s+100u', b'#define V5F_FW_VER        101u', src, count=1)
assert n1 == 1, 'FW_VER anchor not found'
open(TUNE, 'wb').write(src)
print('patched v5f_tune.h: FW_VER=101u, QUAT_ONLY=0u, DEBUG_DIV=24u')

# ---------------- SPI_rx.c ----------------
spi = restore_v100(SPI, MD5_SPI_V100)
if spi is None:
    sys.exit(1)

anchor = b'    for (i = 0u; i < 4u; i++) ch[c++]     = g_v5f_hold.att.q[i];'
assert spi.count(anchor) == 1, 'SPI anchor x%d' % spi.count(anchor)
ins = (g('    /* ---- VER=101 调试模式抽帧 ----\n'
         '     * 654B/帧 × 8.08kHz = 5.3MB/s，USB FS 与上位机都吃不下；\n'
         '     * 按 V5F_CDC_DEBUG_DIV 抽帧(默认 24 -> 337Hz)，放最前面省 CPU。\n'
         '     * 注意：clip 统计窗口(1000 帧)随之从 125ms 变成 ~3s；\n'
         '     *       报帧的 tick 列(tk - s_rep_last_tick)也变为跨 24 帧的间隔。 */\n')
       + b'#if (V5F_CDC_DEBUG_DIV > 1u)\n'
       + b'    {\n'
       + b'        static uint32_t s_div_n;\n'
       + b'        if (++s_div_n < (uint32_t)V5F_CDC_DEBUG_DIV) return;\n'
       + b'        s_div_n = 0u;\n'
       + b'    }\n'
       + b'#endif\n\n')
spi = spi.replace(anchor, ins + anchor)
open(SPI, 'wb').write(spi)
print('patched SPI_rx.c: +%d bytes' % len(ins))

# ---------------- verify ----------------
ok = True
d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_CDC_QUAT_ONLY[^\n]*',
            r'#define V5F_CDC_DEBUG_DIV[^\n]*'):
    for mm in re.finditer(pat, d):
        print('TUNE |', mm.group()[:96])
s = open(SPI, 'rb').read().decode('gbk', errors='replace')
i0 = s.find('VER=101 调试模式抽帧')
i1 = s.find('for (i = 0u; i < 4u; i++) ch[c++]', i0)
print(s[i0 - 4:i1].rstrip())
ok = ('#define V5F_CDC_DEBUG_DIV 24u' in d and 'V5F_CDC_QUAT_ONLY 0u' in d
      and '#define V5F_FW_VER        101u' in d and '#if (V5F_CDC_DEBUG_DIV > 1u)' in s)
print('VERIFY', 'OK' if ok else 'FAIL')
sys.exit(0 if ok else 1)
