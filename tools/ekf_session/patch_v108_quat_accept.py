# -*- coding: utf-8 -*-
"""VER=108: CDC 上报切回**验收模式** —— 只报 EKF 四元数（JustFloat 20 B/帧）。

改动（只碰 v5f_tune.h 两行 + 注释，SPI_rx.c 一字不动）：
  V5F/User/inc/v5f_tune.h
    V5F_FW_VER        107u -> 108u
    V5F_CDC_QUAT_ONLY 0u   -> 1u     (1=验收: 4 x float32 EKF 四元数 + 00 00 80 7F = 20 B)
    V5F_CDC_DEBUG_DIV 24u        保持不变（验收模式下不生效；切回调试模式时用）
  V5F/User/src/SPI_rx.c
    **不改**。验收分支（`#if (V5F_CDC_QUAT_ONLY != 0u)`，就在 justfloat_report() 开头）
    本来就发 `g_v5f_hold.ekf.q[0..3]` + `00 00 80 7F` 共 20 B，并在调试抽帧之前 return。
    本脚本额外做**结构自检**，确认它没被 VER=101 的调试改动破坏。

回退点：v5f_tune.h.bak_v108 = 打补丁前的 VER=107 调试模式内容。
幂等：已是 VER=108/QUAT_ONLY=1u 则什么都不做。

用法: python tools/ekf_session/patch_v108_quat_accept.py
"""
import hashlib
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
SPI = os.path.join(ROOT, 'V5F', 'User', 'src', 'SPI_rx.c')
NCH = 162
FLAGS = 0x07
BAK = TUNE + '.bak_v108'


def g(s):
    return s.encode('gbk')


src = open(TUNE, 'rb').read()
d0 = src.decode('gbk', errors='replace')

# ---------------- 幂等检查 ----------------
if re.search(r'#define V5F_FW_VER\s+108u', d0) and re.search(r'#define V5F_CDC_QUAT_ONLY 1u', d0):
    print('已处于 VER=108 验收模式（QUAT_ONLY=1u），无需改动')
else:
    assert re.search(r'#define V5F_FW_VER\s+107u', d0), 'FW_VER 不是 107u，先确认基线'
    assert re.search(r'#define V5F_CDC_QUAT_ONLY 0u', d0), 'QUAT_ONLY 不是 0u，先确认基线'

    # 回退点
    if not os.path.exists(BAK):
        open(BAK, 'wb').write(src)
        print('backup  %s  (md5 %s)' % (os.path.basename(BAK), hashlib.md5(src).hexdigest()[:8]))
    else:
        print('backup  %s 已存在，保留不覆盖' % os.path.basename(BAK))

    # 1) 输出模式块：QUAT_ONLY 0u -> 1u，注释改成 VER=108
    new = (g('#define V5F_CDC_QUAT_ONLY 1u   /* VER=108 输出模式（验收）:\n'
             '                                 *   1u = 验收: CDC(EP2) 只报 **EKF 四元数**\n'
             '                                 *        (JustFloat: 4 x float32 + 00 00 80 7F = 20 B/帧)\n'
             '                                 *        * 该分支在 justfloat_report() 开头就 return，\n'
             '                                 *          不受 V5F_CDC_DEBUG_DIV 抽帧，速率 = IMU 中断率(~8 kHz)\n'
             '                                 *   0u = 调试: 旧 162 通道 JustFloat 帧(654B/帧, 抽帧后 337 Hz) */\n'))
    m = re.search(rb'#define V5F_CDC_QUAT_ONLY[^\n]*\n(?:[^\n]*\n)*?[^\n]*\*/\n', src)
    assert m, 'QUAT_ONLY 块未找到'
    src = src[:m.start()] + new + src[m.end():]
    # 1b) DEBUG_DIV：已存在则原样保留（VER=101 起就在），不存在才补 —— 避免重复定义
    if b'#define V5F_CDC_DEBUG_DIV' in src:
        print('DEBUG_DIV 已存在，保持原样（不重复插入）')
    else:
        src = src.replace(new, new + g('#define V5F_CDC_DEBUG_DIV 24u  '
                                      '/* VER=101 调试模式抽帧: 8080/24 = 337 Hz */\n'), 1)
        print('DEBUG_DIV 不存在，已补 24u')

    # 2) 版本号
    src, n1 = re.subn(rb'#define V5F_FW_VER\s+107u', b'#define V5F_FW_VER        108u', src, count=1)
    assert n1 == 1, 'FW_VER 锚点未找到'
    open(TUNE, 'wb').write(src)
    print('patched v5f_tune.h: FW_VER=108u, QUAT_ONLY=1u, DEBUG_DIV=24u(不变)')

# ---------------- 校验：v5f_tune.h ----------------
d = open(TUNE, 'rb').read().decode('gbk', errors='replace')
ok = True
for pat in (r'#define V5F_FW_VER[^\n]*', r'#define V5F_CDC_QUAT_ONLY[^\n]*',
            r'#define V5F_CDC_DEBUG_DIV[^\n]*'):
    for mm in re.finditer(pat, d):
        print('TUNE |', mm.group()[:110])
ok &= bool(re.search(r'#define V5F_FW_VER\s+108u', d))
ok &= bool(re.search(r'#define V5F_CDC_QUAT_ONLY 1u', d))
ok &= bool(re.search(r'#define V5F_CDC_DEBUG_DIV 24u', d))
# 定义唯一性 lint（VER=107 里 DIV 已存在，插重复会被这里抓住）
for mac in ('V5F_FW_VER', 'V5F_CDC_QUAT_ONLY', 'V5F_CDC_DEBUG_DIV'):
    n = len(re.findall(r'#define\s+%s\b' % mac, d))
    print('LINT #define %-20s x%d %s' % (mac, n, 'OK' if n == 1 else '!! 重复/缺失'))
    ok &= (n == 1)

# ---------------- 校验：SPI_rx.c 验收分支结构（不改文件） ----------------
s = open(SPI, 'rb').read().decode('gbk', errors='replace')
iq = s.find('#if (V5F_CDC_QUAT_ONLY != 0u)')
ien = s.find('#endif', iq)
idiv = s.find('#if (V5F_CDC_DEBUG_DIV > 1u)')
seg = s[iq:ien] if iq >= 0 else ''
checks = [
    ('验收分支存在', iq >= 0),
    ('发 EKF 四元数 q[0..3]', all(('ekf.q[%d]' % i) in seg for i in range(4))),
    ('20 B 帧尾 00 00 80 7F', 'qbuf[18] = 0x80u' in seg and 'qbuf[19] = 0x7Fu' in seg),
    ('按 20 B 入 CDC', 'hid_up_enqueue(qbuf, 20u)' in seg),
    ('分支内 return（不受调试抽帧）', 'return;' in seg),
    ('调试抽帧在验收分支之后', 0 <= iq < idiv),
    ('调试 162 通道组帧保留', 'for (i = 0u; i < 4u; i++) ch[c++]     = g_v5f_hold.att.q[i];' in s),
]
print('\nSPI_rx.c 结构自检（只读）:')
for name, c in checks:
    print('  [%s] %s' % ('OK' if c else '!!', name))
    ok &= c
if iq >= 0:
    print('\n验收分支原文:')
    print(seg.rstrip())

tag = (108 << 16) | (NCH << 8) | FLAGS
print('\nfw_tag(调试帧第 76 列) 将由 7053831 变为 %d = (108<<16)|(162<<8)|7' % tag)
print('注意: 验收模式的 20 B 流里没有 fw_tag/校验和，`rec.py`/`cols_162` 抓不到 A5 5A 帧属正常。')
print('VERIFY', 'OK' if ok else 'FAIL')
sys.exit(0 if ok else 1)
