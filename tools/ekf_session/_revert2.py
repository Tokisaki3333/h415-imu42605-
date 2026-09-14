# -*- coding: utf-8 -*-
"""回退本轮（VER=44..49）对 EKF 的改动 —— 第二步：确认候选备份含有 VER=43 的 5 列诊断。

VER=42->43 那次的上报改动 = 末尾新增 ekf_mag_gate/mag_bh/mag_r_deg/mag_used/p_yy
（117~121 列）。若候选备份里没有 h->ekf.p_yy / s_mag_gate / s_mag_used 的发布，
说明它是 VER=42 之前的版本，回退会丢诊断列，必须换一个候选。
"""
import glob
import os
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'
REPO = r'C:\Users\33\Documents\v2\h415-imu42605-'
TUNE43 = U + r'\inc\v5f_tune.h.bak_s2p'

MARK = ['s_q_ms', 'V5F_EKF_MAG_SPIKE_DEG', 'V5F_EKF_TILT_AMAG_CAP',
        'if (++n > 64u)', '0x7EC0u', 'V5F_EKF_MAG_BHB_MIN', 'V5F_EKF_DX_MAX_DEG']
DIAG = ['h->ekf.p_yy', 's_mag_gate', 's_mag_used', 's_mag_bh', 's_mag_r']


def rd(p, enc='gbk'):
    return open(p, 'rb').read().decode(enc)


def strip_comments(t):
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', t)


def used(t):
    return set(re.findall(r'\b(V5F_[A-Z][A-Z0-9_]*)\b', strip_comments(t)))


def defined(paths):
    d = set()
    for p in paths:
        try:
            d |= set(re.findall(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)', rd(p, 'gbk')))
        except Exception:
            try:
                d |= set(re.findall(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)',
                                    open(p, 'rb').read().decode('utf-8', 'ignore')))
            except Exception:
                pass
    return d


# 除当前 v5f_tune.h 外的全部头文件 + VER=43 的 tune
HDRS = [p for p in glob.glob(REPO + r'\**\*.h', recursive=True) if p != T]
DEFS43 = defined(HDRS) | set(re.findall(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)', rd(TUNE43)))

print('候选评估（要求：无我的标记 + 含 5 列诊断 + 宏在 VER=43 头文件集合内可解）')
best = None
for c in ['bak_s2n', 'bak_s2l', 'bak_s2k', 'bak_s2h', 'bak_s2g', 'bak_s2f']:
    fp = U + r'\src\proc_ekf.c.' + c
    if not os.path.exists(fp):
        continue
    txt = rd(fp)
    has = [m for m in MARK if m in txt]
    dg = [d for d in DIAG if d in txt]
    miss = sorted(used(txt) - DEFS43)
    ok = (not has) and len(dg) == len(DIAG) and not miss
    print('  %-8s %6dB  标记%s  诊断 %d/5  缺宏 %d  %s'
          % (c, os.path.getsize(fp), has or '无', len(dg), len(miss), '<= 选定' if ok else ''))
    if miss:
        print('           缺:', ', '.join(miss[:6]))
    if ok and best is None:
        best = fp

assert best, '没有同时满足【无我的标记 + 含5列诊断 + 宏一致】的备份 —— 停止，不覆盖任何文件'
print('\n选定: %s' % os.path.basename(best))

shutil.copy2(P, P + '.bak_pre_revert_v49')
shutil.copy2(T, T + '.bak_pre_revert_v49')
shutil.copy2(best, P)
shutil.copy2(TUNE43, T)
print('已回退: proc_ekf.c <- %s ; v5f_tune.h <- bak_s2p (VER=43)' % os.path.basename(best))

p2, t2 = rd(P), rd(T)
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
left = [m for m in MARK if m in p2]
miss = sorted(used(p2) - (defined(HDRS) |
                          set(re.findall(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)', t2)) |
                          set(re.findall(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)', p2))))
dgl = [d for d in DIAG if d in p2]
CK = [('tune VER==43', ver == 43),
      ('我的标记全清', not left),
      ('5 列诊断仍在', len(dgl) == len(DIAG)),
      ('缺宏 0', not miss),
      ('wrap_pi 裸 while 已回', 'while (a >  EKF_PI) a -=' in p2),
      ('M6 掩码回 0x7FC0', '0x7FC0u' in p2 and '0x7EC0u' not in p2),
      ('{ } 平衡', p2.count('{') == p2.count('}')),
      ('/* */ 平衡', p2.count('/*') == p2.count('*/')),
      ('括号差 -4', p2.count('(') - p2.count(')') == -4)]
print()
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    if not v and k == '缺宏 0':
        print('     ', ', '.join(miss[:10]))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', rd(U + r'\src\SPI_rx.c')).group(1))
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
