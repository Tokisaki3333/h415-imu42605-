# -*- coding: utf-8 -*-
"""回退我本轮（VER=44..49）对 EKF 的全部改动。

不碰 git：最后提交是 tools/calib，checkout 会连 VER=10..43 的工作一起抹掉。
只用本目录的 .bak 备份，并且**先用宏一致性校验挑出与 VER=43 tune 头匹配的 proc 备份**，
避免版本错配（proc 引用了 tune 里不存在的宏 -> 编译不过）。

回退前先把当前 VER=49 状态另存为 .bak_pre_revert，什么都不丢。
"""
import os
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

MARK = ['s_q_ms', 'V5F_EKF_MAG_SPIKE_DEG', 'V5F_EKF_TILT_AMAG_CAP',
        'if (++n > 64u)', '0x7EC0u', 'V5F_EKF_MAG_BHB_MIN', 'V5F_EKF_DX_MAX_DEG']


def rd(p):
    return open(p, 'rb').read().decode('gbk')


def used_macros(text):
    return set(re.findall(r'\b(V5F_[A-Z][A-Z0-9_]*)\b', text))


import glob

REPO = r'C:\Users\33\Documents\v2\h415-imu42605-'
ALLSRC = [p for p in glob.glob(REPO + r'\**\*.h', recursive=True)
          + glob.glob(REPO + r'\**\*.c', recursive=True)]


def defined_macros(paths):
    d = set()
    for p in paths:
        try:
            for m in re.finditer(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)', rd(p)):
                d.add(m.group(1))
        except Exception:
            pass
    return d


HDRS = [T, U + r'\inc\SPI_rx.h', U + r'\inc\v5f_proc.h']
TUNE43 = U + r'\inc\v5f_tune.h.bak_s2p'
assert os.path.exists(TUNE43), 'VER=43 tune 备份不存在'
tune43 = rd(TUNE43)
print('VER=43 tune 备份: %d 字节  VER=%s' %
      (os.path.getsize(TUNE43), re.search(r'#define V5F_FW_VER\s+(\d+)u', tune43).group(1)))

cands = ['bak_s2n', 'bak_s2l', 'bak_s2k', 'bak_s2h', 'bak_s2g']
print('\n候选 proc 备份与 VER=43 tune 的宏一致性：')
best = None
for c in cands:
    fp = U + r'\src\proc_ekf.c.' + c
    if not os.path.exists(fp):
        print('  %-8s 不存在' % c)
        continue
    txt = rd(fp)
    miss = sorted(used_macros(txt) - defined_macros(ALLSRC))
    has = [m for m in MARK if m in txt]
    print('  %-8s %6d B  工程内未定义宏 %2d 个  我的标记: %s' % (c, os.path.getsize(fp), len(miss), has or '无'))
    if miss:
        print('           缺: %s' % (', '.join(miss[:8])))
    if not miss and not has and best is None:
        best = fp

if best is None:
    raise SystemExit('\n没有找到既无我的标记、又与 VER=43 tune 宏一致的 proc 备份 —— 停止，不覆盖任何文件')

print('\n选定 proc 基线: %s' % os.path.basename(best))

# ---- 保底：当前状态另存 ----
shutil.copy2(P, P + '.bak_pre_revert_v49')
shutil.copy2(T, T + '.bak_pre_revert_v49')
print('当前 VER=49 已另存为 .bak_pre_revert_v49')

# ---- 回退 ----
shutil.copy2(best, P)
shutil.copy2(TUNE43, T)
print('已回退: proc_ekf.c <- %s ; v5f_tune.h <- bak_s2p(VER=43)' % os.path.basename(best))

# ---- 复核 ----
p2, t2 = rd(P), rd(T)
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
left = [m for m in MARK if m in p2]
miss = sorted(used_macros(p2) - defined_macros([P]) -
              set(re.findall(r'#\s*define\s+(V5F_[A-Z][A-Z0-9_]*)', t2)))
CK = [('tune VER==43', ver == 43),
      ('proc 里我的标记全清', not left),
      ('缺宏 0 个', not miss),
      ('wrap_pi 恢复裸 while', 'while (a >  EKF_PI) a -=' in p2),
      ('M6 掩码回 0x7FC0', '0x7FC0u' in p2 and '0x7EC0u' not in p2),
      ('{ } 平衡', p2.count('{') == p2.count('}')),
      ('/* */ 平衡', p2.count('/*') == p2.count('*/'))]
print()
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    if not v and k == '缺宏 0 个':
        print('     缺:', ', '.join(miss[:12]))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    rd(U + r'\src\SPI_rx.c')).group(1))
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
