# -*- coding: utf-8 -*-
"""回退 —— 第三步：用 diff 判定 .bak_s2n 是否就是"本会话之前"的 proc_ekf.c。

方法：把 .bak_s2n 与当前文件逐行 diff，统计每个差异块的上下文，
      并要求**每个差异块都命中我已知的改动标记**。全部命中 => .bak_s2n 就是原文件。
（上一版的失败原因是判据里的符号名猜错了，不是备份有问题。）
"""
import difflib
import os
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'
CAND = U + r'\src\proc_ekf.c.bak_s2n'
TUNE43 = U + r'\inc\v5f_tune.h.bak_s2p'

# 我本轮改动的"特征串"（出现在每个差异块里）
MARK = ['s_q_ms', 's_ist_last', 'V5F_EKF_MAG_SPIKE_DEG', 's_mag_r_prev', 's_mag_r_seed',
        's_mag_spike', 'V5F_EKF_TILT_AMAG_CAP', 's_am_err', 'V5F_EKF_TILT_AMAG_TOL',
        'level_dps', 'V5F_EKF_TILT_LEV_DPS',
        'if (++n > 64u)', 'while (a >  EKF_PI)', 'while (a < -EKF_PI)',
        '0x7EC0u', '0x7FC0u',
        'V5F_EKF_MAG_BHB_MIN', 'V5F_EKF_MAG_FERR', 'fhb2', 's_mag_dp',
        'V5F_EKF_MAG_R_MAX_DEG', 'q_rot_vec(dq, mf, tmp)', 's_mag_dth', 'exp', 'Exp',
        'V5F_EKF_MAG_SIG_RAD', 'V5F_EKF_DX_MAX_DEG']


def rd(p):
    return open(p, 'rb').read().decode('gbk')


cur = rd(P).split('\n')
old = rd(CAND).split('\n')
sm = difflib.SequenceMatcher(None, old, cur, autojunk=False)
blocks = [b for b in sm.get_opcodes() if b[0] != 'equal']
print('候选 %s: %d 行   当前: %d 行   差异块 %d 个' % (os.path.basename(CAND), len(old), len(cur), len(blocks)))

bad = []
for tag, i1, i2, j1, j2 in blocks:
    txt = '\n'.join(old[i1:i2]) + '\n@@@\n' + '\n'.join(cur[j1:j2])
    hit = [m for m in MARK if m in txt]
    kind = '我的改动' if hit else '★ 非我改动 ★'
    if not hit:
        bad.append((i1, j1, txt))
    print('  %-8s old[%d:%d] new[%d:%d] %-14s %s'
          % (tag, i1, i2, j1, j2, kind, (','.join(sorted(set(hit))[:5]) if hit else '')[:90]))

print()
if bad:
    print('!! 存在 %d 个与我的改动无关的差异块 —— .bak_s2n 不是本会话之前的版本，停止。' % len(bad))
    for i1, j1, txt in bad[:3]:
        print('---- 样本 old 行 %d / new 行 %d ----' % (i1 + 1, j1 + 1))
        print('\n'.join(txt.split('\n')[:24]))
    raise SystemExit(1)

print('==> 所有差异块都命中我的改动标记：.bak_s2n 就是本会话之前的 proc_ekf.c。开始回退。')

# 当前状态另存
shutil.copy2(P, P + '.bak_pre_revert_v49')
shutil.copy2(T, T + '.bak_pre_revert_v49')
shutil.copy2(CAND, P)
shutil.copy2(TUNE43, T)
print('已回退: proc_ekf.c <- bak_s2n ; v5f_tune.h <- bak_s2p')

p2, t2 = rd(P), rd(T)
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
left = [m for m in ['s_q_ms', 'V5F_EKF_MAG_SPIKE_DEG', 'V5F_EKF_TILT_AMAG_CAP',
                    'if (++n > 64u)', '0x7EC0u', 'V5F_EKF_MAG_BHB_MIN',
                    'V5F_EKF_DX_MAX_DEG', 's_am_err', 's_mag_dp'] if m in p2]
CK = [('tune VER==43', ver == 43),
      ('我的标记全清', not left),
      ('wrap_pi 裸 while 已回', 'while (a >  EKF_PI) a -=' in p2),
      ('去陈旧块已回', 'q_rot_vec(dq, mf, tmp)' in p2 and 's_mag_dth' in p2),
      ('M6 掩码回 0x7FC0', '0x7FC0u' in p2 and '0x7EC0u' not in p2),
      ('M7 掩码 0x0100', '0x0100u' in p2),
      ('{ } 平衡', p2.count('{') == p2.count('}')),
      ('/* */ 平衡', p2.count('/*') == p2.count('*/'))]
print()
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', rd(U + r'\src\SPI_rx.c')).group(1))
print('\nfw_tag = %d  (VER=%d, %d ch)  <- 回到本会话之前的版本' % ((ver << 16) | (nch << 8) | 7, ver, nch))
if left:
    print('残留标记:', left)
