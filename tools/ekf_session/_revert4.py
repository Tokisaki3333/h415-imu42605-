# -*- coding: utf-8 -*-
"""回退 —— 最终版：外科式只回退"我本轮改的块"，保留 VER=42->43 的 5 列诊断。

依据：_revert3 的 diff 已经把 17 个差异块干净分类 ——
  11 个块命中我的改动标记（v44 q_s / v45 spike / v46 tiltcap / v47 wrappi /
  v48 m6mask / v49 deadpoint），
  6 个块不含我的标记，内容全是 VER=42->43 的诊断列
  （s_mag_gate/s_mag_used/s_mag_bh/s_mag_r 声明、s_mag_bh 赋值、
   h->ekf.mag_*/p_yy 发布）。
所以：我的块取 .bak_s2n 那一侧（回退），其余块取当前文件那一侧（保留）。
构造上就等于"本会话之前的文件"，不多不少。
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

MARK = ['s_q_ms', 's_ist_last', 'V5F_EKF_MAG_SPIKE_DEG', 's_mag_r_prev', 's_mag_r_seed',
        's_mag_spike', 'V5F_EKF_TILT_AMAG_CAP', 's_am_err', 'V5F_EKF_TILT_AMAG_TOL',
        'level_dps', 'V5F_EKF_TILT_LEV_DPS',
        'if (++n > 64u)', 'while (a >  EKF_PI)', 'while (a < -EKF_PI)',
        '0x7EC0u', '0x7FC0u', 'V5F_EKF_MAG_BHB_MIN', 'V5F_EKF_MAG_FERR', 'fhb2',
        's_mag_dp', 'V5F_EKF_MAG_R_MAX_DEG', 'q_rot_vec(dq, mf, tmp)', 's_mag_dth',
        'V5F_EKF_MAG_SIG_RAD', 'V5F_EKF_DX_MAX_DEG']


def rd(p):
    return open(p, 'rb').read().decode('gbk')


cur_txt = rd(P)
cur = cur_txt.split('\n')
old = rd(CAND).split('\n')
sm = difflib.SequenceMatcher(None, old, cur, autojunk=False)

out, n_rev, n_keep = [], 0, 0
for tag, i1, i2, j1, j2 in sm.get_opcodes():
    if tag == 'equal':
        out.extend(cur[j1:j2])
        continue
    txt = '\n'.join(old[i1:i2]) + '\n@@@\n' + '\n'.join(cur[j1:j2])
    if any(m in txt for m in MARK):
        out.extend(old[i1:i2]); n_rev += 1
    else:
        out.extend(cur[j1:j2]); n_keep += 1

new_txt = '\n'.join(out)
print('回退我的块 %d 个，保留非我块 %d 个' % (n_rev, n_keep))
print('行数: 原 %d -> 回退后 %d   （.bak_s2n %d + 诊断 %d 行）'
      % (len(cur), len(out), len(old), len(out) - len(old)))
assert n_rev == 11 and n_keep == 6, '块分类数目不符（预期 11/6）'

# 结构性自检：回退后里不能再出现"只属于我新增"的标识符
MINE_ONLY = ['s_q_ms', 'V5F_EKF_MAG_SPIKE_DEG', 'V5F_EKF_TILT_AMAG_CAP', 's_am_err',
             'V5F_EKF_MAG_BHB_MIN', 'V5F_EKF_MAG_FERR', 's_mag_dp', 'fhb2',
             's_mag_r_seed', 's_mag_r_prev', 's_mag_spike',
             '0x7EC0u', 'if (++n > 64u)', 'V5F_EKF_DX_MAX_DEG']
# 注：s_ist_last 是我动手之前就有的变量（s_mag_dth 的边沿跟踪），不算我的新增。
left = [m for m in MINE_ONLY if m in new_txt]
assert not left, '回退后仍残留我的标识符: %s' % left
must = [('裸 while 已回', 'while (a >  EKF_PI) a -=' in new_txt and 'while (a < -EKF_PI) a +=' in new_txt),
        ('去陈旧块已回', 'q_rot_vec(dq, mf, tmp)' in new_txt and 's_mag_dth' in new_txt),
        ('M6 掩码 0x7FC0', '0x7FC0u' in new_txt),
        ('M6 无 0x7EC0', '0x7EC0u' not in new_txt),
        ('tilt 门原样', 'h->stat.level_dps < V5F_EKF_TILT_LEV_DPS' in new_txt),
        ('M6 R 原样', 'R[0] = R[4] = R[8] = V5F_EKF_TILT_SIG_RAD * V5F_EKF_TILT_SIG_RAD;' in new_txt),
        ('诊断列保留', all(k in new_txt for k in
                       ['s_mag_gate', 's_mag_used', 's_mag_bh', 'h->ekf.p_yy'])),
        ('{ } 平衡', new_txt.count('{') == new_txt.count('}')),
        ('/* */ 平衡', new_txt.count('/*') == new_txt.count('*/'))]
print()
for k, v in must:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k

# 备份当前 VER=49，再写回
shutil.copy2(P, P + '.bak_pre_revert_v49')
shutil.copy2(T, T + '.bak_pre_revert_v49')
open(P, 'wb').write(new_txt.encode('gbk'))
shutil.copy2(TUNE43, T)
print('\n已回退: proc_ekf.c（仅我的块）; v5f_tune.h <- bak_s2p')

# 回退后与 .bak_s2n 的差异应"恰好只有那 6 个诊断块"
p2 = rd(P).split('\n')
sm2 = difflib.SequenceMatcher(None, old, p2, autojunk=False)
blk = [b for b in sm2.get_opcodes() if b[0] != 'equal']
n_bad = 0
for tag, i1, i2, j1, j2 in blk:
    txt = '\n'.join(old[i1:i2]) + '\n@@@\n' + '\n'.join(p2[j1:j2])
    if any(m in txt for m in MARK):
        n_bad += 1
print('回退后 vs .bak_s2n：差异块 %d，其中含我标记的 %d（应为 0）' % (len(blk), n_bad))
assert n_bad == 0 and len(blk) == 6, '回退结果不是 .bak_s2n + 6 个诊断块'

t2 = rd(T)
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', rd(U + r'\src\SPI_rx.c')).group(1))
print('tune: VER=%d   （我的两条 #define 已随回退消失: %s）'
      % (ver, 'V5F_EKF_MAG_SPIKE_DEG' not in t2 and 'V5F_EKF_TILT_AMAG_CAP' not in t2))
print('\nfw_tag = %d  (VER=%d, %d ch)  <- 回到本会话之前' % ((ver << 16) | (nch << 8) | 7, ver, nch))
