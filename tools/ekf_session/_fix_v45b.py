# -*- coding: utf-8 -*-
"""VER=45 第二步：v5f_tune.h 加 V5F_EKF_MAG_SPIKE_DEG 并升 VER。"""
import re
import shutil

T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
u = open(T, 'rb').read().decode('gbk')

if 'V5F_EKF_MAG_SPIKE_DEG' not in u:
    m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_R_MAX_DEG[^\r\n]*$', u, re.M)
    assert m, '未找到 V5F_EKF_MAG_R_MAX_DEG'
    ins = ('\n/* ★VER=45 地磁"瞬时跳变"判据：仅当 |r|>MAG_R_MAX_DEG 且与上一周期残差\n'
           ' * 之差超过本值时，才判为坏样本丢弃。持续的框架偏置（大而稳定）不丢，\n'
           ' * 否则偏航永远无法被牵引到磁北（VER=44 实测：r 长期 165 度、硬门每周期\n'
           ' * 都在拒、mag_used 恒 0）。20 度。 */\n'
           '#define V5F_EKF_MAG_SPIKE_DEG     20.0f')
    u = u[:m.end()] + ins + u[m.end():]

assert u.count('#define V5F_FW_VER        44u') == 1
u = u.replace('#define V5F_FW_VER        44u', '#define V5F_FW_VER        45u', 1)

data = u.encode('gbk')
shutil.copy2(T, T + '.bak_s2q')
open(T, 'wb').write(data)

h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
ck = [('VER==45', ver == 45),
      ('SPIKE 定义一次', h.count('V5F_EKF_MAG_SPIKE_DEG') == 1),
      ('R_MAX 仍在', h.count('V5F_EKF_MAG_R_MAX_DEG') >= 1)]
for k, v in ck:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k

# C 文件复核（上一步已写入）
P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
c = open(P, 'rb').read().decode('gbk')
ck2 = [('判别在新', 'dr > V5F_EKF_MAG_SPIKE_DEG * DEG2RAD' in c),
       ('旧绝对门已去', 'if (fabsf(r[0]) > V5F_EKF_MAG_R_MAX_DEG * DEG2RAD) {' not in c),
       ('播种在', 'if (!s_mag_r_seed)' in c),
       ('新状态在', 'static uint8_t  s_mag_spike;' in c),
       ('{ } 平衡', c.count('{') == c.count('}')),
       ('/* */ 平衡', c.count('/*') == c.count('*/')),
       ('括号差 -4（改动前既有）', c.count('(') - c.count(')') == -4)]
for k, v in ck2:
    print('  %-24s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=45, 122 ch, 无新增通道)' % ((45 << 16) | (122 << 8) | 7))
