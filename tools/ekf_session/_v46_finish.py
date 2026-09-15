# -*- coding: utf-8 -*-
"""VER=46 收尾：作用域正确的复核 + 读取端同步。"""
import re

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
c = open(R + r'\V5F\User\src\proc_ekf.c', 'rb').read().decode('gbk')
h = open(R + r'\V5F\User\inc\SPI_rx.h', 'rb').read().decode('gbk')
s = open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')
t = open(R + r'\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')

# 只取 ekf_m7_mag 函数体
i0 = c.index('static void ekf_m7_mag')
i1 = c.index('\nstatic void ', i0 + 10)
m7 = c[i0:i1]
print('--- ekf_m7_mag 前 6 行 / 观测块 ---')
for ln in m7.split('\n')[:6]:
    print('   ', ln[:100])
for ln in m7.split('\n'):
    if re.search(r'b0x|b0z|s_H\[|r\[0\]|r\[1\]|R\[0\]|ekf_update|s_mag_dq|s_mag_rx', ln):
        print('   ', ln.strip()[:104])

ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s).group(1))
NEWF = ['mag_rx', 'mag_ry', 'mag_dqx', 'mag_dqy', 'mag_dqz']
CK = [('VER==46', ver == 46),
      ('JF_CH_NUM==133', nch == 133),
      ('M7 内无 atan2', 'atan2f' not in m7),
      ('M7 内无 bh2', 'bh2' not in m7),
      ('M7 内无 fhb 门', 'V5F_EKF_MAG_BHB_MIN' not in m7),
      ('M7 内无 fhb 加权', 'V5F_EKF_MAG_FERR' not in m7),
      ('二维新息', 'r[0] = Bn[0] - b0x;' in m7 and 'r[1] = Bn[1] - b0y;' in m7),
      ('H 偏航列', 's_H[0][8] = -b0y;' in m7 and 's_H[1][8] =  b0x;' in m7),
      ('H 倾角列', 's_H[0][7] =  b0z;' in m7 and 's_H[1][6] = -b0z;' in m7),
      ('掩码 0x01C0', '0x01C0u, V5F_EKF_MAG_K_MAX' in m7),
      ('m=2 且 R 二维', 'ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2' in m7 and 'R[3] = sig2;' in m7),
      ('5 个新量赋值在', all('s_%s =' % f in m7 for f in NEWF)),
      ('publish +5', all('h->ekf.%s =' % f in c for f in NEWF)),
      ('struct +5', all(re.search(r'\b%s\b' % f, h) for f in NEWF)),
      ('channel +5', all('g_v5f_hold.ekf.%s' % f in s for f in NEWF)),
      ('prop 自锁修复仍在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
print()
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
assert all(v for _, v in CK), '有未通过项'
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
print('新列 128 mag_rx 129 mag_ry 130 mag_dqx 131 mag_dqy 132 mag_dqz')

# ---- 读取端 ----
J = R + r'\tools\calib\jf_load.py'
j = open(J, 'rb').read().decode('utf-8')
if 'CH_133' not in j:
    add = ("\nCH_133 = dict(CH_127)\n"
           "CH_133.update({'ekf_mag_rx': 128, 'ekf_mag_ry': 129, 'ekf_mag_dqx': 130,\n"
           "               'ekf_mag_dqy': 131, 'ekf_mag_dqz': 132})\n")
    j = j.replace("\nCH_BY_NCH = {", add + "\nCH_BY_NCH = {", 1)
    j = j.replace('128: CH_127}', '128: CH_127, 133: CH_133}', 1)
    j = j.replace('    if nch == 128:\n        return CH_127\n',
                  '    if nch == 128:\n        return CH_127\n'
                  '    if nch == 133:\n        return CH_133\n', 1)
    open(J, 'wb').write(j.encode('utf-8'))
j2 = open(J, 'rb').read().decode('utf-8')
ok = ('CH_133 = dict(CH_127)' in j2 and '133: CH_133' in j2 and 'if nch == 133' in j2
      and "'ekf_mag_dqz': 132" in j2)
print('  jf_load.py: %s' % ('PASS' if ok else 'FAIL'))
assert ok
K = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
k = open(K, 'rb').read().decode('utf-8', 'ignore')
k = re.sub(r'assert n == \d+', 'assert n == 133', k)
if 'ekf_mag_rx' not in k and re.search(r"'ekf_mag_fhb'", k):
    m = re.search(r"([ \t]*)'ekf_mag_fhb'[ \t]*:[ \t]*\d+[ \t]*,?", k)
    k = k[:m.start()] + (m.group(0) + '\n' + m.group(1) +
                         "'ekf_mag_rx': 128, 'ekf_mag_ry': 129,\n" + m.group(1) +
                         "'ekf_mag_dqx': 130, 'ekf_mag_dqy': 131, 'ekf_mag_dqz': 132,") + k[m.end():]
open(K, 'wb').write(k.encode('utf-8'))
print('  count_cols.py assert:',
      re.findall(r'assert n == (\d+)', open(K, 'rb').read().decode('utf-8', 'ignore')))
