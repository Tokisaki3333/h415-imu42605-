# -*- coding: utf-8 -*-
"""VER=50 收尾：正确校验 + 读取端同步。"""
import re

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
c = open(R + r'\V5F\User\src\proc_ekf.c', 'rb').read().decode('gbk')
h = open(R + r'\V5F\User\inc\SPI_rx.h', 'rb').read().decode('gbk')
s = open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')
t = open(R + r'\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
NEW = ['tilt_dqx', 'tilt_dqy', 'tilt_dqz', 'tilt_prx', 'tilt_pry', 'tilt_prz']
i0 = c.index('static void ekf_m6_tilt')
m6 = c[i0:c.index('\nstatic void ', i0 + 10)]
mm = re.search(r'ekf_update\([^;]*?0x([0-9A-Fa-f]+)u, V5F_EKF_TILT_K_MAX\)', m6, re.S)
print('  M6 实际掩码 = 0x%s' % (mm.group(1) if mm else '?'))
m7m = re.search(r'ekf_update\([^;]*?0x([0-9A-Fa-f]+)u, V5F_EKF_MAG_K_MAX\)',
                c[c.index('static void ekf_m7_mag'):], re.S)
print('  M7 实际掩码 = 0x%s' % (m7m.group(1) if m7m else '?'))
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s).group(1))
CK = [('VER==50', ver == 50), ('JF_CH_NUM==139', nch == 139),
      ('M6 dq 采集在', 's_tilt_dqz = s_dx[IX_Q + 2] * RAD2DEG;' in m6),
      ('M6 pr 采集在', 's_tilt_prz = pr[2];' in m6),
      ('M6 采集只加不改', m6.count('ekf_update(') == 1),
      ('M7 只看偏航', m7m and m7m.group(1).upper() == '0100'),
      ('publish +6', all('h->ekf.%s =' % f in c for f in NEW)),
      ('struct +6', all(re.search(r'\b%s\b' % f, h) for f in NEW)),
      ('channel +6', all('g_v5f_hold.ekf.%s' % f in s for f in NEW)),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))

J = R + r'\tools\calib\jf_load.py'
j = open(J, 'rb').read().decode('utf-8')
if 'CH_139' not in j:
    add = ("\nCH_139 = dict(CH_133)\n"
           "CH_139.update({'ekf_tilt_dqx': 133, 'ekf_tilt_dqy': 134, 'ekf_tilt_dqz': 135,\n"
           "               'ekf_tilt_prx': 136, 'ekf_tilt_pry': 137, 'ekf_tilt_prz': 138})\n")
    j = j.replace("\nCH_BY_NCH = {", add + "\nCH_BY_NCH = {", 1)
    j = j.replace('133: CH_133}', '133: CH_133, 139: CH_139}', 1)
    j = j.replace('    if nch == 133:\n        return CH_133\n',
                  '    if nch == 133:\n        return CH_133\n'
                  '    if nch == 139:\n        return CH_139\n', 1)
    open(J, 'wb').write(j.encode('utf-8'))
j2 = open(J, 'rb').read().decode('utf-8')
ok = 'CH_139 = dict(CH_133)' in j2 and '139: CH_139' in j2 and 'if nch == 139' in j2
print('  jf_load.py CH_139: %s' % ('PASS' if ok else 'FAIL'))
assert ok
K = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
k = open(K, 'rb').read().decode('utf-8', 'ignore')
open(K, 'wb').write(re.sub(r'assert n == \d+', 'assert n == 139', k).encode('utf-8'))
print('  count_cols assert:',
      re.findall(r'assert n == (\d+)', open(K, 'rb').read().decode('utf-8', 'ignore')))
