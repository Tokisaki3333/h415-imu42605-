# -*- coding: utf-8 -*-
"""复核 VER=45 固件状态，并把读取端同步到 128 列。"""
import re

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
c = open(R + r'\V5F\User\src\proc_ekf.c', 'rb').read().decode('gbk')
h = open(R + r'\V5F\User\inc\SPI_rx.h', 'rb').read().decode('gbk')
s = open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')
t = open(R + r'\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s).group(1))
CK = [('VER==45', ver == 45),
      ('机体系判据在', 'fhb2 < V5F_EKF_MAG_BHB_MIN' in c),
      ('R 连续加权', 'V5F_EKF_MAG_FERR / sqrtf(fhb2)' in c),
      ('s_mag_fhb 赋值', 's_mag_fhb = sqrtf(fhb2);' in c),
      ('publish mag_fhb', bool(re.search(r'h->ekf\.mag_fhb\s*=\s*s_mag_fhb;', c))),
      ('struct mag_fhb', 'mag_fhb' in h),
      ('channel mag_fhb', 'g_v5f_hold.ekf.mag_fhb' in s),
      ('prop 自锁修复仍在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('JF_CH_NUM==128', nch == 128),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
assert all(v for _, v in CK), '固件状态未通过'
print('  fw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))

# ---- jf_load.py：CH_127 再追加 mag_fhb=127，并让 128 列也走 CH_127 ----
J = R + r'\tools\calib\jf_load.py'
j = open(J, 'rb').read().decode('utf-8')
if "'ekf_mag_fhb': 127" not in j:
    j = j.replace("'ekf_stage': 125, 'ekf_mag_rej': 126})",
                  "'ekf_stage': 125, 'ekf_mag_rej': 126, 'ekf_mag_fhb': 127})", 1)
    j = j.replace('127: CH_127}', '127: CH_127, 128: CH_127}', 1)
    j = j.replace('    if nch == 127:\n        return CH_127\n',
                  '    if nch == 127:\n        return CH_127\n'
                  '    if nch == 128:\n        return CH_127\n', 1)
    open(J, 'wb').write(j.encode('utf-8'))
j2 = open(J, 'rb').read().decode('utf-8')
print('  jf_load.py: mag_fhb=%s  128:CH_127=%s  ch_for128=%s'
      % ("'ekf_mag_fhb': 127" in j2, '128: CH_127' in j2, 'if nch == 128' in j2))
assert "'ekf_mag_fhb': 127" in j2 and '128: CH_127' in j2 and 'if nch == 128' in j2

# ---- count_cols.py ----
K = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
k = open(K, 'rb').read().decode('utf-8', 'ignore')
k = re.sub(r'assert n == \d+', 'assert n == 128', k)
open(K, 'wb').write(k.encode('utf-8'))
print('  count_cols.py assert:',
      re.findall(r'assert n == (\d+)', open(K, 'rb').read().decode('utf-8', 'ignore')))
print('\n新列 127 = ekf_mag_fhb（机体系水平占比 = 向量有效性判据本身）')
