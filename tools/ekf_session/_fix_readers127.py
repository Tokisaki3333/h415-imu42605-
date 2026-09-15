# -*- coding: utf-8 -*-
"""同步读取端到 127 列：jf_load.py 与 count_cols.py。"""
import re
import shutil

C = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib'
NEWMAP = ("\nCH_127 = dict(CH_122)\n"
          "CH_127.update({'ekf_prop_ok': 122, 'ekf_f_ok': 123, 'ekf_prop_row': 124,\n"
          "               'ekf_stage': 125, 'ekf_mag_rej': 126})\n")

# ---- jf_load.py ----
J = C + r'\jf_load.py'
j = open(J, 'rb').read().decode('utf-8')
assert 'CH_127' not in j
m = re.search(r'\nCH_BY_NCH = \{[^}]*\}\n', j)
assert m and '122: CH_122' in m.group(0), 'CH_BY_NCH 锚点异常'
j = j[:m.start()] + NEWMAP + j[m.start():]
j = j.replace('122: CH_122}', '122: CH_122, 127: CH_127}')
j = j.replace('CH = CH_122', 'CH = CH_127', 1)
j = j.replace('    if nch == 122:\n        return CH_122\n',
              '    if nch == 122:\n        return CH_122\n    if nch == 127:\n        return CH_127\n', 1)
j = j.replace("{90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113,\n"
              "             115: CH_115, 117: CH_117, 122: CH_122}",
              "{90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113,\n"
              "             115: CH_115, 117: CH_117, 122: CH_122, 127: CH_127}")
data = j.encode('utf-8')
shutil.copy2(J, J + '.bak_v44fix')
open(J, 'wb').write(data)
j2 = open(J, 'rb').read().decode('utf-8')
print('jf_load.py:  CH_127 定义 %s | CH_BY_NCH %s | CH=%s | ch_for %s'
      % ('CH_127 = dict(CH_122)' in j2, '127: CH_127' in j2,
         re.search(r'^CH = (\w+)', j2, re.M).group(1),
         'if nch == 127:' in j2))
assert 'CH_127 = dict(CH_122)' in j2 and '127: CH_127' in j2 and 'if nch == 127:' in j2

# ---- count_cols.py ----
K = C + r'\count_cols.py'
k = open(K, 'rb').read().decode('utf-8')
n_before = re.findall(r'assert n == (\d+)', k)
print('count_cols.py: 原 assert %s' % n_before)
k = re.sub(r'assert n == \d+', 'assert n == 127', k)
# KEY 表里补 5 项（若存在 KEY 类似字典）
if 'ekf_p_yy' in k and 'ekf_prop_ok' not in k:
    m2 = re.search(r"([ \t]*)'ekf_p_yy'[ \t]*:[ \t]*\d+[ \t]*,?", k)
    if m2:
        ins = (m2.group(0) +
               "\n%s'ekf_prop_ok': 122, 'ekf_f_ok': 123, 'ekf_prop_row': 124,\n"
               "%s'ekf_stage': 125, 'ekf_mag_rej': 126," % (m2.group(1), m2.group(1)))
        k = k[:m2.start()] + ins + k[m2.end():]
        print('  已补 5 个列名')
    else:
        print('  !! 未找到 p_yy 条目，只改了 assert')
else:
    print('  未发现 KEY 表条目（或已补）')
open(K, 'wb').write(k.encode('utf-8'))
k2 = open(K, 'rb').read().decode('utf-8')
print('  assert 现为 %s ; prop_ok 在 %s' % (re.findall(r'assert n == (\d+)', k2), 'ekf_prop_ok' in k2))
