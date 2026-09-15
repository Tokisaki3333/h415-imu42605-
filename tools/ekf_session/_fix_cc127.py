# -*- coding: utf-8 -*-
"""count_cols.py 同步到 127 列。"""
import re
import shutil

K = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
k = open(K, 'rb').read().decode('utf-8', 'ignore')
print('原 assert:', re.findall(r'assert n == (\d+)', k))
print('有 ekf_p_yy 条目:', bool(re.search(r"'ekf_p_yy'", k)))
k = re.sub(r'assert n == \d+', 'assert n == 127', k)
m = re.search(r"([ \t]*)'ekf_p_yy'[ \t]*:[ \t]*\d+[ \t]*,?", k)
if m and 'ekf_prop_ok' not in k:
    ins = (m.group(0) + '\n' + m.group(1) +
           "'ekf_prop_ok': 122, 'ekf_f_ok': 123, 'ekf_prop_row': 124,\n" +
           m.group(1) + "'ekf_stage': 125, 'ekf_mag_rej': 126,")
    k = k[:m.start()] + ins + k[m.end():]
    print('已补 5 个列名')
elif 'ekf_prop_ok' not in k:
    print('!! 未找到 p_yy 条目，只改了 assert')
shutil.copy2(K, K + '.bak_v44fix')
open(K, 'wb').write(k.encode('utf-8'))
k2 = open(K, 'rb').read().decode('utf-8', 'ignore')
print('现 assert:', re.findall(r'assert n == (\d+)', k2),
      ' prop_ok 在:', 'ekf_prop_ok' in k2)
