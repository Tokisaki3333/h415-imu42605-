# -*- coding: utf-8 -*-
"""recv_v9.py 的列号对齐（0 基真值）。"""
p = r'C:\Users\33\Documents\v2\recv_v9.py'
t = open(p, encoding='utf-8').read()
t = t.replace(
    'FW = 611078.0          # VER=9, 83ch, EKF=0, MAGCAL=1, AC=1',
    'FW = 684039.0          # VER=10, 112ch, EKF=1, MAGCAL=1, AC=1\n'
    'IDX_FW = 76            # 0 基列号，由 tools/calib/count_cols.py 逐行数 ch[c++] 得到\n'
    'IDX_DT = 25\n'
    'IDX_ECHO, IDX_CNT, IDX_LAST = 77, 78, 79')
t = t.replace('float(m[76]) == FW', 'float(m[IDX_FW]) == FW')
t = t.replace('100.0 < float(m[25]) < 200.0', '100.0 < float(m[IDX_DT]) < 200.0')
t = t.replace('dt_sum += float(m[25]) * 1e-6', 'dt_sum += float(m[IDX_DT]) * 1e-6')
t = t.replace('float(m[82]) == 84.0', 'float(m[IDX_LAST]) == 84.0')
t = t.replace('float(m[80])', 'float(m[IDX_ECHO])').replace('float(m[81])', 'float(m[IDX_CNT])')
t = t.replace('if nch and nch >= 83', 'if nch and nch >= 112')
open(p, 'w', encoding='utf-8', newline='\n').write(t)
print('recv_v9.py 已对齐: FW=684039, fw_tag@76, dt_us@25, cmd@77/78/79')
assert 'IDX_FW' in t and 'm[76]' not in t and 'm[25]' not in t
