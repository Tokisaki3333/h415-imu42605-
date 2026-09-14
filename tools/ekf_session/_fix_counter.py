# -*- coding: utf-8 -*-
"""修 count_cols.py：数 ch[c++] 之前必须先剥掉注释 —— 我在注释里写过 "ch[c++]" 这个
字符串来解释历史事故，结果被当成了一列，列数整体 +1。"""
import re
import shutil
import sys

CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
a = "    k = s.count('ch[c++]')"
assert c.count(a) == 1, c.count(a)
c = c.replace(a, """    # ★ 先剥注释再数：注释里出现过 "ch[c++]" 这个字符串（就是在解释列数事故本身），
    #   不剥就会把它当成一列，整张表的列号整体 +1。
    code = re.sub(r'/\\*.*?\\*/', '', ln)
    code = re.sub(r'//.*$', '', code)
    k = code.count('ch[c++]')""", 1)
open(CP, 'w', encoding='utf-8', newline='\n').write(c)
print('count_cols.py: 计数前剥注释')

# ---- jf_load.py 的 CH_BY_NCH 实际长什么样 ----
JL = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()
for l in j.split('\n'):
    if 'CH_BY_NCH' in l or l.strip().startswith('CH = ') or 'CH_113' in l:
        print('  jf_load:', l.strip()[:100])
