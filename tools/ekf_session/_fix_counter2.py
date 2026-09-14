# -*- coding: utf-8 -*-
"""count_cols.py 用**块注释状态机**剥注释 —— 多行注释的中间行没有 "/*"，
按行 split 是剥不掉的（这正是那个幽灵列）。"""
import re

CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()

old_start = c.index('n = 0\npending = 1')
old_end = c.index("print('运行时列数")
tail = c[old_end:]
head = c[:old_start]

body = '''n = 0
pending = 1
rows = []
chain = 0          # 0 = 不在 if/else 链里；1..k = 第 k 个分支
incmt = False      # 块注释状态（跨行！）

for ln in lines:
    # ---- 先按块注释状态机剥注释，再数 ch[c++] ----
    code = ''
    i = 0
    while i < len(ln):
        if incmt:
            j = ln.find('*/', i)
            if j < 0:
                i = len(ln)
            else:
                incmt = False
                i = j + 2
        else:
            j = ln.find('/*', i)
            if j < 0:
                code += ln[i:]
                break
            code += ln[i:j]
            incmt = True
            i = j + 2
    code = code.split('//')[0]
    s = code.strip()
    sraw = ln.strip()

    if 's_rep_tick_ok == 0u' in s:
        chain = 1
    elif chain and s.startswith('} else'):
        chain += 1

    m = re.search(r'for \\(\\w+ = 0u; \\w+ < (\\w+)(?:u)?; \\w+\\+\\+\\)\\s*', s)
    if m:
        b = m.group(1)
        pending = {'3u': 3, '4u': 4, '5u': 5}.get(b, 0)
        if pending is None:
            raise SystemExit('未知循环上界 ' + b)
        s = s[m.end():]
        if not s:
            continue
    k = s.count('ch[c++]')
    if not k:
        if chain and s == '}':
            chain = 0
        continue
    if pending == 0:                     # 有限性扫描循环：不是列
        pending = 1
        continue
    if chain > 1:                        # if/else 链的非首分支：运行时不会执行
        pending = 1
        if s == '}':
            chain = 0
        continue
    nm = ''
    if '/*' in ln:
        nm = ln.split('/*', 1)[1].replace('*/', '').strip()
    base = nm if nm else s[:46]
    for q in range(k * pending):
        rows.append((n + q, base))
    n += k * pending
    pending = 1
    if chain and s == '}':
        chain = 0

'''
c = head + body + tail
open(CP, 'w', encoding='utf-8', newline='\n').write(c)
import ast
ast.parse(c)
print('count_cols.py: 改成块注释状态机')
