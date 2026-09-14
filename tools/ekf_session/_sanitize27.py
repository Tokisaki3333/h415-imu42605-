# -*- coding: utf-8 -*-
"""把 _fix_v27.py 里所有 GBK 编不了的符号换成 ASCII，再调用它。"""
import io

P = r'C:\Users\33\Documents\v2\_fix_v27.py'
t = open(P, encoding='utf-8').read()
MAP = {'\u00b2': '^2', '\u2227': ' and ', '\u2228': ' or ', '\u2248': '~',
       '\u03c3': 'sigma', '\u00d7': 'x', '\u2264': '<=', '\u2265': '>=',
       '\u2261': '==', '\u2192': '->', '\u221a': 'sqrt'}
n = 0
for k, v in MAP.items():
    c = t.count(k)
    if c:
        t = t.replace(k, v)
        n += c
open(P, 'w', encoding='utf-8', newline='\n').write(t)
print('替换 %d 个禁区符号' % n)

# 先验证：把整个脚本里"会写进固件的字符串"逐条做 GBK 预检
import re
bad = []
for m in re.finditer(r'"""((?:[^"\\]|\\.)*)"""', t, re.S):
    s = m.group(1)
    if 'V5F' in s or 'ekf' in s or '|a|' in s:
        try:
            s.encode('gbk')
        except UnicodeEncodeError as e:
            bad.append(str(e))
print('新增文本 GBK 预检:', '全部通过' if not bad else bad[:3])
