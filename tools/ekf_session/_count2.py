# -*- coding: utf-8 -*-
import re

def body_of(src):
    sig = src.find('static void justfloat_report(void)')
    b0 = src.find('{', sig); depth = 0; k = b0; in_blk = False; in_str = False
    while k < len(src):
        c = src[k]
        if in_blk:
            if c == '\n': in_blk = False
        elif in_str:
            if c == '"': in_str = False
        elif c == '/' and src[k+1:k+2] == '*':
            in_blk = True; k += 1
        elif c == '"':
            in_str = True
        elif c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0: break
        k += 1
    return src[b0:k+1]

def count(p):
    src = open(p, 'rb').read().decode('gbk')
    lines = body_of(src).split('\n')
    tot = 0; ent = []
    for idx, ln in enumerate(lines):
        n = ln.count('ch[c++]')
        if n == 0: continue
        m = re.search(r'for\s*\(\s*i\s*=\s*0u?\s*;\s*i\s*<\s*(\d+)u?\s*;', ln)
        if not m and idx > 0:
            m = re.search(r'for\s*\(\s*i\s*=\s*0u?\s*;\s*i\s*<\s*(\d+)u?\s*;', lines[idx-1])
        cnt = int(m.group(1)) * n if m else n
        tot += cnt; ent.append((idx, cnt, ln.strip()))
    decl = int(re.search(r'#define\s+JF_CH_NUM\s+(\d+)u', src).group(1))
    return tot, decl, ent

for p, lab in ((r'h415-imu42605-\V5F\User\src\SPI_rx.c.bak_v78', 'before(=144)'),
               (r'h415-imu42605-\V5F\User\src\SPI_rx.c', 'after(=148)')):
    t, d, ent = count(p)
    print('%-14s counter=%d  decl=%d  diff %+d' % (lab, t, d, t - d))
print()
t, d, ent = count(r'h415-imu42605-\V5F\User\src\SPI_rx.c')
print('全部 ch[c++] 行 (%d 处):' % len(ent))
for idx, cnt, s in ent:
    print('   +%-2d  %s' % (cnt, s[:88]))
