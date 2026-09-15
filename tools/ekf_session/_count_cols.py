# -*- coding: utf-8 -*-
# 精确核对 justfloat_report 的列数（花括号配对取函数体）
import re
p = r'h415-imu42605-\V5F\User\src\SPI_rx.c'
src = open(p, 'rb').read().decode('gbk')
sig = src.find('static void justfloat_report(void)')
assert sig > 0
b0 = src.find('{', sig); assert b0 > 0
depth = 0; k = b0; in_blk = False; in_str = False
while k < len(src):
    c = src[k]
    if in_blk:
        if c == '\n': in_blk = False
    elif in_str:
        if c == '"': in_str = False
    elif c == '/' and k+1 < len(src) and src[k+1] == '*':
        in_blk = True; k += 1
    elif c == '"':
        in_str = True
    elif c == '{':
        depth += 1
    elif c == '}':
        depth -= 1
        if depth == 0:
            break
    k += 1
body = src[b0:k+1]
print('函数体 %d 字节, 起止 %d..%d' % (len(body), b0, k))

lines = body.split('\n')
# 找出所有含 ch[c++] 的行, 标注 for 列数
entries = []
for idx, ln in enumerate(lines):
    n = ln.count('ch[c++]')
    if n == 0: continue
    cnt = n
    m = re.search(r'for\s*\(\s*i\s*=\s*0u?\s*;\s*i\s*<\s*(\d+)u?\s*;', ln)
    if not m and idx > 0:
        m = re.search(r'for\s*\(\s*i\s*=\s*0u?\s*;\s*i\s*<\s*(\d+)u?\s*;', lines[idx-1])
    if m:
        cnt = int(m.group(1)) * n
    entries.append((idx, cnt, ln.strip()))

total_all = sum(e[1] for e in entries)
print('所有分支累加 = %d' % total_all)

# 条件链：找出连续出现的 "if/else if/else + ch[c++]" 组, 每组只执行 1 个分支
extra = 0
chain = []
for idx, cnt, s in entries:
    ctx = lines[idx]
    is_if = ('if (' in ctx) or ('else' in ctx)
    chain.append((idx, cnt, is_if))
run = []
for idx, cnt, is_if in chain:
    if is_if:
        run.append((idx, cnt))
    else:
        if len(run) > 1:
            extra += sum(c for _, c in run) - run[0][1]
            print('  条件链在行 %s: %d 个分支, 各 %s 列 -> 多算 %d'
                  % ([r[0] for r in run], len(run), [c for _, c in run],
                     sum(c for _, c in run) - run[0][1]))
        run = []
if len(run) > 1:
    extra += sum(c for _, c in run) - run[0][1]
    print('  条件链在行 %s: %d 个分支 -> 多算 %d'
          % ([r[0] for r in run], len(run), sum(c for _, c in run) - run[0][1]))

total = total_all - extra
m = re.search(r'#define\s+JF_CH_NUM\s+(\d+)u', src)
decl = int(m.group(1))
print()
print('运行时列数 = %d - %d = %d' % (total_all, extra, total))
print('#define JF_CH_NUM = %d   JF_FRAME_LEN = %d' % (decl, decl*4+6))
print('  %s' % ('一致 PASS' if total == decl else '不一致 FAIL'))
assert total == decl, '列数与 JF_CH_NUM 不一致！'
print()
print('末 12 处:')
for idx, cnt, s in entries[-12:]:
    print('   +%-2d  %s' % (cnt, s[:78]))
