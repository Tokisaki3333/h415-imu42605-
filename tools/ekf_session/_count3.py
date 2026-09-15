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

def strip_comments(s):
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    s = re.sub(r'//[^\n]*', '', s)
    return s

def count(p):
    src = open(p, 'rb').read().decode('gbk')
    body = strip_comments(body_of(src))
    tot = 0; ent = []
    for ln in body.split('\n'):
        n = ln.count('ch[c++]')
        if n == 0: continue
        m = re.search(r'for\s*\([^)]*i\s*<\s*(\d+)u?\s*;[^)]*\)', ln)
        cnt = int(m.group(1)) * n if m else n     # 只认同一行内的 for
        tot += cnt; ent.append((cnt, ln.strip()))
    decl = int(re.search(r'#define\s+JF_CH_NUM\s+(\d+)u', src).group(1))
    # DRDY 间隔那条 if / else if / else 链: 3 个分支各写 1 列, 运行时只走 1 个 -> 扣 2
    if '} else if (tk >= s_rep_last_tick)' in src:
        tot -= 2
        ent.append((-2, '(DRDY if/else 链的 2 个未执行分支, 代码注释已说明)'))
    return tot, decl, ent

print('=== 校准 + 核对 ===')
for p, lab in ((r'h415-imu42605-\V5F\User\src\SPI_rx.c.bak_v78', '修改前'),
               (r'h415-imu42605-\V5F\User\src\SPI_rx.c', '修改后')):
    t, d, _ = count(p)
    print('  %-6s 计数器=%3d  声明=%3d  差 %+d  %s' % (lab, t, d, t-d, 'PASS' if t == d else 'FAIL'))
t1, d1, _ = count(r'h415-imu42605-\V5F\User\src\SPI_rx.c.bak_v78')
t2, d2, _ = count(r'h415-imu42605-\V5F\User\src\SPI_rx.c')
assert t1 == d1 and t2 == d2
assert t2 - t1 == 4, '净增列数不是 4'
print('  净增列数 = %d （新增 4 个罗盘量）  JF_CH_NUM %d -> %d  JF_FRAME_LEN %d -> %d'
      % (t2-t1, d1, d2, d1*4+6, d2*4+6))
print()
t, d, ent = count(r'h415-imu42605-\V5F\User\src\SPI_rx.c')
print('新占的 4 列（应排在最末）:')
for cnt, s in ent[-4:]:
    print('   +%d  %s' % (cnt, s[:76]))
print()
print('PASS: 列数 = JF_CH_NUM = %d' % d)
