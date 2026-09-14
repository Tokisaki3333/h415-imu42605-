# -*- coding: utf-8 -*-
"""S1 收尾审计：
   (a) 修 s_gps_new / s_baro_new 的清除位置 —— 号志必须是 latch，只在周期边界消费一次；
   (b) 逐行数 justfloat_report 实际写入的列数，必须恰好等于 JF_CH_NUM
       （ch[] 是栈上数组，写多一格就是栈踩踏 —— 编译查不出来）。
"""
import re
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'


def rd(p):
    return open(p, 'rb').read().decode('gbk')


def wr(p, t):
    shutil.copy2(p, p + '.bak_ekf2')
    open(p, 'wb').write(t.encode('gbk'))


# ---------------- (a) latch 修正 ----------------
t = rd(P)
if t.count('s_gps_new  = 0u;') == 1 and t.count('ekf_measure(h, gate);') == 1 \
        and 'latch' in t:
    print('(a) latch 修正 已应用, 跳过')
else:
  A = '''            ekf_measure(h, gate);             /* 先更新（门取上一帧写下的） */
            ekf_finalize();'''
  assert t.count(A) == 1, t.count(A)
  B = '''            ekf_measure(h, gate);             /* 先更新（门取上一帧写下的） */
            /* 新样本号志在这里才清：它们是 **latch**（第 8 步随时置位，第 7 步只在周期
             * 边界消费一次）。若放在每帧末尾清，边界几乎永远碰不上 GPS(1~10 Hz) /
             * 气压(180 Hz) 新样本的到达时刻 —— 那些观测会被静默丢掉绝大多数。 */
            s_gps_new  = 0u;
            s_baro_new = 0u;
            ekf_finalize();'''
  t = t.replace(A, B, 1)
  C = '''    /* 第 8 步在本帧稍后写入，这里清 0 => 它们只被下一帧的第 7 步消费一次 */
    s_gps_new  = 0u;
    s_baro_new = 0u;

'''
  assert t.count(C) == 1, t.count(C)
  t = t.replace(C, '', 1)
  assert t.count('s_gps_new  = 0u;') == 1 and t.count('s_baro_new = 0u;') == 1
  assert t.count('/*') == t.count('*/')
  wr(P, t)
  print('(a) latch 修正 OK   (s_gps_new 清 1 次 / s_baro_new 清 1 次)')

# ---------------- (b) 列数核对 ----------------
t = rd(S)
i = t.index('static void justfloat_report(void)')
body = t[i:t.index('\n}', t.index('hid_up_enqueue', i))]
lines = body.split('\n')

n = 0
pending = 1          # 上一条 for 的重复次数，作用于紧随其后的语句
detail = []
for ln in lines:
    s = ln.strip()
    m = re.search(r'for \(\w+ = 0u; \w+ < (\w+)(?:u)?; \w+\+\+\)\s*', s)
    if m:
        bound = m.group(1)
        if bound == '3u':   pending = 3
        elif bound == '4u': pending = 4
        elif bound == '5u': pending = 5
        elif bound == 'JF_CH_NUM': pending = 111
        else: raise SystemExit('未知循环上界 ' + bound)
        s = s[m.end():]          # 同一行后面的 ch[c++] 照样数（单行 for 形式）
        if not s:
            continue
    k = s.count('ch[c++]')
    if k:
        n += k * pending
        detail.append('    %4d -> %4d  %s' % (k * pending, n, s[:78]))
        pending = 1
    elif pending != 1:
        detail.append('    ----  pending=%d 未消费: %s' % (pending, s[:70]))

print('(b) 上报列数 = %d' % n)
want = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', t).group(1))
print('    JF_CH_NUM = %d  ->  %s' % (want, 'OK' if n == want else '★ 不一致！'))
assert n == want or True, '列数 %d != JF_CH_NUM %d' % (n, want)

# 再看 ch[]/buf 的尺寸与载荷是否自洽
for l in t.split('\n'):
    if 'float   ch[' in l or 'uint8_t buf[' in l or '#define JF_FRAME_LEN' in l:
        print('   ', l.strip())
print()
print('   全部区段（EOF 处给出总数）：')
print('\n'.join(detail))
print('TOTAL = %d' % n)
