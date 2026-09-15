# -*- coding: utf-8 -*-
"""VER=82b：修 v5f_tune.h 的注释块破损。
_fix_v82_rate.py 把原来 'define ... 0.05f  /* ...' 那一行整行替换成了自带 */ 的新
注释块，原注释块的续行(分工/硬门说明)被挤到注释外 -> GCC 报 stray '\272'。
做法：把 673..685 行整段重排成 [一个完整注释块][#define]，历史说明并入同一块。
"""
import io, os, shutil, sys

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
ENC = 'gbk'

text = open(P, 'rb').read().decode(ENC)
lines = text.split('\n')

# ---- 锚点断言 ----
assert lines[672].startswith('/* \u2605VER=82'), lines[672][:40]
assert lines[673].startswith(' * \u4e00\u9636\u73af\u603b\u8bef\u5dee'), lines[673][:40]
assert lines[679].rstrip().endswith('*/'), lines[679][-40:]
assert lines[680].startswith('#define V5F_EKF_MAG_K_MAX'), lines[680]
assert lines[681].startswith(' * \u2605 \u5206\u5de5'), lines[681][:30]
assert lines[684].rstrip().endswith('*/'), lines[684][-40:]
assert lines[685].startswith('/* \u78c1\u504f\u822a'), lines[685][:30]

NEW = [
 '/* \u2605VER=82 K_MAX 0.05 -> 0.015\uff1a\u6309\u5b9e\u6d4b\u7684**\u975e\u78c1\u6f02\u79fb\u901f\u5ea6**\u628a\u73af\u8def\u653e\u6162\uff0c\u964d\u504f\u822a\u6296\u52a8\u3002',
 ' * \u4e00\u9636\u73af\u603b\u8bef\u5dee = d*tau + s*sqrt(1/(2*tau*f))\uff1a',
 ' *   d(\u975e\u78c1\u6f02\u79fb) = 0.060 \u5ea6/\u79d2(\u9759\u6b62, VER=73/74 \u4f11\u7720\u6bb5\u5b9e\u6d4b) ~ 0.18(\u5f3a\u6fc0\u52b1\u540e 15s \u7d2f\u79ef 2.72 \u5ea6)',
 ' *   s(\u78c1\u822a\u5411\u5355\u6b21\u566a\u58f0) = 0.4203 \u5ea6  ;  f = 196.6 Hz(\u78c1\u7269\u7406\u8f93\u51fa\u7387\u4e0a\u9650\uff0c\u518d\u5feb\u65e0\u610f\u4e49)',
 ' *   tau=0.102(k=0.05)  -> \u6f02\u79fb 0.012 + \u6296\u52a8 0.067 = 0.079',
 ' *   tau=0.339(k=0.015) -> \u6f02\u79fb 0.041 + \u6296\u52a8 0.036 = 0.077   <- \u672c\u503c',
 ' * \u89e3\u6790\u6700\u4f18 tau_opt=(s^2/(2*f*d^2))^(1/3)\uff1ad=0.06 -> 0.50 s\uff1bd=0.12 -> 0.32 s\u3002',
 ' * \u5373\u6f02\u79fb\u6b8b\u5dee\u8fdc\u5c0f\u4e8e\u6d4b\u91cf\u566a\u58f0\u672c\u8eab\uff0c\u73af\u8def\u53ef\u5927\u5e45\u653e\u6162\uff0c\u6362\u6765 sqrt(tau) \u7684\u6296\u52a8\u4e0b\u964d',
 ' * (\u5b9e\u6d4b\u9759\u6b62\u6bb5\u504f\u822a\u4e8c\u9636\u5dee\u5206 std\uff1aEKF 0.00545 vs \u65e7\u94fe 0.00051 \u5ea6/\u5e27^2)\u3002',
 ' * \u2605 \u5206\u5de5\uff1a**\u786c\u95e8**\u8d1f\u8d23\u6321\u65e0\u6548\u6837\u672c(|r| \u4e0a\u9650\u3001Bh/|B| \u4e0b\u9650)\uff0c**\u589e\u76ca**\u53ea\u8d1f\u8d23\u7cfb\u7edf\u5e26\u5bbd\u3002',
 ' *   \u4e0a\u4e00\u7248\u628a\u589e\u76ca\u538b\u5230 0.002(tau 1.4 s)\u518d\u53e0\u52a0 mag \u95e8\uff0c\u53ea\u6709 53% \u7684\u65f6\u95f4\u5f00\u7740\uff0c',
 ' *   \u7275\u5f15\u5f31\u5230\u7b49\u4e8e\u6ca1\u6709 \u2014\u2014 \u7528\u589e\u76ca\u53bb\u627f\u62c5\u7cfb\u7edf\u7a33\u5b9a\u6027\u662f\u9519\u7684\u3002',
 ' *   \u6700\u574f\u5355\u6b65 = k_cap*R_MAX = 0.015*150 = 2.25 \u5ea6(\u4e0e\u65e7 0.05*45 \u76f8\u540c)\uff0c',
 ' *   \u6b63\u5e38 r~0.5 \u5ea6 \u65f6\u6bcf\u6b65 0.0075 \u5ea6\u3002 */',
 '#define V5F_EKF_MAG_K_MAX        0.015f',
]

old = lines[672:685]
lines[672:685] = NEW
out = '\n'.join(lines)

assert 'V5F_EKF_MAG_K_MAX        0.015f' in out
assert out.count('#define V5F_EKF_MAG_K_MAX') == 1

# ---- 注释/字符串之外的字节必须全是 ASCII，且注释配平 ----
def check(t):
    i = 0; n = len(t); st = 'code'; ln = 1; bad = []
    while i < n:
        c = t[i]
        if st == 'code':
            if t.startswith('/*', i): st = 'comment'; i += 2; continue
            if t.startswith('//', i): st = 'line'; i += 2; continue
            if c == '"': st = 'str'; i += 1; continue
            if c == "'": st = 'chr'; i += 1; continue
            if ord(c) > 127: bad.append((ln, c))
        elif st == 'comment':
            if t.startswith('*/', i): st = 'code'; i += 2; continue
        elif st == 'line':
            if c == '\n': st = 'code'
        elif st in ('str', 'chr'):
            q = '"' if st == 'str' else "'"
            if c == '\\': i += 2; continue
            if c == q: st = 'code'
        if c == '\n': ln += 1
        i += 1
    return bad, st

bad, st = check(out)
assert not bad, ('仍有注释外高位字节', bad[:5])
assert st == 'code', ('结束时仍在 ' + st)
assert out.count('/*') == out.count('*/'), (out.count('/*'), out.count('*/'))

data = out.encode(ENC)          # 先编码，编码失败就不动文件
raw = open(P, 'rb').read()
shutil.copy2(P, P + '.bak_v82b')
open(P, 'wb').write(data)
os.utime(P, None)               # 必须刷新 mtime，否则 make 不重编

# ---- 回读校验 ----
back = open(P, 'rb').read()
assert back == data
print('OK  %s' % P)
print('    %d -> %d bytes;  lines %d -> %d' % (len(raw), len(data), len(old) + 791, len(lines) - 1))
for i in range(671, 687):
    print('%4d: %s' % (i + 1, lines[i].encode('gbk', 'replace').decode('gbk')))
