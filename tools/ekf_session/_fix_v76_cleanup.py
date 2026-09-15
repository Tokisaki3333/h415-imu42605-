# -*- coding: utf-8 -*-
# VER=76 收尾：删掉 b0z / si 两个 set-but-unused 变量（新观测按构造不再需要它们）
import os, shutil
ENC = 'gbk'
p = r'h415-imu42605-\V5F\User\src\proc_ekf.c'

def load(q):
    with open(q, 'rb') as f: return f.read().decode(ENC)
def save(q, s, tag):
    d = s.encode(ENC)
    shutil.copy2(q, q + '.bak_' + tag)
    with open(q, 'wb') as f: f.write(d)
    with open(q, 'rb') as f: assert f.read().decode(ENC) == s
    return len(d)
def sub1(s, a, b, what):
    n = s.count(a); assert n == 1, '[%s] 命中 %d 次' % (what, n)
    return s.replace(a, b, 1)

s = load(p)
s = sub1(s, '    float fhb2, ci, si, b0x, b0y, b0z, sig2;',
            '    float fhb2, ci, b0x, b0y, sig2;', 'decl')
s = sub1(s, '    ci = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);\n'
            '    si = V5F_EKF_DIP_TAN * ci;\n'
            '    b0x = ci * sinf(V5F_MAG_DECL_RAD);\n'
            '    b0y = ci * cosf(V5F_MAG_DECL_RAD);\n'
            '    b0z = -si;\n',
            '    ci = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);\n'
            '    b0x = ci * sinf(V5F_MAG_DECL_RAD);\n'
            '    b0y = ci * cosf(V5F_MAG_DECL_RAD);\n', 'const')
n = save(p, s, 'v76b')
os.utime(p, None)                      # 刷新 mtime，逼构建重编

s2 = load(p)
i = s2.find('static void ekf_m7_mag'); j = s2.find('/* M1 水平位置', i)
blk = s2[i:j]
print('proc_ekf.c %d 字节' % n)
print('M7 内 b0z 出现 %d 次 (应 0, 除注释外)  si 单独赋值 %d 次' %
      (blk.count('b0z = '), blk.count('si = ')))
assert 'b0z = ' not in blk and 'si = ' not in blk
ob, cb = s2.count('{'), s2.count('}')
oc, cc = s2.count('/*'), s2.count('*/')
print('结构  { } %d/%d   /* */ %d/%d' % (ob, cb, oc, cc))
assert ob == cb and oc == cc
print('mtime:'); print(os.path.getmtime(p))
print('PASS: b0z/si 已清除')
