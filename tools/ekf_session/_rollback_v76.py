# -*- coding: utf-8 -*-
# 回滚到 VER=76：用 .bak_v77（VER=77 补丁前的状态，即 VER=76）逐字节恢复
import os, shutil, hashlib, re
ENC = 'gbk'
FILES = [r'h415-imu42605-\V5F\User\inc\v5f_tune.h',
         r'h415-imu42605-\V5F\User\src\proc_ekf.c']

def h(p):
    with open(p, 'rb') as f: return hashlib.md5(f.read()).hexdigest()[:12].upper(), os.path.getsize(p)

print('--- 回滚前 ---')
for p in FILES:
    m, s = h(p); print('  %-16s %7d B  md5 %s' % (os.path.basename(p), s, m))
print('--- 备份 .bak_v77（= VER=76 状态）---')
for p in FILES:
    b = p + '.bak_v77'
    assert os.path.exists(b), '缺少 ' + b
    m, s = h(b); print('  %-16s %7d B  md5 %s' % (os.path.basename(b), s, m))

for p in FILES:
    shutil.copy2(p + '.bak_v77', p)
    os.utime(p, None)                      # 刷新 mtime，逼构建重编

print('--- 回滚后校验 ---')
ok = True
for p in FILES:
    a, sa = h(p); b, sb = h(p + '.bak_v77')
    same = (a == b and sa == sb)
    ok = ok and same
    print('  %-16s %7d B  md5 %s  与 bak_v77 一致=%s  mtime %s'
          % (os.path.basename(p), sa, a, same,
             __import__('time').ctime(os.path.getmtime(p))))
assert ok

t = open(FILES[0], 'rb').read().decode(ENC)
e = open(FILES[1], 'rb').read().decode(ENC)
print()
print('  VER =', re.search(r'#define V5F_FW_VER\s+(\S+)', t).group(1))
for k in ('V5F_EKF_MAG_DEAD_DEG', 'V5F_EKF_MAG_BH_MIN', 'V5F_EKF_MAG_R_MAX_DEG'):
    mm = re.search(r'#define\s+%s\s+(\S+)' % k, t)
    print('  %-24s %s' % (k, mm.group(1) if mm else '(该常量已删除)'))
i = e.find('static void ekf_m7_mag'); j = e.find('/* M1 水平位置', i)
blk = e[i:j]
print('  M7 观测: %s' % ('1 维标量磁航向（VER=76）' if 'ekf_update(R, 1u, r,' in blk else '?'))
print('  M7 用 raw_f_mps2: %s' % ('raw_f_mps2' in blk))
print('  M7 带加计门（VER=77 才有）: %s' % ('V5F_EKF_TILT_AMAG_TOL' in blk))
print('  M7 掩码: %s' % ('0x0100u' if '0x0100u' in blk else '?'))
ob, cb = e.count('{'), e.count('}')
oc, cc = e.count('/*'), e.count('*/')
print('  结构 { } %d/%d   /* */ %d/%d' % (ob, cb, oc, cc))
assert ob == cb and oc == cc
print()
print('  期望 fw_tag = %d (VER=76)' % ((76 << 16) | (144 << 8) | 7))
print('PASS: 已回滚到 VER=76')
