# -*- coding: utf-8 -*-
"""VER=47 -> 48：
 A) 解除地磁新息门造成的死锁：实测隐含偏航误差 mag_r 中位 112 度（p90 117、max 139.9），
    而 V5F_EKF_MAG_R_MAX_DEG = 45 度 -> 每一帧都被拒（mag_used 0%、mag gate 位 0%），
    地磁仍然一次都没能牵引。45 度门是为旧的 atan2 毛刺设的；现在是二维有界观测
    （|r| 天然被 |v0|=0.43 限住），大 |r| 只意味着"误差真的大"，不该锁死。
    已有三重保护足够：死点(|v|<0.30 归零) + k_cap(|K|<=0.05) + chi2 软加权。
    改 45 -> 150 度：覆盖实测最大 139.9，同时仍兜住 NaN/垃圾值。
    K=0.05、349 Hz 下 112 度误差约 20~40 个周期（0.1 s）收敛，不会疯转。
 B) 修 prop_row / stage 的上报：它们在 ekf_publish 之前已被清零，所以恒为 0
    （实测 prop_row/stage 100% 都是 0，等于没上报）。改成在清零前先存快照。
"""
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
n = 0

# ---- A) 门限：只改 tune 头 ----
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'(#define\s+V5F_EKF_MAG_R_MAX_DEG\s+)([0-9.]+)f', u)
assert m and m.group(2) == '45.0', 'MAG_R_MAX_DEG = %s' % (m and m.group(2))
old_line = u[m.start():u.index('\n', m.start())]
u = u[:m.start()] + m.group(1) + '150.0f' + u[m.end():]
print('  ok A1-V5F_EKF_MAG_R_MAX_DEG 45 -> 150')

# ---- B) prop_row / stage 快照 ----
a = "static uint8_t  s_prop_ok;"
assert t.count(a) == 1
t = t.replace(a, a + "\n"
              "static uint8_t  s_pub_row;      /* \u2605VER=48 \u6e05\u96f6\u524d\u7684 s_prop_row \u5feb\u7167\uff08\u4f9b\u4e0a\u62a5\uff09*/\n"
              "static uint8_t  s_pub_stage;    /* \u2605VER=48 \u6e05\u96f6\u524d\u7684 s_stage \u5feb\u7167\uff08\u4f9b\u4e0a\u62a5\uff09*/", 1)
n += 1; print('  ok B1-新增两个快照静态量')

a = "            s_prop_row = 0u;\n"      # 带换行：避开对齐块那行 's_prop_row = 0u; s_stage = ...'
assert t.count(a) == 1, 's_prop_row 锚点 %d 次' % t.count(a)
t = t.replace(a, "            s_pub_row = (uint8_t)s_prop_row;   /* \u2605VER=48 \u5148\u5feb\u7167\u518d\u6e05\u96f6 */\n" + a, 1)
n += 1; print('  ok B2-prop_row 快照')

a = "            s_stage = 0u;\n            ekf_publish(h);"
assert t.count(a) == 1
t = t.replace(a, "            s_pub_stage = (uint8_t)s_stage;   /* \u2605VER=48 \u5148\u5feb\u7167\u518d\u6e05\u96f6 */\n" + a, 1)
n += 1; print('  ok B3-stage 快照')

for fld, src in (('prop_row', 's_pub_row'), ('stage', 's_pub_stage')):
    mm = re.search(r'^([ \t]*)h->ekf\.%s[^\n]*$' % fld, t, re.M)   # 整行替换，格式容错
    assert mm, '未找到 publish %s 行' % fld
    t = t[:mm.start()] + '%s h->ekf.%s = %s;' % (mm.group(1), fld, src) + t[mm.end():]
n += 1; print('  ok B4-publish 改用快照')

dump(P, t, 'gbk', 'v48')

assert u.count('#define V5F_FW_VER        47u') == 1
u = u.replace('#define V5F_FW_VER        47u', '#define V5F_FW_VER        48u', 1)
dump(T, u, 'gbk', 'v48')
print('  ok A2-VER=48')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
rmax = re.search(r'#define\s+V5F_EKF_MAG_R_MAX_DEG\s+(\S+)', t2).group(1)
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==48', ver == 48), ('编辑数==4', n == 4),
      ('R_MAX=150', rmax.startswith('150')),
      ('prop_row 快照在', 's_pub_row = (uint8_t)s_prop_row;' in c),
      ('stage 快照在', 's_pub_stage = (uint8_t)s_stage;' in c),
      ('publish 用快照', 'h->ekf.prop_row = s_pub_row;' in c and 'h->ekf.stage = s_pub_stage;' in c),
      ('死点仍在 H_zero 前', m7.index('s_mag_bh < V5F_EKF_MAG_BH_MIN') < m7.index('H_zero(2u)')),
      ('二维观测仍在', 'r[0] = Bn[0] - b0x;' in m7),
      ('掩码仍 0x01C0', '0x01C0u, V5F_EKF_MAG_K_MAX' in m7),
      ('列数仍 133', nch == 133),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
print('旧行参考: %s' % old_line.strip()[:80])
