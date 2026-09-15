# -*- coding: utf-8 -*-
"""VER=58(坏的) -> 59：回退到 VER=57 基线（= 投影到重力法向平面的二维观测 + 精确角 snap），
并清掉编译告警里的多余声明。

基线说明（用户指示）：
  "投影到重力法向量平面"的二维观测 = 最好结果（v = (Bn[0],Bn[1]) 投到水平面），
  唯一遗留问题是"假点"（180 度鞍点）—— 由精确角 snap 处理。
  => 不要 VER=51-A 的"沿 v0 投影"，不要 VER=56 的符号取反。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

# 1) 回退到 VER=57（.bak_v58 是补丁前的磁盘内容）
shutil.copy2(P + '.bak_v58', P)
c = open(P, 'rb').read().decode('gbk')
m = re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk'))
print('回退后: VER(tune)=%s  { }平衡=%s  括号差=%d (历史遗留 -4)' %
      (m.group(1), c.count('{') == c.count('}'), c.count('(') - c.count(')')))
assert c.count('{') == c.count('}') and c.count('(') - c.count(')') == -4

# 2) 确认 ekf_update 调用完整（两行）
ok = re.search(r'st = ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis\[4\], &s_rej\[3\],\s*\n\s*0x0100u, V5F_EKF_MAG_K_MAX\);', c)
print('ekf_update 两行完整: %s' % bool(ok))
assert ok, 'ekf_update 调用不完整'

# 3) 清掉多余的 uint32_t nq;（编译告警）
n_nq = len(re.findall(r'\bnq\b', c))
print('nq 出现次数 %d' % n_nq)
if n_nq == 1:
    c2 = re.sub(r'^[ \t]*uint32_t nq;[ \t]*\r?\n', '', c, flags=re.M)
    assert len(re.findall(r'\bnq\b', c2)) == 0
    c = c2
    print('  已删除多余声明')
else:
    print('  nq 使用中或有多次，跳过')

# 4) VER 59
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        58u') == 1
u = u.replace('#define V5F_FW_VER        58u', '#define V5F_FW_VER        59u', 1)
shutil.copy2(T, T + '.bak_v59'); open(T, 'wb').write(u.encode('gbk'))
shutil.copy2(P, P + '.bak_v59'); open(P, 'wb').write(c.encode('gbk'))

# 5) 复核
c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==59', ver == 59),
      ('update 两行完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('无 rp 残留', 'rp[' not in m7),
      ('无符号取反', 'r[0] = -r[0]' not in m7),
      ('无 v0 投影', 'kk2' not in m7 and 'kk ' not in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7 and 'r[1] = Bn[1] - b0y;' in m7),
      ('精确角 snap 在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('大误差触发在', 'fabsf(r[0]) + fabsf(r[1]) > 0.5f' in m7),
      ('自校验在', 'if (e1 > e0) {' in m7),
      ('H 三列在', 's_H[1][8] =  b0x;' in m7),
      ('死点 0.12', re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t2).group(1).startswith('0.12')),
      ('nq 已清', 'nq' not in c),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差仍 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
