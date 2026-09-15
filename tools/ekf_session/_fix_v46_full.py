# -*- coding: utf-8 -*-
"""VER=45 -> 46 一次做完：
  A) M7 观测改为「以重力方向为法向的平面上的二维投影」
  B) 追加 5 列判断效果必需的上报（128..132）
  C) 同步读取端与列数

关键列：
  128 mag_rx / 129 mag_ry  新息二维分量（判断残差到底是偏航误差还是倾角伪影）
  130/131/132 mag_dqx/dqy/dqz  本次更新**实际注入**到姿态三轴的角度(度)
      -> 这是"效果"的直接证据：二维观测若正确，修正应主要落在偏航；
         若大量落在倾角，说明掩码/雅可比还有问题。
  沿用：118 mag_bh = |v| 确定程度；119 mag_r = 隐含偏航误差(度)；120 mag_used。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
U = R + r'\V5F\User'
P = U + r'\src\proc_ekf.c'
H = U + r'\inc\SPI_rx.h'
S = U + r'\src\SPI_rx.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


NEWF = [('mag_rx', 'float'), ('mag_ry', 'float'), ('mag_dqx', 'float'),
        ('mag_dqy', 'float'), ('mag_dqz', 'float')]

t = open(P, 'rb').read().decode('gbk')
tn = open(T, 'rb').read().decode('gbk')
for mac in ('V5F_EKF_DIP_TAN', 'V5F_MAG_DECL_RAD', 'V5F_EKF_MAG_SIG_RAD',
            'V5F_EKF_NIS_MAX_2', 'V5F_EKF_MAG_R_MAX_DEG', 'V5F_EKF_MAG_K_MAX'):
    assert re.search(r'#define\s+' + mac + r'\b', tn), '缺少宏 ' + mac
n = 0

# ---- 1) 局部数组 + 新增静态量 ----
a = "    float R[1], r[1], Rt[3][3], Bn[3], mf[3], hh, fhb2;"
assert t.count(a) == 1
t = t.replace(a, "    float R[4], r[2], Rt[3][3], Bn[3], mf[3], fhb2;", 1)
n += 1; print('  ok 1-局部数组 R[4]/r[2]，去 hh')

a = "static float    s_mag_fhb;"
assert t.count(a) == 1
t = t.replace(a, a + "\n"
              "static float    s_mag_rx, s_mag_ry;      /* \u2605VER=46 \u4e8c\u7ef4\u65b0\u606f\u5206\u91cf */\n"
              "static float    s_mag_dqx, s_mag_dqy, s_mag_dqz;  /* \u2605VER=46 \u672c\u6b21\u5b9e\u9645\u6ce8\u5165\u7684\u59ff\u6001\u4fee\u6b63(\u5ea6) */", 1)
n += 1; print('  ok 2-新增 5 个静态量')

# ---- 3) 删除 fhb 硬门 ----
a = t.index("    if (fhb2 < V5F_EKF_MAG_BHB_MIN")
b = t.index("    }\n", a) + len("    }\n")
blk = t[a:b]
assert 'return;' in blk and len(blk) < 400
t = t[:a] + t[b:]
n += 1; print('  ok 3-删 fhb 硬门(%d 字节)' % len(blk))

# ---- 4) 替换观测 + 新息/更新 ----
i0 = t.index("    hh = atan2f(Bn[0], Bn[1]);")
i1 = t.index("    r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);", i0)
j1 = t.index("\n}\n", i1) + 2   # +2：把原函数收尾的 '}' 也吃掉（new 自己以 '}' 结尾）
old = t[i0:j1]
assert 'bh2' in old and '0x0100u, V5F_EKF_MAG_K_MAX' in old and len(old) < 3000
new = """    /* \u2605VER=46 \u4e8c\u7ef4\u6c34\u5e73\u6295\u5f71\u89c2\u6d4b\uff08\u6cd5\u5411 = \u91cd\u529b\u65b9\u5411\uff09\u3002
     *   v  = (Bn[0], Bn[1])  \u5b9e\u6d4b\u78c1\u573a\u8f6c\u5230\u5bfc\u822a\u7cfb\u540e\u7684\u6c34\u5e73\u4e8c\u7ef4\u77e2\u91cf
     *   v0 = (b0x, b0y)      \u5f53\u5730\u78c1\u573a\u7684\u6c34\u5e73\u4e8c\u7ef4\u77e2\u91cf\uff08\u5e38\u91cf\uff09
     *   \u65b9\u5411 -> \u504f\u822a\u5e94\u5f53\u5728\u54ea\uff1b\u5927\u5c0f |v| -> \u786e\u5b9a\u7a0b\u5ea6\u3002
     * \u5168\u7a0b\u65e0\u9664\u6cd5\uff0c\u4e0d\u53ef\u80fd\u50cf atan2 \u90a3\u6837\u628a\u9000\u5316\u653e\u5927\uff1b
     * \u504f\u822a\u7075\u654f\u5ea6\u5c31\u662f |v0| = cos(\u78c1\u503e\u89d2)\uff0c\u4e0e\u673a\u4f53\u59ff\u6001\u65e0\u5173\u3002
     * \u63a9\u7801 0x01C0\uff1aH \u7684\u503e\u89d2\u5217\u975e\u96f6\uff0c\u65b0\u606f\u6309\u51e0\u4f55\u5206\u644a\u5230\u4e09\u8f74\u3002*/
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);                    /* Bn = R(q) f\uff1a\u5bfc\u822a\u7cfb\u78c1\u573a */
    {
        float ci = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);
        float si = V5F_EKF_DIP_TAN * ci;
        float sd = sinf(V5F_MAG_DECL_RAD), cd = cosf(V5F_MAG_DECL_RAD);
        float b0x = ci * sd, b0y = ci * cd, b0z = -si;
        float sig2 = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;
        float rn;
        s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);   /* = |v|\uff1a\u786e\u5b9a\u7a0b\u5ea6 */
        H_zero(2u);
        s_H[0][6] = 0.0f;  s_H[0][7] =  b0z;  s_H[0][8] = -b0y;
        s_H[1][6] = -b0z;  s_H[1][7] = 0.0f;  s_H[1][8] =  b0x;
        r[0] = Bn[0] - b0x;
        r[1] = Bn[1] - b0y;
        s_mag_rx = r[0];
        s_mag_ry = r[1];
        R[0] = sig2; R[1] = 0.0f; R[2] = 0.0f; R[3] = sig2;
        rn = sqrtf(r[0]*r[0] + r[1]*r[1]);
        s_mag_r = (s_mag_bh > 1e-3f) ? (rn / s_mag_bh) * RAD2DEG : 0.0f;
        if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
        } else {
            st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4], &s_rej[3],
                            0x01C0u, V5F_EKF_MAG_K_MAX);
            if (st == 0u) {
                s_gate_bits |= V5F_EKF_GB_MAG;
                s_mag_used = 1u;
                s_mag_dqx = s_dx[IX_Q + 0] * RAD2DEG;
                s_mag_dqy = s_dx[IX_Q + 1] * RAD2DEG;
                s_mag_dqz = s_dx[IX_Q + 2] * RAD2DEG;
            }
        }
    }
}
"""
nb, nc2 = new.count('{'), new.count('}')
pb, pc2 = new.count('('), new.count(')')
print('  new 计数: { %d  } %d   ( %d  ) %d' % (nb, nc2, pb, pc2))
if nb != nc2 or pb != pc2:
    for ln in new.split('\n'):
        d = ln.count('{') - ln.count('}'), ln.count('(') - ln.count(')')
        if d != (0, 0):
            print('    偏差%s  %s' % (d, ln[:90]))
# new 以 ekf_m7_mag 的收尾 '}' 结束，它的开括号在字符串外，故 closes = opens + 1
assert nb + 1 == nc2 and pb == pc2, 'new 块括号不平衡: {%d}%d (%d)%d' % (nb, nc2, pb, pc2)
t = t[:i0] + new + t[j1:]
n += 1; print('  ok 4-二维水平投影观测 + 5 列赋值')

# ---- 5) publish ----
m = re.search(r'^([ \t]*)h->ekf\.mag_fhb[ \t]*=[^\n]*$', t, re.M)
assert m, '未找到 publish mag_fhb 行'
t = t[:m.end()] + ''.join(
    '\n%s h->ekf.%s = %s;' % (m.group(1), f, {'mag_rx': 's_mag_rx', 'mag_ry': 's_mag_ry',
                                             'mag_dqx': 's_mag_dqx', 'mag_dqy': 's_mag_dqy',
                                             'mag_dqz': 's_mag_dqz'}[f]) for f, _ in NEWF) + t[m.end():]
n += 1; print('  ok 5-publish +5')
dump(P, t, 'gbk', 'v46h')

# ---- 6) 结构体 ----
h = open(H, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)float[ \t]+mag_fhb[^\n]*$', h, re.M)
assert m, '未找到 struct mag_fhb 字段'
h = h[:m.end()] + ''.join('\n%s %-8s %s;   /* VER=46 */' % (m.group(1), ty, f) for f, ty in NEWF) + h[m.end():]
dump(H, h, 'gbk', 'v46h')
n += 1; print('  ok 6-struct +5')

# ---- 7) 通道 + JF_CH_NUM ----
s = open(S, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)ch\[c\+\+\][ \t]*=[ \t]*g_v5f_hold\.ekf\.mag_fhb[^\n]*$', s, re.M)
assert m, '未找到 mag_fhb 通道行'
s = s[:m.end()] + ''.join('\n%s ch[c++] = g_v5f_hold.ekf.%s;   /* VER=46 */' % (m.group(1), f)
                          for f, _ in NEWF) + s[m.end():]
mm = re.search(r'#define[ \t]+JF_CH_NUM[ \t]+(\d+)u', s)
assert mm and mm.group(1) == '128', 'JF_CH_NUM=%s' % (mm and mm.group(1))
s = s[:mm.start(1)] + '133' + s[mm.end(1):]
dump(S, s, 'gbk', 'v46h')
n += 1; print('  ok 7-通道 +5, JF_CH_NUM=133')

# ---- 8) VER ----
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        45u') == 1
u = u.replace('#define V5F_FW_VER        45u', '#define V5F_FW_VER        46u', 1)
dump(T, u, 'gbk', 'v46h')
n += 1; print('  ok 8-VER=46')

# ---- 校验 ----
c = open(P, 'rb').read().decode('gbk')
h2 = open(H, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
CK = [('VER==46', ver == 46), ('编辑数==8', n == 8),
      ('无 atan2 方位角', 'hh = atan2f(Bn[0], Bn[1])' not in c),
      ('无 bh2 除法', 'Bn[0]*Bn[2]/bh2' not in c),
      ('无 fhb 硬门', 'if (fhb2 < V5F_EKF_MAG_BHB_MIN' not in c),
      ('无 fhb 加权', 'V5F_EKF_MAG_FERR / sqrtf(fhb2)' not in c),
      ('二维新息', 'r[0] = Bn[0] - b0x;' in c and 'r[1] = Bn[1] - b0y;' in c),
      ('H 偏航列', 's_H[0][8] = -b0y;' in c and 's_H[1][8] =  b0x;' in c),
      ('掩码 0x01C0', '0x01C0u, V5F_EKF_MAG_K_MAX' in c),
      ('m=2', 'ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4]' in c),
      ('dq 捕获在 st==0 内', 's_mag_dqz = s_dx[IX_Q + 2] * RAD2DEG;' in c),
      ('publish +5', all('h->ekf.%s =' % f in c for f, _ in NEWF)),
      ('struct +5', all(re.search(r'\b%s\b' % f, h2) for f, _ in NEWF)),
      ('channel +5', all('g_v5f_hold.ekf.%s' % f in s2 for f, _ in NEWF)),
      ('prop 自锁修复仍在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('JF_CH_NUM==133', nch == 133),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=46, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
print('新列 128 mag_rx 129 mag_ry 130 mag_dqx 131 mag_dqy 132 mag_dqz')

# ---- 9) 读取端 ----
J = R + r'\tools\calib\jf_load.py'
j = open(J, 'rb').read().decode('utf-8')
if 'CH_133' not in j:
    j = j.replace("CH_127 = dict(CH_122)", "CH_127 = dict(CH_122)", 1)
    add = ("\nCH_133 = dict(CH_127)\n"
           "CH_133.update({'ekf_mag_rx': 128, 'ekf_mag_ry': 129, 'ekf_mag_dqx': 130,\n"
           "               'ekf_mag_dqy': 131, 'ekf_mag_dqz': 132})\n")
    j = j.replace("\nCH_BY_NCH = {", add + "\nCH_BY_NCH = {", 1)
    j = j.replace('128: CH_127}', '128: CH_127, 133: CH_133}', 1)
    j = j.replace('    if nch == 128:\n        return CH_127\n',
                  '    if nch == 128:\n        return CH_127\n'
                  '    if nch == 133:\n        return CH_133\n', 1)
    open(J, 'wb').write(j.encode('utf-8'))
j2 = open(J, 'rb').read().decode('utf-8')
assert 'CH_133 = dict(CH_127)' in j2 and '133: CH_133' in j2 and 'if nch == 133' in j2
print('  jf_load.py: CH_133 ok')
K = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
k = open(K, 'rb').read().decode('utf-8', 'ignore')
open(K, 'wb').write(re.sub(r'assert n == \d+', 'assert n == 133', k).encode('utf-8'))
print('  count_cols.py assert:',
      re.findall(r'assert n == (\d+)', open(K, 'rb').read().decode('utf-8', 'ignore')))
