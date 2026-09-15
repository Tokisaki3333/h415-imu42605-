# -*- coding: utf-8 -*-
"""VER=55 -> 56：翻转慢环符号。

数据（VER=55, fw_tag 3640071）：
  开机 idx8 mag_r = 1.16 度（EKF 一开始就在磁北）
  0.57 秒后（这段陀螺仅 0.3 dps，板子静止）mag_r 涨到 50.53 度
  -> 慢环把偏航**越推越偏**（正反馈），随后靠 VER=55 的 snap 拉回。
  => 慢环符号反了：r = v - v0 配 H = dv/dtheta，标准 Kalman 给出的方向与几何相反。
     snap 用几何精确角 + 自校验，不依赖 K，所以它是对的。

改：**只把喂给 ekf_update 的新息取反**；几何量 r 不动（snap 与 mag_rx/ry 诊断仍用它，
    所以上报口径不变，可以从 mag_dqz 的符号变化直接看出环路方向是否被纠正）。
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
a = "            st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4], &s_rej[3],"
assert t.count(a) == 1, 'ekf_update 锚点 %d 次' % t.count(a)
b = ("            /* \u2605VER=56 \u6162\u73af\u7b26\u53f7\u7ffb\u8f6c\uff1a\u5b9e\u6d4b\u5f00\u673a mag_r=1.16 \u5ea6\uff08\u5df2\u5728\u78c1\u5317\uff09\n"
     "             * \u5728\u9759\u6b62\u7684 0.57 \u79d2\u5185\u88ab\u63a8\u5230 50.5 \u5ea6 -> \u6b63\u53cd\u9988\uff0c\u8bf4\u660e\u6162\u73af\u65b9\u5411\u53cd\u4e86\u3002\n"
     "             * \u51e0\u4f55\u91cf r \u4e0d\u52a8\uff08snap \u4e0e mag_rx/ry \u8bca\u65ad\u4ecd\u7528\u5b83\uff09\uff0c\u53ea\u628a\u5582\u7ed9 K \u7684\u65b0\u606f\u53d6\u53cd\u3002 */\n"
     "            r[0] = -r[0]; r[1] = -r[1];\n"
     + a)
t = t.replace(a, b, 1)
dump(P, t, 'gbk', 'v56')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        55u') == 1
u = u.replace('#define V5F_FW_VER        55u', '#define V5F_FW_VER        56u', 1)
dump(T, u, 'gbk', 'v56')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==56', ver == 56),
      ('取反在 update 之前', m7.index('r[0] = -r[0]; r[1] = -r[1];') < m7.index('st = ekf_update(R, 2u, r,')),
      ('诊断在前（口径不变）', m7.index('s_mag_rx = r[0];') < m7.index('r[0] = -r[0];')),
      ('snap 仍用几何 r', 'dpsi = atan2f(crs, dt2);' in m7),
      ('大误差触发仍在', 'fabsf(r[0]) + fabsf(r[1]) > 0.5f' in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
