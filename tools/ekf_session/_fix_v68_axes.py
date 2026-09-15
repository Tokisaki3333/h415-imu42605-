# -*- coding: utf-8 -*-
"""VER=67 -> 68：修正磁力计轴错位 —— X<->Y 互换 + Z 取反（离散穷举判定）。

判据（离散穷举 48 种排列x符号，用"世界磁场矢量的集中度"打分；不做拟合）：
  0.9995  [1 0 2] +1+1-1   方位角 +14.32  磁倾角 +59.02   <- 最优
  0.9928  [1 2 0] +1+1+1
  0.9845  [2 0 1] +1+1+1
最优组合下世界矢量恒定：静止 p50 1.74 度 / 运动 p50 1.58 度；|Bw| 164.5~176.5（±3.5%）
=> 磁三轴相对 IMU 是 (x,y,z) -> (y,x,-z)：**X 与 Y 互换，且 Z 取反**。

VER=66 我改的"X、Y 都取反"是错的，本次替换掉。
残余：世界方位角 +14.32 vs 模型 -7.53，差 21.85 度为常量框架/磁偏角偏差 -> 交给对齐吸收，
      不由环路去追。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
a = "    mf[0] = -mf[0];\n    mf[1] = -mf[1];\n"
assert t.count(a) == 1, '取反两行锚点 %d' % t.count(a)
b = ("    /* \u2605VER=68 \u78c1\u529b\u8ba1\u8f74\u9519\u4f4d\u4fee\u6b63\uff1a(x,y,z) -> (y,x,-z)\u3002\n"
     "     * \u79bb\u6563\u7a77\u4e3e 48 \u79cd\u6392\u5217x\u7b26\u53f7\uff0c\u7528\u4e16\u754c\u78c1\u573a\u77e2\u91cf\u7684\u96c6\u4e2d\u5ea6\u6253\u5206\uff1a\n"
     "     *   [1 0 2] +1+1-1 -> 0.9995\uff08\u6700\u4f18\uff09\uff1b\u6b21\u4f18 [1 2 0] 0.9928\u3001[2 0 1] 0.9845\n"
     "     *   \u6700\u4f18\u7ec4\u5408\u4e0b\u4e16\u754c\u77e2\u91cf\u6052\u5b9a\uff1a\u9759\u6b62 p50 1.74 \u5ea6\u3001\u8fd0\u52a8 p50 1.58 \u5ea6\uff0c\n"
     "     *   |Bw| 164.5~176.5\uff08\u00b13.5%\uff09\u3002=> X \u4e0e Y \u4e92\u6362\uff0c\u4e14 Z \u53d6\u53cd\u3002\n"
     "     * \uff08VER=66 \u7684\u201cX\u3001Y \u90fd\u53d6\u53cd\u201d\u662f\u9519\u7684\uff0c\u672c\u6b21\u66ff\u6362\uff09*/\n"
     "    {\n"
     "        float mt = mf[0];\n"
     "        mf[0] =  mf[1];\n"
     "        mf[1] =  mt;\n"
     "        mf[2] = -mf[2];\n"
     "    }\n")
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}')
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v68'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        67u') == 1
u = u.replace('#define V5F_FW_VER        67u', '#define V5F_FW_VER        68u', 1)
shutil.copy2(T, T + '.bak_v68'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==68', ver == 68),
      ('互换+Z反 在', 'mf[0] =  mf[1];' in m7 and 'mf[2] = -mf[2];' in m7),
      ('旧取反已去', 'mf[0] = -mf[0]' not in m7),
      ('在观测之前', m7.index('mf[0] =  mf[1];') < m7.index('r[0] = Bn[0] - b0x;')),
      ('死区仍在', 'if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {' in m7),
      ('时间窗 snap 仍在', 's_boot_t < V5F_EKF_MAG_ANCHOR_TS' in m7),
      ('二维水平观测仍在', 'r[0] = Bn[0] - b0x;' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('新列仍在', all('g_v5f_hold.ekf.%s' % f in open(U+r'\src\SPI_rx.c','rb').read().decode('gbk') for f in ('mag_vx','mag_yawpre'))),
      ('列数 144', nch == 144),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
