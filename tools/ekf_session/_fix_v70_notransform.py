# -*- coding: utf-8 -*-
"""VER=69 -> 70：删除 EKF 里所有额外的磁轴变换，直接用 mag.f。

自下而上的证据（不拟合，全部用实测）：
  底层 V5F_MAG_A_INIT 已做 (x,y,z)->(-y,-x,+z)（主元素: f0<-raw_y(-), f1<-raw_x(-), f2<-raw_z(+)）
  用底层标定后的 mag.f + 旧链姿态投影世界磁场：
      不加任何变换   集中度 0.9997   抖动 静止p90 0.88 度 / 运动p90 2.56 度   <- 最好
      再加 y,x,-z    集中度 0.9641   抖动 4.66 / 31.58                        <- 我 VER=68（错）
      再加 -x,-y,z   集中度 0.9429   抖动 2.31 / 36.97                        <- 我 VER=66（错）
  => mag.f 已是正确机体矢量；任何 EKF 内变换都在破坏它。
  PC 仿真也早写明：未修改的 mag.f 下 mag_r 出窗 p50 5.57 度、97.7% 落在 12 度死区内。

注：我 VER=67 的补丁实际没打上（锚点用了 V5F_EKF_NIS_MAX_MAG，而代码里是 V5F_EKF_NIS_MAX_2），
    所以"关软加权"这条还没生效，本次不动它，只删轴变换。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
# 删除 VER=68 的互换+Z反 块（以及 VER=66 的取反若还在）
a = t.find("    /* \u2605VER=68 \u78c1\u529b\u8ba1\u8f74\u9519\u4f4d\u4fee\u6b63")
assert a > 0, '未找到 VER=68 注释'
b = t.index("    }\n", t.index("mf[2] = -mf[2];", a)) + len("    }\n")
blk = t[a:b]
assert 'mf[0] =  mf[1];' in blk and 'mf[2] = -mf[2];' in blk and len(blk) < 2000
new = ("    /* \u2605VER=70 \u5df2\u5220\u9664 EKF \u5185\u6240\u6709\u78c1\u8f74\u53d8\u6362\uff1a\u5e95\u5c42 V5F_MAG_A_INIT \u5df2\u7ecf\u505a\u4e86\n"
       "     * (x,y,z)->(-y,-x,+z)\uff0c\u5e95\u5c42\u6807\u5b9a\u540e\u7684 mag.f \u5c31\u662f\u6b63\u786e\u673a\u4f53\u77e2\u91cf\uff1a\n"
       "     * \u7528\u5b83\u6295\u5f71\u51fa\u7684\u4e16\u754c\u78c1\u573a\u6052\u5b9a\u5230 \u9759\u6b62 0.88 \u5ea6 / \u8fd0\u52a8 2.56 \u5ea6\uff1b\n"
       "     * \u518d\u52a0\u4efb\u4f55\u53d8\u6362\u90fd\u4f1a\u628a\u5b83\u5f04\u574f\uff08\u96c6\u4e2d\u5ea6 0.9997 -> 0.94~0.97\uff0c\u8fd0\u52a8\u6296\u52a8 2.6 -> 28~37 \u5ea6\uff09\u3002 */\n")
t = t[:a] + new + t[b:]
assert 'mf[0] =  mf[1];' not in t and 'mf[0] = -mf[0];' not in t
assert t.count('{') == t.count('}')
assert re.search(r'ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_2, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v70'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        69u') == 1
u = u.replace('#define V5F_FW_VER        69u', '#define V5F_FW_VER        70u', 1)
shutil.copy2(T, T + '.bak_v70'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==70', ver == 70),
      ('轴变换全删', 'mf[0] =  mf[1];' not in m7 and 'mf[0] = -mf[0];' not in m7),
      ('mf 仍是原样拷贝', 'mf[i] = h->mag.f[i];' in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7),
      ('死区 12 在', 'if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {' in m7),
      ('时间窗 snap 在', 's_boot_t < V5F_EKF_MAG_ANCHOR_TS' in m7),
      ('k_cap 掩码 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_2, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('列数 144', nch == 144),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
