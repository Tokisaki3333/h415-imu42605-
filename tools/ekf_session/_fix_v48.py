# -*- coding: utf-8 -*-
"""VER=47 -> 48：重力(M6)注入掩码去掉偏航位 bit8（0x7FC0 -> 0x7EC0）。

--- 实测（VER=47，fw_tag 3111431，76620 帧 / 9.54 s）---
  ekf_q 在动（死循环已解）；tilt 门 99.8%（原 0%）；mag 门 100%；mag_used 100%；chi2 0.6%。
  r_leg（旧链）稳如磐石 163.7 度（p90 164.2）。
  EKF 与旧链的偏航差 dyaw 双峰：
     56% 的帧 dyaw = -164.0 度，此时 r_ekf p50 = 0.04 度   <- **EKF 已锁在磁北**（要的牵引成立了）
     44% 的帧 dyaw =  -39.0 度，此时 r_ekf p50 = 123.1 度  <- 被"打飞"再拉回
  => 是极限环，不是测量翻转：地磁把偏航拉到磁北，别的东西又把它打飞 125~164 度。
  对齐没有重跑（p_yy 从未回到 P0），排除。

--- 发动机 ---
p_yy 在 1e-12（协方差塌缩 -> K~1e-8，地磁完全没有权限）与 0.08（K 被 MAG_K_MAX=0.05
顶死）之间摆动，环路没有稳定工作点 —— 这就是"死点"的真正形态。

--- 本轮改动：掐掉重力对偏航的权限 ---
M6 是"重力方向"观测，**重力不含任何偏航信息**；但它的注入掩码 0x7FC0 含 bit8(偏航)，
H 的第 6..8 列（姿态三轴）也都在。VER=46 刚把它的门从 0% 打开到 99.8%，
于是它成了新的强偏航激励源：k_cap=0.010、349 Hz -> tau=0.29 s，
一个持续存在的倾角模型残差足以每秒拽偏航若干度，累积到上百度的"打飞"。
改为 0x7EC0（掩掉 bit8）：重力只能修横滚/俯仰，永远不能修偏航。

保留 VER=45 的地磁"框架偏置"牵引逻辑：实测它已经把四元数牵到了磁北
（56% 的帧 r=0.04 度），这正是之前一直缺的那条牵引；先不动它。
若去掉 bit8 后那 44% 的大残差仍在，则下一个嫌疑就是 VER=45 的偏置积分本身，
届时回退成硬门。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
old = 'ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0], 0x7FC0u, V5F_EKF_TILT_K_MAX);'
assert t.count(old) == 1, 'M6 注入掩码锚点匹配 %d 次' % t.count(old)
new = ('/* \u2605VER=48 \u63a9\u7801 0x7FC0 -> 0x7EC0\uff1a\u53bb\u6389 bit8(\u504f\u822a)\u3002\n'
       '             * \u91cd\u529b\u65b9\u5411\u4e0d\u542b\u4efb\u4f55\u504f\u822a\u4fe1\u606f\uff0c\u8ba9\u5b83\u53bb\u52a8\u504f\u822a\u7269\u7406\u4e0a\u5c31\u662f\u9519\u7684\uff1b\n'
       '             * VER=46 \u628a tilt \u95e8\u4ece 0% \u6253\u5f00\u5230 99.8% \u540e\uff0c\u5b83\u6210\u4e86\u65b0\u7684\u5f3a\u504f\u822a\u6fc0\u52b1\u6e90\n'
       '             * \uff08k_cap 0.010 / 349 Hz -> tau 0.29 s\uff09\uff0c\u628a\u5df2\u7ecf\u88ab\u5730\u78c1\u7275\u5230\u78c1\u5317\u7684\u504f\u822a\n'
       '             * \u53cd\u590d\u6253\u98de 125~164 \u5ea6\uff08\u5b9e\u6d4b dyaw \u53cc\u5cf0\uff09\u3002 */\n'
       '        st = ekf_update(R, 3u, r, V5F_EKF_NIS_MAX_3, &s_nis[3], &s_rej[0], 0x7EC0u, V5F_EKF_TILT_K_MAX);')
t = t.replace(old, new, 1)
assert '0x7FC0u' not in t
dump(P, t, 'gbk', 's2t')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        47u') == 1
u = u.replace('#define V5F_FW_VER        47u', '#define V5F_FW_VER        48u', 1)
dump(T, u, 'gbk', 's2t')

c = open(P, 'rb').read().decode('gbk')
h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==48', ver == 48),
      ('0x7EC0 在', '0x7EC0u' in c),
      ('0x7FC0 已无', '0x7FC0u' not in c),
      ('M7 掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in c),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/')),
      ('括号差仍 -4', c.count('(') - c.count(')') == -4)]
h.encode('gbk'); c.encode('gbk')
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=48, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
