# -*- coding: utf-8 -*-
"""VER=38 -> 39：撤掉单周期修正限幅（把它置大=关闭）。

为什么必须撤：限幅只回滚状态、不回滚协方差 ->
  P[8][8] 越滚越小 -> K 越大 -> 越容易触发限幅 -> 每周期都把修正顶到限幅值执行。
若修正方向有一点点错，它就变成"每周期以最大速率(17度/s)朝错误方向走" = 恒速漂移，
即实测的崩溃性漂移。**限幅把有界误差变成了无界漂移**，这是设计错误。
正确限带宽的方式是**降频**（相关观测不该以 349 Hz 满增益进姿态），下一版改成抽取。
"""
import re
import shutil

T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
u = open(T, 'rb').read().decode('gbk')

DOC = """   /* 暂时置大 = 关闭。
 * 为什么撤：限幅只回滚状态、不回滚协方差 -> P 越来越小 -> K 越大 -> 越容易触发
 * 限幅 -> 每周期都把修正顶到限幅值执行。若修正方向有一点点错，它就变成
 * "每周期以最大速率朝错误方向走"= **恒速漂移**（实测崩溃性漂移）。
 * 限幅把有界误差变成了无界漂移，这是设计错误，不是参数问题。
 * 正确限带宽的方式是**降频**：相关观测不该以 349 Hz 满增益进姿态，
 * 下一版改成对 M6/M7 做抽取。 */"""

for k in ('V5F_EKF_YAW_STEP_MAX_DEG', 'V5F_EKF_TILT_STEP_MAX_DEG'):
    m = re.search(r'#define ' + k + r'\s+[\d.]+f', u)
    assert m, k
    u = u[:m.start()] + '#define ' + k + ' 1000.0f' + DOC + u[m.end():]
    print('  %s -> 1000.0f (关闭)' % k)

assert u.count('#define V5F_FW_VER        38u') == 1
u = u.replace('#define V5F_FW_VER        38u', '#define V5F_FW_VER        39u', 1)
assert u.count('/*') == u.count('*/')
data = u.encode('gbk')
shutil.copy2(T, T + '.bak_s2i')
open(T, 'wb').write(data)

c = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', c).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c',
                         'rb').read().decode('gbk')).group(1))
for k in ('V5F_EKF_YAW_STEP_MAX_DEG', 'V5F_EKF_TILT_STEP_MAX_DEG'):
    print('  %s = %s' % (k, re.search(r'#define ' + k + r'\s+([\d.]+)f', c).group(1)))
print('  VER=%d  fw_tag = %d' % (ver, (ver << 16) | (nch << 8) | 1 | 2 | 4))
