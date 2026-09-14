# -*- coding: utf-8 -*-
"""并入 VER=21：Q_vv 加上"倾角不确定度经重力泄漏进水平加速度"这一项。

实测（VER=20，平移段 1.9~4.0 s）：真实速度约 1 m/s，而 sigma_vel_h 只从 0.012
长到 0.071 —— 差 14 倍。机理：EKF 算 a_nav = R(q)·(f-ba) + g，姿态的倾角不确定度
sigma_tilt 会让重力的一部分漏进水平加速度：a_h,err ≈ g·sigma_tilt。
sigma_tilt 实测 p50 0.63 度 -> 0.108 m/s^2，几秒内就该把 sigma_v 推到 0.2~0.5 m/s。
和 M7 的自适应 R 是同一个道理：**耦合项要进协方差，不然就是撒谎。**
"""
import shutil
import sys

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
t = open(P, 'rb').read().decode('gbk')

a = """    sa2 = V5F_EKF_SIG_A_MPS2 * V5F_EKF_SIG_A_MPS2;
    /* 姿态过程噪声 = 白噪声 + **加速度整流**。"""
b = """    sa2 = V5F_EKF_SIG_A_MPS2 * V5F_EKF_SIG_A_MPS2;
    /* 速度过程噪声也要加上**重力经倾角不确定度的泄漏**：
     * a_nav = R(q)·(f-ba) + g，姿态倾角不确定度 sigma_tilt 会让重力的一部分漏进
     * 水平加速度，a_h,err ≈ g·sigma_tilt。实测 sigma_tilt p50 0.63 度 -> 0.108 m/s^2。
     * 不加这一项，平移时 sigma_vel_h 只长到 0.07 m/s 而真实速度约 1 m/s —— 差 14 倍，
     * 下游会去信一个错了 1 m/s 的速度。（用 s_P 里的倾角方差，即把耦合项算进协方差。） */
    {
        float st2 = s_P[6][6] + s_P[7][7];
        float gst = V5F_EKF_G_MPS2 * sqrtf(st2);
        sa2 += gst * gst;
    }
    /* 姿态过程噪声 = 白噪声 + **加速度整流**。"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1n')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: Q_vv 加倾角泄漏项')

# ---- 顺便把走动段各子判据的实际作用统计出来 ----
import re
import numpy as np
SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
x = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
dt = x[:, 25].astype(np.float64) * 1e-6
tt = np.cumsum(dt)
gb = x[:, 103].astype(np.int32)
isst = x[:, 8] > 0.5
eac = x[:, 81]
alin2 = x[:, 10]**2 + x[:, 11]**2 + x[:, 12]**2
spac = x[:, 54]
sflag = (x[:, 55].astype(np.int64) & 0x04) != 0
zupt = (gb & 0x10) != 0

print()
print('=== 走动/平移段里，ZUPT 的四条子判据各自起了什么作用 ===')
for (t0, t1, nm) in [(1.28, 4.01, '平移段 A'), (4.78, 7.02, '平移段 B'),
                     (9.62, 11.57, '平移段 C'), (17.17, 18.88, '对照:剧烈')]:
    i0, i1 = int(np.searchsorted(tt, t0)), int(np.searchsorted(tt, t1))
    sl = slice(i0, i1)
    n = i1 - i0
    print('  %s (%.2f~%.2f s, %.1f s):' % (nm, t0, t1, t1-t0))
    print('     is_static=1 的帧 %5.1f%%   -> 只靠它就已经关门' % (100*np.mean(isst[sl])))
    print('     e_ac>=0.20  的帧 %5.1f%%   -> 只有 e_ac 能挡的帧 %5.1f%%'
          % (100*np.mean(eac[sl] >= 0.20), 100*np.mean((eac[sl] >= 0.20) & isst[sl])))
    print('     |a_lin|>=0.05g 帧 %5.1f%%  -> 只有它+新判据能挡的帧 %5.1f%%'
          % (100*np.mean(alin2[sl] >= 0.0025), 100*np.mean((alin2[sl] >= 0.0025) & isst[sl] & (eac[sl] < 0.20))))
    print('     zupt 实开 %5.1f%%   sigma_vel_h 从 %.4f 长到 %.4f'
          % (100*np.mean(zupt[sl]), x[i0, 106], x[i1-1, 106]))
