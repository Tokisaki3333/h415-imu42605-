# -*- coding: utf-8 -*-
r"""双跟踪器一致性核验：哪一套把 |a| 稳在 1 g？

固件里两条并存（用户说明是**特意设计**）：
  ① 8 kHz 紧耦合跟踪器 s_accel_bias_lsb（tau_p=13 s，±40 mg 夹紧，见 V5F_ACC_TRACTION_*）
     输出 -> h->imu.accel_g（门控/stat/验收都用它）
  ② EKF 的 ba 状态（M6 用：raw_f_mps2 用**编译期** V5F_ACCEL_BIAS_LSB_*，再减 s_x[IX_BA]）

设计上二者应当收敛到一致（差别只应是"快/慢"分工）。本脚本用静止时 |a| = 1 g 这条硬约束
判谁在漂：分别用 ① 与 ② 的零偏算 |a|，看哪个偏离 1 g。
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
LSBG = np.array([2028.48, 2040.78, 2016.07])       # v5f_proc.h
B0 = np.array([-10.10, -15.42, 43.72])             # 编译期标定（M6 用）
path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260923_060100.bin'
fr = load_frames(path)
a = fr[0] if isinstance(fr, tuple) else fr
n = len(a)
dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
t = np.cumsum(dt)
lsb = np.asarray(a[:, C['accel_lsb0']:C['accel_lsb0'] + 3], float)
ag = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)      # 跟踪器①的输出
bg_r = np.asarray(a[:, C['accel_bias_g0']:C['accel_bias_g0'] + 3], float)   # 跟踪器①的零偏
ba = np.asarray(a[:, C['ekf_ba0']:C['ekf_ba0'] + 3], float)        # 跟踪器②（EKF）
na = np.linalg.norm(ag, axis=1)
g2 = (lsb - B0) / LSBG - ba                                        # M6 实际用的（编译期 + EKF ba）
n2 = np.linalg.norm(g2, axis=1)
print('录像 %s  帧 %d  %.0f s' % (os.path.basename(path), n, t[-1]))
print('t(min)  |a|①跟踪器-1(mg)  |a|②EKF(编译期+ba)-1(mg)  跟踪器零偏(mg)                 ekf.ba(mg)'
      '                  二者差(mg)')
for s in range(0, int(t[-1]) + 1, 60):
    k = min(np.searchsorted(t, s), n - 1)
    d = (bg_r[k] - B0 / LSBG - ba[k]) * 1000.0
    print('%5d   %+10.2f        %+10.2f              (%+6.1f,%+6.1f,%+6.1f)   (%+6.1f,%+6.1f,%+6.1f)   (%+6.1f,%+6.1f,%+6.1f)'
          % (s // 60, (na[k] - 1) * 1000, (n2[k] - 1) * 1000,
             *((bg_r[k] - B0 / LSBG) * 1000.0), *(ba[k] * 1000.0), *d))
print()
print('|a| 偏离 1 g 的统计（全段）：')
print('  ① 跟踪器  : p50 %+.2f mg  p95 %.2f mg  max %.2f mg' %
      ((np.median(na) - 1) * 1000, (np.percentile(na, 95) - 1) * 1000, (na.max() - 1) * 1000))
print('  ② EKF     : p50 %+.2f mg  p95 %.2f mg  max %.2f mg' %
      ((np.median(n2) - 1) * 1000, (np.percentile(n2, 95) - 1) * 1000, (n2.max() - 1) * 1000))
print('  跟踪器相对编译期基准的位移 |bg_r-B0/lsb| 末 = %.1f mg（夹紧上限 V5F_ACC_TRACTION_BIAS_LIM_MG=40 mg）'
      % (np.linalg.norm(bg_r[-1] - B0 / LSBG) * 1000))
print('  两者差 |tracker-ct-ba| 首/末 = %.1f / %.1f mg -> 表观 %.2f / %.2f deg'
      % (np.linalg.norm(bg_r[0] - B0 / LSBG - ba[0]) * 1000,
         np.linalg.norm(bg_r[-1] - B0 / LSBG - ba[-1]) * 1000,
         math.degrees(np.linalg.norm(bg_r[0] - B0 / LSBG - ba[0])),
         math.degrees(np.linalg.norm(bg_r[-1] - B0 / LSBG - ba[-1]))))
