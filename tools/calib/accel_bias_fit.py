# -*- coding: utf-8 -*-
r"""加速度计零偏标定拟合（多姿态静止）：解 |a_meas - b| = 1 g 的最小二乘

为什么需要它：静止单姿态下，"加速度计零偏"与"倾角误差"在数学上不可分辨
（任何**垂直于重力**的零偏都不改变 |a|）⇒ 单姿态数据无法仲裁 `V5F_ACCEL_BIAS_LSB_*`
与 EKF 的 `ba` 谁对。必须换姿态：不同朝向下同一零偏对 |a| 的影响不同，
只有正确零偏才在**所有**姿态下都满足 |a| = 1 g。

数据要求：一段调试帧录像，包含 >=4（最好 6）个**互不相同的静止姿态**，每个 >= 2 s，
姿态间差异尽量大（六面法最好：±x/±y/±z 朝下各一次）。

模型：a_meas(LSB/lsb_per_g) = a_true + b，|a_true| = 1
解：Gauss-Newton 最小化 sum (|a_meas_i - b| - 1)^2

用法: python tools/calib/accel_bias_fit.py R:\imu_xxx.bin
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
LSBG = np.array([2028.48, 2040.78, 2016.07])     # v5f_proc.h
B0_LSB = np.array([-10.10, -15.42, 43.72])       # 当前编译期标定值（待检验）


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260923_060100.bin'
    fr = load_frames(path)
    a = fr[0] if isinstance(fr, tuple) else fr
    n = len(a)
    dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
    dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
    t = np.cumsum(dt)
    w = np.linalg.norm(np.asarray(a[:, C['gyro_dps0']:C['gyro_dps0'] + 3], float), axis=1)
    lsb = np.asarray(a[:, C['accel_lsb0']:C['accel_lsb0'] + 3], float)
    an_lsb = np.linalg.norm(lsb, axis=1)

    # 静止帧
    calm = w < 5.0
    # 取静止段（>=2 s），并按姿态方向聚类（方向夹角 > 25 deg 视为不同姿态）
    segs = []
    i = 0
    while i < n:
        if calm[i]:
            j = i
            while j < n and calm[j]:
                j += 1
            if dt[i:j].sum() >= 2.0:
                v = np.median(lsb[i:j], axis=0)
                v = v / max(np.linalg.norm(v), 1e-9)
                segs.append((i, j, v, dt[i:j].sum()))
            i = j
        else:
            i += 1
    picks = []
    for (i, j, v, T) in segs:
        if all(math.degrees(math.acos(np.clip(np.dot(v, p[2]), -1, 1))) > 25.0 for p in picks):
            picks.append((i, j, v, T))
    print('录像 %s  静止段 %d 个，其中姿态互异 %d 个' % (os.path.basename(path), len(segs), len(picks)))
    for (i, j, v, T) in picks:
        print('   t=%6.1f s  %5.1f s  方向 %s' % (t[i], T, np.array2string(v, precision=3)))
    if len(picks) < 4:
        print()
        print('!! 姿态数不足（%d < 4）⇒ 无法标定零偏。请补录：六个面各朝下、每个 >=3 s 静止。' % len(picks))
        return
    A = np.array([np.median(lsb[i:j], axis=0) / LSBG for (i, j, v, T) in picks])
    b = B0_LSB / LSBG
    print()
    print('拟合（Gauss-Newton，单位 g）：')
    print('  初值（编译期）= (%+.5f, %+.5f, %+.5f) g   |a| 偏离 1 g：%s mg'
          % (b[0], b[1], b[2],
             np.array2string((np.linalg.norm(A - b, axis=1) - 1) * 1000, precision=2)))
    for it in range(60):
        r = np.linalg.norm(A - b, axis=1) - 1.0
        u = (A - b) / np.maximum(np.linalg.norm(A - b, axis=1)[:, None], 1e-12)
        J = -u
        db, *_ = np.linalg.lstsq(J, -r, rcond=None)
        b = b + db
        if np.linalg.norm(db) < 1e-9:
            break
    r = np.linalg.norm(A - b, axis=1) - 1.0
    print('  拟合值        = (%+.5f, %+.5f, %+.5f) g' % (b[0], b[1], b[2]))
    print('  残差 |a|-1    = %s mg   （rms %.2f mg）'
          % (np.array2string(r * 1000, precision=2), math.sqrt(float(np.mean(r ** 2))) * 1000))
    print('  折算 LSB      = (%+.2f, %+.2f, %+.2f)   <- 可直接写进 V5F_ACCEL_BIAS_LSB_*'
          % tuple(b * LSBG))
    print('  与当前编译期之差 = (%+.1f, %+.1f, %+.1f) mg   |差| = %.1f mg'
          % tuple((b - B0_LSB / LSBG) * 1000), np.linalg.norm(b - B0_LSB / LSBG) * 1000)
    print('  （夹紧上限 V5F_ACC_TRACTION_BIAS_LIM_MG = 40 mg：差值超过它，8 kHz 追踪器就顶死）')

    # 缩放一致性：|a_lsb| 直接看（含 scale）
    print()
    print('顺带检查 scale：各姿态 |a_lsb| （应相等）')
    for (i, j, v, T) in picks:
        print('   t=%6.1f s  |a_lsb| = %.1f' % (t[i], np.median(an_lsb[i:j])))


if __name__ == '__main__':
    main()
