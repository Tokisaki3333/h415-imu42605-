# -*- coding: utf-8 -*-
"""
标定一致性诊断：在同一条记录上比较两套标定的 |a_b| - g

静止时 a_b 还原的就是 -重力矢量，模长恒为 g，**与姿态无关**。所以静止段的
    (|a_b| - g)
只反映"这一刻的零偏/标度误差"，不受倾角、不受拉平、不受积分影响 ——
这是判断某套标定在某条记录上好不好用的唯一干净指标。

用法：python bias_consistency.py [记录...]
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'h415-imu42605-', 'tools', 'calib'))
from jf_load import load_jf

G = 9.7985
fps = 8027.0
WIN = int(0.20 * fps)

# 椭圆法（绕 x/y/z 三条记录，每轴两源均值）—— 当前载入固件的值
B_ELL = np.array([-10.10, -15.42, 43.72])
S_ELL = np.array([2028.48, 2040.78, 2016.07])
# 六面法（sixface16.py，±16 g）—— 旧值
B_SIX = np.array([-10.31, -13.66, 45.68])
S_SIX = np.array([2027.501, 2039.529, 2015.479])

RECS = [('绕z 233731_085', 'serial_runtime_20260913_233731_085_export.txt'),
        ('绕y 234829_229', 'serial_runtime_20260913_234829_229_export.txt'),
        ('绕x 000636_751', 'serial_runtime_20260914_000636_751_export.txt'),
        ('正方形 231221_019', 'serial_runtime_20260913_231221_019_export.txt'),
        ('六面 225736_268', 'serial_runtime_20260913_225736_268_export.txt')]


def smooth_rate(q):
    q0, q1 = q[:-1], q[1:]
    w_, x_, y_, z_ = q0.T
    w2, x2, y2, z2 = q1.T
    V = np.stack([w_*x2-x_*w2-y_*z2+z_*y2, w_*y2+x_*z2-y_*w2-z_*x2, w_*z2-x_*y2+y_*x2-z_*w2], 1)
    cs = np.vstack([np.zeros(3), np.cumsum(V, 0)])
    k = np.arange(len(q))
    lo = np.clip(k-WIN, 0, None); hi = np.clip(k+1, None, len(V))
    mv = (cs[hi]-cs[lo])/np.maximum(hi-lo, 1)[:, None]
    return np.degrees(2.0*np.linalg.norm(mv, axis=1)*fps)


def static_means(acc, rate, thr=1.0, minlen=0.4):
    N = len(acc); out, still, i = [], rate < thr, 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/fps >= minlen:
                g_ = int(0.05*fps); sl = slice(i+g_, j-g_)
                if sl.stop-sl.start > int(0.1*fps):
                    out.append(acc[sl].mean(0))
            i = j
        else:
            i += 1
    return np.array(out)


def report(tag, fn):
    p = fn if os.path.exists(fn) else os.path.join('..', fn)
    if not os.path.exists(p):
        print("  %-18s 记录缺失" % tag); return
    a = np.asarray(load_jf(p), dtype=np.float64)
    q = a[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
    acc = a[:, 4:7]
    rate = smooth_rate(q)
    P = static_means(acc, rate)
    print("\n== %-18s  %d 个静止段" % (tag, len(P)))
    for nm, B, S in (('椭圆法(在用)', B_ELL, S_ELL), ('六面法(旧) ', B_SIX, S_SIX)):
        m = (P - B)/S                      # 单位 g（S 是 LSB/g）
        mag = np.linalg.norm(m, axis=1)
        d = (mag - 1.0)*1000               # mg
        print("   %s  |a_b|-1g = %+7.3f mg (中位)  段间散布 %.3f mg  [%s]"
              % (nm, np.median(d), d.std(), ' '.join('%+.2f' % x for x in d)))
    # 让这条记录自己解出"有效零偏"需要动多少（沿 a_b 方向的一维修正）
    m = (P - B_ELL)/S_ELL
    d = (np.linalg.norm(m, axis=1) - 1.0)*1000
    print("   => 本场次相对椭圆法需要沿 a_b 方向修正 %+.2f LSB (b 范数 %.2f -> %.2f)"
          % (-np.median(d)/1000*np.linalg.norm(S_ELL), np.linalg.norm(B_ELL),
             np.linalg.norm(B_ELL) - np.median(d)/1000*np.linalg.norm(S_ELL)))


for tag, fn in RECS:
    report(tag, fn)
