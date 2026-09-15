# -*- coding: utf-8 -*-
"""修正分段后的 VER=48 新数据分析。
正确分段：静止 A = 0~3.15s；运动 = 3.15~20.31s(12987度, 757度/秒)；静止 B = 20.31~末。
"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
b = np.fromfile(P, dtype='<f4').reshape(-1, 122).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


yl, ye = yaw(b[:, 0:4]), yaw(b[:, 89:93])
g = np.linalg.norm(b[:, 26:29], axis=1)
A = t < 3.0            # 起始静止
M = (t >= 3.5) & (t < 20.0)   # 运动（留 0.5s 余量）
B = t > 20.6          # 末尾静止
print('fw_tag %d  时长 %.2f s   静止A %d 帧  运动 %d 帧  静止B %d 帧'
      % (b[0, 76], t[-1], A.sum(), M.sum(), B.sum()))

# ---------- 0) 旧链漂移速度 ----------
print('\n【0】旧链漂移速度')
for nm, m in (('静止A', A), ('静止B', B)):
    k = np.flatnonzero(m)
    span = t[k[-1]] - t[k[0]]
    dl = wrap(yl[k[-1]] - yl[k[0]]); de = wrap(ye[k[-1]] - ye[k[0]])
    rl = np.polyfit(t[k]-t[k[0]], np.unwrap(np.radians(yl[k])), 1)[0]
    re_ = np.polyfit(t[k]-t[k[0]], np.unwrap(np.radians(ye[k])), 1)[0]
    print('  %s %5.2f-%5.2fs(%.2fs): 净变化 旧链 %+7.3f / EKF %+7.3f 度'
          % (nm, t[k[0]], t[k[-1]], span, dl, de))
    print('     线性拟合速率: 旧链 %+7.4f 度/秒   EKF %+7.4f 度/秒' % (np.degrees(rl), np.degrees(re_)))
ka = np.flatnonzero(A); kb = np.flatnonzero(B)
span_ab = t[kb[0]] - t[ka[0]]
dl_ab = wrap(yl[kb[0]] - yl[ka[0]]); de_ab = wrap(ye[kb[0]] - ye[ka[0]])
print('  始末差（A 首 -> B 首, 间隔 %.2fs）: 旧链 %+7.3f 度 -> %+7.4f 度/秒'
      % (span_ab, dl_ab, dl_ab/span_ab))
print('                                    EKF  %+7.3f 度 -> %+7.4f 度/秒'
      % (de_ab, de_ab/span_ab))
print('  两者之差（EKF 相对旧链多转/少转）= %+7.3f 度' % wrap(de_ab - dl_ab))
# 静止段内的漂移速度范围（逐帧、取可靠帧）
rng = []
for nm, m in (('A', A), ('B', B)):
    k = np.flatnonzero(m)
    r = np.abs(wrap(np.diff(ye)))[k[:-1]] / dt[k[:-1]]
    rng.append((nm, np.median(r), np.percentile(r, 90), r.max()))
    print('  静止%s EKF 瞬时速率: p50 %.4f  p90 %.4f  max %.3f 度/秒' % (nm, *rng[-1][1:]) + '')
    r2 = np.abs(wrap(np.diff(yl)))[k[:-1]] / dt[k[:-1]]
    print('  静止%s 旧链瞬时速率: p50 %.4f  p90 %.4f  max %.3f 度/秒' % (nm, np.median(r2), np.percentile(r2, 90), r2.max()))

# ---------- 1) 崩溃式跳变 ----------
print('\n【1】崩溃式跳变（静止段内，EKF 相对旧链）')
for nm, m in (('静止A', A), ('静止B', B)):
    k = np.flatnonzero(m)
    dye = np.abs(wrap(np.diff(ye)))[k[:-1]]
    dyl = np.abs(wrap(np.diff(yl)))[k[:-1]]
    d = np.abs(wrap(np.diff(wrap(ye - yl))))[k[:-1]]
    print('  %s: EKF 单帧|Δyaw| p50 %.4f p99 %.3f max %.3f 度 | 旧链 p50 %.4f max %.3f 度'
          % (nm, np.median(dye), np.percentile(dye, 99), dye.max(),
             np.median(dyl), dyl.max()))
    print('        EKF-旧链 偏置单帧变化: p50 %.4f p99 %.3f max %.3f 度   (max @ t=%.2fs, 该帧|gyro|=%.1f)'
          % (np.median(d), np.percentile(d, 99), d.max(),
             t[k[int(np.argmax(d))]], g[k[int(np.argmax(d))]]))
print('  |EKF-旧链| 偏置: A 段 p50 %.2f 度 -> B 段 p50 %.2f 度   （全程 max %.2f 度）'
      % (np.median(np.abs(wrap(ye[A]-yl[A]))), np.median(np.abs(wrap(ye[B]-yl[B]))),
         np.abs(wrap(ye-yl)).max()))

# ---------- 2) 运动结束->静止 的地磁牵引 ----------
print('\n【2】运动结束（t≈20.31s）进入静止后的地磁牵引')
s = int(np.flatnonzero(t > 20.31)[0])
for W in (0.5, 1.0, 3.0, 8.0):
    Wn = int(W / np.median(dt))
    c = min(N-1, s+Wn)
    print('  进入静止后 %.1fs: Δyaw EKF %+8.3f 度  旧链 %+8.3f 度  差(地磁牵引量) %+8.3f 度'
          % (W, wrap(ye[c]-ye[s]), wrap(yl[c]-yl[s]), wrap(wrap(ye[c]-ye[s]) - wrap(yl[c]-yl[s]))))
mr = b[:, 119]
print('  mag_r 列: min %.6g p50 %.6g max %.6g  非零 %.3f%%' % (mr.min(), np.median(mr), mr.max(), 100*(mr != 0).mean()))
for nm, c in (('mag_gate', 117), ('mag_used', 120), ('mag_bh', 118), ('p_yy', 121)):
    v = b[:, c]
    print('  %-9s min %.4g p50 %.4g max %.4g' % (nm, v.min(), np.median(v), v.max()))

print('\n【3】运动段跟踪（陀螺能力，与磁无关）')
for nm, m in (('运动', M),):
    k = np.flatnonzero(m)
    tl = np.abs(wrap(np.diff(yl)))[k[:-1]].sum()
    te = np.abs(wrap(np.diff(ye)))[k[:-1]].sum()
    print('  总转角 旧链 %.1f 度  EKF %.1f 度  比值 %.5f (%+.3f%%)'
          % (tl, te, te/tl, 100*(te/tl-1)))
    print('  净转角 旧链 %+.2f  EKF %+.2f 度   偏置 %+.2f 度'
          % (wrap(yl[k[-1]]-yl[k[0]]), wrap(ye[k[-1]]-ye[k[0]]),
             wrap(wrap(ye[k[-1]]-ye[k[0]]) - wrap(yl[k[-1]]-yl[k[0]]))))

print('\n【4】源码：gate->ekf_mag_yaw / s_mag_gate 赋值处')
src = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c', 'rb').read().decode('gbk').split('\n')
for i, ln in enumerate(src):
    if re.search(r'ekf_mag_yaw|s_mag_gate', ln):
        print('  %4d %s' % (i+1, ln[:110]))
