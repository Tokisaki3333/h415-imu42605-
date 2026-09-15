# -*- coding: utf-8 -*-
"""无参考下的真残差: 找"暂停平台"(平滑窗速率低), 在每个平台上量
   ① 平台内姿态变化(平台首→平台末) = 该次运动留下、被静止段修正掉的残差
   ② 平台内抖动(噪声)
   ③ 该残差 ÷ 前一段运动的路径 = 无参考 ppm
   两份日志都跑。"""
import numpy as np, re, sys, math
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
pat = re.compile(r'^\[(\d\d):(\d\d):(\d\d\.\d\d\d)\] \[RX\] ((?:[0-9A-Fa-f]{2} ){19}[0-9A-Fa-f]{2})\s*$')
def load(P):
    ts, qs = [], []
    for L in open(P, 'rb').read().decode('ascii', 'replace').split('\n'):
        m = pat.match(L)
        if not m: continue
        ts.append(int(m.group(1))*3600+int(m.group(2))*60+float(m.group(3)))
        qs.append(np.frombuffer(bytes.fromhex(m.group(4).replace(' ', ''))[:16], dtype='<f4'))
    return np.array(ts), np.array(qs, dtype=np.float64)
def ab(a, b): return np.degrees(2*np.arccos(np.clip(np.abs(a*b).sum(1), 0, 1)))
def yawof(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
def upof(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], 1)
for P in (r'serial_runtime_20260916_061641_858_export.txt', r'serial_runtime_20260916_061843_305_export.txt'):
    ts, Q = load(P); N = len(Q); ya, up = yawof(Q), upof(Q)
    dth = ab(Q[:-1], Q[1:])
    W = 240                                   # ~30 ms 窗(时间戳只有 1 ms 分辨率, 不能逐帧求速率)
    span = ts[W:] - ts[:-W]
    rw = np.full(N-W, 1e3)
    g = span > 0.01
    rw[g] = ab(Q[:-W][g], Q[W:][g])/span[g]
    k = np.convolve(rw, np.ones(24)/24, 'same')
    print('\n=== %s  帧 %d  %.2f s  路径 %.0f° ===' % (P.split('\\')[-1], N, ts[-1]-ts[0], dth.sum()))
    low = k < 1.5
    idx = np.where(low)[0]
    sg = [s for s in np.split(idx, np.where(np.diff(idx) > 2000)[0]+1) if len(s) >= 2000]
    print('暂停平台 %d 个:' % len(sg))
    print('  #  起~止(s)      时长  平台内: Δ偏航   Δ倾角   Δ总角   抖动p99 | 前段路径  真残差/路径')
    prev_end = None
    for j, s in enumerate(sg):
        a, b = int(s[0]), int(s[-1])
        dy = ((ya[b]-ya[a]+180) % 360)-180
        dt_ = math.degrees(math.acos(max(-1, min(1, float(up[a]@up[b])))))
        dT = ab(Q[a:a+1], Q[b:b+1])[0]
        jit = np.percentile(dth[a:b], 99)
        if prev_end is not None:
            seg = dth[prev_end:a].sum()
            print('  %d %6.2f~%6.2f %5.2fs  %+7.3f %+8.3f %8.3f %10.4f | %8.0f°  %6.0f ppm' %
                  (j+1, ts[a]-ts[0], ts[b]-ts[0], ts[b]-ts[a], dy, dt_, dT, jit, seg, dT/seg*1e6 if seg > 1 else float('nan')))
        else:
            print('  %d %6.2f~%6.2f %5.2fs  %+7.3f %+8.3f %8.3f %10.4f | %8s   %6s' %
                  (j+1, ts[a]-ts[0], ts[b]-ts[0], ts[b]-ts[a], dy, dt_, dT, jit, '-', '-'))
        prev_end = b
