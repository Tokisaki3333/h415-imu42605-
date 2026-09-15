# -*- coding: utf-8 -*-
"""诊断: 平台内那 2.33° / 1.50° 是"运动残差(收敛)"还是"真实慢转/漂移"?
   看平台内累计转角随时间的形状: 常数斜率=真实慢转(平台判错), 指数收敛=EKF/磁在收敛, 线性漂移=零偏。"""
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
for P, tag in ((r'serial_runtime_20260916_061641_858_export.txt', '061641 高速无参考'),
               (r'serial_runtime_20260916_061843_305_export.txt', '061843 常规')):
    ts, Q = load(P); t0 = ts[0]
    print('\n=== %s ===' % tag)
    for nm, a_t, b_t in (('平台#1', ts[0], ts[0]+3.2), ('平台#2(末)', ts[-1]-4.6, ts[-1])):
        a = int(np.argmin(abs(ts-a_t))); b = int(np.argmin(abs(ts-b_t)))
        if b-a < 800: continue
        cum = ab(Q[a:a+1].repeat(b-a+1, 0), Q[a:b+1])
        ya = yawof(Q[a:b+1])
        print(' %s t=%.2f~%.2f (%.2fs) 累计 %.3f°  每0.4s: 累计/偏航/增量' % (nm, ts[a]-t0, ts[b]-t0, ts[b]-ts[a], cum[-1]))
        step = max(1, int(0.4/((ts[b]-ts[a])/(b-a))))
        prev_c = 0.0
        for k in range(0, b-a, step):
            j = min(k+step, b-a)
            print('    +%4.1fs  cum %6.3f°  yaw %+7.3f°  增量 %6.3f°' %
                  (ts[a+j]-ts[a], cum[j], ya[j]-ya[0], cum[j]-prev_c))
            prev_c = cum[j]
