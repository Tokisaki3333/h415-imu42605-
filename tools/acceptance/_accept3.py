# -*- coding: utf-8 -*-
"""常规工况验收: VER=100 CDC 四元数流(每行一帧 20B)
   ① 格式/完整/模长/速率 ② 有效姿态更新率与每次步长 ③ 角速率剖面(判工况)
   ④ 静止段(窗速率<2dps, >=1s): 偏航/倾角变化与漂移率 ⑤ 起止姿态差"""
import numpy as np, re, sys, math
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P = sys.argv[1] if len(sys.argv) > 1 else r'serial_runtime_20260916_061843_305_export.txt'
pat = re.compile(r'^\[(\d\d):(\d\d):(\d\d\.\d\d\d)\] \[RX\] ((?:[0-9A-Fa-f]{2} ){19}[0-9A-Fa-f]{2})\s*$')
ts, qs, nline, bad = [], [], 0, 0
for L in open(P, 'rb').read().decode('ascii', 'replace').split('\n'):
    if not L.startswith('['): continue
    nline += 1
    m = pat.match(L)
    if not m: bad += 1; continue
    b = bytes.fromhex(m.group(4).replace(' ', ''))
    ts.append(int(m.group(1))*3600+int(m.group(2))*60+float(m.group(3)))
    qs.append(np.frombuffer(b[:16], dtype='<f4'))
    if b[16:20] != b'\x00\x00\x80\x7f': bad += 1
ts = np.array(ts); Q = np.array(qs, dtype=np.float64); N = len(Q)
nn = np.linalg.norm(Q, axis=1)
print('%s' % P.split('\\')[-1])
print('① 行 %d  坏帧 %d  时长 %.2f s  帧 %d  平均 %.0f Hz  |q|偏差 p99 %.2e  非有限 %d' %
      (nline, bad, ts[-1]-ts[0], N, N/(ts[-1]-ts[0]), np.percentile(np.abs(nn-1), 99), int((~np.isfinite(Q)).any(1).sum())))
def ab(a, b): return np.degrees(2*np.arccos(np.clip(np.abs(a*b).sum(1), 0, 1)))
def yawof(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
def upof(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], 1)
ya, up = yawof(Q), upof(Q)
ch = np.abs(np.diff(Q, axis=0)).sum(1) > 1e-9
u = np.where(ch)[0]
step = ab(Q[u], Q[u+1]) if len(u) else np.array([0.0])
ru = len(u)/(ts[-1]-ts[0])
sp = np.percentile(step, [50, 90, 99, 100])
print('② 有效姿态更新率 %.0f Hz (CDC 发 8 kHz 阶梯)  每次步长 p50 %.4f° p90 %.3f° p99 %.3f° max %.3f°' %
      (ru, sp[0], sp[1], sp[2], sp[3]))
print('   步长×更新率 = %.0f dps (等效角速率)' % (sp[0]*ru))
W = 1200
rate = ab(Q[:-W], Q[W:])/((ts[W:]-ts[:-W])/86400.0*24*3600)
rp = np.percentile(rate, [50, 90, 99, 100])
print('③ 0.15s 窗角速率: p50 %.1f  p90 %.1f  p99 %.1f  max %.1f dps  ; >100dps 占比 %.1f%%' %
      (rp[0], rp[1], rp[2], rp[3], 100*np.mean(rate > 100)))
low = rate < 2.0
idx = np.where(low)[0]
sg = [s for s in np.split(idx, np.where(np.diff(idx) > 8000)[0]+1) if len(s) >= 8000]
print('④ 静止段(窗速率<2 dps, >=1 s): %d 段' % len(sg))
print('   #  起~止(s)      时长   偏航变化   倾角变化   偏航漂移     倾角漂移')
for k, s in enumerate(sg):
    a, b = int(s[0]), int(s[-1]+W)
    gy = ((ya[b]-ya[a]+180) % 360)-180
    tp = math.degrees(math.acos(max(-1, min(1, float(up[a]@up[b])))))
    T = ts[b]-ts[a]
    print('   %d %6.2f~%6.2f %6.2fs  %+8.3f° %8.3f°  %+8.2f °/min %+8.2f °/min' %
          (k+1, ts[a]-ts[0], ts[b]-ts[0], T, gy, tp, gy/T*60, tp/T*60))
print('⑤ 起止姿态差: 总 %.2f°  偏航 %+.2f°  倾角 %.2f° ; 路径(Σ|Δ|) %.0f°' %
      (ab(Q[:1], Q[-1:])[0], ((ya[-1]-ya[0]+180) % 360)-180,
       math.degrees(math.acos(max(-1, min(1, float(up[0]@up[-1]))))), step.sum()))
