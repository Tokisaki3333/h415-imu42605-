# -*- coding: utf-8 -*-
# 结构正确性检验（在真实采集上做，不靠猜）
#  A. 各源到达率（自动识别所有"单调递增计数器"列）
#  B. 8 kHz 轮循调度回放：是否丢样本/重样本 + 到达->消费的滞后分布
#  C. 中断预算：从上报的 s_dma1_irq_us 反推每帧/每阶段开销，外推"每帧预测"
#  D. 数学结构：把两个观测拆到相邻两帧 vs 同一时刻批处理，差多少
import numpy as np

P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
T = t[-1]
print('帧数 %d  时长 %.2f s  帧率 %.0f Hz' % (N, T, N/T))

# ---------- A. 单调计数器 = 各源到达事件 ----------
print()
print('=== A. 各源到达率（自动识别单调递增列）===')
cnts = {}
for c in range(NCH):
    v = b[:, c]
    if len(np.unique(v[:200000])) < 20:
        continue
    d = np.diff(v)
    if np.all(d >= 0) and np.sum(d) > 100:
        cnts[c] = np.sum(d)
for c, inc in sorted(cnts.items(), key=lambda kv: -kv[1]):
    d = np.diff(b[:, c])
    per = np.diff(t)[d > 0]
    print('  col %-3d 增量 %7d  = %8.1f Hz   间隔 p50 %8.3f ms  max %8.2f ms'
          % (c, inc, inc/T, np.median(per)*1e3 if len(per) else np.nan,
             per.max()*1e3 if len(per) else np.nan))

# 磁 = 已知 col41
mag_arr = np.where(np.concatenate([[False], np.diff(b[:, 41]) > 0]))[0]
print()
print('  用于回放：磁到达 %d 次 (%.1f Hz)' % (len(mag_arr), len(mag_arr)/T))

# ---------- B. 8 kHz 轮循调度回放 ----------
print()
print('=== B. 8 kHz 轮循调度回放（一帧至多一个源，固定顺序 磁/气压/GPS）===')
def replay(sources, N):
    """sources: {name: set(到达帧号)}；返回每个样本的滞后(帧数)与丢弃数"""
    pend = {k: [] for k in sources}
    arr = {k: np.zeros(N, bool) for k in sources}
    for k, fr in sources.items():
        arr[k][fr] = True
    ptr = 0; order = list(sources)
    lat = {k: [] for k in sources}
    for i in range(N):
        for k in sources:
            if arr[k][i]: pend[k].append(i)
        for j in range(len(order)):
            k = order[(ptr+j) % len(order)]
            if pend[k]:
                lat[k].append(i - pend[k].pop(0))
                ptr = (ptr+j+1) % len(order)
                break
    dropped = {k: len(pend[k]) for k in sources}
    return lat, dropped

lat, dr = replay({'mag': mag_arr}, N)
L = np.array(lat['mag'])
print('  仅磁（真实数据）：样本 %d，未消费(丢弃) %d' % (len(L), dr['mag']))
print('    滞后(帧, 1帧=125us)：p50 %.0f  p90 %.0f  p99 %.0f  max %d  -> max %d us  (=%.1f ms)'
      % (np.percentile(L, 50), np.percentile(L, 90), np.percentile(L, 99), L.max(),
         L.max()*125, L.max()*0.125))

# 三源竞争的合成上界：气压 50 Hz、GPS 5 Hz，相位随机
rng = np.random.default_rng(0)
for trial in range(5):
    bar = np.sort(rng.choice(N, int(50*T), replace=False))
    gps = np.sort(rng.choice(N, max(1, int(5*T)), replace=False))
    lat3, dr3 = replay({'mag': mag_arr, 'baro': bar, 'gps': gps}, N)
    allm = np.array(lat3['mag']); allb = np.array(lat3['baro']); allg = np.array(lat3['gps'])
    print('  三源竞争#%d：磁 max 滞后 %2d 帧(%4d us) p99 %2d | 气压 max %2d 帧 | GPS max %2d 帧 | 丢弃 %s'
          % (trial, allm.max(), allm.max()*125, np.percentile(allm, 99),
             allb.max(), allg.max(), dr3))

# ---------- C. 中断预算 ----------
print()
print('=== C. 中断预算（col17 = 上一个 DMA1 ISR 耗时 us）===')
isr = b[:, 17].copy()
isr[(isr < 1) | (isr > 5000)] = np.nan
print('  ISR 耗时 us: p50 %.1f  p90 %.1f  p99 %.1f  max %.1f   (帧周期 125 us)'
      % tuple(np.nanpercentile(isr, [50, 90, 99, 100])))
print('  占空比: p50 %.1f%%  p99 %.1f%%  max %.1f%%'
      % tuple(100*np.nanpercentile(isr, [50, 99, 100])/125))
# 每阶段开销：用 EKF 四元数变化定位周期边界
qch = np.concatenate([[False], (np.abs(np.diff(b[:, 89]))+np.abs(np.diff(b[:, 90]))
                                + np.abs(np.diff(b[:, 91]))+np.abs(np.diff(b[:, 92]))) > 0])
qb = np.where(qch)[0]
per = np.diff(qb)
Lc = int(np.median(per))
print('  EKF 周期 = %d 帧 (%.1f Hz)，周期边界落在 frame %d 起的每 %d 帧' % (Lc, 8000/Lc, qb[0], Lc))
ph = (np.arange(N) - qb[0]) % Lc
prof = np.array([np.nanmedian(isr[ph == k]) for k in range(Lc)])
print('  各 phase 的 ISR 中位耗时(us)，按大小排序前 8：')
for k in np.argsort(prof)[::-1][:8]:
    print('     phase %2d : %6.2f us   (该 phase 帧数 %d)' % (k, prof[k], (ph == k).sum()))
lo = np.nanmedian(prof[prof < np.median(prof)])
print('  轻帧中位 %.2f us ; 重帧中位 %.2f us ; 最重 phase %.2f us' % (lo, np.median(prof), prof.max()))
print('  外推：若"预测每帧" => 每帧 ≈ 轻帧 + (最重phase - 轻帧) = %.2f us  (占空比 %.1f%%)'
      % (lo + (prof.max()-lo), 100*(lo + (prof.max()-lo))/125))

# ---------- D. 数学：拆帧 vs 批处理 ----------
print()
print('=== D. 把两个观测拆到相邻两帧(125us) vs 同一时刻批处理 ===')
# 一维位置-速度线性系统，两次量测；比较后验差
import numpy.linalg as la
def run(split):
    dtf = 125e-6
    x = np.array([0.0, 1.0]); Pm = np.eye(2)*0.5
    A = np.array([[1.0, dtf], [0.0, 1.0]]); Qm = np.eye(2)*1e-8
    H = np.array([[1.0, 0.0]]); Rm = np.array([[0.25]])
    z1, z2 = 0.10, 0.12
    def upd(x, Pm, z):
        y = z - H@x
        S = H@Pm@H.T + Rm
        K = Pm@H.T@la.inv(S)
        return x + (K@y).ravel(), (np.eye(2)-K@H)@Pm
    if split:
        x, Pm = upd(x, Pm, z1)
        x = A@x; Pm = A@Pm@A.T + Qm        # 预测一帧
        x, Pm = upd(x, Pm, z2)
    else:
        x, Pm = upd(x, Pm, z1)
        x, Pm = upd(x, Pm, z2)
    return x, Pm
xs, Ps = run(True); xb, Pb = run(False)
print('  拆帧: x=[%.10f, %.10f]  P diag=[%.3e, %.3e]' % (xs[0], xs[1], Ps[0, 0], Ps[1, 1]))
print('  批处理: x=[%.10f, %.10f]  P diag=[%.3e, %.3e]' % (xb[0], xb[1], Pb[0, 0], Pb[1, 1]))
print('  位置差 %.3e m (量测噪声 sigma=0.5 m) ; 速度差 %.3e m/s' % (abs(xs[0]-xb[0]), abs(xs[1]-xb[1])))
# 顺序 vs 批处理等价性（同一时刻）
def upd2(x, Pm, z1, z2):
    def u(x, Pm, z):
        y = z - H@x; S = H@Pm@H.T + Rm; K = Pm@H.T@la.inv(S)
        return x + (K@y).ravel(), (np.eye(2)-K@H)@Pm
    x, Pm = u(x, Pm, z1); return u(x, Pm, z2)
x0 = np.array([0.0, 1.0]); P0 = np.eye(2)*0.5
xseq, Pseq = upd2(x0, P0, 0.10, 0.12)
xbat = x0.copy(); Pb2 = P0.copy()
y = np.array([0.10, 0.12]) - np.array([[1.0, 0.0], [1.0, 0.0]])@xbat
H2 = np.array([[1.0, 0.0], [1.0, 0.0]]); R2 = np.eye(2)*0.25
S2 = H2@Pb2@H2.T + R2; K2 = Pb2@H2.T@la.inv(S2)
xbat = xbat + K2@y; Pb2 = (np.eye(2)-K2@H2)@Pb2
print('  同一时刻：顺序 vs 批处理 位置差 %.3e（应≈0，标准结论）' % abs(xseq[0]-xbat[0]))
