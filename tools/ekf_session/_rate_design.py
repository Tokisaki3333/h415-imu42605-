# -*- coding: utf-8 -*-
# 为"磁融合速率"设计做测量
#   需要三个量：
#     d  = EKF 非磁力部分的偏航漂移速度 (度/秒)   —— 环路必须跟得上的斜坡
#     s  = 磁航向单次测量的噪声 (度)              —— 环路会把它放大成抖动
#     τ  = 当前环路等效时间常数                   —— 由 k_cap 与更新率决定
#   目标：在 d*τ（跟不上的残差）与 s*sqrt(1/(2τf))（噪声抖动）之间取最优
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
inn = (b[:,144] - b[:,145] + 180) % 360 - 180          # 带符号航向新息
dqz = b[:,137]; used = (b[:,120] == 1)
stat = (b[:,8] == 1)
gn = np.linalg.norm(b[:,26:29], axis=1); gcl = np.where(gn > 1e4, np.nan, gn)
print('fw_tag %d VER=%d  时长 %.2f s' % (int(round(b[0,76])), int(round(b[0,76]))>>16, t[-1]))

# 磁样本更新率与环路有效速率
cnt = b[:,41]; edges = (np.diff(cnt) != 0).sum()
f_upd = edges / t[-1]
print()
print('=== 环路参数（实测）===')
print('  磁样本率 f_upd = %.1f Hz ；EKF 周期 %.1f Hz ；mag_used 占空 %.1f%%'
      % (f_upd, (np.diff(b[:,130]) != 0).sum()/t[-1], 100*used.mean()))
print('  k_cap = 0.05（固件常量 V5F_EKF_MAG_K_MAX）')
tau = 1.0/(0.05*f_upd)
print('  => 环路等效时间常数 τ = 1/(k_cap*f_upd) = %.4f s  (带宽 %.2f Hz)' % (tau, 1/(2*np.pi*tau)))

# 静止段：量新息的噪声底与低频
print()
print('=== 静止段（旧路径判静 且 |w|<1）的新息统计 ===')
q = stat & (np.nan_to_num(gcl, nan=1e9) < 1.0)
seg = np.zeros(len(b), bool); seg[q] = True
# 用连续段
runs = []
i = 0
while i < len(b):
    if seg[i]:
        j = i
        while j+1 < len(b) and seg[j+1]: j += 1
        if j-i > 300: runs.append((i, j))
        i = j+1
    else:
        i += 1
print('  连续静止段 %d 段, 合计 %.2f s' % (len(runs), sum(t[j]-t[i] for i,j in runs)))
allv = np.concatenate([inn[i:j+1] for i, j in runs])
print('  新息: std %.4f 度  p50|.| %.4f  p90|.| %.4f  max|.| %.3f'
      % (np.std(allv), np.median(np.abs(allv)), np.percentile(np.abs(allv),90), np.abs(allv).max()))
# 高频噪声：相邻样本差（更新处）的 std/sqrt(2)
ch = np.where(np.diff(b[:,134]) != 0)[0]+1
ch = ch[used[ch] & stat[ch] & (np.nan_to_num(gcl[ch],nan=1e9) < 1.0)]
dv = inn[ch]
sel = np.diff(ch) < 100
dn = np.diff(dv)[sel]
s_hi = np.std(dn)/np.sqrt(2)
print('  逐次更新间的新息差 std/sqrt2 = %.4f 度  (= 测量噪声估计 s)' % s_hi)
print('  自相关 lag1(更新序) = %+.3f  (>0 说明有相关误差, 不全是白噪声)'
      % np.corrcoef(dv[:-1], dv[1:])[0,1])

# 抖动：EKF 偏航的高频成分（对旧链比）
def yaw_of(q):
    w,x,y,z = q[:,0],q[:,1],q[:,2],q[:,3]
    n=np.sqrt(w*w+x*x+y*y+z*z); n[n<1e-9]=1e-9
    return np.degrees(np.arctan2(2*(w/n)*(z/n)+2*(x/n)*(y/n),1-2*((y/n)**2+(z/n)**2)))
ye = yaw_of(b[:,89:93]); yl = yaw_of(b[:,0:4])
print()
print('=== 抖动（相邻帧偏航的二阶差分，静止段）===')
for nm, y in (('EKF', ye), ('旧链', yl)):
    vals = []
    for i, j in runs:
        d2 = y[i+2:j+1] - 2*y[i+1:j] + y[i:j-1]
        vals.append(d2)
    v = np.concatenate(vals)
    print('  %-4s 二阶差分 std = %.5f 度/帧^2   rms = %.5f' % (nm, np.std(v), np.sqrt(np.mean(v**2))))

# 非磁漂移速度：从"没有磁修正"的历史段取（VER=73/74 有 12 度死区）
print()
print('=== 非磁漂移速度 d 的估计 ===')
print('  历史实测（VER=73/74 有 12 度死区、环路休眠的静止段）:')
print('    |psi| 0.43 -> 1.27 度 / 14 s  =>  d ≈ 0.060 度/秒')
print('  本次数据（VER=81，环路一直工作）用稳态误差反推:')
print('    ψ_ss ≈ d*τ ；实测静止 |新息| p50 %.4f 度, τ=%.4f s => d ≈ %.4f 度/秒'
      % (np.median(np.abs(allv)), tau, np.median(np.abs(allv))/tau))
print('    （后者偏大是因为静止新息主要由测量噪声而不是漂移决定）')

# 最优 τ
print()
print('=== 设计：最优环路时间常数 ===')
for d in (0.03, 0.06, 0.12, 0.25):
    tau_opt = (s_hi**2/(2*f_upd*d**2))**(1/3)
    k_opt = 1.0/(tau_opt*f_upd)
    err_d = d*tau_opt
    err_n = s_hi*np.sqrt(1.0/(2*tau_opt*f_upd))
    print('  d=%.2f 度/秒 -> τ_opt %.3f s, k=%.4f (现 0.05), 漂移残差 %.4f 度, 噪声抖动 %.4f 度, 合计 %.4f'
          % (d, tau_opt, k_opt, err_d, err_n, err_d+err_n))
print()
print('  当前 τ=%.4f: 漂移残差(d=0.06) %.4f 度, 噪声抖动 %.4f 度, 合计 %.4f'
      % (tau, 0.06*tau, s_hi*np.sqrt(1/(2*tau*f_upd)), 0.06*tau + s_hi*np.sqrt(1/(2*tau*f_upd))))
for T in (0.1,0.3,0.5,1.0,2.0,5.0):
    k = 1.0/(T*f_upd)
    print('  若 τ=%.1f s -> k_cap=%.5f : 漂移残差(d=0.06) %.4f 度, 噪声抖动 %.4f 度, 合计 %.4f'
          % (T, k, 0.06*T, s_hi*np.sqrt(1/(2*T*f_upd)), 0.06*T + s_hi*np.sqrt(1/(2*T*f_upd))))
