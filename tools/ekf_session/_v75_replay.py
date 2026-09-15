# -*- coding: utf-8 -*-
# 离线重放：把 VER=74 录制里"因 P 塌到地板而失效"的修正，换成"加了方差下限后的修正"，
# 看残差轨迹会变成什么样。这是对 VER=75 的验证，不是预测。
#
# 依据（两条都是实测/闭式，不是拟合）：
#  1) 注入约定已实测: psi_new = psi + 施加的修正
#  2) 每个周期的"扰动" d = 实测Δpsi - 实测施加量   (与施加量无关的那部分)
#  3) 加下限后 K 恒在 k_cap: |K[8][0]|=0.05, |K[8][1]|=0.048
#     -> 施加量' = K[8]·rk,  rk = -(R_z(psi)v0 - v0)
import numpy as np

P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474
v0 = np.array([B0X, B0Y])
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.02) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
psi = np.degrees(np.arctan2(B0X*b[:,131]-B0Y*b[:,130], B0X*b[:,130]+B0Y*b[:,131]))
used = b[:,120] == 1
dqz = b[:,137]

def capped_dx(psi_deg):
    """加方差下限后本周期的施加量(度)。K 已恒在 k_cap 上限。"""
    p = np.radians(psi_deg)
    c, s = np.cos(p), np.sin(p)
    v = np.array([v0[0]*c - v0[1]*s, v0[0]*s + v0[1]*c])
    rk = -(v - v0)                      # rk = z - h(x)
    return (-0.05*rk[0] - 0.048*rk[1]) * 57.2957795

def rho(psi_deg):                       # 一阶牵引强度: d(施加)/d(psi)
    h = 1e-4
    return (capped_dx(psi_deg+h) - capped_dx(psi_deg-h)) / (2*h)

print('加下限后的单周期修正量 (对比实测在 P 塌陷时的值):')
for p in (1, 3, 10, 20, 30, 49, 60, 90, 120, 160):
    print('  |psi|=%3d 度 -> 施加 %+7.3f 度/更新 = %7.1f 度/秒 (193.8Hz)   牵引增益 %.3f' %
          (p, capped_dx(p), capped_dx(p)*193.8, rho(p)))

# mag 周期边界
ch = np.where(np.diff(psi) != 0)[0] + 1
ch = ch[good[ch] & good[ch-1]]
i0, i1 = ch[:-1], ch[1:]
dtc = t[i1] - t[i0]
ok = (dtc > 1e-4) & (dtc < 2e-2)
i0, i1, dtc = i0[ok], i1[ok], dtc[ok]
dpsi = (psi[i1] - psi[i0] + 180.0) % 360.0 - 180.0
applied_meas = np.where(used[i0], dqz[i0], 0.0)
dist = dpsi - applied_meas
print()
print('=== 重放: 残差用 ψ\'_{k+1} = ψ\'_k + d_k + 加下限后的施加量 ===')
print('  时段(s)     实测|psi|p50  实测|psi|max   重放|psi|p50  重放|psi|max   重放<10度占比')
for lo, hi in [(0,5),(5,10),(10,15),(15,20),(20,25),(25,30),(30,35),(35,40),(40,44)]:
    m = (t[i1] >= lo) & (t[i1] < hi)
    if m.sum() < 50: continue
    idx = np.where(m)[0]
    ps = psi[i0][idx[0]]
    traj = [ps]
    for k in idx:
        # 一阶牵引 + 扰动(用实测 d_k) ; 用隐式近似避免大步长过冲
        d = dist[k]
        # 解析小步: 施加量对 psi 线性化
        g = rho(ps)
        step = 1.0/(1.0 + max(0.0, -g))        # 隐式阻尼(牵引使 psi 减小)
        ps = ps + (capped_dx(ps) + d) * step
        traj.append(ps)
    traj = np.array(traj)
    print('%5.0f~%-5.0f %12.2f %13.2f %13.2f %14.2f %13.1f%%'
          % (lo, hi, np.median(np.abs(psi[i0][idx])), np.abs(psi[i0][idx]).max(),
             np.median(np.abs(traj[:-1])), np.abs(traj[:-1]).max(),
             100*(np.abs(traj[:-1]) < 10).mean()))
