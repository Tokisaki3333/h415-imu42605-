# -*- coding: utf-8 -*-
"""读 cap.py 落盘的二进制静态记录：每样本 32 B = float64 帧序号 + float64 主机时刻(s) + float32 q[4]。

时间基准用帧序号 / 实测帧率（等间隔）；主机时刻只用来核对有没有丢帧。
用法: python cap_read.py static_zero.bin [--allan]
"""
import sys
import numpy as np

REC = np.dtype([('frm', '<f8'), ('thost', '<f8'), ('q', '<f4', 4)])


def load(path):
    b = open(path, 'rb').read()
    n = len(b) // REC.itemsize
    a = np.frombuffer(b[:n * REC.itemsize], dtype=REC)
    q = a['q'].astype(np.float64)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    frm = a['frm']
    thost = a['thost']
    # 帧率：用首末帧序号与主机时长
    fps = (frm[-1] - frm[0]) / (thost[-1] - thost[0])
    t = (frm - frm[0]) / fps
    return t, q, fps, thost


def allan(t, q, taus=None):
    """q 的累积角(deg) 的重叠 Allan 偏差，返回 (tau, sigma_deg)"""
    q0 = np.array([q[0, 0], -q[0, 1], -q[0, 2], -q[0, 3]])
    aw, ax, ay, az = q0
    bw, bx, by, bz = q.T
    rw = aw*bw - ax*bx - ay*by - az*bz
    rx = aw*bx + ax*bw + ay*bz - az*by
    ry = aw*by - ax*bz + ay*bw + az*bx
    rz = aw*bz + ax*by - ay*bx + az*bw
    s = np.where(rw >= 0, 1.0, -1.0)
    nv = np.sqrt(rx*rx + ry*ry + rz*rz)
    th = 2.0*np.arctan2(nv, np.abs(rw))
    ang = np.degrees(th)[:, None] * np.stack([rx, ry, rz], 1) * s[:, None] / np.maximum(nv, 1e-30)[:, None]
    dt = np.median(np.diff(t))
    M = len(ang)
    if taus is None:
        taus = np.unique(np.round(np.logspace(np.log10(2), np.log10(M//8), 60)).astype(int))
    out = []
    for m in taus:
        d = ang[2*m:] - 2*ang[m:-m] + ang[:-2*m]
        var = (d*d).sum(0)/(2.0*(M-2*m))
        out.append((m*dt, np.sqrt(var.sum())))
    return np.array(out)


if __name__ == '__main__':
    path = sys.argv[1]
    t, q, fps, thost = load(path)
    print("样本 %d  时长 %.1f s  帧率 %.3f fps  dt=%.4f s" % (len(t), t[-1], fps, np.median(np.diff(t))))
    # 漂移必须相对**起始姿态**，不是相对单位四元数（上电参考早已不是 identity）
    q0 = np.array([q[0, 0], -q[0, 1], -q[0, 2], -q[0, 3]])
    aw, ax, ay, az = q0
    bw, bx, by, bz = q[-1]
    rw = aw*bw-ax*bx-ay*by-az*bz; rx = aw*bx+ax*bw+ay*bz-az*by
    ry = aw*by-ax*bz+ay*bw+az*bx; rz = aw*bz+ax*by-ay*bx+az*bw
    nv = np.sqrt(rx*rx+ry*ry+rz*rz)
    e = 2*np.degrees(np.arctan2(nv, abs(rw)))
    ev = e*np.array([rx, ry, rz])/max(nv, 1e-30)*(1 if rw >= 0 else -1)
    print("末端 q = %s" % np.array2string(q[-1], precision=6))
    print("相对起始姿态的漂移 = %s deg   |%.4f| deg" % (np.array2string(ev, precision=4), e))
    print("                      %.4f deg/h   %.2f deg/天（线性外推，长记录才准）"
          % (e/(t[-1]/3600), e/(t[-1]/86400)))
    if '--allan' in sys.argv:
        res = allan(t, q)
        print("\n  tau(s)    sigma(deg)   sigma(deg/h)   N(deg/rt-h)")
        for tau, s in res:
            if tau > 1e9:
                continue
            print("  %8.3f  %10.5f  %12.4f  %12.4f" % (tau, s, s/tau*3600, s/np.sqrt(tau)*60))
        r = np.array(res)
        sr = r[:, 1]/r[:, 0]*3600.0          # 速率的 Allan 偏差 deg/h
        i = int(np.argmin(sr))
        print("\n速率 Allan 最小 = %.4f deg/h @ tau=%.1f s  ->  零偏不稳定性 B = %.4f deg/h"
              % (sr[i], r[i, 0], sr[i]/0.664))
        print("  (tau 搜索上限 %.0f s，独立簇数约 %.0f，不确定度约 ±%.0f%%)"
              % (r[-1, 0], t[-1]/r[i, 0], 100/np.sqrt(max(t[-1]/r[i, 0], 1))))
