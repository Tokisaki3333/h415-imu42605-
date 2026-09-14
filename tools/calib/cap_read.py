# -*- coding: utf-8 -*-
"""读 cap.py 落盘的二进制记录。

每样本 = float64 帧序号 + float64 主机时刻(s) + float32 v[N]
  N=4 : v = 四元数 w,x,y,z
  N=7 : v = 四元数 w,x,y,z + 加速度计三轴原始值(LSB)
通道数从 <file>.meta 读，没有则按 4 路（兼容旧文件）。
时间基准 = 帧序号 / 实测帧率（等间隔）；主机时刻只用于核对丢帧。

用法:
  python cap_read.py static_zero.bin            # 漂移
  python cap_read.py static_zero.bin --allan    # + Allan
  python cap_read.py static_zero.bin --raw      # + 加速度原始值统计
"""
import os, sys
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
ACC_LSB_PER_G = 2048.0            # ±16 g 档（spi_hw.c ACCEL_CONFIG0=0x03）；改量程要同步改这里
G = 9.80665


def nchan(path):
    meta = path + '.meta'
    if os.path.exists(meta):
        return int(open(meta).read().split()[0])
    return 4


def load(path):
    n = nchan(path)
    rec = np.dtype([('frm', '<f8'), ('thost', '<f8'), ('v', '<f4', n)])
    b = open(path, 'rb').read()
    k = len(b) // rec.itemsize
    a = np.frombuffer(b[:k * rec.itemsize], dtype=rec)
    v = a['v'].astype(np.float64)
    q = v[:, :4].copy()
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    frm, thost = a['frm'], a['thost']
    fps = (frm[-1] - frm[0]) / (thost[-1] - thost[0])
    t = (frm - frm[0]) / fps
    return t, q, v[:, 4:], fps, thost


def qmul(a, b):
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw], 1)


def rel_rotvec(q0, q1):
    """q0^-1 (x) q1 的旋转矢量(deg)"""
    inv = np.array([q0[0], -q0[1], -q0[2], -q0[3]])
    r = qmul(np.tile(inv, (len(q1), 1)), q1)
    s = np.where(r[:, 0] >= 0, 1.0, -1.0)
    v = r[:, 1:] * s[:, None]
    n = np.linalg.norm(v, axis=1)
    return np.degrees(2*np.arctan2(n, np.abs(r[:, 0])))[:, None] * v / np.maximum(n, 1e-30)[:, None]


def allan(t, ang):
    """ang(deg, Nx3) 的重叠 Allan 偏差 -> (tau_s, sigma_deg_合成)"""
    dt = float(np.median(np.diff(t)))
    M = len(ang)
    taus = np.unique(np.round(np.logspace(np.log10(2), np.log10(M//8), 60)).astype(int))
    out = []
    for m in taus:
        d = ang[2*m:] - 2*ang[m:-m] + ang[:-2*m]
        out.append((m*dt, float(np.sqrt(((d*d).sum(0)/(2.0*(M-2*m))).sum()))))
    return np.array(out)


if __name__ == '__main__':
    path = sys.argv[1]
    t, q, extra, fps, thost = load(path)
    print("样本 %d  通道 %d  时长 %.1f s  帧率 %.3f fps  dt=%.4f s"
          % (len(t), 4 + extra.shape[1], t[-1], fps, np.median(np.diff(t))))

    ang = rel_rotvec(q[0], q)
    e = ang[-1]
    print("末端 q = %s" % np.array2string(q[-1], precision=6))
    print("相对起始姿态的漂移 = %s deg   |%.4f| deg" % (np.array2string(e, precision=4), np.linalg.norm(e)))
    print("                      %.4f deg/h   %.2f deg/天（线性外推，长记录才准）"
          % (np.linalg.norm(e)/(t[-1]/3600), np.linalg.norm(e)/(t[-1]/86400)))

    if extra.shape[1] >= 3:
        a = extra[:, :3]
        g_lsb = a / ACC_LSB_PER_G
        print("\n加速度计原始值统计 (LSB)：")
        for i, ax in enumerate('xyz'):
            print("  %s: 均值 %+9.2f  1sigma %7.2f  峰值 %8.0f   -> %+8.3f g 均值, 噪声 %.2f mg"
                  % (ax, a[:, i].mean(), a[:, i].std(), np.abs(a[:, i]).max(),
                     g_lsb[:, i].mean(), g_lsb[:, i].std()*1000))
        m = np.linalg.norm(g_lsb, axis=1)
        print("  |a| = %.4f g  1sigma %.2f mg   (静止时应等于 1 g)" % (m.mean(), m.std()*1000))
        print("  等效比力 m/s^2: %s" % np.array2string(a.mean(0)/ACC_LSB_PER_G*G, precision=4))
        print("  倾角(由加速度计, deg): roll %.3f  pitch %.3f"
              % (np.degrees(np.arctan2(g_lsb[:, 1].mean(), g_lsb[:, 2].mean())),
                 np.degrees(np.arctan2(-g_lsb[:, 0].mean(),
                                       np.hypot(g_lsb[:, 1].mean(), g_lsb[:, 2].mean())))))

    if '--allan' in sys.argv:
        res = allan(t, ang)
        sr = res[:, 1] / res[:, 0] * 3600.0
        print("\n  tau(s)    sigma(deg)   sigma(deg/h)   N(deg/rt-h)")
        for i in range(0, len(res), max(1, len(res)//14)):
            print("  %8.3f  %10.5f  %12.4f  %12.4f"
                  % (res[i, 0], res[i, 1], sr[i], res[i, 1]/np.sqrt(res[i, 0])*60))
        k = int(np.argmin(sr))
        print("\n速率 Allan 最小 = %.4f deg/h @ tau=%.1f s -> 零偏不稳定性 B = %.4f deg/h"
              % (sr[k], res[k, 0], sr[k]/0.664))
        print("  (tau 上限 %.0f s，独立簇数约 %.0f，不确定度约 ±%.0f%%)"
              % (res[-1, 0], t[-1]/res[k, 0], 100/np.sqrt(max(t[-1]/res[k, 0], 1))))
