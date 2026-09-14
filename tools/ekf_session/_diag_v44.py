# -*- coding: utf-8 -*-
"""离线复算：判定 VER=44 里 s_q_ms 到底等于什么。

已知（日志内）：mag_f(42) 原始磁样本、ekf_q(89) 当前姿态、mag_r(119) 固件算出的残差。
待判：固件里 Bn = R(q_?) f_s 中的 q_? 是什么。

H1: Bn = R(q_now) f_s            (等价于"完全不补偿陈旧")
H2: Bn = R(q_now)^T f_s          (转置/逆)
H3: Bn = R(q_s) f_s 且 q_s = q_now  -> 与 H1 相同

若 H1 ~= 119(172度)  -> s_q_ms 就是 q_now，180 度来自别处（EkF 姿态本身）
若 H1 很小而 119 = 172 -> s_q_ms != q_now，存储/取用有 bug
"""
import numpy as np

b = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 122).astype(np.float64)
D = -7.53  # V5F_MAG_DECL_DEG


def q2R(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - w * z); R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y); R[:, 2, 1] = 2 * (y * z + w * x); R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


q = b[:, 89:93]
f = b[:, 42:45]
R = q2R(q)
Bn = np.einsum('nij,nj->ni', R, f)
BnT = np.einsum('nji,nj->ni', R, f)          # R^T f

az = np.degrees(np.arctan2(Bn[:, 0], Bn[:, 1]))
azT = np.degrees(np.arctan2(BnT[:, 0], BnT[:, 1]))
r_h1 = wrap(D - az)
r_h2 = wrap(D - azT)
r_fw = b[:, 119]

bh_h1 = np.hypot(Bn[:, 0], Bn[:, 1])
bh_fw = b[:, 118]

g = np.linalg.norm(b[:, 26:29], axis=1)
rest = g < 5.0
print('静止帧 %d / %d' % (rest.sum(), len(b)))
print()
print('  量                        静止p50    全场p50    固件p50')
print('  |r| H1 R(q_now) f_s      %8.3f  %8.3f' % (np.median(np.abs(r_h1[rest])), np.median(np.abs(r_h1))))
print('  |r| H2 R(q_now)^T f_s    %8.3f  %8.3f' % (np.median(np.abs(r_h2[rest])), np.median(np.abs(r_h2))))
print('  |r| 固件(119)             %8.3f  %8.3f' % (np.median(np.abs(r_fw[rest])), np.median(np.abs(r_fw))))
print()
print('  bh H1                    %8.4f  %8.4f' % (np.median(bh_h1[rest]), np.median(bh_h1)))
print('  bh 固件(118)             %8.4f  %8.4f' % (np.median(bh_fw[rest]), np.median(bh_fw)))
print()
print('  磁样本模 |f|             %8.4f  %8.4f' % (np.median(np.linalg.norm(f[rest], axis=1)),
                                                    np.median(np.linalg.norm(f, axis=1))))
# 固件残差 vs H1 的一致性（同号同幅？）
ok = np.abs(r_fw) > 1e-6
print('  固件与 H1 之差 p50       %8.3f 度' % np.median(np.abs(wrap(r_fw[ok] - r_h1[ok]))))
print('  固件与 H2 之差 p50       %8.3f 度' % np.median(np.abs(wrap(r_fw[ok] - r_h2[ok]))))
print()
print('  ekf yaw 静止段 p50       %8.2f 度' % np.median(np.degrees(np.arctan2(
    2 * (q[rest, 0] * q[rest, 3] + q[rest, 1] * q[rest, 2]),
    1 - 2 * (q[rest, 2] ** 2 + q[rest, 3] ** 2)))))
