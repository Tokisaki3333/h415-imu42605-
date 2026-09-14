# -*- coding: utf-8 -*-
"""170 度框架误差的归属：旧链 vs EKF。

122 通道下（CH_112）：att q=0, gyro=26, mag_f=42, mag_Bw=46, psi_true=49, ekf_q=89。
判据：
  r_legacy = wrap(D - atan2(mag_Bw[0], mag_Bw[1]))
  r_ekf    = 119 列（固件算的）
若 r_legacy ~ 0  -> 旧链正常，170 度是 EKF 自己的框架/对齐错
若 r_legacy ~ 170 -> psi_true/atan2 约定本身有问题（两边共用），不是 EKF 的锅
"""
import numpy as np

b = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 122).astype(np.float64)
D = -7.53


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


g = np.linalg.norm(b[:, 26:29], axis=1)
rest = g < 5.0

ql = b[:, 0:4]
qe = b[:, 89:93]
Bw = b[:, 46:49]                 # 旧链导航系地磁
psi_true = b[:, 49]
r_ekf = b[:, 119]

az_leg = np.degrees(np.arctan2(Bw[:, 0], Bw[:, 1]))
r_leg_d = wrap(D - az_leg)
r_leg_psi = wrap(D - psi_true)

print('静止帧 %d/%d' % (rest.sum(), len(b)))
print()
print('  量                                静止p50    全场p50    全场p90')
def pr(nm, v):
    print('  %-30s %8.2f  %8.2f  %8.2f' % (nm, np.median(v[rest]), np.median(v), np.percentile(v, 90)))

pr('r_ekf (119列, vs D)', r_ekf)
pr('r_legacy = D - atan2(Bw)', r_leg_d)
pr('r_legacy = D - psi_true', r_leg_psi)
pr('|Bw| 模', np.linalg.norm(Bw, axis=1))
pr('Bw 俯仰角 (deg)', np.degrees(np.arcsin(np.clip(Bw[:, 2] / np.maximum(np.linalg.norm(Bw, axis=1), 1e-9), -1, 1))))
pr('psi_true', psi_true)
pr('yaw(旧链 att q)', yaw(ql))
pr('yaw(ekf_q)', yaw(qe))
pr('yaw(ekf) - yaw(legacy)', wrap(yaw(qe) - yaw(ql)))
pr('yaw(ekf) - psi_true', wrap(yaw(qe) - psi_true))

print('\n  ---- 关键对照 ----')
print('  r_ekf 与 r_legacy(atan2 Bw) 之差 p50 %.2f 度' % np.median(np.abs(wrap(r_ekf - r_leg_d))))
print('  旧链 att q 与 ekf_q 的夹角 p50 %.2f 度' % np.median(np.degrees(2*np.arccos(np.clip(
    np.abs(np.einsum('ij,ij->i', ql/np.linalg.norm(ql, axis=1, keepdims=True),
                      qe/np.linalg.norm(qe, axis=1, keepdims=True))), -1, 1)))))
