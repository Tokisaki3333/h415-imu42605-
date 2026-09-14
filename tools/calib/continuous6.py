# -*- coding: utf-8 -*-
"""
连续六面搬运（不关串口）—— 直接查搬运过程：
  ★ 本脚本绑定 ±4 g 档（8192 LSB/g）的旧记录；固件已改 ±16 g，
    新数据要把 8192 改成 2048，S/B 也要重解。
  1) 有没有磕碰：加速度削顶(±4g=±32768)、角速度削顶(±2000dps)
  2) 逐静止段的倾角误差（方向无关，加速度计当真值）
  3) 末次静止 vs 首次静止的 yaw（z）为何回不去

帧：7 路 float = q(4) + accel_raw(3)，32 B（8 个 float 槽，最后一槽是帧尾）
"""
import re
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=160)
G0 = 9.80665
RX = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*)+)')
FN = 'serial_runtime_20260913_224354_704_export.txt'
S = np.array([8114.619, 8163.620, 8067.570]) / G0      # LSB/(m/s^2)，六面成对法
B = np.array([-38.357, -56.534, 184.037])              # LSB

frames = RX.findall(open(FN, errors='ignore').read())
b = b''.join(bytes.fromhex(s.replace(' ', '')) for s in frames)
a = np.frombuffer(b, dtype='<f4').reshape(-1, 8)[:, :7].astype(np.float64)
q = a[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
acc = a[:, 4:7]
N = len(q); fps = 8027.0; dt = 1.0/fps
print("帧数 %d  时长 %.1f s  fps %.0f" % (N, N/fps, fps))

# 角速度（四元数增量）
dot = np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)
rate = np.degrees(2*np.arccos(dot))/dt
amag = np.linalg.norm(acc, axis=1)/8192.0

print("\n" + "=" * 90)
print("一、有没有削顶（磕碰特征）")
print("=" * 90)
print("  加速度 |a|max = %.4f g  (%.0f LSB)   打到 ±4g 的帧数 = %d"
      % (amag.max(), np.abs(acc).max(), int((np.abs(acc) >= 32760).sum())))
print("  角速度 |w|max = %.1f dps               打到 ±2000dps 的帧数 = %d"
      % (rate.max(), int((rate >= 1990).sum())))
print("  角速度分位: p50 %.1f  p90 %.1f  p99 %.1f  p99.9 %.1f dps"
      % tuple(np.percentile(rate, [50, 90, 99, 99.9])))
big = np.flatnonzero(amag > 1.5)
print("  |a| > 1.5 g 的帧数 = %d" % len(big))
if len(big):
    print("    出现时刻(s): %s" % np.array2string(big[:20]/fps, precision=2))
    print("    这些帧的 |a| (g): %s" % np.array2string(amag[big[:20]], precision=3))

print("\n" + "=" * 90)
print("二、静止段划分（|w| < 10 dps 连续 > 0.5 s）")
print("=" * 90)
still = rate < 10.0
segs = []
i = 0
while i < len(still):
    if still[i]:
        j = i
        while j < len(still) and still[j]:
            j += 1
        if (j-i)/fps > 0.5:
            segs.append((i, j))
        i = j
    else:
        i += 1
print("  共 %d 段静止" % len(segs))


def Rmat(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def yaw(q):
    w, x, y, z = q
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


print("\n%-4s %8s %8s %8s %9s %9s %9s %9s" %
      ('#', 't0(s)', 'dur(s)', '|a|g', '倾角误差', 'yaw', 'a_x LSB', 'a_z LSB'))
info = []
for k, (s, e) in enumerate(segs):
    st = slice(s+max(int(0.2*fps), 0), e-max(int(0.2*fps), 0))
    if st.stop <= st.start:
        st = slice(s, e)
    m = acc[st].mean(0)
    qm = q[st].mean(0); qm /= np.linalg.norm(qm)
    f = (m - B)/S
    u_meas = f/np.linalg.norm(f)
    u_est = Rmat(qm).T @ np.array([0.0, 0.0, 1.0])
    err = np.degrees(np.arccos(np.clip(np.dot(u_meas, u_est), -1, 1)))
    info.append(dict(t0=s/fps, dur=(e-s)/fps, q=qm, acc=m, err=err, yaw=yaw(qm)))
    print("%-4d %8.2f %8.2f %8.4f %9.4f %9.3f %9.1f %9.1f"
          % (k, s/fps, (e-s)/fps, np.linalg.norm(m)/8192.0, err, yaw(qm), m[0], m[2]))

if len(info) >= 2:
    print("\n" + "=" * 90)
    print("三、首末对比")
    print("=" * 90)
    a0, a1 = info[0], info[-1]
    f0, f1 = (a0['acc']-B)/S, (a1['acc']-B)/S
    print("  加速度计倾角 首: roll %+7.4f pitch %+7.4f | 末: roll %+7.4f pitch %+7.4f"
          % (np.degrees(np.arctan2(f0[1], f0[2])), np.degrees(np.arctan2(-f0[0], np.hypot(f0[1], f0[2]))),
             np.degrees(np.arctan2(f1[1], f1[2])), np.degrees(np.arctan2(-f1[0], np.hypot(f1[1], f1[2])))))
    print("  四元数倾角   首: roll %+7.4f pitch %+7.4f | 末: roll %+7.4f pitch %+7.4f"
          % (np.degrees(np.arctan2(2*(a0['q'][0]*a0['q'][1]+a0['q'][2]*a0['q'][3]), 1-2*(a0['q'][1]**2+a0['q'][2]**2))),
             np.degrees(np.arcsin(np.clip(2*(a0['q'][0]*a0['q'][2]-a0['q'][3]*a0['q'][1]), -1, 1))),
             np.degrees(np.arctan2(2*(a1['q'][0]*a1['q'][1]+a1['q'][2]*a1['q'][3]), 1-2*(a1['q'][1]**2+a1['q'][2]**2))),
             np.degrees(np.arcsin(np.clip(2*(a1['q'][0]*a1['q'][2]-a1['q'][3]*a1['q'][1]), -1, 1)))))
    print("\n  倾角误差  首 %.4f deg  ->  末 %.4f deg   增量 %+.4f deg"
          % (a0['err'], a1['err'], a1['err']-a0['err']))
    print("  yaw       首 %+7.3f deg -> 末 %+7.3f deg   (加速度计测不到 yaw，仅供参考)"
          % (a0['yaw'], a1['yaw']))
    print("\n  逐段倾角误差增量：")
    for i in range(1, len(info)):
        print("    %d -> %d   %.4f -> %.4f   %+.4f deg   (间隔 %5.2f s，其间最大|w| %.0f dps)"
              % (i-1, i, info[i-1]['err'], info[i]['err'], info[i]['err']-info[i-1]['err'],
                 info[i]['t0']-(info[i-1]['t0']+info[i-1]['dur']),
                 rate[int(info[i-1]['t0']*fps):int(info[i]['t0']*fps)+1].max() if i < len(info) else 0))
