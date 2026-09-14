# -*- coding: utf-8 -*-
"""val_ekf.py 的转动段检出重写：原来用"每帧偏航变化 > 0.05 度"，慢转时每帧只有
0.01 度 -> 永远检不出来（实测转动帧占比报 0.0%）。改用**陀螺 z 速率的平滑值**
判转动段，并直接输出每段 "EKF 转角 / 旧链转角" 的比值 —— 这是判断预积分是否
把每一帧都积进去的最硬指标（VER=14 实测比值 0.7115，理论 16/23=0.6957）。"""
P = r'C:\Users\33\Documents\v2\tools\calib\val_ekf.py'
t = open(P, encoding='utf-8').read()

a = """# 分段：找出旧链偏航的台阶（转动）
dyl = np.diff(yl)
mv = np.abs(dyl) > 0.05
print('  旧链偏航总转角 %.2f 度，转动帧占比 %.1f%%' % (np.abs(dyl).sum(), 100.0*mv.mean()))
if mv.sum() > 10:
    # 用 0.2 s 窗口比较两路的增量（与常数偏置无关）
    k = max(int(0.2/dt.mean()), 1)
    cum_e = np.concatenate([[0], np.cumsum(dyl * 0 + np.diff(ye))])
    cum_l = np.concatenate([[0], np.cumsum(dyl)])
    # 在每段内部比较增量
    idx = np.where(np.diff(mv.astype(np.int8)) != 0)[0]
    segs = [(idx[i], idx[i+1]) for i in range(len(idx)-1)]
    tot_e = tot_l = 0.0
    print('  转动段（旧链为准）与 EKF 的转角对比：')
    for s0, s1 in segs:
        if s1 - s0 < 20 or abs(yl[s1] - yl[s0]) < 5.0:
            continue
        de = wrap(ye[s1] - ye[s0]); dl = wrap(yl[s1] - yl[s0])
        tot_e += de; tot_l += dl
        print('    t=%6.2f~%6.2f s  旧链 %+8.2f 度   EKF %+8.2f 度   差 %+7.3f 度'
              % (t[s0], t[s1], dl, de, de - dl))
    if tot_l:
        print('    合计: 旧链 %+.2f 度  EKF %+.2f 度  累计差 %+.3f 度（%.4f%%）'
              % (tot_l, tot_e, tot_e - tot_l, 100*(tot_e - tot_l)/tot_l))"""

b = """# 分段：转动段由**陀螺 z 速率**判定（慢转时每帧偏航只变 0.01 度，按帧差永远检不出）
gz = a[:, 28]                       # 旧链校正后角速度 z，dps
w3 = np.linalg.norm(a[:, 26:29], axis=1)
# 0.1 s 滑窗平滑，去掉单帧毛刺
k = max(int(0.1 / dt.mean()), 1)
sm = np.convolve(w3, np.ones(k)/k, mode='same')
turn = sm > 8.0                     # dps：超过它算"在转"
idx = np.where(np.diff(turn.astype(np.int8)) != 0)[0]
segs = []
for i in range(len(idx) - 1):
    s0, s1 = idx[i], idx[i+1]
    if turn[min(s0+1, len(turn)-1)] and (s1 - s0) > k:
        segs.append((s0, s1))
print('  转动段数 %d（判据：0.1 s 平滑 |w| > 8 dps）  转动帧占比 %.1f%%'
      % (len(segs), 100.0*turn.mean()))
print('  每段：EKF 转角 / 旧链转角   —— 纯陀螺跟踪能力，与磁无关')
tot_e = tot_l = 0.0
for s0, s1 in segs:
    de = yeu[s1] - yeu[s0]
    dl = ylu[s1] - ylu[s0]
    if abs(dl) < 10.0:
        continue
    tot_e += de; tot_l += dl
    print('    t=%6.2f~%6.2f s (%5.2f s)  旧链 %+8.2f 度  EKF %+8.2f 度  '
          '比值 %6.4f  差 %+7.3f 度'
          % (t[s0], t[s1], t[s1]-t[s0], dl, de, de/dl if dl else float('nan'), de-dl))
if tot_l:
    print('    合计 旧链 %+.2f  EKF %+.2f  比值 %6.4f  累计差 %+.3f 度（%.4f%%）'
          % (tot_l, tot_e, tot_e/tot_l, tot_e - tot_l, 100*(tot_e-tot_l)/tot_l))
    print('    ★ 比值应 ~1.0000。VER=14 实测 0.7115（= 16/23，预积分漏了 7 帧）')"""

assert t.count(a) == 1, t.count(a)
t = t.replace(a, b, 1)

# yaw 展开必须有，插到 yaw 定义之后
a2 = """isr = a[:, 17]"""
assert t.count(a2) == 1
t = t.replace(a2, """# 展开后的连续偏航（去 ±180 跳变），用于按段比较转角
yeu = np.degrees(np.unwrap(np.radians(ye)))
ylu = np.degrees(np.unwrap(np.radians(yl)))

isr = a[:, 17]""", 1)

open(P, 'w', encoding='utf-8', newline='\n').write(t)
import ast
ast.parse(t)
print('val_ekf.py: 转动段检出改为速率判据 + 输出 EKF/旧链 转角比值')
