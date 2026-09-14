# -*- coding: utf-8 -*-
"""
±16 g 档（2048 LSB/g）六面标定：零偏 + 三轴倍率比例。
本地重力 g = 9.7985 m/s²（用户给出），不用标准 9.80665。

记录：7 路 float = q(4) + accel_raw(3) LSB，帧尾 00 00 80 7F，32 B
成对法（±同轴两次）：
    b            = (y+ + y-) / 2                 [LSB]
    标度 S [LSB/(m/s²)] = (|y+| + |y-|) / 2 / g
    标度 [LSB/g]        = (|y+| + |y-|) / 2       （与 g 无关）
对放置倾斜二阶稳健；用矢量模量 |a|-g 做绝对校验。
"""
import re
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=170)
G = 9.7985                       # 本地重力 m/s^2
LSB_PER_G_NOM = 2048.0           # ±16 g 档标称
RX = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*)+)')
FN = 'serial_runtime_20260913_225736_268_export.txt'
fps = 8027.0

frames = RX.findall(open(FN, errors='ignore').read())
b = b''.join(bytes.fromhex(s.replace(' ', '')) for s in frames)
a = np.frombuffer(b, dtype='<f4').reshape(-1, 8)[:, :7].astype(np.float64)
q = a[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
acc = a[:, 4:7]
N = len(q)
print("帧数 %d  时长 %.1f s  g=%.4f m/s^2  标称 %.0f LSB/g" % (N, N/fps, G, LSB_PER_G_NOM))

rate = np.degrees(2*np.arccos(np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)))/ (1/fps)
amag = np.linalg.norm(acc, axis=1)/LSB_PER_G_NOM
print("削顶检查: |a|max %.3f g (%.0f LSB)  打到 ±32760 的帧 %d   |w|max %.0f dps"
      % (amag.max(), np.abs(acc).max(), int((np.abs(acc) >= 32760).sum()), rate.max()))

# ---- 静止段 ----
still = rate < 10.0
segs = []
i = 0
while i < len(still):
    if still[i]:
        j = i
        while j < len(still) and still[j]:
            j += 1
        if (j-i)/fps > 0.8:
            segs.append((i, j))
        i = j
    else:
        i += 1

print("\n" + "=" * 108)
print("静止段（用陀螺 |w|<10dps 连续 >0.8s 判定）")
print("=" * 108)
print("%-4s %8s %7s %10s %10s %10s %8s %9s" %
      ('#', 't0(s)', 'dur(s)', 'ax LSB', 'ay LSB', 'az LSB', '|a| g', '主轴'))
S6 = []
for k, (s, e) in enumerate(segs):
    st = slice(s+int(0.2*fps), e-int(0.2*fps))
    if st.stop <= st.start:
        st = slice(s, e)
    m = acc[st].mean(0)
    ax_ = int(np.argmax(np.abs(m)))
    S6.append(m)
    print("%-4d %8.2f %7.2f %10.1f %10.1f %10.1f %10.5f %8s%+d"
          % (k, s/fps, (e-s)/fps, m[0], m[1], m[2], np.linalg.norm(m)/LSB_PER_G_NOM,
             'xyz'[ax_], int(np.sign(m[ax_]))))
S6 = np.array(S6)

# ---- 六面配对：按主轴+符号归类，各取一组 ----
print("\n" + "=" * 108)
print("成对法标定（本地 g = %.4f m/s^2）" % G)
print("=" * 108)
pick = {}
for i, m in enumerate(S6):
    k = int(np.argmax(np.abs(m)))
    key = ('xyz'[k], 1 if m[k] > 0 else -1)
    pick.setdefault(key, []).append((i, m, k))

B = np.zeros(3); S = np.zeros(3)
print("  轴    y+(LSB)      y-(LSB)      零偏[LSB]    零偏[mg]     标度[LSB/g]   相对2048")
for ax in 'xyz':
    k = 'xyz'.index(ax)
    gp = pick.get((ax, 1)); gn = pick.get((ax, -1))
    if not gp or not gn:
        print("   %s   缺 %s 面" % (ax, 'y+' if not gp else 'y-')); continue
    yp = np.mean([m[k] for _, m, _ in gp])
    yn = np.mean([m[k] for _, m, _ in gn])
    B[k] = (yp + yn)/2.0
    sc = (abs(yp) + abs(yn))/2.0                       # LSB/g
    S[k] = sc / G                                      # LSB/(m/s^2)
    print("   %s  %+10.2f  %+10.2f  %+10.2f  %+9.3f   %10.3f   %+8.4f %%"
          % (ax, yp, yn, B[k], B[k]/LSB_PER_G_NOM*1000, sc, (sc/LSB_PER_G_NOM-1)*100))

print("\n  矢量模量校验（校正后 |a| 应为 %.4f m/s^2 = %.4f g）：" % (G, 1.0))
for i, m in enumerate(S6):
    f = (m - B)/S
    print("     段%-2d  |a| = %.5f m/s^2   偏差 %+8.3f mg   (原始 |a| = %.5f g)"
          % (i, np.linalg.norm(f), (np.linalg.norm(f)-G)/G*1000, np.linalg.norm(m)/LSB_PER_G_NOM))

print("\n  放置偏轴角（去零偏后，反映夹具质量）：")
for i, m in enumerate(S6):
    f = (m - B)/S
    k = int(np.argmax(np.abs(f)))
    off = np.degrees(np.arctan2(np.linalg.norm(np.delete(f, k)), abs(f[k])))
    print("     段%-2d 主轴 %s%+d   偏轴 %.3f deg" % (i, 'xyz'[k], int(np.sign(f[k])), off))
