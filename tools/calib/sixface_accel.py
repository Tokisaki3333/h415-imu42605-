# -*- coding: utf-8 -*-
"""
加速度计六面标定 + 第 7 组（回初始平面）的四元数漂移评估。

★ 本脚本与下面 7 条记录**绑定在 ±4 g 档（8192 LSB/g）**，脚本里的 8192 和标度换算
  都是那一档的。固件已改成 ±16 g（2048 LSB/g，见 spi_hw.c），
  **新数据必须先把 8192 全改成 2048，并重解 S/B**，否则标度差 4 倍（静默失效）。

记录：7 路 float = q(w,x,y,z) + accel_raw(3) LSB，帧尾 00 00 80 7F，32 B/帧
      （32 B = 7 个 float 数据 + 4 B 帧尾，帧尾也占一个 float 槽 -> 按 8 列再切 7 列）

模型：y = g·M·u + b     y = 实测 LSB，u = 真重力在机体系方向(±体轴)，M = 标度+交叉，b = 零偏
  成对法（±g 同轴两次）：b = (y+ + y-)/2，S·g = (|y+| + |y-|)/2   —— 对放置倾斜二阶稳健
  12 参数全拟合：6 面 x 3 = 18 方程解 12 未知，可解但会被放置误差污染
"""
import re
import numpy as np

np.set_printoptions(precision=6, suppress=True, linewidth=160)
G0 = 9.80665
RX = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*)+)')

F = [('222919_239', 'z+'), ('222934_128', 'z-'), ('222951_145', 'y-'),
     ('223007_913', 'y+'), ('223026_247', 'x+'), ('223107_311', 'x-'),
     ('223149_605', 'z+ 回初始平面')]
PRE = 'serial_runtime_20260913_'


def load(tag):
    frames = RX.findall(open(PRE + tag + '_export.txt', errors='ignore').read())
    b = b''.join(bytes.fromhex(s.replace(' ', '')) for s in frames)
    a = np.frombuffer(b, dtype='<f4').reshape(-1, 8)[:, :7].astype(np.float64)
    q = a[:, :4]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q[4000:-4000], a[4000:-4000, 4:7]        # 丢首尾 0.5 s


def qmul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                     a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])


def rv(q):
    s = 1.0 if q[0] >= 0 else -1.0
    v = q[1:] * s
    n = np.linalg.norm(v)
    return np.zeros(3) if n < 1e-14 else np.degrees(2*np.arctan2(n, abs(q[0]))) * v / n


def q_rp(q):
    w, x, y, z = q
    return (np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))),
            np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1))))


def a_rp(f):
    return (np.degrees(np.arctan2(f[1], f[2])),
            np.degrees(np.arctan2(-f[0], np.hypot(f[1], f[2]))))


D = {}
for tag, face in F:
    q, acc = load(tag)
    m = acc.mean(0)
    D[face] = dict(q=q, m=m, tag=tag,
                   noise=np.linalg.norm(acc - m, axis=1).std()*1000/8192.0)

# ---------- 一、成对法（稳健） ----------
S = np.zeros(3); B = np.zeros(3)
for i, ax in enumerate('xyz'):
    yp, yn = D['%s+' % ax]['m'][i], D['%s-' % ax]['m'][i]
    B[i] = (yp + yn) / 2.0
    S[i] = (abs(yp) + abs(yn)) / 2.0 / G0
Sc = S * G0                                    # LSB/g

print("=" * 104)
print("一、各面实测 + 成对法标定（丢首尾 0.5 s）")
print("=" * 104)
print("%-20s %5s %8s %8s %8s %7s %7s %9s %9s" %
      ('记录', '主轴', 'y_x', 'y_y', 'y_z', '|a|g', '噪声mg', '偏轴(原始)', '偏轴(去零偏)'))
for tag, face in F:
    d = D[face]
    m = d['m']
    k = int(np.argmax(np.abs(m)))
    raw = np.degrees(np.arctan2(np.linalg.norm(np.delete(m, k)), abs(m[k])))
    mm = (m - B) / S
    kk = int(np.argmax(np.abs(mm)))
    cor = np.degrees(np.arctan2(np.linalg.norm(np.delete(mm, kk)), abs(mm[kk])))
    d['k'] = k; d['raw'] = raw; d['cor'] = cor
    print("%-20s %5s %8.2f %8.2f %8.2f %7.5f %7.3f %9.3f %9.3f"
          % (tag, face, m[0], m[1], m[2], np.linalg.norm(m)/8192.0, d['noise'], raw, cor))

print("\n  成对法结果（对放置倾斜二阶稳健：3.3 deg 只带 0.17%% 标度误差、0.8 mg 零偏误差）")
for i, ax in enumerate('xyz'):
    print("   %s: 零偏 %+8.3f LSB = %+7.3f mg      标度 %9.3f LSB/g (%+7.3f %%)"
          % (ax, B[i], B[i]/8192.0*1000, Sc[i], (Sc[i]/8192.0-1)*100))

# ---------- 二、12 参数全拟合（含交叉项） ----------
U = np.zeros((6, 3))
for i, (tag, face) in enumerate(F[:6]):
    k = D[face]['k']
    U[i, k] = 1.0 if D[face]['m'][k] > 0 else -1.0
Y = np.array([D[f]['m'] for _, f in F[:6]])
A = np.hstack([G0*U, np.ones((6, 1))])
M = np.zeros((3, 3)); Bf = np.zeros(3)
for k in range(3):
    sol, *_ = np.linalg.lstsq(A, Y[:, k], rcond=None)
    M[k, :] = sol[:3]; Bf[k] = sol[3]

print("\n" + "=" * 104)
print("二、12 参数全拟合（对角+零偏+对称/非对称交叉）")
print("=" * 104)
print("  对角 LSB/g = %s   偏差 %s %%"
      % (np.array2string(np.diag(M)*G0, precision=3),
         np.array2string((np.diag(M)*G0/8192.0-1)*100, precision=4)))
print("  零偏 = %s LSB = %s mg" % (np.array2string(Bf, precision=3),
                                  np.array2string(Bf/8192.0*1000, precision=3)))
print("  交叉项 %%（对同行对角归一）:")
for i in range(3):
    for j in range(3):
        if i != j:
            print("     %s <- %s : %+7.4f %%" % ('xyz'[i], 'xyz'[j], M[i, j]/M[i, i]*100))
print("  fit 残差:")
Mi = np.linalg.inv(M)
for i, (tag, face) in enumerate(F[:6]):
    f = Mi @ (D[face]['m'] - Bf)
    print("     %-12s 校正后 [%9.4f %9.4f %9.4f] m/s^2  主轴偏差 %+8.4f mg  水平 %7.4f mg"
          % (tag, f[0], f[1], f[2],
             (f[int(np.argmax(np.abs(U[i])))] - U[i][int(np.argmax(np.abs(U[i])))]*G0)*1000/G0,
             np.linalg.norm(f - np.dot(f, U[i])*U[i])*1000/G0))

# ---------- 三、第 7 组：四元数漂移评估 ----------
print("\n" + "=" * 104)
print("三、第 7 组（回初始平面）—— 四元数漂移评估")
print("=" * 104)
f1 = (D['z+']['m'] - B) / S
f7 = (D['z+ 回初始平面']['m'] - B) / S
r1a, p1a = a_rp(f1); r7a, p7a = a_rp(f7)
r1q, p1q = q_rp(D['z+']['q'].mean(0)); r7q, p7q = q_rp(D['z+ 回初始平面']['q'].mean(0))
print("  第1组        加速度计 roll %+8.4f pitch %+8.4f | 四元数 roll %+8.4f pitch %+8.4f"
      % (r1a, p1a, r1q, p1q))
print("  第7组        加速度计 roll %+8.4f pitch %+8.4f | 四元数 roll %+8.4f pitch %+8.4f"
      % (r7a, p7a, r7q, p7q))
print("\n  放置重复性（两次同一平面）: droll %+7.4f  dpitch %+7.4f  合成 %.4f deg"
      % (r7a-r1a, p7a-p1a, np.hypot(r7a-r1a, p7a-p1a)))
print("  四元数看到的倾角变化      : droll %+7.4f  dpitch %+7.4f  合成 %.4f deg"
      % (r7q-r1q, p7q-p1q, np.hypot(r7q-r1q, p7q-p1q)))
dr, dp = (r7q-r1q)-(r7a-r1a), (p7q-p1q)-(p7a-p1a)
print("\n  ★ 测量期间四元数相对加速度计基准的倾角漂移 = (q7-q1)-(a7-a1)")
print("     droll %+7.4f  dpitch %+7.4f   合成 %.4f deg" % (dr, dp, np.hypot(dr, dp)))

print("\n  各组内静态段四元数自身漂移（段首->段尾）：")
for tag, face in F:
    q = D[face]['q']
    e = rv(qmul(np.array([q[0, 0], -q[0, 1], -q[0, 2], -q[0, 3]]), q[-1]))
    t = len(q)/8027.0
    print("     %-20s %5.1f s  漂 %.4f deg  (%.1f deg/h)  分量 [%+.4f %+.4f %+.4f]"
          % (tag, t, np.linalg.norm(e), np.linalg.norm(e)/(t/3600), e[0], e[1], e[2]))
