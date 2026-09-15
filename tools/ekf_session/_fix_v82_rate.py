# -*- coding: utf-8 -*-
"""
VER=82：按实测的"非磁力部分漂移速度"把磁环放慢，降低偏航抖动

实测（VER=81，41.87 s / 148 列）：
  f_upd = 196.6 Hz（磁物理率，已是上限，不再提高）
  磁航向单次测量噪声 s = 0.4203 度（连续更新差估计，lag1 自相关 +0.206）
  非磁漂移速度 d：静止 0.060 度/秒（VER=73/74 十二度死区、环路休眠时实测
                  14 s 漂 0.84 度）；强激励段 0.18 度/秒（15 s 累积 2.72 度）
  偏航抖动（静止段二阶差分 std）：EKF 0.00545 vs 旧链 0.00051 度/帧^2 = 10.7 倍
  当前环路 τ = 1/(k_cap*f_upd) = 0.1017 s（k_cap=0.05）

设计（一阶环总误差 = d*τ + s*sqrt(1/(2τf))）：
  τ      k_cap    漂移残差(d=0.12)  噪声抖动   合计
  0.102  0.05     0.012            0.067     0.079   <- 现在
  0.34   0.015    0.041            0.036     0.077   <- 本版
  0.51   0.010    0.061            0.030     0.091
  解析最优 τ_opt=(s^2/(2 f d^2))^(1/3): d=0.06 -> 0.50 s ; d=0.12 -> 0.32 s
  => 非磁漂移只有 0.06~0.18 度/秒（远小于 0.42 度的测量噪声），环路可以大幅放慢；
     放慢只按 sqrt(τ) 换抖动下降，而漂移残差仍远小于测量噪声本身。

改动：V5F_EKF_MAG_K_MAX 0.05 -> 0.015（τ 0.102 -> 0.34 s，抖动降 1.9 倍）。
      更新率、掩码、H、观测形式、其余门/常量全部不动。
版本 81 -> 82。
"""
import os, shutil
ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')

def load(p):
    with open(p, 'rb') as f: return f.read().decode(ENC)
def save(p, s, tag):
    d = s.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f: f.write(d)
    with open(p, 'rb') as f: assert f.read().decode(ENC) == s
    os.utime(p, None); return len(d)
def sub1(s, a, b, what):
    n = s.count(a); assert n == 1, '[%s] 命中 %d 次' % (what, n)
    return s.replace(a, b, 1)

t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        81u', '#define V5F_FW_VER        82u', 'VER')

old = [l for l in t.split('\n') if l.startswith('#define V5F_EKF_MAG_K_MAX')]
assert len(old) == 1, old
print('原行: %s' % old[0])
new = ('/* ★VER=82 0.05 -> 0.015：按实测的非磁漂移速度把环路放慢，降偏航抖动。\n'
       ' * 一阶环总误差 = d*tau + s*sqrt(1/(2*tau*f))：\n'
       ' *   d(非磁漂移) = 0.060 度/秒(静止, VER=73/74 死区休眠时实测) ~ 0.18(强激励)\n'
       ' *   s(磁航向单次噪声) = 0.4203 度  ;  f = 196.6 Hz\n'
       ' *   tau=0.102(k=0.05) -> 漂移 0.012 + 抖动 0.067 = 0.079\n'
       ' *   tau=0.34 (k=0.015)-> 漂移 0.041 + 抖动 0.036 = 0.077  <- 本值\n'
       ' * 即：漂移残差远小于测量噪声本身，环路可大幅放慢，换来 sqrt(tau) 的抖动下降。\n'
       ' * 实测抖动(静止段偏航二阶差分 std) EKF 0.00545 vs 旧链 0.00051 度/帧^2。 */\n'
       '#define V5F_EKF_MAG_K_MAX        0.015f')
t = sub1(t, old[0], new, 'K_MAX')
n = save(TUNE, t, 'v82')

t2 = load(TUNE)
assert '#define V5F_FW_VER        82u' in t2
assert '0.015f' in t2
for l in t2.split('\n'):
    if l.startswith('#define V5F_EKF_MAG_K_MAX') or l.startswith('#define V5F_FW_VER'):
        print('  ' + l.strip())
print('v5f_tune.h %d 字节' % n)
import math
print('  新 tau = %.4f s (f=196.6Hz) ; 预期抖动 %.4f 度 (原 0.0665) ; 漂移残差(d=0.12) %.4f 度'
      % (1/(0.015*196.6), 0.4203*math.sqrt(1/(2*(1/(0.015*196.6))*196.6)), 0.12/(0.015*196.6)))
print('PASS: VER=82 已写入')
