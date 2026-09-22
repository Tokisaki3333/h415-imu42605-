# -*- coding: utf-8 -*-
"""标定 159/160 两列的时间单位: 用同帧的 mag_age_ms (col148, 由固件用 1e-5f 从 10ns 计数算出) 做尺子.
   结论: 差值(159-160) 的量子 = 1, 且 差值 * 1e-3(ms) == col148  =>  单位就是 us, 不需要任何倍数.
"""
import numpy as np, os
P = r'R:\raw_v9.bin'
NCH = 162
raw = np.fromfile(P, dtype='<f4'); N = raw.size//NCH
b = raw[:N*NCH].reshape(N, NCH).astype(np.float64)
u8 = raw[:N*NCH].reshape(N, NCH).view(np.uint8)
xk = np.bitwise_xor.reduce(u8[:, :161*4], axis=1).astype(np.uint16) ^ np.uint16(0x5A5A)
ok = (np.nan_to_num(b[:, 161]) == xk)
ok &= np.isfinite(b).all(1)
ag = (b[:, 159] - b[:, 160]) % (1 << 24)
age = b[:, 148] * 1000.0                      # ms -> us
m = ok & (age > 0)
print('帧 %d  校验通过 %.3f%%  有效 %d' % (N, 100*ok.mean(), m.sum()))
print('  差(159-160) us : 中位 %.1f  p10 %.1f  p90 %.1f' %
      (np.median(ag[m]), np.percentile(ag[m], 10), np.percentile(ag[m], 90)))
print('  col148*1000 us : 中位 %.1f  p10 %.1f  p90 %.1f' %
      (np.median(age[m]), np.percentile(age[m], 10), np.percentile(age[m], 90)))
r = ag[m]/np.maximum(age[m], 1e-9)
print('  比值 差/col148 : 中位 %.5f  p10 %.5f  p90 %.5f  (=1 => 同为 us)' %
      (np.median(r), np.percentile(r, 10), np.percentile(r, 90)))
d = np.diff(ag[m].astype(np.int64)); d = d[d != 0]
from math import gcd
g = 0
for v in np.unique(np.abs(d))[:5000]:
    g = gcd(g, int(v))
print('  相邻非零差额的 gcd = %d  (量子=1 单位; 若是 10us 计数则 gcd>=10)' % g)
print('  差值的唯一取值数 %d, 最小非零 %d, 最大 %d' % (len(np.unique(ag[m])), d.min() if len(d) else 0, d.max() if len(d) else 0))
# 也直接核对两列本身是否整数(us 计数被截断成整数)
for c in (159, 160):
    fr = b[ok, c]
    print('  col%d: 非整数帧 %.3f%%  值域 %.0f..%.0f' % (c, 100*(np.abs(fr-np.round(fr)) > 1e-6).mean(), fr.min(), fr.max()))
