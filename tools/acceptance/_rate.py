# -*- coding: utf-8 -*-
"""VER=100 CDC(JustFloat) 录像：量帧率、姿态真实更新率、每个更新重复的帧数。

用途：确认"采样率 vs 估计器速率"是否一致（抽查 23:1 硬抽取）。
usage: python _rate.py <export.txt> [duration_s]
"""
import sys, re, numpy as np

path = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else None

rows = []
for ln in open(path, 'r', errors='replace'):
    m = re.search(r'\[RX\]\s*([0-9A-Fa-f ]+)', ln)
    if not m:
        continue
    h = m.group(1).replace(' ', '')
    if len(h) >= 40:
        rows.append(bytes.fromhex(h[:40]))

n = len(rows)
a = np.frombuffer(b''.join(rows), dtype='<f4').reshape(n, 5)
q = a[:, :4]
tail = a[:, 4].view(np.uint32) if hasattr(a[:, 4], 'view') else None

ch = np.abs(np.diff(q, axis=0)).sum(1) > 0
idx = np.flatnonzero(np.concatenate(([True], ch, [True])))
runs = np.diff(idx)

print('frames        %d' % n)
print('tail==0x7F800000  %s' % (np.all(a[:, 4] == np.float32(np.inf)) if tail is None else ''))
if dur:
    print('frame rate    %.1f Hz' % (n / dur))
print('q updates     %d  (%.3f of frames)' % (int(ch.sum()), ch.mean()))
if dur:
    print('attitude rate %.1f Hz' % (ch.sum() / dur))
print('dup frames    %d  (%.1f%%)' % (n - ch.sum(), 100.0 * (n - ch.sum()) / n))
print('runlen        mean %.2f  median %g  p10 %g  p90 %g  max %g'
      % (runs.mean(), np.median(runs), np.percentile(runs, 10),
         np.percentile(runs, 90), runs.max()))
print('|q|-1 max     %.3e' % np.abs(np.linalg.norm(q, axis=1) - 1.0).max())
