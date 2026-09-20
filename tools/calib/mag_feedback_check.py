# -*- coding: utf-8 -*-
r"""校验磁牵引的**反馈方向**：施加的 dq 到底是把残差拉回去了，还是推得更远。

数据里只有观测量，所以直接做回归（不需要知道固件内部的注入约定）：

  x = rx[k]        （第 k 帧的 yaw 方向磁残差，e1 = gn x b_b 上的分量）
  y = rx[k+1]-rx[k]（下一帧残差的变化）
  ⇒  y ≈ -g*x  g>0 是负反馈（牵引有效）；y ≈ +g*x 是正反馈（越修越大 = 符号/轴向错）

  另一组：y vs 同一帧发布的 dq_yaw 分量（dq 在重力方向上的投影）
  ⇒  斜率 ≈ -1 表示施加的修正半径与残差一致且方向正确；≈ +1 表示方向反了。

用法:
  python tools/calib/mag_feedback_check.py R:\imu_20260921_030649.bin
  python tools/calib/mag_feedback_check.py <bin> --seg 13.9 14.6
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import mag_motion_diag as G


def fit(x, y, tag):
    """返回 (斜率, 相关系数, 样本数)，对 |x|>0.05 的点做最小二乘。"""
    m = np.abs(x) > 0.05
    if m.sum() < 20:
        print('  %-28s 样本太少 (%d)' % (tag, m.sum()))
        return
    xx, yy = x[m], y[m]
    b, a = np.polyfit(xx, yy, 1)
    r = np.corrcoef(xx, yy)[0, 1]
    print('  %-28s n=%5d  斜率 %+7.3f  相关系数 %+6.3f  中位 |x| %.2f°'
          % (tag, m.sum(), b, r, np.median(np.abs(xx))))


def seg_report(D, t0, t1, tag):
    t, rx, ry = D['t'], D['rx'], D['ry']
    k0 = int(np.flatnonzero(t >= t0)[0])
    k1 = int(np.flatnonzero(t <= t1)[-1])
    sl = slice(k0, k1)
    print('%s  t=%.2f~%.2f s（%d 帧）  |w| p50 %.0f dps  used %.1f%%'
          % (tag, t0, t1, k1 - k0, np.percentile(D['w'][sl], 50), 100 * D['used'][sl].mean()))
    fit(rx[sl][:-1], np.diff(rx[sl]), 'y=Δrx  x=rx   (负=有效)')
    fit(ry[sl][:-1], np.diff(ry[sl]), 'y=Δry  x=ry   (负=有效)')
    fit(D['res'][sl][:-1], np.diff(D['res'][sl]), 'y=Δres x=res  (负=有效)')
    dqm = D['used'][sl].astype(bool)[1:]
    dqy = D['dqy'][sl][1:]
    dqt = D['dqt'][sl][1:]
    fit(dqy[dqm], np.diff(rx[sl])[dqm], 'y=Δrx  x=dq_yaw (≈-1 正确)')
    fit(dqt[dqm], np.diff(ry[sl])[dqm], 'y=Δry  x=dq_tilt(≈-1 正确)')
    fit(dqy[dqm], rx[sl][:-1][dqm], 'y=rx   x=dq_yaw (≈+1 才是拉回)')
    print('  Σ|dq_yaw| %.1f°  Σ|dq_tilt| %.1f°  rx %.2f->%.2f  res %.2f->%.2f'
          % (np.abs(D['dqy'][sl]).sum(), np.abs(D['dqt'][sl]).sum(),
             rx[sl][0], rx[sl][-1], D['res'][sl][0], D['res'][sl][-1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=r'R:\imu_20260921_030649.bin')
    ap.add_argument('--seg', nargs=2, type=float, action='append', default=None,
                    help='额外分析区间 t0 t1（可重复）')
    a = ap.parse_args()
    G.C.selfcheck(verbose=False)
    D = G.load(a.log)
    print('%s  VER=%d  %d 帧  %.1fs' % (os.path.basename(a.log), D['rep']['ver'], D['n'], D['t'][-1]))
    seg_report(D, 0.0, 1.0, '① 起始静置')
    seg_report(D, 3.0, 10.0, '② 剧烈运动段')
    seg_report(D, 13.90, 14.60, '③ 停稳后异常段')
    seg_report(D, 15.0, 29.5, '④ 末尾静置')
    for s in (a.seg or []):
        seg_report(D, s[0], s[1], '⑤ 指定区间')
    return 0


if __name__ == '__main__':
    sys.exit(main())
