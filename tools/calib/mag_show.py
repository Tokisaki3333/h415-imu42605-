# -*- coding: utf-8 -*-
r"""matplotlib 窗口内**实时播放**录像的运动动画（不导出视频/GIF/帧序列）。

四格：
  ① 姿态视图：世界系固定；绿=模型磁场 b^_n（常量）；红=用 EKF 姿态把标定后磁场转回世界系
     （应贴着绿的）；粉虚线=用旧链 att.q 转出来的（会乱飞）；三色短轴=EKF 机体系；
     青=重力方向。红绿夹角就是"世界系磁场方向残差"。左下角是逐帧读数。
  ② 该残差的时间历程：EKF vs 旧链（对数轴）+ 游标          （物理不变量，不需绝对参考）
  ③ |w| 与削顶帧（红点）
  ④ 地磁施加的修正：yaw / tilt 分量 + mag_rs（倾斜降权倍数，对数副轴）

帧率：默认**不限帧率**。静态内容只画一次并存成背景，每帧只重画游标/矢量/读数再 blit
      （满幅重绘 ~184 ms/帧 → 区域 blit ~2 ms/区域），所以能跑到 ~60-100 fps。
  --speed 0.2    放慢 5 倍细看
  --all          每一帧都画（不丢帧，能多快就多快 = 慢动作）
  --fps 60       额外限制渲染上限（0=不限，默认）

用法:
  python tools/calib/mag_show.py R:\imu_20260921_030649.bin
  python tools/calib/mag_show.py <bin> --t0 1 --t1 12
  python tools/calib/mag_show.py <bin> --t0 1 --t1 12 --speed 0.25
  python tools/calib/mag_show.py <bin> --snapshot out.png --at 7.5     # 单帧存图（调试用）
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import cols_162 as C
import mag360_cal as M

import matplotlib
try:
    matplotlib.use('TkAgg')
except Exception:
    pass
import matplotlib.pyplot as plt
from matplotlib.transforms import Bbox
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

c = C.CH_162
DIP_TAN = 1.70
NPT_MAX = 3000          # ②③④ 曲线显示用抽稀点数（只影响观感，不影响数值）
ZERO = np.zeros(3)


class View:
    """正交投影：世界系 -> 屏幕（等效 elev/azim 视角），纯 2D 画，比 mplot3d 快得多。"""

    def __init__(self, elev=22.0, azim=-58.0):
        el, az = np.radians(elev), np.radians(azim)
        self.right = np.array([-np.sin(az), np.cos(az), 0.0])
        self.vdir = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        self.up = np.cross(self.right, self.vdir)

    def proj(self, p):
        p = np.atleast_2d(np.asarray(p, float))
        return np.column_stack([p @ self.right, p @ self.up])


def build(path):
    a, _ = C.load_frames(path)
    rep = C.frame_report(a)
    A, Cv = M.read_current_AC()
    dt = a[:, c['dt_us']].astype(float) * 1e-6
    t = np.cumsum(dt)
    w = np.linalg.norm(a[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
    clip = ((a[:, c['flags']].astype(int) >> 12) & 1).astype(bool)
    ys = (A @ a[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float).T + Cv[:, None]).T

    def uf(qcol):
        q = a[:, c[qcol]:c[qcol] + 4].astype(float)
        R = np.stack([M.quat_to_R(r) for r in q])
        u = np.einsum('nij,nj->ni', R, ys)
        return u / np.linalg.norm(u, axis=1, keepdims=True), R

    u_ekf, R_ekf = uf('ekf_q0')
    u_leg, _ = uf('att_q0')
    mb_e = u_ekf.mean(0); mb_e /= np.linalg.norm(mb_e)
    mb_l = u_leg.mean(0); mb_l /= np.linalg.norm(mb_l)
    ci = 1.0 / np.sqrt(1 + DIP_TAN ** 2)
    b0 = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -DIP_TAN * ci])
    gn = a[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
    gn = gn / np.linalg.norm(gn, axis=1, keepdims=True)
    dqv = a[:, c['ekf_mag_dqx']:c['ekf_mag_dqz'] + 1].astype(float)
    yawc = np.einsum('ni,ni->n', dqv, gn)
    tilt = np.sqrt(np.maximum(np.einsum('ni,ni->n', dqv, dqv) - yawc * yawc, 0.0))
    return dict(rep=rep, t=t, w=w, clip=clip, u_ekf=u_ekf, u_leg=u_leg, R_ekf=R_ekf, b0=b0,
                res_e=np.degrees(np.arccos(np.clip(u_ekf @ mb_e, -1, 1))),
                res_l=np.degrees(np.arccos(np.clip(u_leg @ mb_l, -1, 1))),
                yawc=yawc, tilt=tilt, path=np.cumsum(w * dt),
                th=a[:, c['ekf_mag_r_deg']].astype(float),
                rs=a[:, c['ekf_mag_rs']].astype(float), n=len(t))


BASE = 't=%6.2fs  |w|=%5.0f dps  th=%5.1fdeg  res: EKF %5.1f / old %6.1f deg  mag_rs=%.0f'


class Anim:
    def __init__(self, D, t0, t1):
        self.D = D
        self.V = View()
        self.t0, self.t1 = t0, t1
        fig = self.fig = plt.figure(figsize=(12.0, 7.2), dpi=100)
        ax3 = self.ax3 = fig.add_subplot(2, 2, 1)
        ax2 = self.ax2 = fig.add_subplot(2, 2, 2)
        ax1 = self.ax1 = fig.add_subplot(2, 2, 3)
        ax4 = self.ax4 = fig.add_subplot(2, 2, 4)

        ax3.set_xlim(-1.3, 1.3); ax3.set_ylim(-1.3, 1.3)
        ax3.set_aspect('equal'); ax3.set_axis_off()
        P = self.V.proj
        for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
            q = P([ZERO, v]); ax3.plot(q[:, 0], q[:, 1], color='0.82', lw=1)
        q = P([ZERO, D['b0']]); ax3.plot(q[:, 0], q[:, 1], color='g', lw=3.5, alpha=0.85)
        q = P([ZERO, (0, 0, -1.0)]); ax3.plot(q[:, 0], q[:, 1], color='c', lw=2.5, alpha=0.8)
        ax3.set_title('① 姿态（世界系固定）  红=磁场残差矢量  绿=模型磁场  青=重力  粉虚=旧链',
                      fontsize=9)

        art = self.art = {}
        art['ue'], = ax3.plot([], [], color='r', lw=2.5)
        art['uet'], = ax3.plot([], [], 'o', color='r', ms=5)
        art['ul'], = ax3.plot([], [], color='m', lw=1.2, ls=':')
        art['bx'], = ax3.plot([], [], color='r', lw=2.2)
        art['by'], = ax3.plot([], [], color='g', lw=2.2)
        art['bz'], = ax3.plot([], [], color='b', lw=2.2)

        s = max(1, D['n'] // NPT_MAX)
        ts = D['t'][::s]
        ax2.semilogy(ts, np.maximum(D['res_e'][::s], 1e-3), 'r-', lw=1, label='EKF（磁牵引）')
        ax2.semilogy(ts, np.maximum(D['res_l'][::s], 1e-3), 'm:', lw=1, label='旧链 att.q')
        ax2.set_ylabel('残差 [deg]'); ax2.set_xlabel('t [s]'); ax2.grid(alpha=0.3)
        ax2.legend(fontsize=8, loc='upper left'); ax2.set_xlim(t0, t1)
        ax2.set_title('② 世界系磁场方向残差（物理不变量）', fontsize=9)

        ax1.plot(ts, D['w'][::s], 'b-', lw=0.8, label='|w| [dps]')
        cl = np.flatnonzero(D['clip'])
        ax1.plot(D['t'][cl], D['w'][cl], 'r.', ms=2, label='削顶帧')
        ax1.set_ylabel('|w| [dps]'); ax1.set_xlabel('t [s]'); ax1.grid(alpha=0.3)
        ax1.legend(fontsize=8); ax1.set_xlim(t0, t1); ax1.set_title('③ 角速率与削顶', fontsize=9)

        ax4.plot(ts, D['yawc'][::s], 'g-', lw=0.8, label='yaw 分量')
        ax4.plot(ts, D['tilt'][::s], 'r-', lw=0.8, alpha=0.8, label='tilt 分量')
        ax4.set_ylabel('dq [deg/次]'); ax4.set_xlabel('t [s]'); ax4.grid(alpha=0.3)
        ax4.set_xlim(t0, t1); ax4.legend(fontsize=8, loc='upper left')
        ax4b = self.ax4b = ax4.twinx()
        ax4b.semilogy(ts, np.maximum(D['rs'][::s], 1e-2), color='0.55', lw=0.8, label='mag_rs')
        ax4b.set_ylabel('mag_rs', color='0.45'); ax4b.legend(fontsize=8, loc='upper right')
        ax4.set_title('④ 地磁施加的修正（yaw/tilt 分量）', fontsize=9)

        curs = []
        for ax in (ax2, ax1, ax4):
            curs.append(ax.axvline(t0, color='k', lw=0.9, alpha=0.7))
        self.curs = curs
        self.mov3 = [art['ue'], art['uet'], art['ul'], art['bx'], art['by'], art['bz']]
        self.txt = fig.text(0.006, 0.010, '', fontsize=9, family='monospace', va='bottom')
        self.txt.set_in_layout(False)
        fig.subplots_adjust(left=0.055, right=0.945, top=0.930, bottom=0.085,
                            wspace=0.30, hspace=0.32)
        self.regions = []
        self.bg_txt = None
        self.bb_txt = None
        self.dirty = True
        fig.canvas.mpl_connect('resize_event', self._on_resize)

    def _on_resize(self, _ev):
        self.dirty = True

    def capture(self):
        cv = self.fig.canvas
        rend = cv.get_renderer()
        self.regions = []
        for ax, arts in ((self.ax3, self.mov3),
                         (self.ax2, [self.curs[0]]),
                         (self.ax1, [self.curs[1]]),
                         (self.ax4, [self.curs[2]])):
            bb = ax.get_tightbbox(rend)
            self.regions.append([bb, cv.copy_from_bbox(bb), ax, arts])
        w, h = cv.get_width_height()
        self.bb_txt = Bbox.from_extents(2, 2, w - 2, 24)
        self.bg_txt = cv.copy_from_bbox(self.bb_txt)

    def draw_full(self):
        self.fig.canvas.draw()
        self.capture()
        self.dirty = False

    def set_frame(self, k, extra=''):
        D, V, art = self.D, self.V, self.art
        ue, ul, R = D['u_ekf'][k], D['u_leg'][k], D['R_ekf'][k]
        P = V.proj([ZERO, ue, ZERO, ul,
                    ZERO, R[:, 0] * 0.75, ZERO, R[:, 1] * 0.75, ZERO, R[:, 2] * 0.75])
        art['ue'].set_data(P[0:2, 0], P[0:2, 1])
        art['uet'].set_data(P[1:2, 0], P[1:2, 1])
        art['ul'].set_data(P[2:4, 0], P[2:4, 1])
        art['bx'].set_data(P[4:6, 0], P[4:6, 1])
        art['by'].set_data(P[6:8, 0], P[6:8, 1])
        art['bz'].set_data(P[8:10, 0], P[8:10, 1])
        tk = D['t'][k]
        self.txt.set_text((BASE + extra) % (tk, D['w'][k], D['th'][k],
                                            D['res_e'][k], D['res_l'][k], D['rs'][k]))
        for cur in self.curs:
            cur.set_xdata([tk, tk])

    def blit(self):
        cv = self.fig.canvas
        if self.dirty:
            self.draw_full()
            return
        for bb, bg, ax, arts in self.regions:
            cv.restore_region(bg)
            for a in arts:
                ax.draw_artist(a)
            cv.blit(bb)
        cv.restore_region(self.bg_txt)
        self.txt.draw(cv.get_renderer())
        cv.blit(self.bb_txt)
        cv.flush_events()


def play(A, t0, t1, speed, fps, every):
    D = A.D
    idx = np.flatnonzero((D['t'] >= t0) & (D['t'] <= t1))
    if idx.size == 0:
        print('区间 %.2f~%.2f s 内没有帧' % (t0, t1))
        return
    n = idx.size
    t_base = float(D['t'][idx[0]])
    min_dt = (1.0 / fps) if fps > 0 else 0.0
    sync = not every
    plt.ion()
    A.draw_full()
    tw = time.perf_counter()
    t_next = tw
    m = drawn = 0
    t_last = tw
    try:
        while m < n:
            k = int(idx[m]); m += 1
            now = time.perf_counter()
            if sync:
                target = tw + (D['t'][k] - t_base) / max(speed, 1e-6)
                if target > now:
                    left = target - now
                    while left > 0.02:
                        time.sleep(0.02); left = target - time.perf_counter()
                    time.sleep(max(0.0, left))
                elif target < now - 0.030:
                    continue
            if min_dt:
                now = time.perf_counter()
                if now < t_next:
                    time.sleep(t_next - now)
            A.set_frame(k)
            A.blit()
            t_next = time.perf_counter() + min_dt
            drawn += 1
            if drawn % 30 == 0:
                el = t_next - tw
                print('\r  %.0f fps（%.1f ms/帧）  数据 %.1f/%d 帧  t=%.2fs   '
                      % (drawn / max(el, 1e-9), el * 1000 / drawn, m, n, D['t'][k]),
                      end='', flush=True)
            t_last = time.perf_counter()
    except KeyboardInterrupt:
        pass
    el = t_last - tw
    print('\n  共画 %d 帧 / 数据 %d 帧  实测 %.1f fps  %.1f ms/帧（不导出任何文件）%s'
          % (drawn, n, drawn / max(el, 1e-9), el * 1000 / max(drawn, 1), ' ' * 12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=r'R:\imu_20260921_030649.bin')
    ap.add_argument('--t0', type=float, default=None)
    ap.add_argument('--t1', type=float, default=None)
    ap.add_argument('--speed', type=float, default=1.0, help='播放倍速（1=实时，0.25=慢放4倍）')
    ap.add_argument('--fps', type=float, default=0.0, help='渲染帧率上限，0=不限（默认）')
    ap.add_argument('--all', dest='every', action='store_true', help='每帧都画（不丢帧，慢动作）')
    ap.add_argument('--snapshot', default=None, help='只存一帧 PNG（调试/存档用）')
    ap.add_argument('--at', type=float, default=None, help='快照时刻(s)')
    a = ap.parse_args()

    C.selfcheck(verbose=False)
    D = build(a.log)
    t0 = 0.0 if a.t0 is None else a.t0
    t1 = D['t'][-1] if a.t1 is None else a.t1
    print('%s  VER=%d  %.1fs  行程 %.0f°  区间 %.2f~%.2f s'
          % (os.path.basename(a.log), D['rep']['ver'], D['t'][-1], D['path'][-1], t0, t1))

    A = Anim(D, t0, t1)
    if a.snapshot:
        k = int(np.argmin(np.abs(D['t'] - (a.at if a.at is not None else 0.5 * (t0 + t1)))))
        A.set_frame(k)
        A.fig.savefig(a.snapshot, dpi=110)
        print('快照 t=%.2f s -> %s' % (D['t'][k], a.snapshot))
        return 0

    print('窗口内实时播放：%s%s（静态内容只画一次，逐帧只重画游标/矢量后 blit）'
          % ('每帧都画' if a.every else '实时节流，渲染跟不上自动丢帧',
             '' if a.fps <= 0 else '，上限 %.0f fps' % a.fps))
    play(A, t0, t1, a.speed, a.fps, a.every)
    print('播放结束（窗口保持打开，关掉即可）')
    plt.ioff()
    plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
