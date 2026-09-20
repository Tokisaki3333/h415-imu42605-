# -*- coding: utf-8 -*-
r"""matplotlib 窗口里**实时播放**一次录像的运动动画（不导出任何视频/GIF）。

四格：
  ① 3D 姿态：世界系固定；绿=模型磁场 b^_n（常量）；红=用 EKF 姿态把标定后磁场转回世界系
     （应贴着绿的）；粉虚线=用旧链 att.q 转出来的（会乱飞）；三色短轴=EKF 机体系；
     青=重力方向。红绿夹角就是"世界系磁场方向残差"。
  ② 该残差的时间历程：EKF vs 旧链（对数轴）+ 游标          （物理不变量，不需绝对参考）
  ③ |w| 与削顶帧（红点）
  ④ 地磁施加的修正：yaw / tilt 分量 + mag_rs（倾斜降权倍数，对数副轴）

用法:
  python tools/calib/mag_show.py R:\imu_20260921_030649.bin            # 播全程
  python tools/calib/mag_show.py <bin> --t0 1 --t1 12 --fps 15        # 只播快段
  python tools/calib/mag_show.py <bin> --snapshot out.png --at 7.5    # 只存一帧（调试用）
（图元只建一次、逐帧 set_data_3d 更新，不重画、不 tight_layout，所以不卡。）
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C
import mag360_cal as M

import matplotlib
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

c = C.CH_162
DIP_TAN = 1.70


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


def make_fig(D, t0, t1):
    fig = plt.figure(figsize=(13, 8))
    ax3 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2)
    ax1 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)

    ax3.set_xlim(-1.25, 1.25); ax3.set_ylim(-1.25, 1.25); ax3.set_zlim(-1.25, 1.25)
    ax3.set_box_aspect((1, 1, 1)); ax3.view_init(elev=22, azim=-58)
    ax3.set_xticks([]); ax3.set_yticks([]); ax3.set_zticks([])
    for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
        ax3.plot([0, v[0]], [0, v[1]], [0, v[2]], color='0.8', lw=1)
    ax3.plot([0, D['b0'][0]], [0, D['b0'][1]], [0, D['b0'][2]], color='g', lw=3.5, alpha=0.85)
    ax3.plot([0, 0], [0, 0], [0, -1.0], color='c', lw=2.5, alpha=0.8)      # 重力
    art = {}
    art['ue'], = ax3.plot([0, 0], [0, 0], [0, 1], color='r', lw=2.5)
    art['uet'], = ax3.plot([0], [0], [1], 'o', color='r', ms=5)
    art['ul'], = ax3.plot([0, 0], [0, 0], [0, 1], color='m', lw=1.2, ls=':')
    art['bx'], = ax3.plot([0, 0], [0, 0], [0, 0], color='r', lw=2.2)
    art['by'], = ax3.plot([0, 0], [0, 0], [0, 0], color='g', lw=2.2)
    art['bz'], = ax3.plot([0, 0], [0, 0], [0, 0], color='b', lw=2.2)
    art['ti'] = ax3.set_title('', fontsize=9)

    ax2.semilogy(D['t'], np.maximum(D['res_e'], 1e-3), 'r-', lw=1, label='EKF（磁牵引）')
    ax2.semilogy(D['t'], np.maximum(D['res_l'], 1e-3), 'm:', lw=1, label='旧链 att.q')
    ax2.set_ylabel('残差 [deg]'); ax2.set_xlabel('t [s]'); ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc='upper left'); ax2.set_xlim(t0, t1)
    ax2.set_title('② 世界系磁场方向残差（物理不变量）', fontsize=9)

    ax1.plot(D['t'], D['w'], 'b-', lw=0.8, label='|w| [dps]')
    ax1.plot(D['t'][D['clip']], D['w'][D['clip']], 'r.', ms=2, label='削顶帧')
    ax1.set_ylabel('|w| [dps]'); ax1.set_xlabel('t [s]'); ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8); ax1.set_xlim(t0, t1); ax1.set_title('③ 角速率与削顶', fontsize=9)

    ax4.plot(D['t'], D['yawc'], 'g-', lw=0.8, label='yaw 分量')
    ax4.plot(D['t'], D['tilt'], 'r-', lw=0.8, alpha=0.8, label='tilt 分量')
    ax4.set_ylabel('dq [deg/次]'); ax4.set_xlabel('t [s]'); ax4.grid(alpha=0.3)
    ax4.set_xlim(t0, t1); ax4.legend(fontsize=8, loc='upper left')
    ax4b = ax4.twinx()
    ax4b.semilogy(D['t'], np.maximum(D['rs'], 1e-2), color='0.55', lw=0.8, label='mag_rs')
    ax4b.set_ylabel('mag_rs', color='0.45'); ax4b.legend(fontsize=8, loc='upper right')
    ax4.set_title('④ 地磁施加的修正（yaw/tilt 分量）', fontsize=9)

    curs = (ax2.axvline(t0, color='k', lw=0.9, alpha=0.7),
            ax1.axvline(t0, color='k', lw=0.9, alpha=0.7),
            ax4.axvline(t0, color='k', lw=0.9, alpha=0.7))
    fig.tight_layout()
    return fig, ax3, art, curs


def set_frame(D, art, curs, k):
    ue, ul, R = D['u_ekf'][k], D['u_leg'][k], D['R_ekf'][k]
    art['ue'].set_data_3d([0, ue[0]], [0, ue[1]], [0, ue[2]])
    art['uet'].set_data_3d([ue[0]], [ue[1]], [ue[2]])
    art['ul'].set_data_3d([0, ul[0]], [0, ul[1]], [0, ul[2]])
    for nm, j in (('bx', 0), ('by', 1), ('bz', 2)):
        e = R[:, j] * 0.75
        art[nm].set_data_3d([0, e[0]], [0, e[1]], [0, e[2]])
    art['ti'].set_text('t=%6.2fs  |w|=%5.0f dps  θ=%5.1f°  残差: EKF %5.1f° / 旧链 %6.1f°  mag_rs=%.0f'
                       % (D['t'][k], D['w'][k], D['th'][k], D['res_e'][k], D['res_l'][k], D['rs'][k]))
    for cur in curs:
        cur.set_xdata([D['t'][k], D['t'][k]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=r'R:\imu_20260921_030649.bin')
    ap.add_argument('--fps', type=float, default=12.0)
    ap.add_argument('--t0', type=float, default=None)
    ap.add_argument('--t1', type=float, default=None)
    ap.add_argument('--snapshot', default=None, help='只存一帧 PNG（调试/存档用）')
    ap.add_argument('--at', type=float, default=None, help='快照时刻(s)')
    ap.add_argument('--speed', type=float, default=1.0, help='播放倍速（1=实时）')
    a = ap.parse_args()

    C.selfcheck(verbose=False)
    D = build(a.log)
    t0 = 0.0 if a.t0 is None else a.t0
    t1 = D['t'][-1] if a.t1 is None else a.t1
    dt = float(np.median(np.diff(D['t'])))
    fps = a.fps * a.speed
    stride = max(1, int(round((1.0 / fps) / max(dt, 1e-4))))
    idx = np.flatnonzero((D['t'] >= t0) & (D['t'] <= t1))[::stride]
    print('%s  VER=%d  %.1fs  行程 %.0f°  ->  播放 %d 帧 @%.0f fps（窗口内实时，不导出）'
          % (os.path.basename(a.log), D['rep']['ver'], D['t'][-1], D['path'][-1], len(idx), fps))

    fig, ax3, art, curs = make_fig(D, t0, t1)
    if a.snapshot:
        k = int(np.argmin(np.abs(D['t'] - (a.at if a.at is not None else 0.5 * (t0 + t1)))))
        set_frame(D, art, curs, k)
        fig.savefig(a.snapshot, dpi=110)
        print('快照 t=%.2f s -> %s' % (D['t'][k], a.snapshot))
        return 0

    plt.ion()
    plt.show(block=False)
    t_start = time.time()
    try:
        for m, k in enumerate(idx):
            set_frame(D, art, curs, k)
            ax3.set_title(art['ti'].get_text(), fontsize=9)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            target = t_start + (m + 1) / fps
            slack = target - time.time()
            if slack > 0:
                plt.pause(slack)
            elif m % 30 == 0:
                plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    print('播放结束（窗口保持打开，关掉即可）')
    plt.ioff()
    plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
