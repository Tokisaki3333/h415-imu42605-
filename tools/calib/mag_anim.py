# -*- coding: utf-8 -*-
r"""渲染一次录像的运动动画（GIF）。

四格：
  ① 3D 姿态：世界系固定；绿=模型磁场 b^_n（常量）；红=用 EKF 姿态把标定后磁场转回世界系
     的方向（应当贴着绿的）；粉虚线=用旧链 att.q 转出来的（会乱飞）；三色短轴=EKF 机体系。
     红绿之间的夹角就是"世界系磁场方向残差"。
  ② 时间历程：该残差 EKF vs 旧链（对数纵轴）+ 游标。
  ③ |w| 与削顶帧（红点）+ 累计行程。
  ④ 施加的地磁修正：yaw 分量 / tilt 分量（度/次）+ mag_rs（对数副轴）。

用法:
  python tools/calib/mag_anim.py R:\imu_20260921_030649.bin --fps 12
  python tools/calib/mag_anim.py <bin> --t0 0 --t1 12 --out R:\fast.gif   # 只渲染快段
  python tools/calib/mag_anim.py <bin> --preview R:\frame.png --at 7.5    # 只出一帧 PNG
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C
import mag360_cal as M

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                     # noqa: E402
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
from matplotlib.animation import FuncAnimation      # noqa: E402

c = C.CH_162


def build(path):
    a, _ = C.load_frames(path)
    rep = C.frame_report(a)
    A, Cv = M.read_current_AC()
    dt = a[:, c['dt_us']].astype(float) * 1e-6
    t = np.cumsum(dt)
    w = np.linalg.norm(a[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
    clip = ((a[:, c['flags']].astype(int) >> 12) & 1).astype(bool)
    ys = (A @ a[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float).T + Cv[:, None]).T

    def unit_field(qcol):
        q = a[:, c[qcol]:c[qcol] + 4].astype(float)
        R = np.stack([M.quat_to_R(r) for r in q])
        u = np.einsum('nij,nj->ni', R, ys)
        return u / np.linalg.norm(u, axis=1, keepdims=True), R

    u_ekf, R_ekf = unit_field('ekf_q0')
    u_leg, _ = unit_field('att_q0')
    mb_e = u_ekf.mean(0); mb_e /= np.linalg.norm(mb_e)
    mb_l = u_leg.mean(0); mb_l /= np.linalg.norm(mb_l)
    res_e = np.degrees(np.arccos(np.clip(u_ekf @ mb_e, -1, 1)))
    res_l = np.degrees(np.arccos(np.clip(u_leg @ mb_l, -1, 1)))

    # 模型磁场（世界系，常量）：用 0.5*(EKF 与旧链的平均方向) 仅用于画图基准，
    # 真正的"模型" b0 由 v5f_tune.h 的 decl/dip 给出（这里一并算出来画）
    ci = 1.0 / np.sqrt(1 + 1.70 ** 2)
    b0 = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -1.70 * ci])

    gn = a[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
    gn = gn / np.linalg.norm(gn, axis=1, keepdims=True)
    dqv = a[:, c['ekf_mag_dqx']:c['ekf_mag_dqz'] + 1].astype(float)
    yawc = np.einsum('ni,ni->n', dqv, gn)
    tilt = np.sqrt(np.maximum(np.einsum('ni,ni->n', dqv, dqv) - yawc * yawc, 0.0))
    return dict(a=a, rep=rep, dt=dt, t=t, w=w, clip=clip, u_ekf=u_ekf, u_leg=u_leg,
                R_ekf=R_ekf, mb_e=mb_e, mb_l=mb_l, res_e=res_e, res_l=res_l, b0=b0,
                yawc=yawc, tilt=tilt, path=np.cumsum(w * dt),
                rs=a[:, c['ekf_mag_rs']].astype(float), th=a[:, c['ekf_mag_r_deg']].astype(float),
                used=a[:, c['ekf_mag_used']] > 0.5, nis=a[:, c['ekf_nis0'] + 4].astype(float))


def seg(p, o, r=1.0):
    """返回 3 条线用的 (xs, ys, zs)：从 o 出发沿三个坐标轴。"""
    return ([o[0], o[0] + r * p[0, 0]], [o[1], o[1] + r * p[1, 0]], [o[2], o[2] + r * p[2, 0]])


def draw_frame(fig, axs, D, k, tlim):
    ax3, ax2, ax1, ax4 = axs
    R = D['R_ekf'][k]
    ax3.clear()
    ax3.set_xlim(-1.2, 1.2); ax3.set_ylim(-1.2, 1.2); ax3.set_zlim(-1.2, 1.2)
    ax3.set_box_aspect((1, 1, 1))
    ax3.view_init(elev=22, azim=-58)
    ax3.set_xticks([]); ax3.set_yticks([]); ax3.set_zticks([])
    # 世界系轴（灰）
    for v, nm in ((np.array([1, 0, 0.]), 'X'), (np.array([0, 1, 0.]), 'Y'), (np.array([0, 0, 1.]), 'Z')):
        ax3.plot([0, v[0]], [0, v[1]], [0, v[2]], color='0.75', lw=1)
    # 模型磁场（绿，常量）
    ax3.plot([0, D['b0'][0]], [0, D['b0'][1]], [0, D['b0'][2]], color='g', lw=3, alpha=0.8)
    # 实测磁场（EKF 姿态转回世界）——应贴着绿
    ue = D['u_ekf'][k]
    ax3.plot([0, ue[0]], [0, ue[1]], [0, ue[2]], color='r', lw=2.5)
    ax3.plot([ue[0]], [ue[1]], [ue[2]], 'o', color='r', ms=5)
    # 实测磁场（旧链姿态转回世界）——会乱飞
    ul = D['u_leg'][k]
    ax3.plot([0, ul[0]], [0, ul[1]], [0, ul[2]], color='m', lw=1, ls=':', alpha=0.8)
    # 机体系三轴（EKF）
    for j, col in ((0, 'r'), (1, 'g'), (2, 'b')):
        e = R[:, j] * 0.75
        ax3.plot([0, e[0]], [0, e[1]], [0, e[2]], color=col, lw=2.2)
    # 重力（世界系 -Z，蓝）
    ax3.plot([0, 0], [0, 0], [0, -1.0], color='c', lw=2)
    ax3.set_title('t=%5.2fs  |w|=%4.0f dps  theta=%4.1f°  残差 EKF %4.1f° / 旧链 %5.1f°'
                  % (D['t'][k], D['w'][k], D['th'][k], D['res_e'][k], D['res_l'][k]), fontsize=9)

    for ax, xlim in ((ax2, None), (ax1, None), (ax4, None)):
        for ln in ax.lines:
            if ln.get_label() == 'cur':
                ln.remove()
    for ax in (ax2, ax1, ax4):
        xl = tlim if xlim is None else xlim
        ax.plot([D['t'][k], D['t'][k]], ax.get_ylim(), color='k', lw=0.8, alpha=0.6, label='cur')
    ax2.set_title('② 世界系磁场方向残差（物理不变量）', fontsize=9)
    ax1.set_title('③ 角速率与削顶', fontsize=9)
    ax4.set_title('④ 地磁施加的修正（yaw/tilt 分量）', fontsize=9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log', nargs='?', default=r'R:\imu_20260921_030649.bin')
    ap.add_argument('--fps', type=float, default=12.0)
    ap.add_argument('--t0', type=float, default=None)
    ap.add_argument('--t1', type=float, default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--preview', default=None, help='只出一帧 PNG')
    ap.add_argument('--at', type=float, default=None, help='预览帧的时刻(s)')
    a = ap.parse_args()

    C.selfcheck(verbose=False)
    D = build(a.log)
    n = len(D['t'])
    m = np.ones(n, bool)
    if a.t0 is not None:
        m &= D['t'] >= a.t0
    if a.t1 is not None:
        m &= D['t'] <= a.t1
    idx = np.flatnonzero(m)[::max(1, int(round((1.0 / a.fps) / np.median(D['dt']))))]
    print('录像 %s  VER=%d  帧 %d  %.1fs  行程 %.0f°  渲染 %d 帧 @%.0f fps'
          % (os.path.basename(a.log), D['rep']['ver'], n, D['t'][-1], D['path'][-1],
             len(idx), a.fps))

    fig = plt.figure(figsize=(13, 8.5))
    ax3 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2)
    ax1 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)
    tlim = (D['t'][0], D['t'][-1])
    ax2.semilogy(D['t'], np.maximum(D['res_e'], 1e-3), 'r-', lw=1, label='EKF（磁牵引）')
    ax2.semilogy(D['t'], np.maximum(D['res_l'], 1e-3), 'm:', lw=1, label='旧链 att.q')
    ax2.set_ylabel('残差 [deg]'); ax2.set_xlabel('t [s]'); ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc='upper left'); ax2.set_xlim(*tlim)
    ax1.plot(D['t'], D['w'], 'b-', lw=0.8, label='|w| [dps]')
    ax1.plot(D['t'][D['clip']], D['w'][D['clip']], 'r.', ms=2, label='削顶帧')
    ax1.set_ylabel('|w| [dps]'); ax1.set_xlabel('t [s]'); ax1.grid(alpha=0.3)
    ax1.legend(fontsize=8); ax1.set_xlim(*tlim)
    ax4.plot(D['t'], D['yawc'], 'g-', lw=0.8, label='yaw 分量')
    ax4.plot(D['t'], D['tilt'], 'r-', lw=0.8, alpha=0.8, label='tilt 分量')
    ax4.set_ylabel('dq [deg/次]'); ax4.set_xlabel('t [s]'); ax4.grid(alpha=0.3)
    ax4.set_xlim(*tlim); ax4.legend(fontsize=8, loc='upper left')
    ax4b = ax4.twinx()
    ax4b.semilogy(D['t'], np.maximum(D['rs'], 1e-2), color='0.5', lw=0.8, label='mag_rs（倾斜降权）')
    ax4b.set_ylabel('mag_rs', color='0.4'); ax4b.legend(fontsize=8, loc='upper right')

    def upd(i):
        draw_frame(fig, (ax3, ax2, ax1, ax4), D, idx[i], tlim)
        fig.tight_layout()
        return ()

    if a.preview:
        k = int(np.argmin(np.abs(D['t'] - (a.at if a.at is not None else D['t'][idx[len(idx) // 2]]))))
        draw_frame(fig, (ax3, ax2, ax1, ax4), D, k, tlim)
        fig.tight_layout(); fig.savefig(a.preview, dpi=110)
        print('预览帧 t=%.2f s -> %s' % (D['t'][k], a.preview))
        return 0

    out = a.out or (os.path.splitext(a.log)[0] + '_anim.gif')
    anim = FuncAnimation(fig, upd, frames=len(idx), interval=1000.0 / a.fps, blit=False)
    anim.save(out, writer='pillow', fps=a.fps)
    plt.close(fig)
    print('动画已写出: %s  (%.1f MB)' % (out, os.path.getsize(out) / 1e6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
