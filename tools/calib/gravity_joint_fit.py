# -*- coding: utf-8 -*-
"""
重力方向联合拟合：同时解 加速度计测量阵 + 零偏 与 陀螺残余校正阵。

动机：用椭圆法（只吃加速度计自己的量长约束）标出来的加速度计、和用闭合法标出来的陀螺，
在"两个静止点之间的重力方向转角"上对不上 4~16 倍。单靠任一边都判不出是谁的错。
本脚本把两边放进**同一组方程**：

    每个静止点 i：   R(q_i)^T ẑ  =  normalize( M_a · (acc_i - b) )        (2 个独立方程)

    q_i 由固件四元数的增量经陀螺残余阵 K 修正得到：
        q_i = q0 ⊗ exp( K · θ_i ),   θ_i = log( q_ref^-1 ⊗ q_i^fw )

未知量：M_a(9) + b(3) + K(9) + q0(每条记录 3) = 30
方程：  三条单轴记录共 ~39 个静止点 x 2 = ~78

残差里 R 与 u 都来自重力方向，所以绕垂直轴的偏航对两边同时不可观测、自动抵消，
不会污染 M_a 与 K 的可分性。

用法：python tools/calib/gravity_joint_fit.py
"""
import os, sys
import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
for _p in (_here, os.path.join(_here, 'h415-imu42605-', 'tools', 'calib')):
    if os.path.isfile(os.path.join(_p, 'jf_load.py')):
        sys.path.insert(0, _p)
        break
from jf_load import load_jf

FPS = 8027.0
DT = 1.0/FPS
W_A = 4000
SA_THR = 1.15          # mg
GYRO_B0 = np.array([1.0334, 0.8494, 12.0910])
GYRO_S = np.array([16.2753, 16.4366, 16.4235])
ACC_S0 = np.array([2028.48, 2040.78, 2016.07])
ACC_B0 = np.array([-10.10, -15.42, 43.72])

RECS = [('z-2337', 'serial_runtime_20260913_233731_085_export.txt'),
        ('y-2348', 'serial_runtime_20260913_234829_229_export.txt'),
        ('x-0006', 'serial_runtime_20260914_000636_751_export.txt'),
        ('z-0140', 'serial_runtime_20260914_014002_763_export.txt'),
        ('y-0142', 'serial_runtime_20260914_014224_171_export.txt'),
        ('x-0145', 'serial_runtime_20260914_014529_487_export.txt')]


def find_rec(fn):
    """记录通常放在仓库根目录或其上一级（工作区根），逐个试"""
    cands = [fn, os.path.join('..', fn),
             os.path.join(_here, '..', '..', '..', fn),
             os.path.join(_here, '..', '..', '..', '..', fn)]
    for c in cands:
        if os.path.exists(c):
            return c
    return None


def expq(v):
    t = np.linalg.norm(v)
    if t < 1e-13:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.concatenate([[np.cos(t*0.5)], np.sin(t*0.5)*v/t])


def logq(q):
    q = q/np.linalg.norm(q)
    w = float(np.clip(q[0], -1.0, 1.0))
    v = q[1:]
    n = np.linalg.norm(v)
    if n < 1e-13:
        return np.zeros(3)
    return v/n*(2.0*np.arctan2(n, w))


def qmul(a, b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                     a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def up_of(q):
    """R(q)^T ẑ —— 世界竖直方向在机体系里的表示"""
    w, x, y, z = q
    return np.array([2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)])


def collect(fn, SG=0.0):
    A = np.asarray(load_jf(fn), dtype=np.float64)
    N = len(A)
    q = A[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
    acc = A[:, 4:7]
    if A.shape[1] >= 10:
        dps = (A[:, 7:10] - GYRO_B0)/GYRO_S
        rate = np.linalg.norm(dps, axis=1)
    else:
        # 7 通道老记录没有陀螺原始值，速率从固件四元数差分取（只用于挑静止段）
        r0 = np.degrees(2*np.arccos(np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)))*FPS
        rate = np.concatenate([[r0[0]], r0])
    a_b0 = (acc - ACC_B0)/ACC_S0
    M = np.cumsum(np.vstack([np.zeros((1, 3)), a_b0]), 0)
    kk = np.arange(N)
    sa = np.abs((M[np.minimum(kk+1, N)]-M[np.maximum(kk+1-W_A, 0)]) /
                np.maximum(np.minimum(kk+1, N)-np.maximum(kk+1-W_A, 0), 1)[:, None]
                - (M[np.maximum(kk+1-W_A, 0)]-M[np.maximum(kk+1-2*W_A, 0)]) /
                np.maximum(np.maximum(kk+1-W_A, 0)-np.maximum(kk+1-2*W_A, 0), 1)[:, None]
                ).max(1)*1000
    still = (sa < SA_THR) & (rate < 1.0)
    segs, i = [], 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/FPS > 0.4:
                segs.append((i, j))
            i = j
        else:
            i += 1
    accs, qs = [], []
    # 总是重积分：SG=0 时它必须精确复现固件姿态，这就是一次自检
    qs_corr = reintegrate_no_rect(q, a_b0, segs, SG)
    for idx, (s, e) in enumerate(segs):
        sl = slice(s+int(0.1*FPS), max(s+int(0.1*FPS)+1, e-int(0.1*FPS)))
        accs.append(acc[sl].mean(0))
        qm = qs_corr[sl].mean(0); qm /= np.linalg.norm(qm)
        qs.append(qm)
    return np.array(accs), np.array(qs)


def reintegrate_no_rect(q, a_b, segs, SG):
    """扣掉加速度整流后重新积分姿态。

    机理：陀螺输出 = 真实角速度 + S_g * a（a 为该时刻的**全部**非重力加速度，含向心项）。
    非重力加速度 = 实测比力 - 预测重力方向。参考系问题：固件四元数的参考 z 不是重力方向，
    所以先用第一个静止点定出 w = R(q_static) * u_meas（世界竖直在固件参考系中的表示）。
    角速度修正量（度）：w_corr = w_fw - S_g * a_lin * dt（S_g 单位 dps/g，a_lin 单位 g）。
    """
    # 由第一个静止点定 w
    s0, e0 = segs[0]
    sl = slice(s0+int(0.1*FPS), max(s0+int(0.1*FPS)+1, e0-int(0.1*FPS)))
    u = a_b[sl].mean(0); u /= np.linalg.norm(u)
    qm = q[sl].mean(0); qm /= np.linalg.norm(qm)
    w = Rm(qm) @ u
    w /= np.linalg.norm(w)
    # 逐样本增量（度），机体系
    qc = q.copy(); qc[:, 1:] *= -1.0
    aw, ax, ay, az = qc[:-1].T
    bw, bx, by, bz = q[1:].T
    vx = aw*bx+ax*bw+ay*bz-az*by
    vy = aw*by-ax*bz+ay*bw+az*bx
    vz = aw*bz+ax*by-ay*bx+az*bw
    ww = aw*bw-ax*bx-ay*by-az*bz
    n = np.sqrt(vx*vx+vy*vy+vz*vz)
    ang = 2.0*np.arctan2(n, ww)
    rv = np.stack([vx, vy, vz], 1)/np.maximum(n, 1e-15)[:, None]*ang[:, None]
    # 预测重力方向（机体系）= R(q)^T w，逐样本向量化
    w_, x_, y_, z_ = q.T
    gx = (1-2*(y_*y_+z_*z_))*w[0] + 2*(x_*y_+w_*z_)*w[1] + 2*(x_*z_-w_*y_)*w[2]
    gy = 2*(x_*y_-w_*z_)*w[0] + (1-2*(x_*x_+z_*z_))*w[1] + 2*(y_*z_+w_*x_)*w[2]
    gz = 2*(x_*z_+w_*y_)*w[0] + 2*(y_*z_-w_*x_)*w[1] + (1-2*(x_*x_+y_*y_))*w[2]
    a_lin = a_b - np.stack([gx, gy, gz], 1)          # 单位 g
    # rv 是**弧度**，修正量按度算，必须统一
    rv = rv - SG*a_lin[:-1]*DT*np.pi/180.0
    # 逐样本积分（纯标量，避免 numpy 调用开销）
    out = np.empty((len(q), 4))
    qw, qx, qy, qz = q[0]
    out[0] = (qw, qx, qy, qz)
    for k in range(len(rv)):
        vx, vy, vz = rv[k]
        th = np.sqrt(vx*vx+vy*vy+vz*vz)
        if th > 1e-12:
            s = np.sin(th*0.5)/th; c = np.cos(th*0.5)
        else:
            s = 0.5; c = 1.0
        dx, dy, dz = vx*s, vy*s, vz*s
        nw = qw*c - qx*dx - qy*dy - qz*dz
        nx = qw*dx + qx*c + qy*dz - qz*dy
        ny = qw*dy - qx*dz + qy*c + qz*dx
        nz = qw*dz + qx*dy - qy*dx + qz*c
        if (k & 1023) == 0:
            r = np.sqrt(nw*nw+nx*nx+ny*ny+nz*nz)
            nw, nx, ny, nz = nw/r, nx/r, ny/r, nz/r
        qw, qx, qy, qz = nw, nx, ny, nz
        out[k+1] = (qw, qx, qy, qz)
    if SG == 0.0:
        # 自检：SG=0 时重新积分应精确复现固件姿态
        d = np.abs(np.clip((out*q).sum(1), -1, 1))
        print("      [自检] SG=0 重积分 vs 固件姿态 最大偏差 %.4f deg"
              % np.degrees(2*np.arccos(d.min())))
    return out


def unpack(p, nrec):
    Ma = p[0:9].reshape(3, 3)
    b = p[9:12]
    # 陀螺只拟合**每条记录沿其转轴的一个标度修正** s_r。
    # 为什么不是完整的 3x3：R(exp(Kθ))ᵀw 对 (Kθ) 中沿 w 的分量完全不敏感（绕竖直轴转
    # 不改变重力方向），所以每条记录只观测到 Kn_r 垂直于 w_r 的 2 个分量 ——
    # 每条记录 2 个方程对 K 的 9 个未知量，有 3 维零空间。实测放开 9 参数时拟合会把
    # 0.92% 塞进最不可观测的方向（z 轴，而绕 z 那条记录 n·w = 0.02）。故只留可辨识部分。
    s = p[12:12+nrec]
    ang = p[12+nrec:].reshape(-1, 2)
    # 每条记录一个未知量 w_r：世界竖直方向在「固件参考系」里的单位向量。
    # 固件四元数上电初始化为单位四元数，其参考 z 与世界竖直**不对齐**（实测各记录
    # 上电姿态 1.3~178.8 deg 都有），所以不能拿 R(q)^T ẑ 当重力方向用。
    w = np.stack([np.sin(ang[:, 0])*np.cos(ang[:, 1]),
                  np.sin(ang[:, 0])*np.sin(ang[:, 1]),
                  np.cos(ang[:, 0])], 1)
    return Ma, b, s, w


def resid(p, data):
    Ma, b, s, w = unpack(p, len(data))
    out = []
    for ridx, (accs, qs) in enumerate(data):
        qref = qs[0]
        for i in range(len(accs)):
            th = logq(qmul(qconj(qref), qs[i]))       # 固件增量旋转矢量（相对本记录首点）
            Rc = Rm(expq(s[ridx]*th))                 # 沿转轴的标度修正后
            model = Rc.T @ w[ridx]                    # 重力方向（机体系）
            # 3 分量残差、不归一化：同时约束方向与模长，钉死 Ma 的整体缩放
            out.extend(list(model - Ma @ (accs[i] - b)))
    return np.array(out)


def Rm(q):
    w_, x_, y_, z_ = q
    return np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-w_*z_), 2*(x_*z_+w_*y_)],
                     [2*(x_*y_+w_*z_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-w_*x_)],
                     [2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)]])


def jac(p, data, eps=1e-6):
    r0 = resid(p, data)
    J = np.zeros((len(r0), len(p)))
    for k in range(len(p)):
        pp = p.copy(); pp[k] += eps
        J[:, k] = (resid(pp, data) - r0)/eps
    return J, r0


def main():
    SG = float(os.environ.get('SG', '0.0'))       # 陀螺加速度敏感度补偿 dps/g
    FIX_MA = os.environ.get('FIX_MA', '0') == '1'      # 钉死加速度计阵，只解陀螺 s_r
    data = []
    for tag, fn in RECS:
        path = find_rec(fn)
        if path is None:
            print("缺 %s" % fn); return
        accs, qs = collect(path, SG)
        print("%-6s 静止点 %2d 个" % (tag, len(accs)))
        # 诊断：现用标定下，固件姿态与加速度计重力方向的夹角（应当 ~0.1-0.6 deg）
        errs = []
        for i in range(len(accs)):
            u = (accs[i] - ACC_B0)/ACC_S0
            u = u/np.linalg.norm(u)
            errs.append(np.degrees(np.arccos(np.clip(np.dot(up_of(qs[i]), u), -1, 1))))
        print("       现用标定下 姿态 vs 加速度计 夹角: 中位 %.4f 最大 %.4f deg"
              % (np.median(errs), np.max(errs)))
        data.append((accs, qs))

    # 初值：w_r 由本记录首点的加速度方向定（θ=0 时 model 就等于 w_r）
    Ma0 = np.diag(1.0/ACC_S0)
    w0 = []
    for accs, qs in data:
        u = Ma0 @ (accs[0] - ACC_B0)
        u = u/np.linalg.norm(u)
        w0.append([np.arccos(np.clip(u[2], -1, 1)), np.arctan2(u[1], u[0])])
    p = np.concatenate([Ma0.ravel(), ACC_B0, np.ones(len(data)), np.array(w0).ravel()])
    print("\n参数量 %d，残差量 %d" % (len(p), len(resid(p, data))))

    lam = 1e-3
    r = resid(p, data)
    c = (r**2).sum()
    # FIX_MA=1：把 M_a 与 b 钉死在现用标定上，只解陀螺 s_r 与 w_r。
    # 用来判断 s_r 是不是在吸收加速度计的轴间各向异性（两者对相位的贡献同量级）。
    FREE = np.ones(len(p), bool)
    if FIX_MA:
        FREE[0:12] = False
        print("（FIX_MA=1：加速度计阵与零偏钉死在现用标定，只解 s_r 与 w_r）")
    print("初始 RMS 残差 %.5f（= %.4f deg 量级）" % (np.sqrt(c/len(r)), np.degrees(np.sqrt(c/len(r)))))
    for it in range(120):
        J, r = jac(p, data)
        g = J.T @ r
        H = J.T @ J
        for _ in range(30):
            try:
                dp = np.linalg.solve(H + lam*np.diag(np.maximum(np.diag(H), 1e-12)), -g)
            except np.linalg.LinAlgError:
                lam *= 10; continue
            pn = p + dp*FREE
            rn = resid(pn, data)
            cn = (rn**2).sum()
            if cn < c:
                p, r, c = pn, rn, cn
                lam = max(lam*0.3, 1e-9)
                break
            lam *= 3.0
        else:
            break
    print("迭代 %d 次后 RMS 残差 %.5f（%.4f deg）" % (it+1, np.sqrt(c/len(r)), np.degrees(np.sqrt(c/len(r)))))

    Ma, b, sfit, w = unpack(p, len(data))
    S = 1.0/np.diag(Ma)
    print("\n===== 拟合结果 =====")
    print("加速度计 测量阵 M_a =\n%s" % np.array2string(Ma, precision=6, suppress_small=False))
    print("  -> 逐轴标度 1/diag = %s LSB/g（现用 %s）"
          % (np.array2string(S, precision=2), np.array2string(ACC_S0, precision=2)))
    print("  -> 与现用值偏 %+.4f%% / %+.4f%% / %+.4f%%"
          % tuple((S/ACC_S0-1)*100))
    off = Ma - np.diag(np.diag(Ma))
    print("  -> 非对角（相对对角）= %s" % np.array2string(off/np.diag(Ma), precision=5))
    print("加速度计零偏 b = %s LSB（现用 %s） 差 %s"
          % (np.array2string(b, precision=2), np.array2string(ACC_B0, precision=2),
             np.array2string(b-ACC_B0, precision=2)))
    print("陀螺沿各记录转轴的标度修正 s_r = %s" % np.array2string(sfit, precision=6))
    print("  -> 偏差 %s %%"
          % np.array2string((sfit-1.0)*100, precision=4))

    # 每条记录的残差
    Ma_, b_, s_, w_ = unpack(p, len(data))
    print("\n每条记录: w_r = 世界竖直方向在固件参考系里（与固件 z 轴夹角）")
    for ridx, (tag, fn) in enumerate([(t, f) for t, f in RECS]):
        accs, qs = data[ridx]
        qref = qs[0]
        errs = []
        for i in range(len(accs)):
            th = logq(qmul(qconj(qref), qs[i]))
            model = Rm(expq(s_[ridx]*th)).T @ w_[ridx]
            u = Ma_ @ (accs[i] - b_)
            errs.append(np.degrees(np.arccos(np.clip(
                np.dot(model/np.linalg.norm(model), u/np.linalg.norm(u)), -1, 1))))
        print("   %-4s 固件z轴与竖直夹角 %.3f deg   残差 中位 %.4f 最大 %.4f deg"
              % (tag, np.degrees(np.arccos(np.clip(w_[ridx][2], -1, 1))),
                 np.median(errs), np.max(errs)))


if __name__ == '__main__':
    main()
