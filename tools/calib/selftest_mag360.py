# -*- coding: utf-8 -*-
"""mag360_cal 的自检：造合成日志（已知 A/C/w0）跑通 加载->选样->拟合->判据 全链。

覆盖：
  1) 带帧格式(A5 5A ... 5A A5) 与 纯 payload 格式(带随机相位) 都能定相位；
  2) 全球面(多轴整圈) => 9 参数可辨识：残差小、秩判据只报 1 个标度零空间、半分一致；
  3) 只做平面 360(纯偏航、水平) => 必须报"数据不足"，不许悄悄给出结果。

用法: python tools/calib/selftest_mag360.py
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cols_162 as C
import mag360_cal as M

TMP = os.environ.get('TEMP', '.')


def make_log(n=12000, mode='sphere', seed=1, payload_only=False, phase=37):
    rng = np.random.default_rng(seed)
    c = C.CH_162
    dt = 0.002966
    A_true = np.array([[3.8e-05, -5.66e-03, 2.5e-04],
                       [-5.83e-03, 3.77e-04, -6.9e-05],
                       [3.7e-04, 9.1e-04, 6.18e-03]])
    C_true = np.array([2.93e-02, 2.09e-02, 1.89e-02])
    ci = 1.0 / np.sqrt(1 + 2.08 ** 2)
    w0 = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -2.08 * ci])

    qs = np.zeros((n, 4))
    gyro = np.zeros((n, 3))
    mag = np.zeros((n, 3))
    q = np.array([1.0, 0.0, 0.0, 0.0])
    axis = np.array([0.0, 0.0, 1.0])
    rate = 40.0
    left = 0
    for i in range(n):
        if left <= 0:
            if mode == 'sphere':
                axis = rng.normal(size=3)
                axis /= np.linalg.norm(axis)
                rate = float(rng.uniform(25.0, 50.0))
            else:
                axis = np.array([0.0, 0.0, 1.0])
                rate = 40.0
            left = max(int(round(360.0 / (rate * dt))), 1)
        ang = np.radians(rate) * dt
        dq = np.concatenate(([np.cos(ang / 2)], axis * np.sin(ang / 2)))
        q = np.array([dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3],
                      dq[0] * q[1] + dq[1] * q[0] + dq[2] * q[3] - dq[3] * q[2],
                      dq[0] * q[2] - dq[1] * q[3] + dq[2] * q[0] + dq[3] * q[1],
                      dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1] + dq[3] * q[0]])
        q /= np.linalg.norm(q)
        left -= 1
        qs[i] = q
        gyro[i] = axis * rate
        R = M.quat_to_R(q)
        mag[i] = np.linalg.solve(A_true, R.T @ w0 - C_true) + rng.normal(0, 0.35, 3)

    frames = np.zeros((n, C.NCH), dtype=np.float32)
    frames[:, c['att_q0']:c['att_q0'] + 4] = qs
    frames[:, c['flags']] = 0x04
    frames[:, c['gyro_dps0']:c['gyro_dps0'] + 3] = gyro
    frames[:, c['gyro_lsb0']:c['gyro_lsb0'] + 3] = gyro * 16.384
    acc = np.array([M.quat_to_R(qq).T @ np.array([0, 0, 1.0]) for qq in qs])
    frames[:, c['accel_g0']:c['accel_g0'] + 3] = acc
    frames[:, c['accel_lsb0']:c['accel_lsb0'] + 3] = acc * 2048
    frames[:, c['mag_lsb0']:c['mag_lsb0'] + 3] = mag
    frames[:, c['ist_cnt']] = np.arange(n) // 2
    frames[:, c['dt_us']] = dt * 1e6
    frames[:, c['temp_celsius']] = 30.0
    frames[:, c['fw_tag']] = float(C.FW_TAG_EXPECT)
    for i in range(n):
        frames[i, C.NCH - 1] = float(C.chk(frames[i, :C.NCH - 1].astype('<f4').tobytes()))
    raw = frames.tobytes()
    pl = C.NCH * 4
    if payload_only:
        raw = b'\x00' * (phase * 4) + raw
    else:
        out = bytearray()
        for i in range(n):
            out += b'\xa5\x5a' + bytes([pl & 0xFF, pl >> 8]) + raw[i * pl:(i + 1) * pl] + b'\x5a\xa5'
        raw = bytes(out)
    path = os.path.join(TMP, 'mag360_selftest_%s_%s.txt' % (mode, 'pl' if payload_only else 'fr'))
    with open(path, 'w') as f:
        for i in range(0, len(raw), 648):
            f.write('[RX] ' + raw[i:i + 648].hex(' ').upper() + '\n')
    return path, A_true, C_true, w0


def run(path, A_true):
    a, info = C.load_frames(path)
    rep = C.frame_report(a)
    assert abs(rep['fw_tag'] - C.FW_TAG_EXPECT) < 0.5, 'fw_tag 不对: %s' % rep['fw_tag']
    assert rep['checksum_ok'] > 0.99, '校验和通过率 %.2f' % rep['checksum_ok']
    sel, q, gyro, w, am, m = M.select(a, 50.0, 0.02, False)
    idx = np.flatnonzero(sel)
    assert len(idx) > 500, '选样太少 %d' % len(idx)
    Mm = M.design(q, gyro, m, w, 20.0, idx)
    A, Cv, w0, sv, Vt = M.fit_raw(Mm)
    A, Cv, w0 = M.normalize_scale(A, Cv, w0, m[idx])
    e = M.eval_resid(A, Cv, q, m, idx, w0)
    n_null, grp, _ = M.rank_diag(Mm)
    hs = M.halfsplit(q, gyro, m, w, idx, 20.0, w0)
    relA = float(np.linalg.norm(A - A_true) / np.linalg.norm(A_true))
    print('  fmt=%-8s N=%5d  零空间 %d  半分 %s  ||A-Ar||/||Ar|| %.4f  残差 p50 %.3f p90 %.3f'
          % (info['format'], len(idx), n_null,
             ('%.3f/%.2f' % hs) if hs else 'n/a', relA,
             np.percentile(e, 50), np.percentile(e, 90)))
    return n_null, hs, relA, float(np.percentile(e, 90)), A, Cv


def main():
    C.selfcheck()
    print('--- 情形1: 全球面 + 带帧 ---')
    p, At, Ct, w0t = make_log(12000, 'sphere', 1, False)
    n1, hs1, relA1, e90_1, A1, C1 = run(p, At)
    assert n1 == 1, '全球面零空间应为 1，得到 %d' % n1
    assert hs1 and hs1[0] < 0.15, '半分一致性差: %s' % (hs1,)
    assert e90_1 < 0.5, '残差过大 %.3f' % e90_1

    print('--- 情形2: 全球面 + 纯 payload(随机相位) ---')
    p, At, Ct, w0t = make_log(12000, 'sphere', 2, True, phase=37)
    n2, hs2, relA2, e90_2, A2, C2 = run(p, At)
    assert n2 == 1 and hs2 and hs2[0] < 0.15 and e90_2 < 0.5, 'payload 情形失败'
    assert np.linalg.norm(A2 - A1) / np.linalg.norm(A1) < 0.05, '两种格式结果不一致'

    print('--- 情形3: 只做平面 360（必须判"数据不足"）---')
    p, At, Ct, w0t = make_log(12000, 'planar', 3, False)
    n3, hs3, relA3, e90_3, A3, C3 = run(p, At)
    print('  零空间维数 %d（>1 即报"有参数未被激励"）  半分 %s' % (n3, hs3))
    assert n3 > 1, '平面 360 未被判为秩不足'
    print('\nSELFTEST OK')


if __name__ == '__main__':
    main()
