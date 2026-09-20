# -*- coding: utf-8 -*-
"""VER=101 调试帧(JustFloat, 162 通道)的权威列名表。

来源：V5F/User/src/SPI_rx.c `justfloat_report()` 里 `ch[c++]` 的**代码顺序**，
      外加两处"静态多写、运行只占 1 列"的修正：
        - 26 列 dt(us) 是 if/else 三分支（0 / delta / 0），运行时只占 1 列；
        - 末列 161 先写 0.0f 占位，随后被校验和覆盖。
自检：ANCHOR 里的列号必须与历史录像上验证过的锚点一致（fw_tag=76, ekf_q=89,
      mag_lsb=38, mag_f=42, mag_norm=45, mag_gate=117, mag_r_deg=119,
      mag_used=120, tick=159/160, checksum=161）。
"""
import numpy as np

# (组名, 列数)  —— 顺序即代码顺序
GROUPS = [
    ('att_q', 4), ('v_nav', 3), ('level_dps', 1), ('is_static', 1), ('acc_traction', 1),
    ('a_lin', 3), ('accel_bias_g', 3),
    ('flags', 1), ('dma1_irq_us', 1),
    ('gyro_lsb', 3), ('accel_lsb', 3), ('temp_celsius', 1), ('dt_us', 1),
    ('gyro_dps', 3), ('gyro_bias_dps', 3), ('accel_g', 3),
    ('bias_evidence_gran', 1), ('att_tilt', 1), ('vel_soft', 1),
    ('mag_lsb', 3), ('ist_cnt', 1), ('mag_f', 3), ('mag_norm', 1),
    ('mag_Bw', 3), ('psi_true_deg', 1), ('mag_trust', 1),
    ('baro_press_pa', 1), ('baro_temp_c', 1), ('bmp_cnt', 1),
    ('gps_speed', 1), ('gps_rmc_flags', 1), ('gps_rmc_cnt', 1),
    ('gps_alt_m', 1), ('gps_fix_quality', 1), ('gps_sat_num', 1), ('gps_hdop', 1),
    ('gps_gga_flags', 1), ('gps_gga_cnt', 1), ('gps_vdop', 1), ('gps_gsa_cnt', 1),
    ('gps_course_deg', 1),
    ('gps_lat_i', 1), ('gps_lat_frac', 1), ('gps_lon_i', 1), ('gps_lon_frac', 1),
    ('gps_sats_view', 1), ('gps_snr_avg', 1), ('gps_snr_min', 1), ('gps_snr_n', 1),
    ('gps_gsv_cnt', 1), ('gps_talkers', 1),
    ('fw_tag', 1), ('cmd_echo', 1), ('cmd_cnt', 1), ('cmd_last', 1),
    ('baro_press_avg', 1), ('stat_e_ac', 1), ('stat_ac_bypass', 1),
    ('ekf_p', 3), ('ekf_v', 3), ('ekf_q', 4), ('ekf_a_nav', 3), ('ekf_ba', 3),
    ('ekf_bg', 3), ('ekf_b_baro', 1), ('ekf_gate_bits', 1),
    ('ekf_sigma_yaw', 1), ('ekf_sigma_pos_h', 1), ('ekf_sigma_vel_h', 1),
    ('ekf_sigma_tilt_deg', 1), ('ekf_nis', 5),
    ('ekf_pzz', 1), ('ekf_pbb', 1), ('ekf_p_p_zz', 1), ('ekf_p_p_bb', 1),
    ('ekf_mag_gate', 1), ('ekf_mag_bh', 1), ('ekf_mag_r_deg', 1), ('ekf_mag_used', 1),
    ('ekf_p_yy', 1), ('ekf_prop_ok', 1), ('ekf_f_ok', 1), ('ekf_prop_row', 1),
    ('ekf_stage', 1), ('ekf_mag_rej', 1), ('ekf_mag_fhb', 1),
    ('ekf_mag_rx', 1), ('ekf_mag_ry', 1), ('ekf_mag_vx', 1), ('ekf_mag_vy', 1),
    ('ekf_mag_v0x', 1), ('ekf_mag_v0y', 1), ('ekf_mag_yawpre', 1),
    ('ekf_mag_dqx', 1), ('ekf_mag_dqy', 1), ('ekf_mag_dqz', 1),
    ('ekf_tilt_dqx', 1), ('ekf_tilt_dqy', 1), ('ekf_tilt_dqz', 1),
    ('ekf_tilt_prx', 1), ('ekf_tilt_pry', 1), ('ekf_tilt_prz', 1),
    ('ekf_mag_cmp_thm', 1), ('ekf_mag_cmp_thp', 1),
    ('ekf_mag_cmp_amn', 1), ('ekf_mag_cmp_mhn', 1),
    ('ekf_mag_age_ms', 1), ('ekf_mag_rs', 1), ('ekf_tilt_inv_ms', 1),
    ('ekf_mag_hold', 1), ('ekf_grav_ok', 1), ('ekf_grav_nis', 1),
    ('mag_ok', 1), ('gate_ekf_mag_yaw', 1), ('gate_ekf_tilt', 1),
    ('ekf_mag_gate_out', 1), ('ekf_mag_used_out', 1),
    ('tick_tk', 1), ('tick_md', 1), ('checksum', 1),
]

NCH = 162


def _expand():
    idx = {}
    c = 0
    for name, n in GROUPS:
        if n == 1:
            idx[name] = c
        else:
            for k in range(n):
                idx['%s%d' % (name, k)] = c + k
        c += n
    return idx, c


CH_162, TOTAL = _expand()

ANCHOR = {
    'att_q0': 0, 'flags': 16, 'gyro_lsb0': 18, 'accel_lsb0': 21, 'dt_us': 25,
    'gyro_dps0': 26, 'accel_g0': 32, 'mag_lsb0': 38, 'ist_cnt': 41,
    'mag_f0': 42, 'mag_norm': 45, 'psi_true_deg': 49, 'mag_trust': 50,
    'fw_tag': 76, 'ekf_q0': 89, 'ekf_sigma_tilt_deg': 107, 'ekf_nis0': 108,
    'ekf_mag_gate': 117, 'ekf_mag_r_deg': 119, 'ekf_mag_used': 120,
    'ekf_mag_rej': 126, 'ekf_mag_dqx': 135, 'ekf_mag_cmp_thm': 144,
    'ekf_mag_age_ms': 148, 'ekf_mag_rs': 149, 'ekf_grav_ok': 152,
    'mag_ok': 154, 'tick_tk': 159, 'tick_md': 160, 'checksum': 161,
}

VER_EXPECT = 101
FW_TAG_EXPECT = (VER_EXPECT << 16) | (NCH << 8) | 0x07   # flags: EKF_EN|MAG_CAL_EN|DET_AC_EN


def selfcheck(verbose=True):
    ok = True
    bad = [(k, v, CH_162.get(k)) for k, v in ANCHOR.items() if CH_162.get(k) != v]
    if TOTAL != NCH:
        print('FAIL: 组数合计 %d != %d' % (TOTAL, NCH))
        ok = False
    if bad:
        for k, want, got in bad:
            print('FAIL: %-22s want %3d got %s' % (k, want, got))
        ok = False
    if verbose and ok:
        print('cols_162: 162 列 / %d 组 / 锚点全部命中' % len(GROUPS))
    return ok


# ----------------------------- 帧解析 -----------------------------

def _hex_bytes(path):
    import re
    buf = bytearray()
    rx = re.compile(rb'\[RX\]\s*([0-9A-Fa-f ]+)')
    any_hex = re.compile(rb'(?:[0-9A-Fa-f]{2}[ ]?)+')
    with open(path, 'rb') as f:
        for ln in f:
            m = rx.search(ln)
            seg = m.group(1) if m else (any_hex.match(ln.strip()) and ln.strip() or b'')
            if not seg:
                continue
            toks = seg.split()
            try:
                buf += bytes(int(t, 16) for t in toks if len(t) == 2)
            except ValueError:
                continue
    return bytes(buf)


def load_frames(path, max_frames=None):
    """返回 (np.ndarray(N,162) float32, info dict)。自动识别两种落盘格式：
       A) 带帧：A5 5A | len(u16 LE)=648 | payload | 5A A5
       B) 无帧：纯 payload 流（用 fw_tag 常值 + 校验和定相位）
    """
    raw = _hex_bytes(path)
    pl = NCH * 4
    info = {'bytes': len(raw)}

    # --- A) 带帧扫描 ---
    import struct
    frames = []
    i = 0
    n = len(raw)
    while i + 4 + pl <= n:
        if raw[i] == 0xA5 and raw[i + 1] == 0x5A:
            ln = raw[i + 2] | (raw[i + 3] << 8)
            if ln == pl:
                end = i + 4 + pl
                if end + 2 <= n and raw[end] == 0x5A and raw[end + 1] == 0xA5:
                    frames.append(np.frombuffer(raw[i + 4:end], dtype='<f4'))
                    i = end + 2
                    continue
        i += 1
    if len(frames) > 16:
        a = np.stack(frames)
        info['format'] = 'framed'
        info['frames'] = len(frames)
        return a, info

    # --- B) 纯 payload：定相位 ---
    best = None
    for p in range(NCH):
        if len(raw) < (p + 1) * 4 + pl * 8:
            continue
        a = np.frombuffer(raw[p * 4:], dtype='<f4')
        m = (a.size - CH_162['fw_tag']) // NCH
        if m < 16:
            continue
        f = a[:m * NCH].reshape(m, NCH)
        tag = f[:, CH_162['fw_tag']]
        if not np.all(tag == tag[0]):
            continue
        t0 = float(tag[0])
        ver = int(t0) >> 16
        ch = (int(t0) >> 8) & 0xFF
        if not (1e5 < t0 < 1.7e7 and 80 <= ver <= 130 and ch == NCH):
            continue
        # 校验和一致性
        good = 0
        for k in range(0, min(m, 200)):
            fr = f[k]
            xk = 0
            b = fr[:NCH - 1].astype('<f4').tobytes()
            for byte in b:
                xk ^= byte
            if abs(float(fr[CH_162['checksum']]) - float(xk & 0xFF)) < 0.5:
                good += 1
        info.setdefault('cand', []).append((p, t0, good / min(m, 200)))
        if best is None or good > best[2]:
            best = (p, t0, good / min(m, 200), f)
    if best is None:
        raise SystemExit('无法定位帧：既没有 A5 5A 帧头，也没有可用的 fw_tag 相位')
    p, t0, okfrac, f = best
    info.update({'format': 'payload', 'phase': p, 'fw_tag': t0,
                 'checksum_ok': okfrac, 'frames': len(f)})
    if okfrac < 0.9:
        print('警告：校验和通过率只有 %.1f%%，相位可能不对' % (100 * okfrac))
    return f, info


def frame_report(a, info=None):
    """打印版本指纹/校验和/物理量范围，用来确认"这一帧到底够不够、对不对"。"""
    rep = {}
    rep['frames'] = len(a)
    tag = float(np.median(a[:, CH_162['fw_tag']]))
    rep['fw_tag'] = tag
    rep['ver'] = int(tag) >> 16
    rep['ch'] = (int(tag) >> 8) & 0xFF
    rep['flags'] = int(tag) & 0xFF
    ok = 0
    for k in range(min(len(a), 300)):
        fr = a[k]
        xk = 0
        for byte in fr[:NCH - 1].astype('<f4').tobytes():
            xk ^= byte
        if abs(float(fr[CH_162['checksum']]) - float(xk & 0xFF)) < 0.5:
            ok += 1
    rep['checksum_ok'] = ok / min(len(a), 300)
    q = a[:, CH_162['att_q0']:CH_162['att_q0'] + 4]
    rep['att_q_norm_err'] = float(np.abs(np.linalg.norm(q, axis=1) - 1).max())
    g = a[:, CH_162['accel_g0']:CH_162['accel_g0'] + 3]
    rep['accel_abs_median'] = float(np.median(np.linalg.norm(g, axis=1)))
    m = a[:, CH_162['mag_lsb0']:CH_162['mag_lsb0'] + 3]
    rep['mag_lsb_range'] = [float(m[:, i].min()) for i in range(3)] + \
                           [float(m[:, i].max()) for i in range(3)]
    rep['mag_lsb_abs_median'] = float(np.median(np.abs(m)))
    rep['compiled_tag'] = FW_TAG_EXPECT
    return rep


if __name__ == '__main__':
    import sys
    ok = selfcheck()
    if len(sys.argv) > 1:
        a, info = load_frames(sys.argv[1])
        print('info:', {k: v for k, v in info.items() if k != 'cand'})
        for k, v in frame_report(a).items():
            print('  %-20s %s' % (k, v))
    sys.exit(0 if ok else 1)
