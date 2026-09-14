# -*- coding: utf-8 -*-
"""收尾：① count_cols 的 KEY 对齐（tilt=107 / vel_h=106）
        ② jf_load 补 CH_112 与 CH_113（权威表，索引按逐行核出的运行时列号）"""
import re
import shutil
import sys

CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
a = "'sigma_pos_h': 105, 'sigma_vel_h': 107, 'sigma_tilt': 106, 'nis': 108}"
b = "'sigma_pos_h': 105, 'sigma_vel_h': 106, 'sigma_tilt': 107, 'nis': 108}"
assert c.count(a) == 1, c.count(a)
open(CP, 'w', encoding='utf-8', newline='\n').write(c.replace(a, b, 1))
print('count_cols KEY: tilt=107, vel_h=106')

JL = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()

# --- CH_112（VER=10..19 的 112 列布局）---
if 'CH_112' not in j:
    TBL = '''
# ---- VER=10..19：112 列，**权威表**（列号由 tools/calib/count_cols.py 逐行核出）----
#   关键列: dt_us=25  fw_tag=76  下行三列=77/78/79  press_avg=80  e_ac/ac_bypass=81/82
#           EKF 块 = 83..111
CH_112 = {
    'q': 0, 'v_nav': 4, 'level_dps': 7, 'is_static': 8, 'acc_traction': 9,
    'a_lin': 10, 'accel_bias_g': 13, 'flags': 16, 'dma1_irq_us': 17,
    'gyro_lsb': 18, 'accel_lsb': 21, 'temp_imu': 24, 'dt_us': 25,
    'gyro_dps': 26, 'gyro_bias_dps': 29, 'accel_g': 32, 'bias_evidence_gran': 35,
    'att_tilt': 36, 'vel_soft': 37, 'mag_lsb': 38, 'ist_cnt': 41,
    'mag_f': 42, 'mag_norm': 45, 'mag_Bw': 46, 'psi_true': 49, 'mag_trust': 50,
    'baro_press': 51, 'baro_temp': 52, 'baro_cnt': 53,
    'gps_speed': 54, 'gps_rmc_flags': 55, 'gps_rmc_cnt': 56,
    'gps_alt': 57, 'gps_fixq': 58, 'gps_sv': 59, 'gps_hdop': 60,
    'gps_gga_flags': 61, 'gps_gga_cnt': 62, 'gps_vdop': 63, 'gps_gsa_cnt': 64,
    'gps_course': 65,
    'gps_lat_int': 66, 'gps_lat_frac': 67, 'gps_lon_int': 68, 'gps_lon_frac': 69,
    'gps_sats_view': 70, 'gps_snr_avg': 71, 'gps_snr_min': 72, 'gps_snr_n': 73,
    'gps_gsv_cnt': 74, 'gps_gsv_talkers': 75, 'fw_tag': 76,
    'cmd_echo': 77, 'cmd_cnt': 78, 'cmd_last': 79,
    'baro_press_avg': 80, 'stat_e_ac': 81, 'stat_ac_bypass': 82,
    'ekf_p': 83, 'ekf_v': 86, 'ekf_q': 89, 'ekf_a_nav': 93,
    'ekf_ba': 96, 'ekf_bg': 99, 'ekf_b_baro': 102, 'ekf_gate_bits': 103,
    'ekf_sigma_yaw_deg': 104, 'ekf_sigma_pos_h': 105, 'ekf_sigma_vel_h': 106,
    'ekf_nis': 107,
}

# EKF gate_bits（列 103）的位名
EKF_GB = [
    (0x0001, 'gps_pos'), (0x0002, 'gps_alt'), (0x0004, 'baro'),
    (0x0008, 'gps_vel'), (0x0010, 'zupt'), (0x0020, 'tilt'),
    (0x0040, 'mag_yaw'), (0x0080, 'aligned'), (0x0100, 'step'),
    (0x0200, 'chi2_rej'), (0x0400, 'origin'), (0x0800, 'acc_sat'),
]

'''
    a = 'CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80}'
    assert j.count(a) == 1
    j = j.replace(a, TBL.lstrip('\n') +
                  'CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112}', 1)
    j = j.replace('CH = CH_80', 'CH = CH_112', 1)
    a = """    if nch == 80:
        return CH_80"""
    assert j.count(a) == 1
    j = j.replace(a, a + """
    if nch == 112:
        return CH_112""", 1)
    print('jf_load.py: CH_112 已加')

# --- CH_113（VER=20：插 ekf_sigma_tilt_deg 到 107，其后整体后移一位）---
if 'CH_113' not in j:
    a = 'CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112}'
    assert j.count(a) == 1, j.count(a)
    j = j.replace(a, """# ---- VER=20：113 列（在 112 列上插 ekf_sigma_tilt_deg=107，107 起整体后移一位）----
CH_113 = dict(CH_112)
CH_113.update({k: (v + 1 if v >= 107 else v) for k, v in list(CH_113.items())})
CH_113.update({'ekf_sigma_tilt_deg': 107})

CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113}""", 1)
    j = j.replace('CH = CH_112', 'CH = CH_113', 1)
    a = """    if nch == 112:
        return CH_112"""
    assert j.count(a) == 1
    j = j.replace(a, a + """
    if nch == 113:
        return CH_113""", 1)
    j = j.replace('已知 78/80 与所有 <=92 的历史版', '已知 78/80/112/113 与所有 <=92 的历史版', 1)
    print('jf_load.py: CH_113 已加')

open(JL, 'w', encoding='utf-8', newline='\n').write(j)

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import importlib
import jf_load
importlib.reload(jf_load)
c112, c113 = jf_load.ch_for(112), jf_load.ch_for(113)
print()
print('ch_for(112): fw_tag %d  sigma_vel_h %d  nis %d' % (
    c112['fw_tag'], c112['ekf_sigma_vel_h'], c112['ekf_nis']))
print('ch_for(113): fw_tag %d  sigma_vel_h %d  sigma_tilt %d  nis %d' % (
    c113['fw_tag'], c113['ekf_sigma_vel_h'], c113['ekf_sigma_tilt_deg'], c113['ekf_nis']))
span = {'q': 4, 'v_nav': 3, 'a_lin': 3, 'accel_bias_g': 3, 'gyro_lsb': 3, 'accel_lsb': 3,
        'gyro_dps': 3, 'gyro_bias_dps': 3, 'accel_g': 3, 'mag_lsb': 3, 'mag_f': 3,
        'mag_Bw': 3, 'ekf_p': 3, 'ekf_v': 3, 'ekf_q': 4, 'ekf_a_nav': 3, 'ekf_ba': 3,
        'ekf_bg': 3, 'ekf_nis': 5}
for nm, cc, tot in (('CH_112', c112, 112), ('CH_113', c113, 113)):
    used = {}
    bad = 0
    for k, v in cc.items():
        for q in range(span.get(k, 1)):
            if (v+q) in used:
                print('  ★ %s 列 %d 冲突: %s vs %s' % (nm, v+q, used[v+q], k))
                bad += 1
            used[v+q] = k
    miss = [i for i in range(tot) if i not in used]
    print('%s: 覆盖 %d/%d, 冲突 %d, 未命名 %s' % (nm, len(used), tot, bad, miss))
