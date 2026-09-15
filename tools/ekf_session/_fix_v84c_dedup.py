# -*- coding: utf-8 -*-
"""修正 VER=84：sigma_tilt_deg 是既有字段，去掉我重复加的那个。
   新列变 5 个 -> JF_CH_NUM 154, mag_rs=149 tilt_inv_ms=150 mag_hold=151 grav_ok=152 grav_nis=153"""
import os, shutil

ROOT = r'C:\Users\33\Documents\v2\h415-imu42605-'
P_EKF = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
P_H = os.path.join(ROOT, r'V5F\User\inc\SPI_rx.h')
P_C = os.path.join(ROOT, r'V5F\User\src\SPI_rx.c')
P_L = os.path.join(ROOT, r'tools\calib\jf_load.py')


def rw(p, e, fn):
    t = open(p, 'rb').read().decode(e)
    t2 = fn(t)
    d = t2.encode(e)
    open(p, 'wb').write(d)
    os.utime(p, None)
    assert open(p, 'rb').read() == d
    print('  fixed %-16s %6d -> %6d B' % (os.path.basename(p), len(t.encode(e)), len(d)))


def rm1(t, s, what):
    assert t.count(s) == 1, '%s: %d 次' % (what, t.count(s))
    return t.replace(s, '', 1)


rw(P_EKF, 'gbk', lambda t: rm1(t, '\n       h->ekf.sigma_tilt_deg = s_tilt_sig_deg;', 'publish dup'))
rw(P_H, 'gbk', lambda t: rm1(
    t, '\n        float    sigma_tilt_deg;   /* \u2605VER=84 \u503e\u89d2 1sigma(\u5ea6) */', 'hdr dup'))
rw(P_C, 'gbk', lambda t: rm1(
    t, '\n        ch[c++] = g_v5f_hold.ekf.sigma_tilt_deg;   /* \u2605VER=84 \u503e\u89d2 1sigma(\u5ea6) */', 'report dup'))
rw(P_C, 'gbk', lambda t: t.replace('#define JF_CH_NUM     155u', '#define JF_CH_NUM     154u', 1))
rw(P_L, 'utf-8', lambda t: t.replace(
    "CH_155 = dict(CH_149)\n"
    "CH_155.update({'ekf_sigma_tilt_deg': 149, 'ekf_mag_rs': 150, 'ekf_tilt_inv_ms': 151,\n"
    "               'ekf_mag_hold': 152, 'ekf_grav_ok': 153, 'ekf_grav_nis': 154})  # VER=84\n"
    "CH_BY_NCH[155] = CH_155\n"
    "CH = CH_155",
    "CH_154 = dict(CH_149)\n"
    "CH_154.update({'ekf_mag_rs': 149, 'ekf_tilt_inv_ms': 150, 'ekf_mag_hold': 151,\n"
    "               'ekf_grav_ok': 152, 'ekf_grav_nis': 153})  # VER=84\n"
    "CH_BY_NCH[154] = CH_154\n"
    "CH = CH_154", 1))

print()
print('fw_tag = %d ; frame = %d B' % ((84 << 16) | (154 << 8) | 7, 154 * 4 + 6))
