# -*- coding: utf-8 -*-
"""把上报块注释里的列号改成**运行时** 0 基真值（静态计数把它们整体 +2 了）。"""
import shutil

S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
t = open(S, 'rb').read().decode('gbk')
n0 = len(t)

old_hdr = '''     * ★ 下面一律 **0 基**列号，与 PC 端 tools/calib/jf_load.py 的表一致
     *   （0 基 25 = 当帧 dt_us，0 基 78 = fw_tag，0 基 79~81 = 下行验证三列，
     *     0 基 82 = 气压软件平均，0 基 83/84 = e_ac / ac_bypass）。
     *  85..87   ekf.p[3]'''
new_hdr = '''     * ★ 下面一律 **0 基**列号，与 tools/calib/count_cols.py（逐行核列数）一致：
     *     0 基 25 = 当帧 dt_us，0 基 76 = fw_tag，77~79 = 下行验证三列，
     *     80 = 气压软件平均，81/82 = e_ac / ac_bypass。
     *   ★ 为什么曾经写成 78：静态数 ch[c++] 会把"当帧 DRDY 间隔"那条 if/else 的
     *     三个分支都算成列，于是它之后的列号整体 +2。运行时只有一个分支。
     *  83..85   ekf.p[3]'''
assert t.count(old_hdr) == 1
t = t.replace(old_hdr, new_hdr, 1)

for a, b in [
    ('     *  88..90   ekf.v[3]',  '     *  86..88   ekf.v[3]'),
    ('     *  91..94   ekf.q[4]',  '     *  89..92   ekf.q[4]'),
    ('     *  95..97   ekf.a_nav[3]', '     *  93..95   ekf.a_nav[3]'),
    ('     *  98..100  ekf.ba[3]', '     *  96..98   ekf.ba[3]'),
    ('     * 101..103  ekf.bg[3]', '     *  99..101  ekf.bg[3]'),
    ('     * 104       ekf.b_baro', '     * 102       ekf.b_baro'),
    ('     * 105       ekf.gate_bits', '     * 103       ekf.gate_bits'),
    ('     * 106       sigma_yaw_deg', '     * 104       sigma_yaw_deg'),
    ('     * 107       sigma_pos_h', '     * 105       sigma_pos_h'),
    ('     * 108       sigma_vel_h', '     * 106       sigma_vel_h'),
    ('     * 109..113  ekf.nis[5]', '     * 107..111  ekf.nis[5]'),
    ('     * ★ 0~84 列号一个都不动，新量只追加在尾部。 */',
     '     * ★ 0~82 列号一个都不动，新量只追加在尾部。 */'),
]:
    assert t.count(a) == 1, (a, t.count(a))
    t = t.replace(a, b, 1)

assert len(t) > n0 and t.count('/*') == t.count('*/')
shutil.copy2(S, S + '.bak_ekf4')
open(S, 'wb').write(t.encode('gbk'))
print('注释列号已改为运行时 0 基真值')
