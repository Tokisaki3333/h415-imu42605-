# -*- coding: utf-8 -*-
"""VER=14 -> 15：预积分必须每一帧都做，不能只在 stage 0..15 里做。

实测（VER=14，北->东->南->西 台阶）：
    旧链偏航台阶 -92.16 度，EKF 只跟了 -65.57 度，比值 0.7115
    阶段机一周期 = 16 帧预积分 + 7 帧观测更新 = 23 帧
    16/23 = 0.6957  —— 对上。
即每 23 帧丢掉 7 帧的陀螺/加计，姿态少转 30%；磁环只好事后往回拉 4 秒
（trace 里 t=7~11 s 板子静止、EKF 偏航却以 9~12 dps 被拽回来）。

修法：把预积分（含 s_dt_e 累加）提到阶段判断之外，每帧无条件做；
      协方差按行传播仍只在 stage 0..15 做（16 行摊 16 帧，一行不多一行不少）。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')

a = """    if (s_stage < V5F_EKF_DECIM) {
        /* 预积分 + 协方差传播一行（每帧只有这点活） */
        raw_w_rads(h, w);
        raw_f_mps2(h, f);
        /* 必须**减去估计的陀螺零偏**再积分。误差状态里 F 写的是 dth' = -R dbg，
         * 那正是在说"标称角速度 = 量测 - bg_hat"。第一版忘了减：bg 被新息推着一路涨
         * （实测 -803 dps，真值只有 0.0x dps）却从不反馈到姿态，等于让偏航误差全被
         * 一个假偏置状态吃掉 —— 偏航环看着很稳（sigma 0.06 度），实际是拿 bg 当垃圾桶。
         * 加计那一路本来就是对的（下面 dv 里减了 ba*dt_e）。 */
        for (i = 0u; i < 3u; i++) {
            s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
            s_dvb[i] += f[i] * dt;
        }
        s_dt_e += dt;
        if (s_F_ok && s_prop_row < EKF_N) ekf_prop_row();
        s_stage++;
    } else {"""
b = """    /* ---------------- 预积分：**每一帧**都要做 ----------------
     * ★ 必须放在阶段判断之外。第一版把它写在 `s_stage < DECIM` 里，于是那 7 个
     *   观测更新帧（stage 16..22）的陀螺/加计数据被整个丢掉 —— 一周期 23 帧里
     *   只积了 16 帧，姿态少转 16/23 = 0.6957。实测一个 92.16 度的台阶 EKF 只跟了
     *   65.57 度，比值 0.7115，与 0.6957 对上。后果是磁环一直在后面追（板子静止时
     *   EKF 偏航还被以 9~12 dps 拽回来补缺口），v/p 同样按 70% 缩放。
     * 必须减估计的陀螺零偏：误差状态里 F 写的是 dth' = -R dbg，就是在说
     *   "标称角速度 = 量测 - bg_hat"。加计那一路本来就是对的（下面 dv 里减 ba*dt_e）。 */
    raw_w_rads(h, w);
    raw_f_mps2(h, f);
    for (i = 0u; i < 3u; i++) {
        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
        s_dvb[i] += f[i] * dt;
    }
    s_dt_e += dt;

    if (s_stage < V5F_EKF_DECIM) {
        /* 协方差：本帧传播一行。16 行正好摊在 stage 0..15 这 16 帧里，
         * 到 stage 16 时已经传播完毕，随后 7 帧做观测更新与收尾。 */
        if (s_F_ok && s_prop_row < EKF_N) ekf_prop_row();
        s_stage++;
    } else {"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1g')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 预积分提出阶段机（每帧都做）')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        14u') == 1
u = u.replace('#define V5F_FW_VER        14u', '#define V5F_FW_VER        15u', 1)
# 同时收紧磁偏航 R：NIS 实测 5.2 -> R 大约给了 sqrt(5.2)=2.3 倍富余
a2 = '#define V5F_MAG_YAW_R_DEG      0.7f'
assert u.count(a2) == 1
u = u.replace(a2, '#define V5F_MAG_YAW_R_DEG      0.5f', 1)
shutil.copy2(T, T + '.bak_s1g')
open(T, 'wb').write(u.encode('gbk'))
print('VER 14 -> 15；V5F_MAG_YAW_R_DEG 0.7 -> 0.5（实测 nis_mag p50 5.2，R 给大了）')
print('fw_tag 期望 = %d' % ((15 << 16) | (112 << 8) | 1 | 2 | 4))

for l in t.split('\n'):
    if 's_dt_e +=' in l or 'raw_f_mps2(h, f);' in l or 'ekf_prop_row();' in l:
        print('   ', l.strip()[:88])
