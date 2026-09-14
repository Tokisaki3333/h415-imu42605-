# -*- coding: utf-8 -*-
"""VER=22 -> 23：修两个我引入的 bug。

① mag 基线自举死锁：mag_norm 开机时是 0（IST 还没出样本），基线把 0 锁住，
   之后 dev ~0.94 > 0.10，而跟踪只在 dev<0.10 时跑 -> 永远追不上、门永远 0%。
   实测 VER=22 整份数据 mag 门 0.00%。
   修：只在**物理合理**（0.5~2.0）时锁初值；再加"长时间打不开就重新自举"的自愈。

② M5 的 ZUPT 不再依赖旧链 is_static。
   实测（VER=22）静止段 is_static 只有 6~8%，而 e_ac<0.2 是 100%、
   ||a_off|-1|<3% 是 99.9% —— is_static 是唯一拦路虎。它的判据是**陀螺直流电平**，
   剧烈运动后在线牵引冻结在错值上 -> 静止时 DC 仍超阈。
   后果致命：3.87 g 冲击把速度积到 6.8 m/s，随后静止段 ZUPT 不开 -> 速度不下来
   -> Δp 漂 8.95 m、全程 |p_h| 到 15.7 m。
   ★ 丢掉 is_static **不损失能力**：它本来就是直流判据，对平滑平移同样瞎
     （项目自己量过"60 s 匀速走动 is_static 100%"）。真正对平移有效的是
     交流能量 e_ac 与 ||a|-1|，两条都已在门里。
"""
import re
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


# -------- ① mag 基线：只在合理范围锁初值 + 自愈 --------
sub("static uint8_t  s_mn_seen;",
    "static uint8_t  s_mn_seen;\nstatic uint16_t s_mn_bad;             /* 基线连续打不开的次数（自愈用）*/",
    'mnvar')
sub("""        float dev;
        if (!s_mn_seen) { s_mn_lp = h->mag.mag_norm; s_mn_seen = 1u; }
        dev = fabsf(h->mag.mag_norm - s_mn_lp);
        if (dev < V5F_EKF_MAG_NORM_DEV) {
            s_mn_lp += (h->mag.mag_norm - s_mn_lp) * V5F_EKF_MAG_NORM_ALPHA;
        }""",
    """        float dev, mnow = h->mag.mag_norm;
        /* 初值只在**物理合理**时锁：开机时 mag_norm 是 0（IST 还没出样本），
         * 把 0 锁成基线 -> dev 恒 ~0.94 -> 跟踪分支永不执行 -> 门永远打不开。
         * 实测 VER=22 整份数据 mag 门 0.00%，就是这个自举死锁。 */
        if (!s_mn_seen) {
            if (mnow > V5F_EKF_MAG_NORM_LO && mnow < V5F_EKF_MAG_NORM_HI) {
                s_mn_lp = mnow;
                s_mn_seen = 1u;
            }
            dev = 0.0f;                       /* 还没基线：先放行 */
        } else {
            dev = fabsf(mnow - s_mn_lp);
            if (dev < V5F_EKF_MAG_NORM_DEV) {
                s_mn_lp += (mnow - s_mn_lp) * V5F_EKF_MAG_NORM_ALPHA;
            }
            /* 自愈：连续打不开太久（约 3 s）就认为基线本身错了，重新自举 */
            if (dev >= V5F_EKF_MAG_NORM_DEV) {
                if (s_mn_bad < 0xFFFFu) s_mn_bad++;
                if (mnow > V5F_EKF_MAG_NORM_LO && mnow < V5F_EKF_MAG_NORM_HI
                    && s_mn_bad > V5F_EKF_MAG_NORM_REBASE) {
                    s_mn_lp = mnow;
                    s_mn_bad = 0u;
                    dev = 0.0f;
                }
            } else {
                s_mn_bad = 0u;
            }
        }""", 'mnboot')

# -------- ② M5 去掉 is_static --------
sub("""    gate->ekf_zupt = (uint8_t)(h->stat.is_static
                    && (h->stat.e_ac < V5F_EKF_ZUPT_EAC_MAX)
                    && amag_ok
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));""",
    """    /* ★ 不再要 is_static。实测（VER=22）静止段 is_static 只有 6~8%，而
     *   e_ac<0.2 是 100%、||a_off|-1|<3% 是 99.9% —— 它是唯一的拦路虎。
     *   它的判据是**陀螺直流电平**，剧烈运动后在线牵引冻结在错值上 -> 静止时
     *   DC 仍超阈。后果致命：3.87 g 冲击把速度积到 6.8 m/s，随后静止段 ZUPT 不开
     *   -> 速度不下来 -> Δp 漂 8.95 m、|p_h| 到 15.7 m（而真值是 10×20 cm）。
     *   丢掉它**不损失能力**：直流判据对平滑平移同样瞎（项目实测"60 s 匀速走动
     *   is_static 100%"），真正对平移有效的是 e_ac 与 ||a|-1|，两条都已在门里。 */
    gate->ekf_zupt = (uint8_t)((h->stat.e_ac < V5F_EKF_ZUPT_EAC_MAX)
                    && amag_ok
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));""", 'zupt')

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1p')
open(P, 'wb').write(t.encode('gbk'))
chk = open(P, 'rb').read().decode('gbk')
for k, v in [('mag 初值范围检查', 'mnow > V5F_EKF_MAG_NORM_LO' in chk),
             ('mag 自愈', 's_mn_bad > V5F_EKF_MAG_NORM_REBASE' in chk),
             ('ZUPT 无 is_static', 'h->stat.is_static' not in chk.split('gate->ekf_zupt')[1][:600])]:
    print('  %-18s %s' % (k, v))
    assert v, k

# -------- 常量 --------
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_MAG_NORM_ALPHA   0.002f'
assert u.count(a) == 1
u = u.replace(a, a + """  /* 基线跟踪系数（约 500 周期 ≈ 1.4 s）*/
#define V5F_EKF_MAG_NORM_LO      0.50f   /* 基线初值只在这个区间内才锁（开机时 mag_norm=0）*/
#define V5F_EKF_MAG_NORM_HI      2.00f
#define V5F_EKF_MAG_NORM_REBASE  1000u   /* 基线连续打不开这么多周期（约 3 s）就重新自举 */""", 1)
assert u.count('#define V5F_FW_VER        22u') == 1
u = u.replace('#define V5F_FW_VER        22u', '#define V5F_FW_VER        23u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1p')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: 基线常量 + VER 23')
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c',
                         'rb').read().decode('gbk')).group(1))
print()
print('fw_tag 期望 = %d' % ((23 << 16) | (nch << 8) | 1 | 2 | 4))
