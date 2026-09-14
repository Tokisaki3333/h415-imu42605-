# -*- coding: utf-8 -*-
"""VER=45 -> 46：重力(倾角)校正不得用"转动快慢"当门，改为按 |a|-1 连续加权。

--- 实测（VER=44 日志，18.73 s）---
  tilt 门(0x20) 开启率：静止 85.3% / 微动 0.5% / 慢转 0.0% / 快转 0.0%
  对应的 |a|-1   ：静止 0.0001 / 微动 0.0064 / 慢转 0.0283 / 快转 1.1394
  sigma_tilt p50 ：静止 4.59 度 / 快转 31.79 度
  -> 板子一动，重力校正就被关掉，XY 只剩陀螺积分 -> 你看到的"XY 在飘"。

--- 错在哪 ---
原门：
    gate->ekf_tilt = (!sat && acc_valid
                      && (fabsf(am2 - 1.0f) < V5F_EKF_TILT_AMAG_TOL)
                      && ((h->stat.level_dps < V5F_EKF_TILT_LEV_DPS) || ac_bypass));
第三条"转动角速率小于阈值"是**物理上错误**的判据：匀速转动不破坏重力测量
（只是离心加速度，机体中心处为 w^2*r，慢转 55 dps 时 < 0.01 m/s^2）。
真正破坏重力测量的是**线加速度**，而它已经由第二条 |a|-1 表达。
结果：慢转时 |a|-1 只有 2.8%，加计完全可用，门却 0% 开 -> 倾角靠陀螺自由漂移。

--- 修法 ---
1) 去掉 level_dps 那一项（以及 ac_bypass 兜底），只保留"模值合理性"的**上限**保护：
       |a|-1 < V5F_EKF_TILT_AMAG_CAP (0.35)
   这不是"信任门"，只是剔除撞击类异常样本。
2) 把 |a|-1 从硬门搬进 **R**（连续加权，与地磁/气压同一套纪律）：
       sigma_tilt_lin = |a|-1  (rad)   [因为 f_lin 引起的倾角误差 ~ |f_lin|/g]
       R_tilt = TILT_SIG^2 + sigma_tilt_lin^2
   |a|-1=2.8% 时 R 只略增 -> 校正照样强；|a|-1=35% 时 R 很大 -> 更新很弱但**不归零**，
   倾角不会在运动期间累积漂移。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
n = 0


def sub(a, b, nm):
    global t, n
    c = t.count(a)
    assert c == 1, '锚点[%s] 匹配 %d 次（应为 1）' % (nm, c)
    t = t.replace(a, b, 1)
    n += 1
    print('  ok  %s' % nm)


# ---- 1. s_am_err ----
sub("static uint8_t  s_mag_spike;",
    "static uint8_t  s_mag_spike;\n"
    "static float    s_am_err;       /* |a|-1，加计模值偏差 -> 倾角观测的附加方差 */",
    '1-s_am_err')

# ---- 2. 门：去掉 level_dps 判据 ----
i0 = t.index('gate->ekf_tilt = (uint8_t)(!sat')
i1 = t.index(';', i0) + 1
oldg = t[i0:i1]
assert len(oldg) < 400 and 'level_dps' in oldg and 'TILT_AMAG_TOL' in oldg, oldg
newg = ('gate->ekf_tilt = (uint8_t)(!sat && h->imu.acc_valid\n'
        '                        && (fabsf(am2 - 1.0f) < V5F_EKF_TILT_AMAG_CAP));\n'
        '        /* |a|-1 交给 R 连续加权（见 ekf_m6_tilt / s_am_err），此处只挡撞击类异常。\n'
        '         * 原第三条 level_dps < TILT_LEV_DPS 已删：匀速转动不破坏重力测量，\n'
        '         * 实测慢转 |a|-1 仅 2.8% 而门 0% 开 -> 运动期倾角只有陀螺积分 -> XY 飘。 */\n'
        '        s_am_err = fabsf(am2 - 1.0f);')
t = t[:i0] + newg + t[i1:]
n += 1
print('  ok  2-门去掉 level_dps')

# ---- 3. M6 的 R 改为连续加权 ----
sub("    R[0] = R[4] = R[8] = V5F_EKF_TILT_SIG_RAD * V5F_EKF_TILT_SIG_RAD;",
    "    /* ★VER=46 连续加权：线加速度 f_lin 引起的倾角误差约为 |f_lin|/g = |a|-1 (rad)，\n"
    "     * 把它作为附加方差并入 R，于是\"加计可不可信\"变成权重大小而不是开/关门，\n"
    "     * 运动期间倾角仍然被持续牵引（原硬门在 |gyro|>TILT_LEV_DPS 时 0% 开 -> XY 漂）。*/\n"
    "    R[0] = R[4] = R[8] = V5F_EKF_TILT_SIG_RAD * V5F_EKF_TILT_SIG_RAD + s_am_err * s_am_err;",
    '3-M6 R 加权')

dump(P, t, 'gbk', 's2r')

# ---- 4. tune：新增 CAP，升 VER ----
u = open(T, 'rb').read().decode('gbk')
print('  [tune] 原 TILT_AMAG_TOL / TILT_LEV_DPS 值：',
      re.findall(r'#define\s+(V5F_EKF_TILT_AMAG_TOL|V5F_EKF_TILT_LEV_DPS)\s+([^\s/]+)', u))
if 'V5F_EKF_TILT_AMAG_CAP' not in u:
    m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_TILT_AMAG_TOL[^\r\n]*$', u, re.M)
    assert m, '未找到 V5F_EKF_TILT_AMAG_TOL'
    ins = ('\n/* ★VER=46 倾角(重力)观测的模值**上限**保护：|a|-1 超过本值才丢弃样本。\n'
           ' * 它只是剔除撞击类异常，不是"信任门"——加计的可信度由 R 连续加权表达\n'
           ' * （R += (|a|-1)^2）。原 TOL(0.05/0.03 量级) 在慢转 |a|-1=2.8% 时把门关死。0.35。 */\n'
           '#define V5F_EKF_TILT_AMAG_CAP     0.35f')
    u = u[:m.end()] + ins + u[m.end():]
assert u.count('#define V5F_FW_VER        45u') == 1
u = u.replace('#define V5F_FW_VER        45u', '#define V5F_FW_VER        46u', 1)
dump(T, u, 'gbk', 's2r')

# ---- 5. 校验 ----
c = open(P, 'rb').read().decode('gbk')
h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
ck = [('VER==46', ver == 46), ('编辑数==3', n),
      ('门不再用 level_dps', 'level_dps < V5F_EKF_TILT_LEV_DPS' not in c),
      ('CAP 已用', 'V5F_EKF_TILT_AMAG_CAP' in c),
      ('s_am_err 赋值在', 's_am_err = fabsf(am2 - 1.0f);' in c),
      ('M6 R 加权在', '+ s_am_err * s_am_err;' in c),
      ('tune CAP 在', h.count('V5F_EKF_TILT_AMAG_CAP') >= 1),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
h.encode('gbk'); c.encode('gbk')
for k, v in ck:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=46, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
