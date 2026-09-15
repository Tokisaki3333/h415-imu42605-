# -*- coding: utf-8 -*-
"""
VER=81：按用户判断撤掉"倾角不可信就不观测"的门 + 删掉不可能触发的几何死点

用户的原话与逻辑：
  "姿态的重力法向量就是多路径融合的最优值，能用就用；类似的不要旁路传感器；
   不需要门控来处理'分不清偏航错与倾角错'，因为已经投影到重力法向量平面了，
   一定是偏航错，若有需要根据姿态逆解算一次即可。"
  => 投影轴用的是**姿态自己的**重力法向 a_up = R(q_hat)^T z_nav（多路径融合的最优值），
     测量与预测都在该平面内；两向量都在同一个平面里，失配只能是绕 a_up 的旋转，
     即偏航错。所以：
       · 删掉 VER=80 加的 gate->att_tilt 门（不需要）；
       · 删掉 BH_MIN 死点（它的触发条件是重力法向与磁北重合=真磁极，本纬度不可能；
         早先用户已指示可删）。
     保留的只剩数值保护（除零）与样本门、整角对齐、|f| 守卫、R_MAX/K_MAX。
版本 80 -> 81。
"""
import os, shutil
ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
_SUB = {'\u2202': 'd', '\u2261': '==', '\u00e2': 'a_up', '\u2192': '->', '\u00b7': '.',
        '\u00b1': '+/-', '\u00d7': 'x', '\u2264': '<=', '\u2265': '>=', '\u2248': '~',
        '\u0302': '', '\u1e91': 'z', '\u1e90': 'Z', '\u0177': 'y', '\u00ee': 'i', '\u00f4': 'o'}
def san(s):
    for k, v in _SUB.items(): s = s.replace(k, v)
    s.encode(ENC); return s
def load(p):
    with open(p, 'rb') as f: return f.read().decode(ENC)
def save(p, s, tag):
    d = s.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f: f.write(d)
    with open(p, 'rb') as f: assert f.read().decode(ENC) == s
    os.utime(p, None); return len(d)
def sub1(s, a, b, what):
    n = s.count(a); assert n == 1, '[%s] 命中 %d 次' % (what, n)
    return s.replace(a, b, 1)

e = load(EKF)

old_gate = san('''    /* ★VER=80 门控复用旧链"加速度牵引姿态"的门（处理函数 5 算出）：
     * 256 ms 窗 |off| 在 1g+-3% 且净 |w|<2 dps；实测占空比 静止 92% / 剧烈 32%。
     * 该门开 = 姿态倾角可信 = 本观测可信。磁场观测几何上分不清"偏航错"与
     * "倾角错"，所以倾角不可信时这条观测必须停。 */
    if (!gate->att_tilt) {
        if (s_rej[3] < 250u) s_rej[3]++;
        return;
    }
''')
e = sub1(e, old_gate,
         san('''    /* ★VER=81 不再设"倾角可信"门：投影轴取的是**姿态自己**的重力法向
     * a_up = R(q_hat)^T z_nav（多路径融合的最优值），测量与预测都已投到该平面内，
     * 两向量同处一个平面，失配只能是绕 a_up 的旋转 = 偏航错。倾角错在这一步
     * 已经被投影消掉了，不需要再来一道门。 */
'''), 'gate-rm')

old_bh = san('''    /* ★VER=75 保留的死点：|v| 过小说明姿态/投影面已不可信 */
    if (s_mag_bh < V5F_EKF_MAG_BH_MIN) {
        s_mag_r = 0.0f;
        s_mag_rx = 0.0f; s_mag_ry = 0.0f;
        s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;
        if (s_rej[3] < 250u) s_rej[3]++;
        s_gate_bits |= V5F_EKF_GB_CHI2;
        return;
    }
''')
e = sub1(e, old_bh,
         san('''    /* ★VER=81 已删除几何死点 BH_MIN：它的触发条件是"重力法向与磁北重合"
     * （即到了真磁极），本纬度磁倾 64.3 度、水平分量恒 0.4333，不可能发生。 */
'''), 'bh-rm')
n_e = save(EKF, e, 'v81')

t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        80u', '#define V5F_FW_VER        81u', 'VER')
n_t = save(TUNE, t, 'v81')

t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        81u' in t2
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
ob, cb = e2.count('{'), e2.count('}')
oc, cc = e2.count('/*'), e2.count('*/')
print('proc_ekf.c { } %d/%d   /* */ %d/%d' % (ob, cb, oc, cc))
assert ob == cb and oc == cc
for nm, ok in (('att_tilt 门已删', 'gate->att_tilt' not in blk),
               ('BH_MIN 死点已删', 'V5F_EKF_MAG_BH_MIN' not in blk),
               ('重力轴仍由姿态解出', 'rot_nb(Rt, up_nav, ab)' in blk),
               ('预测仍同一姿态', 'rot_nb(Rt, b0v, fp)' in blk),
               ('观测 1 维', blk.count('ekf_update(R, 1u, r,') == 1),
               ('H 只有偏航项', blk.count('s_H[0][8] = 1.0f') == 1),
               ('mask 0x0100', blk.count('0x0100u') == 1),
               ('样本门保留', 's_mag_cnt_upd' in blk),
               ('snap 保留', 'dpsi' in blk),
               ('|f| 守卫保留', 'f2 < 0.25f' in blk),
               ('R_MAX 保留', 'V5F_EKF_MAG_R_MAX_DEG' in blk)):
    print('  [%s] %s' % ('OK' if ok else '!!', nm)); assert ok, nm
print('v5f_tune.h %d B ; proc_ekf.c %d B' % (n_t, n_e))
print('PASS: VER=81 已写入')
