# -*- coding: utf-8 -*-
"""补齐 VER=20 剩下的三处（SPI_rx.c 报列 / count_cols / jf_load）。用精确锚点。"""
import re
import shutil

S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
t = open(S, 'rb').read().decode('gbk')
m = re.search(r'#define JF_CH_NUM[^\n]*', t)
print('现状 repr:', repr(m.group(0)[:60]))
print('锚点计数:', t.count(m.group(0)[:24]))

# 用正则替换，避免空白/注释差异
n0 = len(t)
t2 = re.sub(r'#define JF_CH_NUM\s+112u', '#define JF_CH_NUM     113u', t, count=1)
assert t2 != t, 'JF_CH_NUM 没改到'
t = t2
t2 = re.sub(r'/\* 454 = 帧头2\+帧长2\+载荷448\+帧尾2 \*/',
            '/* 458 = 帧头2+帧长2+载荷452+帧尾2 */', t, count=1)
assert t2 != t, 'FRAME_LEN 注释没改到'
t = t2

a = '    ch[c++] = g_v5f_hold.ekf.sigma_vel_h;\n'
assert t.count(a) == 1, ('sigma_vel_h 锚点', t.count(a))
t = t.replace(a, a + '    ch[c++] = g_v5f_hold.ekf.sigma_tilt_deg;   /* 倾角 1sigma（度）*/\n', 1)

# 注释里的列号说明：正则整体替换那一段
old = re.search(r'     \* 105       sigma_pos_h[^\n]*\n     \* 106       sigma_vel_h[^\n]*\n     \* 107\.\.111  ekf\.nis\[5\][^\n]*', t)
if old:
    t = t.replace(old.group(0),
                  '     * 105       sigma_pos_h     水平位置 1sigma m\n'
                  '     * 106       sigma_tilt_deg  倾角(水平两轴合成) 1sigma（度）—— 剧烈运动里\n'
                  '     *                          r_yaw = 真实偏航误差 + tan(I)*倾角误差，倾角误差\n'
                  '     *                          只能从这里读；也是验证 M7 自适应 R 的唯一途径\n'
                  '     * 107       sigma_vel_h     水平速度 1sigma m/s\n'
                  '     * 108..112  ekf.nis[5]      归一化新息平方：位置/速度/气压/重力/磁偏航。\n'
                  '     *                          应分别趋近 2/2/1/3/1；远大于维数 -> R 给小了。', 1)
else:
    print('  （列号注释段没匹配到，跳过；不影响功能）')
t = re.sub(r'     \* ★ 0~82 列号一个都不动[^\n]*\n',
           '     * ★ 0~106 列号一个都不动，107 起为新增/顺移（共 113 列）。 */\n', t, count=1)
assert len(t) > n0 and t.count('/*') == t.count('*/')
shutil.copy2(S, S + '.bak_s1l')
open(S, 'wb').write(t.encode('gbk'))
print('SPI_rx.c: JF_CH_NUM=113, sigma_tilt 已加')

# ---- count_cols.py ----
CP = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
c = open(CP, encoding='utf-8').read()
c = re.sub(r'assert n == 112,', 'assert n == 113,', c, count=1)
c = re.sub(r"'sigma_pos_h': 105, 'sigma_vel_h': 106, 'nis': 107\}",
           "'sigma_pos_h': 105, 'sigma_vel_h': 107, 'sigma_tilt': 106, 'nis': 108}", c, count=1)
open(CP, 'w', encoding='utf-8', newline='\n').write(c)
print('count_cols.py 已同步')

# ---- jf_load.py ----
JL = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib\jf_load.py'
j = open(JL, encoding='utf-8').read()
a = 'CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112}'
assert j.count(a) == 1, j.count(a)
j = j.replace(a, """# ---- VER=20：113 列（在 112 列上插了 ekf_sigma_tilt_deg，107 起的列整体后移一位）----
CH_113 = dict(CH_112)
CH_113.update({k: (v + 1 if v >= 107 else v) for k, v in list(CH_113.items())})
CH_113.update({'ekf_sigma_tilt_deg': 107, 'ekf_sigma_vel_h': 108, 'ekf_nis': 109})

CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113}""", 1)
j = j.replace('CH = CH_112', 'CH = CH_113', 1)
a = """    if nch == 112:
        return CH_112"""
assert j.count(a) == 1
j = j.replace(a, a + """
    if nch == 113:
        return CH_113""", 1)
j = j.replace('已知 78/80/112 与', '已知 78/80/112/113 与', 1)
open(JL, 'w', encoding='utf-8', newline='\n').write(j)
print('jf_load.py 已同步')
