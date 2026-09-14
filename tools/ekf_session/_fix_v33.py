# -*- coding: utf-8 -*-
"""VER=32 -> 33：气压掩码 0x0004 -> 0x0024（δp_z + δv_z，不含 b_baro）。"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
a = '&s_nis[2], &s_rej[2], 0x0004u);'
assert t.count(a) == 1
t = t.replace(a, '&s_nis[2], &s_rej[2], 0x0024u);', 1)

old = """        /* ★ 掩码 = 0x0004：**只准动 dp_z**（重力所在的那根位置轴）。
         *   气压测的就是高度，没有资格碰速度（速度自有 ZUPT 与 GPS 多普勒），
         *   也没有资格碰绝对基准 b_baro（那归 GPS 高度 M2）。实测放宽到
         *   p_z+v_z+b_baro 会让差分观测把垂直速度也一起改，而它并没有那个信息。 */"""
if t.count(old) != 1:
    # 上一版写的是中文 d 前缀，做个宽松匹配
    m = re.search(r'        /\* ★ 掩码 = 0x0004.*?\*/\n', t, re.S)
    assert m, '找不到旧注释'
    old = m.group(0).rstrip('\n')
assert t.count(old) == 1, t.count(old)

new = """        /* ★ 掩码 = 0x0024：dp_z + dv_z，**不含 b_baro**。正确边界是：
         *   · dv_z 合法：气压是位置量，位置-速度的耦合来自动力学模型
         *     （P[p_z][v_z] 由 F 建立），位置观测据此修垂直速度是标准 Kalman 行为，
         *     不是凭空造信息。必须放开，否则垂直速度只能靠积分一路漂。
         *   · b_baro 不合法：它是**绝对**基准，只有 GPS 高度 M2 有资格定它；
         *     没有 M2 时它没有观测者（每周期归零已强制它保持惰性）。
         *   · 姿态/零偏一律不许：气压对姿态没有任何信息。
         *   注：VER=30 的速度发散真凶是被切掉的 ba（泄漏无处可去），不是这一项。 */"""
t = t.replace(old, new, 1)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2b')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        32u') == 1
u = u.replace('#define V5F_FW_VER        32u', '#define V5F_FW_VER        33u', 1)
sw(T, u, 'gbk', 's2b')

c = open(P, 'rb').read().decode('gbk')
print('M3 -> 0x0024:', '&s_rej[2], 0x0024u);' in c)
for m in ['0x0E3F', '0x8024', '0x0024', '0x0E18', '0x0E38', '0x7FC0', '0x71C0']:
    print('   %s x%d' % (m, c.count(m)))
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
print('VER=%d  fw_tag = %d' % (ver, (ver << 16) | (nch << 8) | 1 | 2 | 4))
