# -*- coding: utf-8 -*-
"""VER=48 -> 49：真正的死点门 —— 用**机体系**水平分量，不是导航系。

--- 用户实测描述（关键）---
  "偏航角所绕的轴连接到磁北时数据会疯转"
  偏航角所绕的轴 = 机体 z 轴。当机体 z 与磁场方向重合（板子俯仰到磁倾角附近、
  机体系磁场几乎全落在 z 上）时，f_x,f_y -> 0。

--- 为什么旧门没用 ---
旧门判据是 bh2 = Bn[0]^2 + Bn[1]^2，Bn = R(q)f 是**导航系**磁场。
导航系（近水平）下 bh 恒等于当地磁场的水平分量占比，与姿态无关，
实测 VER=47：mag_bh 恒在 0.436~0.470，bh<0.30 占 0.00% —— **从未触发过一次**。
而真正的死点在这个姿态下 bh 反而**很大**（磁场沿机体 z，转到导航系仍是大水平分量），
所以旧门在死点上是完全瞎的。

--- 数学 ---
方位角 hh = atan2(Bn_x, Bn_y)，Bn = R(q)f。
  yaw 灵敏度 dh/dpsi = -1（恒定）—— 前提是 f 精确。
  但 f 并不精确（标定残差、硬磁等，量级 sigma_B）。
  机体系水平分量 fhb = hypot(f_x,f_y) 是 yaw 信息的**载波**：
      yaw 信息量 ~ fhb，而误差 ~ sigma_B / fhb。
  fhb -> 0 时误差发散，于是地磁把"纯姿态误差"当成 yaw 误差，
  以 K 满额灌进偏航 -> **疯转**。VER=45 的"框架偏置积分"会把这个垃圾
  当成持续偏置 latch 下来，后果更重。

--- 修法（连续加权 + 硬底线）---
  1) 硬底线：fhb < V5F_EKF_MAG_BHB_MIN(0.15) -> 丢样本，并且 **s_mag_r_seed 清零**，
     死点期间不许 latch 偏置。
  2) 连续加权：R_mag += (V5F_EKF_MAG_FERR / fhb)^2
     fhb=0.45(正常) -> 附加 sigma 约 1.3 度，影响很小；
     fhb=0.15(底线) -> 约 3.8 度，权重明显下降；
     fhb 再小则被硬门挡掉。
  导航系那道 bh2 门保留（防导航系磁场真正垂直的极端情况），两道并存。
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


# 1) 变量与计数
sub("    float R[1], r[1], Rt[3][3], Bn[3], mf[3], hh;",
    "    float R[1], r[1], Rt[3][3], Bn[3], mf[3], hh, fhb2;",
    '1a-局部变量 fhb2')
sub("static uint8_t  s_mag_spike;",
    "static uint8_t  s_mag_spike;\n"
    "static uint8_t  s_mag_dp;       /* ★VER=49 机体系死点丢弃计数（诊断）*/",
    '1b-s_mag_dp')

# 2) 机体系死点门（插在 mf 取值之后，早于一切计算）
#    注意该行全文出现 2 次（另一处在对齐块），必须限定到 M7 函数体内
_anchor = "    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];\n"
_f0 = t.index('static void ekf_m7_mag')
_i0 = t.index(_anchor, _f0)
_lim = t.index('static void ', _f0 + 10)
assert t.count(_anchor, _f0, _lim) == 1, 'M7 内 mf 取值行出现 %d 次' % t.count(_anchor, _f0, _lim)
t = t[: _i0 + len(_anchor)] + (
    "    /* \u2605VER=49 \u771f\u6b63\u7684\u6b7b\u70b9\u95e8\uff1a**\u673a\u4f53\u7cfb**\u6c34\u5e73\u5206\u91cf\u3002\n"
    "     * yaw \u6240\u7ed5\u7684\u8f74 = \u673a\u4f53 z\uff1b\u5b83\u4e0e\u78c1\u573a\u65b9\u5411\u91cd\u5408\u65f6 f_x,f_y -> 0\uff0c\n"
    "     * yaw \u4fe1\u606f\u91cf ~ fhb -> 0 \u800c\u8bef\u5dee ~ sigma_B/fhb -> \u53d1\u6563\uff0c\u4e8e\u662f\u6570\u636e\u75af\u8f6c\u3002\n"
    "     * \u65e7\u95e8\u7528\u7684\u662f\u5bfc\u822a\u7cfb bh2\uff08\u6052 0.436~0.470\uff0c\u4ece\u672a\u89e6\u53d1\uff09\uff0c\u5728\u6b7b\u70b9\u4e0a\u5b8c\u5168\u778e\u3002*/\n"
    "    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];\n"
    "    if (fhb2 < V5F_EKF_MAG_BHB_MIN * V5F_EKF_MAG_BHB_MIN) {\n"
    "        s_mag_r_seed = 0u;          /* \u6b7b\u70b9\u671f\u95f4\u4e0d\u8bb8 latch \u6846\u67b6\u504f\u7f6e */\n"
    "        if (s_mag_dp < 250u) s_mag_dp++;\n"
    "        if (s_rej[3] < 250u) s_rej[3]++;\n"
    "        s_gate_bits |= V5F_EKF_GB_CHI2;\n"
    "        return;\n"
    "    }\n") + t[_i0 + len(_anchor):]
n += 1
print('  ok  2-\u673a\u4f53\u7cfb\u6b7b\u70b9\u95e8')

# 3) R 连续加权
sub("        R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;",
    "        /* \u2605VER=49 \u6b7b\u70b9\u8fde\u7eed\u52a0\u6743\uff1a\u65b9\u4f4d\u89d2\u8bef\u5dee ~ sigma_B/fhb\uff0c\n"
    "         * fhb \u8d8a\u5c0f\u5730\u78c1\u5bf9 yaw \u7684\u6743\u9650\u8d8a\u5c0f\uff08\u800c\u4e0d\u662f\u7a81\u7136\u5f00/\u5173\uff09\u3002*/\n"
    "        {\n"
    "            float sr2 = V5F_EKF_MAG_FERR / sqrtf(fhb2);\n"
    "            R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD + sr2 * sr2;\n"
    "        }",
    '3-R 连续加权')

dump(P, t, 'gbk', 's2u')

# 4) tune
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_BH_MIN[^\r\n]*$', u, re.M)
assert m, '未找到 V5F_EKF_MAG_BH_MIN'
ins = ('\n/* \u2605VER=49 \u6b7b\u70b9\u95e8\uff08**\u673a\u4f53\u7cfb**\uff0c\u4e0d\u662f\u5bfc\u822a\u7cfb\uff09\u3002\n'
       ' * yaw \u6240\u7ed5\u7684\u8f74 = \u673a\u4f53 z\uff1b\u5b83\u4e0e\u78c1\u573a\u65b9\u5411\u91cd\u5408\u65f6\uff0c\n'
       ' * fhb = hypot(f_x,f_y) -> 0\uff0cyaw \u4fe1\u606f\u91cf\u8d8b\u96f6\u800c\u8bef\u5dee\u53d1\u6563 -> \u6570\u636e\u75af\u8f6c\u3002\n'
       ' * \u5bfc\u822a\u7cfb bh \u5728\u8fd9\u4e2a\u59ff\u6001\u4e0b\u53cd\u800c\u5f88\u5927\uff08\u5b9e\u6d4b\u6052 0.44~0.47\uff09\uff0c\u6240\u4ee5\u65e7\u95e8\u5f62\u540c\u865a\u8bbe\u3002\n'
       ' * \u5e73\u677f\u6c34\u5e73\u65f6 fhb = cos(\u78c1\u503e\u89d2) \u7ea6 0.47\uff1b0.15 \u7ea6\u5bf9\u5e94\u78c1\u573a\u5bf9\u673a\u4f53 z \u7684\u504f\u79bb 8.6 \u5ea6\u3002 */\n'
       '#define V5F_EKF_MAG_BHB_MIN      0.15f\n'
       '/* \u2605VER=49 \u5730\u78c1\u573a\u6a21\u578b\u8bef\u5dee\uff08\u5355\u4f4d\u77e2\u91cf\u91cf\u7ea7\uff09\uff0c\u7528\u4e8e\u6b7b\u70b9\u8fde\u7eed\u52a0\u6743\uff1a\n'
       ' * R_mag += (FERR/fhb)^2\u3002fhb=0.45 \u65f6\u9644\u52a0\u7ea6 1.3 \u5ea6\uff1bfhb=0.15 \u65f6\u7ea6 3.8 \u5ea6\u3002 */\n'
       '#define V5F_EKF_MAG_FERR         0.010f')
u = u[:m.end()] + ins + u[m.end():]
assert u.count('#define V5F_FW_VER        48u') == 1
u = u.replace('#define V5F_FW_VER        48u', '#define V5F_FW_VER        49u', 1)
dump(T, u, 'gbk', 's2u')

c = open(P, 'rb').read().decode('gbk')
h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==49', ver == 49), ('编辑数==3', n == 3),
      ('机体系门在', 'fhb2 < V5F_EKF_MAG_BHB_MIN * V5F_EKF_MAG_BHB_MIN' in c),
      ('死点重置 seed', 's_mag_r_seed = 0u;          /*' in c),
      ('R 加权在', 'V5F_EKF_MAG_FERR / sqrtf(fhb2)' in c),
      ('导航系门仍在', 'bh2 < V5F_EKF_MAG_BH_MIN * V5F_EKF_MAG_BH_MIN' in c),
      ('M6 掩码仍 0x7EC0', '0x7EC0u' in c),
      ('tune 两条在', h.count('V5F_EKF_MAG_BHB_MIN') == 1 and h.count('V5F_EKF_MAG_FERR') == 1),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/')),
      ('括号差仍 -4', c.count('(') - c.count(')') == -4)]
h.encode('gbk'); c.encode('gbk')
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=49, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
