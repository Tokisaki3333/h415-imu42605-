# -*- coding: utf-8 -*-
"""VER=48 -> 49：地磁注入掩码 0x01C0 -> 0x0100（只修偏航，不修倾角）。

VER=48 实测（fw_tag 3179783，29.34 s）：
  启动：prop_ok=1 100%、f_ok=1 100%、mag_used=1 **93.9%**  <- 地磁终于开始工作
  修正去向：|dq| x/y/z = 1.29/0.70/1.86 度，占比 33.5/18.1/48.4%
  但 mag_r p50 = 81.4 度（静止段 82.8 / 89.3），且**静止段 EKF 总转角 13565 / 15867 度**
  （|gyro| p50 只有 0.2~0.3 dps）=> 地磁在静止时把姿态按 ~650 度/秒持续驱动，不收敛。
  同时 mag_bh p50 = 0.5166，而由 DIP_TAN=2.08 推出的标称 |v0| = cos(64.26) = 0.4335
  => 实测水平分量比模型大 19%，两者不一致。

病因：二维观测**同时**约束倾角与偏航（掩码 0x01C0）。而磁场模型 B0 的磁倾角有约
  10.5 度标定误差 -> 地磁要求一个"错误"的倾角 -> 与重力观测(M6)对抗 ->
  姿态被两边来回拉 -> 静止时也持续自转、mag_r 永远停在 80~90 度。
  倾角本来就有重力这个可靠绝对基准，**不该让带标定误差的地磁去修**。

修法：掩码改回 0x0100 —— 仍用二维线性新息（无除法、无退化放大），
      但只注入偏航。倾角交给重力。这样地磁不再和重力对抗，偏航才有机会收敛。
"""
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


c = open(P, 'rb').read().decode('gbk')
# 先查 EKF_N（上一轮我把传播行数当成了 17，必须核实）
hdr = open(U + r'\inc\v5f_proc.h', 'rb').read().decode('gbk', 'ignore')
m = re.search(r'#define\s+EKF_N\s+(\S+)', hdr) or re.search(r'#define\s+EKF_N\s+(\S+)', c)
print('  EKF_N = %s' % (m.group(1) if m else '未找到'))

a = "0x01C0u, V5F_EKF_MAG_K_MAX"
assert c.count(a) == 1, '掩码锚点 %d 次' % c.count(a)
c = c.replace(a, "0x0100u, V5F_EKF_MAG_K_MAX", 1)
dump(P, c, 'gbk', 'v49')

t = open(T, 'rb').read().decode('gbk')
assert t.count('#define V5F_FW_VER        48u') == 1
t = t.replace('#define V5F_FW_VER        48u', '#define V5F_FW_VER        49u', 1)
dump(T, t, 'gbk', 'v49')

c2 = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c2.index('static void ekf_m7_mag')
m7 = c2[i0:c2.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==49', ver == 49),
      ('掩码=0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7 and '0x01C0u' not in m7),
      ('二维观测仍在', 'r[0] = Bn[0] - b0x;' in m7),
      ('H 三列仍在(供协方差)', 's_H[1][8] =  b0x;' in m7),
      ('死点仍在', 's_mag_bh < V5F_EKF_MAG_BH_MIN' in m7),
      ('门限仍 150', '150.0f' in t2),
      ('列数仍 133', nch == 133),
      ('{ } 平衡', c2.count('{') == c2.count('}')),
      ('/* */ 平衡', c2.count('/*') == c2.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
