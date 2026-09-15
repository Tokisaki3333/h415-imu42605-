# -*- coding: utf-8 -*-
"""VER=87 重做：v5f_tune.h 从 .bak_v87 还原后，按"定义行 + 其注释续行"精确替换"""
import os, shutil

ROOT = r'C:\Users\33\Documents\v2\h415-imu42605-'
P_T = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
BAK = P_T + '.bak_v87'

t = open(BAK, 'rb').read().decode('gbk')          # VER=86 状态
print('  从备份还原: %d B' % len(t.encode('gbk')))

lines = t.split('\n')
i = next(k for k, l in enumerate(lines) if l.startswith('#define V5F_MAG_YAW_R_DEG'))
j = i + 1
while not lines[j].startswith('#define'):
    j += 1
print('  替换 L%d..L%d' % (i + 1, j))
NEW = [
 '#define V5F_MAG_YAW_R_DEG      0.5f    /* \u78c1\u822a\u5411\u5355\u6b21\u91c7\u6837\u566a\u58f0(1sigma)\u3002',
 '                                        * \u2605VER=87 \u5b83\u53ea\u7528\u4e8e YAW_P_MIN \u4e0e R \u9879\u57fa\u51c6\uff0c',
 '                                        * **\u4e0d\u518d**\u5f53\u4f5c\u78c1\u7684\u53ef\u4fe1\u5ea6\uff08\u89c1 MAG_R_DEG\uff09\u3002 */',
 '/* \u2605VER=87 \u78c1\u7684**\u603b**\u822a\u5411\u4e0d\u786e\u5b9a\u5ea6\uff08\u542b\u5b8f\u89c2\u8bef\u5dee\uff09\uff0c\u5b83\u51b3\u5b9a\u78c1\u73af\u5e26\u5bbd\u3002',
 ' * \u5b9e\u6d4b(VER=86 \u91c7\u96c6, thm - \u65e7\u59ff\u6001\u504f\u822a\u9010\u6bb5\u91cf)\uff1a\u9759\u6b62\u6bb5\u6bb5\u5185 std 0.36~0.5 \u5ea6\uff1b',
 ' *   \u8fd0\u52a8\u6bb5\u6bb5\u5185 std 4.4~14.1 \u5ea6\uff1b\u6bb5\u95f4\u4e2d\u4f4d\u6570\u8df3\u53d8 +21.6 / -21.7 / +12.4 / +21.1 / -19.0 \u5ea6\uff1b',
 ' *   \u603b\u6563\u5e03 p50 2.37  p90 15.2  p99 26.8  max 69  std 8.4 \u5ea6\u3002',
 ' * \u7528 2.5 \u5ea6 -> K=P/(P+R)=0.85 -> tau=0.009 s\uff0c\u504f\u822a\u5b8c\u5168\u88ab\u78c1\u5e26\u8d70\uff08\u5b9e\u6d4b\u62d6\u5f17 10~33 \u5ea6\uff09\u3002',
 ' * \u53d6 20 \u5ea6(\u7ea6 p99)\uff1a\u81ea\u6d3d\u5e73\u8861 sigma_yaw ~3.4 \u5ea6, tau ~0.28 s\u3002',
 ' * \u6ce8\u610f\uff1a**\u4e0d\u80fd**\u7528\u94b3\u589e\u76ca\u53bb\u9650\u5e26\u5bbd\u2014\u2014\u90a3\u4f1a\u8ba9 P \u4e0e\u5b9e\u9645\u589e\u76ca\u4e0d\u4e00\u81f4(VER=84/85 \u6559\u8bad)\uff1b',
 ' *       \u9650\u5e26\u5bbd\u53ea\u80fd\u9760 R \u8bf4\u771f\u8bdd\u3002 */',
 '#define V5F_EKF_MAG_R_DEG       20.0f',
]
lines[i:j] = NEW
t2 = '\n'.join(lines)
d = t2.encode('gbk')
open(P_T, 'wb').write(d)
os.utime(P_T, None)
assert open(P_T, 'rb').read() == d
print('  写回: %d B (还原后 %d B, 差 %+d)' % (len(d), len(t.encode('gbk')), len(d) - len(t.encode('gbk'))))
