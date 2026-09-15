# -*- coding: utf-8 -*-
# 1) 根目录本会话文件全部搬进 tools/ekf_session/（保证不丢）
# 2) jf_load.py 补上当前 148 列的通道映射 CH_148
import os, shutil, glob, re

ROOT = r'C:\Users\33\Documents\v2'
DEST = os.path.join(ROOT, r'h415-imu42605-\tools\ekf_session')
os.makedirs(DEST, exist_ok=True)

pats = ['*.py', '*.md', '*.txt']
n_copy = 0
for p in pats:
    for f in glob.glob(os.path.join(ROOT, p)):
        name = os.path.basename(f)
        if name.startswith('__'):        # 跳过 __pycache__ 之类
            continue
        d = os.path.join(DEST, name)
        if os.path.abspath(f) == os.path.abspath(d):
            continue
        shutil.copy2(f, d)
        n_copy += 1
print('搬入/更新 %d 个文件到 tools/ekf_session/' % n_copy)

# ---- jf_load.py: 追加 CH_148 ----
p = os.path.join(ROOT, r'h415-imu42605-\tools\calib\jf_load.py')
src = open(p, encoding='utf-8').read()
if 'CH_148' in src:
    print('jf_load.py 已有 CH_148，跳过')
else:
    add = '''

# ---- ★VER=78 起：148 列（新增 4 个"罗盘量"）----
# 128 mag_rx / 129 mag_ry / 130 mag_vx / 131 mag_vy / 132 mag_v0x / 133 mag_v0y
# 134 mag_yawpre / 135..137 mag_dqx,dqy,dqz / 138..140 tilt_dq{x,y,z}
# 141..143 tilt_pr{x,y,z} / 144..147 mag_cmp_{thm,thp,amn,mhn}
CH_148 = dict(CH_127)
CH_148.update({'ekf_mag_rx': 128, 'ekf_mag_ry': 129,
               'ekf_mag_vx': 130, 'ekf_mag_vy': 131,
               'ekf_mag_v0x': 132, 'ekf_mag_v0y': 133,
               'ekf_mag_yawpre': 134,
               'ekf_mag_dqx': 135, 'ekf_mag_dqy': 136, 'ekf_mag_dqz': 137,
               'ekf_tilt_dqx': 138, 'ekf_tilt_dqy': 139, 'ekf_tilt_dqz': 140,
               'ekf_tilt_prx': 141, 'ekf_tilt_pry': 142, 'ekf_tilt_prz': 143,
               'mag_cmp_thm': 144,   # 罗盘实测航向(度)
               'mag_cmp_thp': 145,   # 罗盘预测航向(度)
               'mag_cmp_amn': 146,   # 姿态"上"与加计夹角(度)
               'mag_cmp_mhn': 147})  # 重力法平面内磁场模长
CH_BY_NCH[148] = CH_148
CH = CH_148
'''
    src = src.rstrip('\n') + add
    open(p, 'w', encoding='utf-8', newline='\n').write(src)
    print('jf_load.py: 已追加 CH_148 并设为默认')

# 校验
s2 = open(p, encoding='utf-8').read()
assert 'CH_148' in s2 and 'mag_cmp_thm' in s2
import importlib.util
spec = importlib.util.spec_from_file_location('jfl', p)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
print('jf_load.py 载入 OK: CH_BY_NCH 键 = %s ; 默认 CH 列数键 = %s'
      % (sorted(m.CH_BY_NCH.keys()), m.CH.get('mag_cmp_mhn')))
print('  CH_148 通道数 = %d' % (max(m.CH_148.values()) + 1))
