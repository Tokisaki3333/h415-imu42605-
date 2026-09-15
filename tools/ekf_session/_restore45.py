# -*- coding: utf-8 -*-
"""把 VER=46 那 4 个文件恢复到补丁前（.bak_v46h），供修正 j1 后重打。"""
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
for f in (U + r'\src\proc_ekf.c', U + r'\inc\SPI_rx.h',
          U + r'\src\SPI_rx.c', U + r'\inc\v5f_tune.h'):
    shutil.copy2(f + '.bak_v46h', f)
    print('restored', f.split('\\')[-1])
c = open(U + r'\src\proc_ekf.c', 'rb').read().decode('gbk')
print('  proc_ekf.c: 无 mag_rx %s ; { } 平衡 %s'
      % ('s_mag_rx' not in c, c.count('{') == c.count('}')))
