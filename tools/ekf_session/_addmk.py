# -*- coding: utf-8 -*-
"""把 proc_ekf.c 加进 MounRiver 生成的 subdir.mk。"""
p = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\obj\User\src\subdir.mk'
t = open(p, 'rb').read().decode('utf-8')
if 'proc_ekf' in t:
    print('已存在，跳过')
else:
    pairs = [
        ('../User/src/proc_attitude.c \\', '../User/src/proc_attitude.c \\\n../User/src/proc_ekf.c \\'),
        ('./User/src/proc_attitude.d \\',   './User/src/proc_attitude.d \\\n./User/src/proc_ekf.d \\'),
        ('./User/src/proc_attitude.o \\',   './User/src/proc_attitude.o \\\n./User/src/proc_ekf.o \\'),
    ]
    for a, b in pairs:
        assert t.count(a) == 1, (a, t.count(a))
        t = t.replace(a, b, 1)
    open(p, 'wb').write(t.encode('utf-8'))
    print('subdir.mk OK')

for l in open(p, 'rb').read().decode('utf-8').split('\n'):
    if 'proc_ekf' in l:
        print('   ', l)
