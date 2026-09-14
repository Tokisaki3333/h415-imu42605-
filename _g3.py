import io
p = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\obj\User\src\subdir.mk'
L = io.open(p, encoding='gbk', errors='replace').read().split('\n')
for i in (50, 51, 52, 53, 54):
    print('%4d %s' % (i+1, L[i]))
