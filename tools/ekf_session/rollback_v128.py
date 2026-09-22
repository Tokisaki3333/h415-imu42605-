# -*- coding: utf-8 -*-
"""回滚 VER=128 牵引速度改动（只动 v5f_tune.h / proc_ekf.c 两个文件）。
门控（VER=126/127 恒定门限那一段）**不动**。"""
import os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

TUNE = os.path.join(B.REPO, 'V5F', 'User', 'inc', 'v5f_tune.h')
EKF = os.path.join(B.REPO, 'V5F', 'User', 'src', 'proc_ekf.c')
TAG = '.bak_v128'

B.restore(TUNE, TAG)
B.restore(EKF, TAG)

tune = open(TUNE, 'rb').read().decode('gbk')
ekf = open(EKF, 'rb').read().decode('gbk')
ok = True
ok &= bool(re.search(r'#define V5F_FW_VER\s+127u', tune))
ok &= ('VER=128' not in tune)
ok &= ('VER=128' not in ekf)
ok &= (ekf.count('        float k_max = V5F_MAG_EPOCH_DT_S / V5F_MAG_YAW_TAU_MIN_S;') == 1)
# 门控段（VER=127）必须原样在
ok &= bool(re.search(r'#define V5F_MAG_DIST_THR_DPS\s+20\.0f', tune))
ok &= bool(re.search(r'#define V5F_MAG_DIST_KS\s+0\.0f', tune))
ok &= bool(re.search(r'#define V5F_MAG_DIST_WMAX_DPS\s+5\.0f', tune))
ok &= bool(re.search(r'#define V5F_MAG_DIST_HOLD_N\s+4011u', tune))
ok &= bool(re.search(r'#define V5F_MAG_ERR_LIM\s+0\.10f', tune))
ok &= (ekf.count('if (!clipped && wm < V5F_MAG_DIST_WMAX_DPS && rate > thr)') == 1)
ok &= (ekf.count('VER=127: KS=0') == 1)
e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
ok &= (e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'))
print('VER   :', re.search(r'#define V5F_FW_VER\s+\d+u', tune).group())
print('K_max :', re.search(r'float k_max = V5F_MAG_EPOCH_DT_S[^\n]*', ekf).group())
print('ROLLBACK', 'OK' if ok else 'FAIL')
sys.exit(0 if ok else 1)
