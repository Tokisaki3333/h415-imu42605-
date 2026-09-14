import io, os, re
root = r'C:\Users\33\Documents\v2\h415-imu42605-'
pats = ['JF_CH_NUM', 'JF_FRAME_LEN', 'sqrtf', 'math.h', 'V5F_MAG']
for dp, dn, fn in os.walk(root):
    if any(x in dp for x in ('.git', 'build', 'Debug', 'Release')):
        continue
    for f in fn:
        if not f.endswith(('.c', '.h')):
            continue
        p = os.path.join(dp, f)
        try:
            s = io.open(p, encoding='gbk', errors='replace').read()
        except Exception:
            continue
        for pat in pats:
            for i, ln in enumerate(s.split('\n')):
                if pat in ln:
                    rel = os.path.relpath(p, root)
                    print('%-46s %-12s %4d  %s' % (rel, pat, i+1, ln.strip()[:110]))
