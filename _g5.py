import io, os, re
root = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
for dp, dn, fn in os.walk(root):
    for f in sorted(fn):
        if not f.endswith(('.c','.h')):
            continue
        p = os.path.join(dp, f)
        s = io.open(p, encoding='gbk', errors='replace').read()
        hits = []
        for i, ln in enumerate(s.split('\n')):
            if re.search(r'\bmag\b|magnet|磁力计|磁航向|yaw|偏航', ln) and not ln.strip().startswith('*'):
                hits.append((i+1, ln.strip()))
        if hits:
            print('---- %s ----' % os.path.relpath(p, root))
            for i, ln in hits:
                print('  %4d %s' % (i, ln[:120]))
