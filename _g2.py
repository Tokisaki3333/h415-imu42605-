import io, os, re
root = r'C:\Users\33\Documents\v2\h415-imu42605-'
for dp, dn, fn in os.walk(os.path.join(root, 'V5F')):
    if '.git' in dp:
        continue
    for f in fn:
        if f.lower().endswith(('.mk', '.mak')) or f == 'makefile':
            p = os.path.join(dp, f)
            s = io.open(p, encoding='gbk', errors='replace').read()
            for i, ln in enumerate(s.split('\n')):
                if re.search(r'CFLAGS|Werror|Wall|Wextra|Ofast|unused', ln, re.I):
                    print('%-40s %4d %s' % (os.path.relpath(p, root), i+1, ln.strip()[:150]))
