import io, os, re
root = r'C:\Users\33\Documents\v2\h415-imu42605-'
for dp, dn, fn in os.walk(root):
    if any(x in dp for x in ('.git','LVGL')):
        continue
    for f in fn:
        if not f.endswith(('.c','.h')):
            continue
        p = os.path.join(dp, f)
        try:
            s = io.open(p, encoding='gbk', errors='replace').read()
        except Exception:
            continue
        if 'hid_up_enqueue' in s or 'hid_up' in s:
            for i, ln in enumerate(s.split('\n')):
                if re.search(r'hid_up|UP_RING|up_ring|UP_BUF|up_buf|8192|4096|2048', ln):
                    print('%-46s %4d %s' % (os.path.relpath(p, root), i+1, ln.strip()[:130]))
