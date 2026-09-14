import io, os, re
root = r'C:\Users\33\Documents\v2\h415-imu42605-'
pat = re.compile(r'shared_mem_t|SHM_SIZE|sizeof\(\s*gps_rmc|static_assert|_Static_assert|0x[0-9A-Fa-f]{4}\s*\+\s*g_shm|gps_rmc')
for dp, dn, fn in os.walk(root):
    if any(x in dp for x in ('.git', 'LVGL', 'obj')):
        continue
    for f in fn:
        if not f.endswith(('.c', '.h')):
            continue
        p = os.path.join(dp, f)
        try:
            s = io.open(p, encoding='utf-8').read()
        except Exception:
            try: s = io.open(p, encoding='gbk', errors='replace').read()
            except Exception: continue
        for i, ln in enumerate(s.split('\n')):
            if pat.search(ln) and 'gps_rmc_chan_t' not in ln and 'g_v5f_hold.gps_rmc' not in ln and 'g_shm->gps_rmc.' not in ln:
                print('%-44s %4d %s' % (os.path.relpath(p, root), i+1, ln.strip()[:110]))
