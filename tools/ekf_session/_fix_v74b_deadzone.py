# -*- coding: utf-8 -*-
"""
VER=74（按用户对"死区"的正确定义重写）

用户的定义：死区**只**应针对"偏航角所在线（偏航旋转轴）与磁北矢量趋近重合"这一
几何退化情形 —— 磁场几乎与偏航轴平行时，水平投影消失，此时推断出的"磁北方向"
本身没有意义，绝不能拿去融合。它与"残差小不小"无关。

对应到本工程，这个几何量就是 s_mag_bh = |Bn_h| = hypot(Bn[0],Bn[1]) ——
导航系水平面里磁场的水平分量，也正是"投影到重力法向量平面"的那个平面是否可信
的读数。VER=73 录制实测（v0 当校验字滤掉 4.95% 污染帧）：
    * 干净静止/正常几何：|v| = 0.416 ~ 0.44（= cos(64.3°) = 0.4333 的应有值）；
    * 12 < mag_r < 150（原正常融合档）：|v| p50 = 0.416；
    * mag_r > 150（被 R_MAX 拒掉的那一档）：|v| p10/p50/p90 = 0.135/0.181/0.212
      —— 磁场几乎贴上偏航轴，几何退化确实成立。
即：R_MAX 一直在替 BH_MIN 干"几何退化保护"的活，而 BH_MIN=0.12 太低挡不住。

改动 1 —— V5F_EKF_MAG_BH_MIN 0.12 -> 0.28（这就是用户说的死区，放在正确的位置）
  0.28 = 0.65 * |v0|(0.4333)。|v| 一旦落到这个比例以下，说明重力法平面已经不可信
  （|v| 对纯偏航误差不敏感，只随倾角/投影面误差变小），推断出的磁北没有意义。
  正常几何 0.416~0.44 有充足余量通过；退化档 0.135~0.212 被挡住。

改动 2 —— V5F_EKF_MAG_DEAD_DEG 12 -> 0（删除"小误差就不修"的通用死区）
  12 度是按"磁角典型误差可达 10 度"设的，与几何退化无关，且实测有害：
  开机静止 14 s 真实 |psi| 只有 0.43~1.27 度；剧烈运动后回到静止 36~44 s
  |psi| = 10.36 度而 mag_r = 10.84~10.95，恰好卡在 12 度下面 ——
  环路拒绝拉回，这 10.4 度纯粹是那个通用死区留下的残渣。
  置 0 后每次新磁样本都融合（单步仍由 k_cap 限幅 0.05*|r|）。

改动 3 —— V5F_EKF_MAG_R_MAX_DEG 179 -> 150（回退我上一版的错误改动）
  实测它拒掉的那些帧 |v| 只有 0.18，正是几何退化档；放开它等于在磁北最没有
  意义的时候去融合。保留 150 作为第二道退化保护（BH_MIN 修好后它会很少触发）。
"""
import os, shutil

ENC = 'gbk'
TUNE = r'h415-imu42605-\V5F\User\inc\v5f_tune.h'

def load(p):
    with open(p, 'rb') as f:
        return f.read().decode(ENC)

def save(p, text, tag):
    data = text.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f:
        f.write(data)
    with open(p, 'rb') as f:
        assert f.read().decode(ENC) == text
    return len(data)

def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '[%s] 锚点命中 %d 次' % (what, n)
    return t.replace(old, new, 1)

t = load(TUNE)

# 1) BH_MIN：这才是用户定义的死区
t = sub1(t,
    '#define V5F_EKF_MAG_BH_MIN       0.12f  /* 磁偏航增益上限（tau 约 1.4 s 的牵引）*/',
    '/* ★VER=74 这就是用户定义的**死区**：偏航旋转轴与磁北矢量趋近重合（磁场几\n'
    ' * 乎平行于偏航轴）时，水平投影消失、投影面本身不可信，此时算出的"磁北方向"\n'
    ' * 没有意义，绝不能融合。几何量就是 s_mag_bh = |Bn_h|（导航系水平面里的磁场\n'
    ' * 水平分量），也就是"投影到重力法向量平面"那个平面可不可信的读数：\n'
    ' * 纯偏航误差不改变它，只有倾角/投影面误差会让它变小。\n'
    ' * 实测 VER=73：干净几何 0.416~0.44（=cos(64.3)=0.4333 应有值）；\n'
    ' * 退化档（原被 R_MAX 拒掉的帧）|v| = 0.135/0.181/0.212。\n'
    ' * 取 0.28 = 0.65*|v0|：正常几何余量充足，退化档全部挡住。\n'
    ' * 原值 0.12 太低（挡不住 0.13~0.21 那一档，活全让 R_MAX 替它干了）。 */\n'
    '#define V5F_EKF_MAG_BH_MIN       0.28f',
    'BH_MIN')

# 2) 删除通用小误差死区
t = sub1(t, '#define V5F_EKF_MAG_DEAD_DEG      3.0f',
    '/* ★VER=74 置 0：删掉"残差小就不修"的通用死区。它不是几何退化保护，\n'
    ' * 而实测有害 —— 剧烈运动后回到静止时 |psi|=10.36 度、mag_r=10.84~10.95\n'
    ' * 恰好卡在原 12 度门下面，环路拒绝拉回，那 10.4 度就是它留下的残渣。\n'
    ' * 死区职责交给 BH_MIN（几何退化）；单步幅度仍由 k_cap 限幅。 */\n'
    '#define V5F_EKF_MAG_DEAD_DEG      0.0f', 'DEAD')

# 3) 回退 R_MAX
t = sub1(t,
    '#define V5F_EKF_MAG_R_MAX_DEG    179.0f   /* 磁偏航硬新息门：|r| 超它整帧丢弃 */',
    '/* ★VER=74 回退 179 -> 150：实测被它拒掉的帧 |v| 只有 0.135~0.212\n'
    ' * （干净几何是 0.416~0.44），正是几何退化档 —— 放开等于在磁北最没有意义的\n'
    ' * 时候去融合。保留 150 作为第二道退化保护（BH_MIN 修好后它很少触发）。 */\n'
    '#define V5F_EKF_MAG_R_MAX_DEG    150.0f   /* 磁偏航硬新息门：|r| 超它整帧丢弃 */',
    'R_MAX')

n = save(TUNE, t, 'v74b')
t2 = load(TUNE)
assert '#define V5F_FW_VER        74u' in t2
assert '#define V5F_EKF_MAG_BH_MIN       0.28f' in t2
assert '#define V5F_EKF_MAG_DEAD_DEG      0.0f' in t2
assert '#define V5F_EKF_MAG_R_MAX_DEG    150.0f' in t2
print('v5f_tune.h %d 字节' % n)
for ln in t2.split('\n'):
    if ('V5F_FW_VER' in ln or 'MAG_BH_MIN' in ln or 'MAG_DEAD_DEG' in ln
        or 'MAG_R_MAX_DEG' in ln):
        print('   ' + ln.strip())
print()
print('PASS: VER=74（按用户定义重写）已写入')
