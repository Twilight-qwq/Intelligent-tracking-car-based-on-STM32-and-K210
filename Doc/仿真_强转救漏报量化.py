# -*- coding: utf-8 -*-
"""
弯道开转点几何仿真 v4（只读分析，不碰任何源码）

v4 修正（用户 2026-11-23 澄清）：
  - 视觉报弯点：146 = 文档推算(存档§4，模型非实测)；150 = 门禁地板(C−30)，不是报弯时刻。
  - 两者殊途同归——视觉146报弯但里程<150 → 被当假信号，车继续走到150才认 → 开转=150+20=170。
  - 核心规则：开转 = max(报弯, 门禁地板C−30) + TURN_ARM_CM
  - 真正决定成败的是"报弯晚过地板多少"。若实测 I5 > 地板：
        TURN_ARM_CM = 170 − max(I5, 150)   （首弯；存档§12第4条）
    I5≤150 → ARM 保持20（开转170）；I5=165 → ARM 改5（开转仍170）。
  - 之前 v3 的 VISION_OFFSET=15（"165 报→185 转"）是用户举例、非数据，已删除。

几何（工程分析记录.md §1/§2，硬事实）：
  - 内拐点160 / C(弯口)180 / 外拐点200；理想开转 = C − R·tan(θ/2)，R=8.5 → 171.5
  - 出弯点 = 开转 + 2·R·tan(θ/2)；出弯>外拐点(C+20·tan(θ/2)=200) = 出线没分
  - 强转触发 = 读数 > C + FORCE；FORCE 可为负（宏是 float）
  - 评分：出弯≤理想出弯188.5 满分；188.5~200 扣分(贴外线)；>200 没分(出线)

用法：python 仿真_强转救漏报量化.py [--trace] [输出文件]
"""

import math
import random
import sys

TRACE = ('--trace' in sys.argv)
_TRACE_COUNT = 0

# ===== 几何常量（工程分析记录.md §1/§2，硬事实）=====
C_MILE  = [180.0, 325.0, 330.0]   # 中心线拐点（弯口）
ANGLES  = [90.0, 103.0, 90.0]
R_MID   = 8.5                     # 转弯半径取中值（理论7.2/实测折算10.0）
HALF_LANE = 20.0                  # 车道半宽（中心到中心40÷2）
EARLY   = 30.0                    # TURN_DIST_EARLY_CM → 门禁地板 = C − 30
LATE    = 10.0                    # TURN_DIST_LATE_CM
ARM_CM  = 20.0                    # TURN_ARM_CM（默认；实测报弯晚于地板时按公式改小）
COOLDOWN_CM = 70.0

# ===== 视觉报弯点 =====
# 文档推算首弯 146.3（存档§4：内拐点132.3进视野 + 6帧166ms走14cm），模型非实测。
# 用相对C的偏移表示，便于套到三个弯：报弯 ≈ C − VISION_OFFSET。
VISION_OFFSET = 34.0   # 180−34=146（首弯文档推算值）。因<门禁地板30cm，会被兜到150才认。

# ===== 噪声 =====
LIGHT_JITTER = 8.0    # 视觉报弯里程抖动 σ(cm)
ODO_SIG  = 0.01       # 里程读数随机误差比例 σ
CALIB_ERR = 5.0       # 里程读数固定偏差(cm)，用户指定 5cm

STEP_CM = 0.6

GEOM = []
for _c, _a in zip(C_MILE, ANGLES):
    _t = math.tan(math.radians(_a / 2.0))
    _d = R_MID * _t
    GEOM.append(dict(d=_d, ideal_on=_c - _d, ideal_off=_c + _d,
                     outer=_c + HALF_LANE * _t, floor=_c - EARLY))


def grade(turn_on, g):
    off = turn_on + 2.0 * g['d']
    if off <= g['ideal_off']:
        return 'full', off
    if off <= g['outer']:
        return 'penalty', off
    return 'zero', off


def sim_bend(i, missed, force_on, force_cm, vision_offset, arm_cm,
             calib_err, light_jit, odo_sig):
    g, Cx = GEOM[i], C_MILE[i]
    odo_err = random.gauss(0.0, odo_sig)
    report_at = (Cx - vision_offset) + random.gauss(0.0, light_jit)   # 视觉报弯真实里程(I5)

    seg = 0.0
    armed = False
    arm_at = 0.0
    mode = None
    turn_on = None

    while turn_on is None and seg < 900:
        seg += STEP_CM
        read = seg * (1.0 + odo_err) + calib_err

        # 路径A：强转（FORCE 可为负）
        if (force_on and missed and not armed
                and seg >= COOLDOWN_CM and read > (Cx + force_cm)):
            turn_on = seg
            mode = 'force'
            global _TRACE_COUNT
            _TRACE_COUNT += 1
            if TRACE and _TRACE_COUNT <= 8:
                out("[审计] 弯%d C=%.0f FORCE=%+.1f 触发线=%.1f 读数=%.1f 真实开转=%.1f 理想=%.1f 晚=%.1fcm" %
                    (i+1, Cx, force_cm, Cx+force_cm, read, seg, g['ideal_on'], seg-g['ideal_on']))

        # 路径B：视觉报弯 -> 里程门禁[C−30, C+10] -> ARM -> 开转
        # 关键：视觉在146报弯(read<150)会被当假信号；继续走到地板150才认 → 兜住。
        if (not missed) and not armed and seg >= report_at:
            if (Cx - EARLY) <= read <= (Cx + LATE):
                armed = True
                arm_at = seg
        if armed and turn_on is None and seg >= arm_at + arm_cm:
            turn_on = seg
            mode = 'vision'

    if turn_on is None:
        return ('miss', None, None, None, 'miss')
    late = turn_on - g['ideal_on']
    grad, off = grade(turn_on, g)
    return (mode, round(turn_on, 1), round(late, 1), round(off, 1), grad)


def sim_lap(force_on=False, force_cm=-9.0, p_miss=0.03, vision_offset=VISION_OFFSET,
            arm_cm=ARM_CM, calib_err=CALIB_ERR, light_jit=LIGHT_JITTER, odo_sig=ODO_SIG):
    res = dict(force=0, vision=0, miss=0, full=0, penalty=0, zero=0,
               force_lates=[], vision_lates=[])
    for i in range(3):
        missed = random.random() < p_miss
        mode, turn_on, late, off, grad = sim_bend(
            i, missed, force_on, force_cm, vision_offset, arm_cm,
            calib_err, light_jit, odo_sig)
        if mode == 'miss':
            res['miss'] += 1
            continue
        res[mode] += 1
        res[grad] += 1
        (res['force_lates'] if mode == 'force' else res['vision_lates']).append(late)
    return res


def run(n=3000, **kw):
    return [sim_lap(**kw) for _ in range(n)]


def sd(xs):
    if len(xs) < 2:
        return 0.0
    m = sum(xs) / len(xs)
    return (sum((x - m) ** 2 for x in xs) / len(xs)) ** 0.5


_LINES = []
def out(s=""):
    print(s)
    _LINES.append(s)


def report(name, laps):
    n = len(laps)
    full = sum(l['full'] for l in laps)
    pen = sum(l['penalty'] for l in laps)
    zero = sum(l['zero'] for l in laps)
    clean = sum(1 for l in laps if l['full'] == 3)
    fsum = sum(l['force'] for l in laps)
    vl = [x for l in laps for x in l['vision_lates']]
    fl = [x for l in laps for x in l['force_lates']]
    out("=" * 64)
    out(name)
    out("  满分 / 扣分 / 没分 : %5.1f%% / %5.1f%% / %5.1f%%" %
        (100.0*full/(n*3), 100.0*pen/(n*3), 100.0*zero/(n*3)))
    out("  整圈全满分         : %5.1f%%  (%d/%d)" % (100.0*clean/n, clean, n))
    out("  强转触发次数       : %d" % fsum)
    if vl:
        out("  视觉开转相对理想   : 均值%+.1fcm [%+.1f,%+.1f]" % (sum(vl)/len(vl), min(vl), max(vl)))
    if fl:
        out("  强转开转相对理想   : 均值%+.1fcm [%+.1f,%+.1f]" % (sum(fl)/len(fl), min(fl), max(fl)))


if __name__ == '__main__':
    random.seed(20261123)

    out("【几何基准】R=8.5；理想开转=C−R·tan(θ/2)；理想出弯=C+8.5；门禁地板=C−30；外拐点=C+20·tan(θ/2)")
    for i, (c, a) in enumerate(zip(C_MILE, ANGLES)):
        g = GEOM[i]
        out("  弯%d θ=%.0f° C=%.0f 地板=%.0f 理想开转=%.1f 理想出弯=%.1f 外拐点=%.1f" %
            (i+1, a, c, g['floor'], g['ideal_on'], g['ideal_off'], g['outer']))

    out("\n【规则验证】开转 = max(报弯, 地板) + ARM（文档推算报弯146，被地板150兜住 → 170）")
    report("报弯146 + ARM20（现状，3%%漏报）", run(3000, force_on=False, vision_offset=34.0, arm_cm=20.0))

    out("\n【实测 I5 晚于地板的敏感性】首弯地板150")
    report("I5≈165 + ARM20不改（开转185）", run(3000, force_on=False, vision_offset=15.0, arm_cm=20.0))
    report("I5≈165 + ARM改5（开转170）",   run(3000, force_on=False, vision_offset=15.0, arm_cm=5.0))

    out("\n【强转 FORCE 档】全开（视觉全漏报，隔离强转本身）；FORCE 负值才对")
    for fc in (-12.0, -9.0, -5.0, 0.0, 10.0, 30.0):
        report("FORCE=%+.1f" % fc, run(3000, force_on=True, force_cm=fc, p_miss=1.0))

    out("\n【强转 FORCE=-9 + 真实3%%漏报】只有漏报弯走强转")
    report("FORCE=-9 / 3%%漏报", run(3000, force_on=True, force_cm=-9.0, p_miss=0.03))

    out("\n说明：开转 = max(实测I5, 地板C−30) + ARM；实测 I5≤150 → ARM=20 不动；")
    out("      I5>150 → ARM = 170 − max(I5,150)（首弯）。146/150 都被地板兜成170，无需二选一。")
    out("      FORCE 必须负值：正数触发线≥C=180，永远晚于理想171.5。")

    paths = [a for a in sys.argv[1:] if a != '--trace']
    if paths:
        with open(paths[-1], 'w', encoding='utf-8') as f:
            f.write("\n".join(_LINES) + "\n")
        out("\n[报告已写入] " + paths[-1])
