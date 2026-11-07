# -*- coding: utf-8 -*-
"""
============================================================
 vision.py  ·  相机几何 + 找线
------------------------------------------------------------
 对外只有两个函数：
     find_lane(img)   扫完整个画面，返回一堆数字
     print_table()    启动时把算出来的表打出来看看

 上电时（import 这一下）会按 config.py 里的
 高度 / 低头角度 / 视场角 把扫描行和像素间距算出来。
============================================================
"""

import math
import config as C

IMG_W, IMG_H = 160, 120
CENTER_X = IMG_W // 2


# ============================================================
#  一、由"装多高、低多少头、视场多大"推出扫描行
# ============================================================

F0 = (IMG_W / 2.0) / math.tan(math.radians(C.HFOV_DEG) / 2.0)   # 焦距（像素）
_S = math.sin(math.radians(C.CAM_PITCH_DEG))
_C = math.cos(math.radians(C.CAM_PITCH_DEG))
_H = C.CAM_H_CM / 100.0


def row_slant(v):
    """第 v 行对应的斜距（米）。地平线以上返回 0。"""
    dy = (v - IMG_H / 2.0) / F0
    wz = -_C * dy - _S
    if wz > -1e-6:
        return 0.0
    return -_H / wz


def row_dist(v):
    """第 v 行正中央对应的车前水平距离（米）。"""
    t = row_slant(v)
    if t <= 1e-6:
        return 0.0
    dy = (v - IMG_H / 2.0) / F0
    return (-_S * dy + _C) * t


def row_gap_of(v):
    """第 v 行：外侧黑线中心 -> 车道中心，差多少像素。"""
    t = row_slant(v)
    if t <= 1e-6:
        return 0
    return int(F0 * 0.25 / t + 0.5)


def pick_rows():
    """在"两根线都塞得进画面"的行里，平均挑 N_SCAN 行。"""
    ok = []
    for v in range(IMG_H):
        g = row_gap_of(v)
        if C.GAP_MIN <= g <= C.GAP_MAX:
            ok.append(v)
    if len(ok) < C.N_SCAN:
        ok = list(range(12, 60))            # 兜底，说明参数填得离谱
    out = []
    n = len(ok) - 1
    for i in range(C.N_SCAN):
        v = ok[int(i * n / (C.N_SCAN - 1.0) + 0.5)] + C.ROW_TWEAK
        if v < 0:
            v = 0
        elif v > IMG_H - 1:
            v = IMG_H - 1
        out.append(v)
    out.reverse()                           # 改成 近 -> 远
    return out


SCAN_ROWS = pick_rows()                     # 5 个行号，近的在前
ROW_GAP = [row_gap_of(v) for v in SCAN_ROWS]   # 对应的像素间距，运行时会被修正
SCAN_WEIGHT = (3, 3, 2, 2, 1)


def horizon_row():
    """地平线在第几行。这行以上是天空，不用扫。"""
    for v in range(IMG_H):
        if row_slant(v) > 0:
            return v
    return IMG_H - 1


def print_table():
    hz = horizon_row()
    print("焦距 F=%.1f px   地平线在第 %d 行   最近能看 %.2f m"
          % (F0, hz, row_dist(IMG_H - 1)))
    for i in range(C.N_SCAN):
        print("  行 %3d  车前 %4.2f m  间距 %3d px  权重 %d"
              % (SCAN_ROWS[i], row_dist(SCAN_ROWS[i]), ROW_GAP[i], SCAN_WEIGHT[i]))


# ============================================================
#  二、扫一行
# ============================================================

def scan_row(img, y, gi):
    """扫描第 y 行。返回 (车道中心 x, 黑点数, 单线标记)。

    单线标记：0 = 两根都在   1 = 只剩外侧那根   2 = 只剩内圈那根
              -1 = 一根都没有
    车道中心找不到时 x = -1。
    """
    # ---- 1. 读一整行 ----
    gp = img.get_pixel
    vals = []
    add = vals.append
    x = C.X_MIN
    while x <= C.X_MAX:
        add(gp(x, y))
        x += 1

    # ---- 2. 这一行自己的黑白分界线（所以不怕灯光变）----
    lo = min(vals)
    hi = max(vals)
    if hi - lo < C.CONTRAST_MIN:
        return -1, 0, -1
    thr = lo + int((hi - lo) * C.BLACK_RATIO)

    # ---- 3. 找黑段 ----
    n = len(vals)
    first = -1
    last = -1
    cnt = 0
    dark = 0
    i = 0
    while i < n:
        if vals[i] < thr:
            j = i
            while j < n and vals[j] < thr:
                j += 1
            dark += j - i
            if C.RUN_MIN <= (j - i) <= C.RUN_MAX:
                c = (i + j - 1) // 2 + C.X_MIN
                if cnt == 0:
                    first = c
                last = c
                cnt += 1
            i = j
        else:
            i += 1

    if cnt == 0:
        return -1, dark, -1

    gap = ROW_GAP[gi]

    # ---- 两根都在，间距对得上：最可靠 ----
    if cnt >= 2:
        half = (last - first) // 2
        d = half - gap
        if d < 0:
            d = -d
        if d <= C.GAP_TOL:
            ROW_GAP[gi] = (gap * 15 + half) >> 4   # 自适应修正间距表
            return (first + last) // 2, dark, 0

    # ---- 只剩一根：判断它是外侧还是内圈 ----
    if first < CENTER_X - C.SIDE_DEAD:
        return first + gap, dark, 1
    if last > CENTER_X + C.SIDE_DEAD:
        return last - gap, dark, 2
    return -1, dark, -1


# ============================================================
#  三、扫全部行，合成一个结果
# ============================================================

def find_lane(img):
    """返回 (误差, 远处误差, 有效行数, 横线条数, 远处有效行数, 右弯信号)。"""
    n = len(SCAN_ROWS)
    sum_w = 0
    sum_e = 0
    far_w = 0
    far_e = 0
    hits = 0
    far_hits = 0
    bars = 0
    corner = 0
    span = C.X_MAX - C.X_MIN + 1

    for i in range(n):
        y = SCAN_ROWS[i]
        mid, dark, only = scan_row(img, y, i)

        if dark * 10 >= span * 8:               # 这一行 80% 都是黑的 = 横线
            bars += 1

        # 最近两行只剩外侧那根 = 内圈线拐走了 = 前面是右弯
        if only == 1 and i <= 1:
            if 0 <= mid - ROW_GAP[i] < C.CORNER_X:
                corner = 1

        if mid < 0:
            continue

        e = mid - CENTER_X
        w = SCAN_WEIGHT[i]
        sum_w += w
        sum_e += w * e
        hits += 1

        if i >= n - 2:                          # 最后两行 = 最远的两行
            far_w += w
            far_e += w * e
            far_hits += 1

        if C.DEBUG_DRAW:
            img.draw_circle(mid, y, 2, color=0, fill=True)

    if sum_w == 0:
        return 0, 0, 0, bars, 0, corner

    err = sum_e // sum_w
    far = far_e // far_w if far_w > 0 else err
    return err, far, hits, bars, far_hits, corner
