# -*- coding: utf-8 -*-
"""vision.py —— 相机几何 + 找线。

每帧从 160x120 灰度图里算出"车道中心在第几列"，
再和画面中心(第 80 列)比较，差值就是偏差 err（像素）。
对外只有 find_lane() 和 print_table()。
"""

import math
import config as C

IMG_W, IMG_H = 160, 120          # 画面尺寸，和 main.py 的 QQVGA 一致
CENTER_X = IMG_W // 2            # 画面中心列 = 80


# ============================================================
#  一、相机几何：由安装参数推出该扫哪几行
# ============================================================

# 针孔相机焦距（像素）。半宽 80px 对应半视场角，F0 = 半宽 / tan(半角)
F0 = (IMG_W / 2.0) / math.tan(math.radians(C.HFOV_DEG) / 2.0)

# 低头角的正余弦、镜头高度（米），后面每行都要用，先算好
_S = math.sin(math.radians(C.CAM_PITCH_DEG))
_C = math.cos(math.radians(C.CAM_PITCH_DEG))
_H = C.CAM_H_CM / 100.0


def row_slant(v):
    """第 v 行的斜距（米）。地平线以上返回 0。"""
    dy = (v - IMG_H / 2.0) / F0
    wz = -_C * dy - _S           # 这条光线朝下的分量
    if wz > -1e-6:
        return 0.0               # 朝上或水平 = 打不到地面
    return -_H / wz              # 相似三角形：高度 / 朝下比例 = 光线长度


def row_dist(v):
    """第 v 行正中央对应的车前水平距离（米）。"""
    t = row_slant(v)
    if t <= 1e-6:
        return 0.0
    dy = (v - IMG_H / 2.0) / F0
    return (-_S * dy + _C) * t   # 把斜距投影到地面上


def row_gap_of(v):
    """第 v 行：外侧黑线中心到车道中心差多少像素（半车道宽 0.25m）。"""
    t = row_slant(v)
    if t <= 1e-6:
        return 0
    return int(F0 * 0.25 / t + 0.5)


def pick_rows():
    """在"两条线都塞得进画面"的行里，等间隔挑 N_SCAN 行，由近到远返回。"""
    ok = []
    for v in range(IMG_H):
        g = row_gap_of(v)
        if C.GAP_MIN <= g <= C.GAP_MAX:      # 太远分不清、太近撑出画面
            ok.append(v)
    if len(ok) < C.N_SCAN:
        ok = list(range(12, 60))             # 兜底：参数填得离谱时

    out = []
    n = len(ok) - 1
    for i in range(C.N_SCAN):
        v = ok[int(i * n / (C.N_SCAN - 1.0) + 0.5)] + C.ROW_TWEAK
        if v < 0:
            v = 0
        elif v > IMG_H - 1:
            v = IMG_H - 1
        out.append(v)
    out.reverse()                            # 改成由近到远
    return out


# 上电时算一次，之后全程序共用
SCAN_ROWS = pick_rows()                      # 5 个行号，近的在前
ROW_GAP = [row_gap_of(v) for v in SCAN_ROWS] # 对应的半间距，运行时会自适应修正

# 近处看得准给 3 分，远处容易抖给 1 分
SCAN_WEIGHT = (3, 3, 2, 2, 1)


def horizon_row():
    """地平线在第几行（这行以上是天空，不用扫）。"""
    for v in range(IMG_H):
        if row_slant(v) > 0:
            return v
    return IMG_H - 1


def print_table():
    """启动时打印扫描行表，用来核对高度/角度填得对不对。"""
    hz = horizon_row()
    print("焦距 F=%.1f px   地平线在第 %d 行   最近能看 %.2f m"
          % (F0, hz, row_dist(IMG_H - 1)))
    for i in range(C.N_SCAN):
        print("  行 %3d  车前 %4.2f m  间距 %3d px  权重 %d"
              % (SCAN_ROWS[i], row_dist(SCAN_ROWS[i]), ROW_GAP[i], SCAN_WEIGHT[i]))


# ============================================================
#  二、扫一行，找出车道中心
# ============================================================

def scan_row(img, y, gi):
    """扫第 y 行，返回 (车道中心列, 黑点数, 单线标记)。

    单线标记：0=两根都在  1=只剩外侧(左)  2=只剩内圈(右)  -1=没有
    找不到中心时第一项为 -1。
    """
    # 1. 读整行（跳过两端暗角）
    gp = img.get_pixel
    vals = []
    add = vals.append
    x = C.X_MIN
    while x <= C.X_MAX:
        add(gp(x, y))
        x += 1

    # 2. 这一行自己的黑白分界线（自适应阈值，不怕灯光不均）
    lo = min(vals)
    hi = max(vals)
    if hi - lo < C.CONTRAST_MIN:             # 对比度太低 = 这行没线
        return -1, 0, -1
    thr = lo + int((hi - lo) * C.BLACK_RATIO)

    # 3. 找连续黑段，长度合格才算"一条线"
    n = len(vals)
    first = -1                  # 第一个合格黑段的中心列
    last = -1                   # 最后一个合格黑段的中心列
    cnt = 0                     # 合格黑段个数
    dark = 0                    # 本行黑色像素总数（判断横线用）
    i = 0
    while i < n:
        if vals[i] < thr:
            j = i
            while j < n and vals[j] < thr:
                j += 1
            dark += j - i

            if C.RUN_MIN <= (j - i) <= C.RUN_MAX:    # 太短是噪点，太宽是糊片
                c = (i + j - 1) // 2 + C.X_MIN       # 黑段中心，换回真实列号
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

    # ---- 两根都在：间距对得上才敢用中点 ----
    if cnt >= 2:
        half = (last - first) // 2
        d = half - gap
        if d < 0:
            d = -d
        if d <= C.GAP_TOL:
            # 实测和理论差一点就挪 1/16，参数填得略偏也能自己收敛
            ROW_GAP[gi] = (gap * 15 + half) >> 4
            return (first + last) // 2, dark, 0

    # ---- 只剩一根：靠"偏左还是偏右"判断是外侧还是内圈 ----
    if first < CENTER_X - C.SIDE_DEAD:
        return first + gap, dark, 1          # 左=外侧线，中心在它右边
    if last > CENTER_X + C.SIDE_DEAD:
        return last - gap, dark, 2           # 右=内圈线，中心在它左边
    # 卡在正中间分不清内外，猜错方向会反，宁可不给
    return -1, dark, -1


# ============================================================
#  三、扫全部行，加权合成一个结果
# ============================================================

def find_lane(img):
    """返回 (误差, 远处误差, 有效行数, 横线条数, 远处有效行数, 右弯信号)。"""
    n = len(SCAN_ROWS)
    sum_w = 0        # 权重总和
    sum_e = 0        # 偏差×权重 累加
    far_w = 0        # 远端权重和
    far_e = 0        # 远端偏差×权重 累加
    hits = 0         # 找到中心的行数
    far_hits = 0     # 远端找到的行数
    bars = 0         # 疑似横线的行数
    corner = 0       # 右弯信号
    span = C.X_MAX - C.X_MIN + 1

    for i in range(n):
        y = SCAN_ROWS[i]
        mid, dark, only = scan_row(img, y, i)

        # 整行八成是黑的 = 横线，不是赛道线
        if dark * 10 >= span * 8:
            bars += 1

        # 最近两行只剩外侧线，且推出的中心很靠左 => 内圈线已拐走 => 右弯
        if only == 1 and i <= 1:
            if 0 <= mid - ROW_GAP[i] < C.CORNER_X:
                corner = 1

        if mid < 0:
            continue                    # 这行没找到，跳过

        e = mid - CENTER_X              # 正数=车道在右=车偏左=要往右打
        w = SCAN_WEIGHT[i]
        sum_w += w
        sum_e += w * e
        hits += 1

        if i >= n - 2:                  # 最远两行
            far_w += w
            far_e += w * e
            far_hits += 1

        if C.DEBUG_DRAW:
            img.draw_circle(mid, y, 2, color=0, fill=True)

    if sum_w == 0:
        return 0, 0, 0, bars, 0, corner  # 全部丢线

    err = sum_e // sum_w                 # 加权平均，整除（后面要发整数）
    far = far_e // far_w if far_w > 0 else err   # 远端没数据就用整体顶替
    return err, far, hits, bars, far_hits, corner
