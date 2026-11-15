# -*- coding: utf-8 -*-
"""vision.py —— 相机几何 + 找线。

每帧从 160x120 灰度图里算出"车道中心在第几列"，
再和画面中心(第 80 列)比较，差值就是偏差 err（像素）。
对外只有 find_lane() 和 print_scan_table()。
"""

import math
import config

IMAGE_WIDTH, IMAGE_HEIGHT = 160, 120     # 画面尺寸，和 main.py 的 QQVGA 一致
CENTER_COLUMN = IMAGE_WIDTH // 2         # 画面中心列 = 80


# ============================================================
#  一、相机几何：由安装参数推出该扫哪几行
# ============================================================

# 针孔相机焦距（像素）。半宽 80px 对应半视场角，F = 半宽 / tan(半角)
FOCAL_LENGTH_PX = (IMAGE_WIDTH / 2.0) / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2.0)

# 低头角的正余弦、镜头高度（米），后面每行都要用，先算好
PITCH_SIN = math.sin(math.radians(config.CAMERA_PITCH_DEG))
PITCH_COS = math.cos(math.radians(config.CAMERA_PITCH_DEG))
CAMERA_HEIGHT_M = config.CAMERA_HEIGHT_CM / 100.0


def slant_distance_of_row(row):
    """第 row 行的斜距（米）。地平线以上返回 0。"""
    row_offset = (row - IMAGE_HEIGHT / 2.0) / FOCAL_LENGTH_PX
    ray_downward = -PITCH_COS * row_offset - PITCH_SIN   # 这条光线朝下的分量
    if ray_downward > -1e-6:
        return 0.0                                       # 朝上或水平 = 打不到地面
    return -CAMERA_HEIGHT_M / ray_downward               # 相似三角形：高度 / 朝下比例 = 光线长度


def ground_distance_of_row(row):
    """第 row 行正中央对应的车前水平距离（米）。"""
    slant = slant_distance_of_row(row)
    if slant <= 1e-6:
        return 0.0
    row_offset = (row - IMAGE_HEIGHT / 2.0) / FOCAL_LENGTH_PX
    return (-PITCH_SIN * row_offset + PITCH_COS) * slant  # 把斜距投影到地面上


def half_lane_gap_of_row(row):
    """第 row 行：外侧黑线中心到车道中心差多少像素（半车道宽 0.25m）。"""
    slant = slant_distance_of_row(row)
    if slant <= 1e-6:
        return 0
    return int(FOCAL_LENGTH_PX * 0.25 / slant + 0.5)


def choose_scan_rows():
    """在"两条线都塞得进画面"的行里，等间隔挑 SCAN_ROW_COUNT 行，由近到远返回。"""
    usable_rows = []
    for row in range(IMAGE_HEIGHT):
        half_gap = half_lane_gap_of_row(row)
        if config.LINE_GAP_MIN_PX <= half_gap <= config.LINE_GAP_MAX_PX:   # 太远分不清、太近撑出画面
            usable_rows.append(row)
    if len(usable_rows) < config.SCAN_ROW_COUNT:
        usable_rows = list(range(12, 60))                # 兜底：参数填得离谱时

    scan_rows = []
    last_index = len(usable_rows) - 1
    for step in range(config.SCAN_ROW_COUNT):
        row = usable_rows[int(step * last_index / (config.SCAN_ROW_COUNT - 1.0) + 0.5)] + config.SCAN_ROW_OFFSET
        if row < 0:
            row = 0
        elif row > IMAGE_HEIGHT - 1:
            row = IMAGE_HEIGHT - 1
        scan_rows.append(row)
    scan_rows.reverse()                                  # 改成由近到远
    return scan_rows


# 上电时算一次，之后全程序共用
SCAN_ROWS = choose_scan_rows()                           # 5 个行号，近的在前
ROW_HALF_GAP = [half_lane_gap_of_row(row) for row in SCAN_ROWS]  # 对应的半间距，运行时会自适应修正

# 近处看得准给 3 分，远处容易抖给 1 分
SCAN_ROW_WEIGHT = (3, 3, 2, 2, 1)


def find_horizon_row():
    """地平线在第几行（这行以上是天空，不用扫）。"""
    for row in range(IMAGE_HEIGHT):
        if slant_distance_of_row(row) > 0:
            return row
    return IMAGE_HEIGHT - 1


def print_scan_table():
    """启动时打印扫描行表，用来核对高度/角度填得对不对。"""
    horizon = find_horizon_row()
    print("焦距 F=%.1f px   地平线在第 %d 行   最近能看 %.2f m"
          % (FOCAL_LENGTH_PX, horizon, ground_distance_of_row(IMAGE_HEIGHT - 1)))
    for index in range(config.SCAN_ROW_COUNT):
        print("  行 %3d  车前 %4.2f m  间距 %3d px  权重 %d"
              % (SCAN_ROWS[index], ground_distance_of_row(SCAN_ROWS[index]),
                 ROW_HALF_GAP[index], SCAN_ROW_WEIGHT[index]))


# ============================================================
#  二、扫一行，找出车道中心
# ============================================================

def scan_one_row(image, row, row_index):
    """扫第 row 行，返回 (车道中心列, 黑点数, 单线标记)。

    单线标记：0=两根都在  1=只剩外侧(左)  2=只剩内圈(右)  -1=没有
    找不到中心时第一项为 -1。
    """
    # 1. 读整行（跳过两端暗角）
    get_pixel = image.get_pixel
    pixel_values = []
    append_value = pixel_values.append
    column = config.SCAN_COLUMN_FIRST
    while column <= config.SCAN_COLUMN_LAST:
        append_value(get_pixel(column, row))
        column += 1

    # 2. 这一行自己的黑白分界线（自适应阈值，不怕灯光不均）
    darkest = min(pixel_values)
    brightest = max(pixel_values)
    if brightest - darkest < config.CONTRAST_MIN_LEVEL:       # 对比度太低 = 这行没线
        return -1, 0, -1
    threshold = darkest + int((brightest - darkest) * config.BLACK_THRESHOLD_RATIO)

    # 3. 找连续黑段，长度合格才算"一条线"
    pixel_count = len(pixel_values)
    first_line_column = -1          # 第一个合格黑段的中心列
    last_line_column = -1           # 最后一个合格黑段的中心列
    line_count = 0                  # 合格黑段个数
    dark_pixel_count = 0            # 本行黑色像素总数（判断横线用）
    index = 0
    while index < pixel_count:
        if pixel_values[index] < threshold:
            end_index = index
            while end_index < pixel_count and pixel_values[end_index] < threshold:
                end_index += 1
            dark_pixel_count += end_index - index

            if config.LINE_RUN_MIN_PX <= (end_index - index) <= config.LINE_RUN_MAX_PX:
                center_column = (index + end_index - 1) // 2 + config.SCAN_COLUMN_FIRST
                if line_count == 0:
                    first_line_column = center_column
                last_line_column = center_column
                line_count += 1

            index = end_index
        else:
            index += 1

    if line_count == 0:
        return -1, dark_pixel_count, -1

    half_gap = ROW_HALF_GAP[row_index]

    # ---- 两根都在：间距对得上才敢用中点 ----
    if line_count >= 2:
        measured_half_gap = (last_line_column - first_line_column) // 2
        gap_error = measured_half_gap - half_gap
        if gap_error < 0:
            gap_error = -gap_error
        if gap_error <= config.LINE_GAP_TOLERANCE_PX:
            # 实测和理论差一点就挪 1/16，参数填得略偏也能自己收敛
            ROW_HALF_GAP[row_index] = (half_gap * 15 + measured_half_gap) >> 4
            return (first_line_column + last_line_column) // 2, dark_pixel_count, 0

    # ---- 只剩一根：靠"偏左还是偏右"判断是外侧还是内圈 ----
    if first_line_column < CENTER_COLUMN - config.SIDE_DEAD_ZONE_PX:
        return first_line_column + half_gap, dark_pixel_count, 1    # 左=外侧线，中心在它右边
    if last_line_column > CENTER_COLUMN + config.SIDE_DEAD_ZONE_PX:
        return last_line_column - half_gap, dark_pixel_count, 2     # 右=内圈线，中心在它左边
    # 卡在正中间分不清内外，猜错方向会反，宁可不给
    return -1, dark_pixel_count, -1


# ============================================================
#  三、扫全部行，加权合成一个结果
# ============================================================

def find_lane(image):
    """返回 (误差, 远处误差, 有效行数, 横线条数, 远处有效行数, 右弯信号)。"""
    row_count = len(SCAN_ROWS)
    weight_sum = 0                  # 权重总和
    weighted_error_sum = 0          # 偏差×权重 累加
    far_weight_sum = 0              # 远端权重和
    far_weighted_error_sum = 0      # 远端偏差×权重 累加
    found_row_count = 0             # 找到中心的行数
    far_found_row_count = 0         # 远端找到的行数
    bar_row_count = 0               # 疑似横线的行数
    corner_signal = 0               # 右弯信号
    scan_span = config.SCAN_COLUMN_LAST - config.SCAN_COLUMN_FIRST + 1

    for row_index in range(row_count):
        row = SCAN_ROWS[row_index]
        lane_center, dark_count, single_line = scan_one_row(image, row, row_index)

        # 整行八成是黑的 = 横线，不是赛道线
        if dark_count * 10 >= scan_span * 8:
            bar_row_count += 1

        # 最近两行只剩外侧线，且推出的中心很靠左 => 内圈线已拐走 => 右弯
        if single_line == 1 and row_index <= 1:
            if 0 <= lane_center - ROW_HALF_GAP[row_index] < config.CORNER_EDGE_COLUMN:
                corner_signal = 1

        if lane_center < 0:
            continue                    # 这行没找到，跳过

        column_error = lane_center - CENTER_COLUMN    # 正数=车道在右=车偏左=要往右打
        weight = SCAN_ROW_WEIGHT[row_index]
        weight_sum += weight
        weighted_error_sum += weight * column_error
        found_row_count += 1

        if row_index >= row_count - 2:                  # 最远两行
            far_weight_sum += weight
            far_weighted_error_sum += weight * column_error
            far_found_row_count += 1

        if config.DEBUG_DRAW:
            image.draw_circle(lane_center, row, 2, color=0, fill=True)

    if weight_sum == 0:
        return 0, 0, 0, bar_row_count, 0, corner_signal   # 全部丢线

    err = weighted_error_sum // weight_sum          # 加权平均，整除（后面要发整数）
    far_err = far_weighted_error_sum // far_weight_sum if far_weight_sum > 0 else err  # 远端没数据就用整体顶替
    return err, far_err, found_row_count, bar_row_count, far_found_row_count, corner_signal
