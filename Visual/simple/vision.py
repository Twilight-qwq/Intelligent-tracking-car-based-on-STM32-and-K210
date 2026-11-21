# -*- coding: utf-8 -*-
"""vision.py —— 相机几何 + 找线。

每帧从 160x120 灰度图里算出"车道中心在第几列"，
再和画面中心(第 80 列)比较，差值就是偏差 error（像素）。
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


def axial_distance_of_row(row):
    """第 row 行：沿光轴量的轴向距离（米），从镜片到这条光线打到地面的那个点。

    地平线以上（光线不朝下）返回 0，说明这条线打不到地面。
    """
    tan_angle_from_axis = (row - IMAGE_HEIGHT / 2.0) / FOCAL_LENGTH_PX
    vertical_component = -PITCH_COS * tan_angle_from_axis - PITCH_SIN   # 这条光线朝下的分量
    if vertical_component > -1e-6:
        return 0.0                                       # 朝上或水平 = 打不到地面
    return -CAMERA_HEIGHT_M / vertical_component               # 相似三角形：镜头高度 ÷ 竖直分量 = 轴向距离


def horizontal_distance_of_row(row):
    """第 row 行正中央对应的车前水平距离（米）。"""
    axial_distance = axial_distance_of_row(row)
    if axial_distance <= 1e-6:
        return 0.0
    tan_angle_from_axis = (row - IMAGE_HEIGHT / 2.0) / FOCAL_LENGTH_PX
    horizontal_component = -PITCH_SIN * tan_angle_from_axis + PITCH_COS   # 这条光线朝前的分量
    return horizontal_component * axial_distance        # 轴向距离 × 朝前分量 = 地面上的水平距离


def half_lane_width_of_row(row):
    """第 row 行：外侧黑线中心到车道中心差多少像素。

    也就是半车道宽 config.LANE_HALF_WIDTH_M 在这一行上占几个像素。
    两条线都看得见时用它来配对；只剩一条线时用它把中心推出来。
    """
    axial_distance = axial_distance_of_row(row)
    if axial_distance <= 1e-6:
        return 0
    return int(FOCAL_LENGTH_PX * config.LANE_HALF_WIDTH_M / axial_distance + 0.5)


def choose_scan_rows():
    """在"两条线都塞得进画面"的行里，等间隔挑 SCAN_ROW_COUNT 行，由近到远返回。"""
    usable_rows = []
    for row in range(IMAGE_HEIGHT):
        half_lane_width_px = half_lane_width_of_row(row)
        if config.HALF_LANE_WIDTH_MIN_PX <= half_lane_width_px <= config.HALF_LANE_WIDTH_MAX_PX:   # 太远分不清、太近撑出画面
            usable_rows.append(row)
    if len(usable_rows) < config.SCAN_ROW_COUNT:
        usable_rows = list(range(12, 60))                # 兜底：参数填得离谱时

    scan_rows = []
    last_usable_index = len(usable_rows) - 1
    for step in range(config.SCAN_ROW_COUNT):
        row = usable_rows[int(step * last_usable_index / (config.SCAN_ROW_COUNT - 1.0) + 0.5)] + config.SCAN_ROW_SHIFT
        if row < 0:
            row = 0
        elif row > IMAGE_HEIGHT - 1:
            row = IMAGE_HEIGHT - 1
        scan_rows.append(row)
    scan_rows.reverse()                                  # 改成由近到远
    return scan_rows


# 上电时算一次，之后全程序共用
SCAN_ROWS = choose_scan_rows()                           # 5 个行号，近的在前
ROW_HALF_LANE_WIDTH_PX = [half_lane_width_of_row(row) for row in SCAN_ROWS]  # 对应的半车道宽（像素），运行中会被微调

# 近处看得准给 3 分，远处容易抖给 1 分
SCAN_ROW_WEIGHT = (3, 3, 2, 2, 1)


def find_horizon_row():
    """地平线在第几行（这行以上是天空，不用扫）。"""
    for row in range(IMAGE_HEIGHT):
        if axial_distance_of_row(row) > 0:
            return row
    return IMAGE_HEIGHT - 1


def print_scan_table():
    """启动时打印扫描行表，用来核对高度/角度填得对不对。"""
    horizon = find_horizon_row()
    print("焦距 F=%.1f px   地平线在第 %d 行   最近能看 %.2f m"
          % (FOCAL_LENGTH_PX, horizon, horizontal_distance_of_row(IMAGE_HEIGHT - 1)))
    for row_index in range(config.SCAN_ROW_COUNT):
        print("  行 %3d  车前 %4.2f m  半车道宽 %3d px  权重 %d"
              % (SCAN_ROWS[row_index], horizontal_distance_of_row(SCAN_ROWS[row_index]),
                 ROW_HALF_LANE_WIDTH_PX[row_index], SCAN_ROW_WEIGHT[row_index]))


# ============================================================
#  二、扫一行，找出车道中心
# ============================================================

def scan_one_row(image, row, row_index):
    """扫第 row 行，返回 (车道均值, 黑点数, 单线标记)。

    单线标记：0=两根都在  1=只剩外侧(左)  2=只剩内圈(右)  -1=没有
    找不到中心时第一项为 -1。
    """
    # 1. 读整行（跳过两端暗角）
    get_pixel = image.get_pixel
    pixel_values = []
    append_pixel = pixel_values.append
    column_index = config.SCAN_COLUMN_FIRST
    while column_index <= config.SCAN_COLUMN_LAST:
        append_pixel(get_pixel(column_index, row))
        column_index += 1

    # 2. 这一行自己的黑白分界线（自适应阈值，不怕灯光不均）
    darkest_level = min(pixel_values)   #最暗的像素的灰度值
    brightest_level = max(pixel_values) #最亮的像素的灰度值
    if brightest_level - darkest_level < config.CONTRAST_MIN_LEVEL:       # 对比度太低 = 这行没线
        return -1, 0, -1    #差值比70小不作数
    black_threshold = darkest_level + int((brightest_level - darkest_level) * config.BLACK_THRESHOLD_RATIO)
    #阈值 = 最暗灰度值 + 0.4 * (最亮灰度值 - 最暗灰度值)
    #取0.4不取0.5是为了防止把灰色过渡带也判黑


    # 3. 找连续黑段，长度合格才算"一条线"
    row_pixel_count = len(pixel_values)
    left_line_column = -1           # 第一个合格黑段的中心列
    right_line_column = -1          # 最后一个合格黑段的中心列
    line_count = 0                  # 合格黑段个数
    dark_pixel_count = 0            # 本行黑色像素总数（判断横线用）
    pixel_index = 0
    while pixel_index < row_pixel_count:
        if pixel_values[pixel_index] < black_threshold:#判断是否为黑
            run_end_index = pixel_index #从当前黑色像素开始往后推到最后一个连着的黑像素

            #会不会导致黑线部分有一点反光就测不到的情况???

            while run_end_index < row_pixel_count and pixel_values[run_end_index] < black_threshold:
                run_end_index += 1
            dark_pixel_count += run_end_index - pixel_index

            if config.BLACK_RUN_MIN_PX <= (run_end_index - pixel_index) <= config.BLACK_RUN_MAX_PX:
                black_run_center_column = (pixel_index + run_end_index - 1) // 2 + config.SCAN_COLUMN_FIRST
                #黑线的中心的像素列的列号
                if line_count == 0:#目前没检测到其他的黑段
                    left_line_column = black_run_center_column
                #左边的黑线中心只if了一次
                right_line_column = black_run_center_column
                line_count += 1
                #右边的黑线每次循环都会更新,最后留下来的就是右侧黑线中心

            pixel_index = run_end_index #中间已经是黑色,不需要再来回判断
        else:#测到白像素
            pixel_index += 1

    if line_count == 0:#左右线没有一条测到
        return -1, dark_pixel_count, -1 #返回(车道均值, 黑点数, 单线标记)

    half_lane_width_px = ROW_HALF_LANE_WIDTH_PX[row_index]

    # ---- 两根都在：间距对得上才敢用中点 ----
    if line_count >= 2:
        measured_half_lane_width_px = (right_line_column - left_line_column) // 2
        half_width_error = measured_half_lane_width_px - half_lane_width_px
        if half_width_error < 0:
            half_width_error = -half_width_error
        if half_width_error <= config.HALF_WIDTH_TOLERANCE_PX:
            # 实测和理论差一点就挪 1/16，参数填得略偏也能自己收敛
            ROW_HALF_LANE_WIDTH_PX[row_index] = (half_lane_width_px * 15 + measured_half_lane_width_px) >> 4
            return (left_line_column + right_line_column) // 2, dark_pixel_count, 0
            # 返回 (左右均值, 黑点数, 单线标记)

    # ---- 只剩一根：靠"偏左还是偏右"判断是外侧还是内圈 ----
    if left_line_column < CENTER_COLUMN - config.SINGLE_LINE_DEAD_ZONE_PX:
        return left_line_column + half_lane_width_px, dark_pixel_count, 1    # 左=外侧线，中心在它右边
    if right_line_column > CENTER_COLUMN + config.SINGLE_LINE_DEAD_ZONE_PX:
        return right_line_column - half_lane_width_px, dark_pixel_count, 2     # 右=内圈线，中心在它左边
    # 卡在正中间分不清内外，猜错方向会反，宁可不给
    return -1, dark_pixel_count, -1
    # 返回 (左右均值, 黑点数, 单线标记)


# ============================================================
#  三、扫全部行，加权合成一个结果
# ============================================================

def find_lane(image):
    """返回 (误差, 远处误差, 有效行数, 横线条数, 远处有效行数, 右弯信号)。"""
    scan_row_count = len(SCAN_ROWS)
    weight_sum = 0                  # 权重总和
    weighted_error_sum = 0          # 偏差×权重 累加
    far_weight_sum = 0              # 远端权重和
    far_weighted_error_sum = 0      # 远端偏差×权重 累加
    found_row_count = 0             # 找到中心的行数
    far_found_row_count = 0         # 远端找到的行数
    bar_row_count = 0               # 疑似横线的行数,用于终点线判断
    corner_signal = 0               # 右弯信号
    scan_width_px = config.SCAN_COLUMN_LAST - config.SCAN_COLUMN_FIRST + 1

    for row_index in range(scan_row_count):
        row = SCAN_ROWS[row_index]
        lane_center, row_dark_pixel_count, single_line = scan_one_row(image, row, row_index)

        # 整行80%是黑的 = 横线，不是赛道线
        if row_dark_pixel_count * 10 >= scan_width_px * 8:
            bar_row_count += 1

        # 最近两行只剩外侧线，且推出的中心很靠左 => 内圈线已拐走 => 右弯
        if single_line == 1 and row_index <= 1:
            if 0 <= lane_center - ROW_HALF_LANE_WIDTH_PX[row_index] < config.CORNER_LEFT_LINE_COLUMN:
            #倒推左线列号 : (左+右)//2 - (右-左)//2
                corner_signal = 1

        if lane_center < 0:
            continue                    # 这行没找到，跳过

        column_error = lane_center - CENTER_COLUMN    # 正数=车道在右=车偏左=要往右打
        weight = SCAN_ROW_WEIGHT[row_index]
        weight_sum += weight
        weighted_error_sum += weight * column_error
        found_row_count += 1

        if row_index >= scan_row_count - 2:                  # 最远两行
            far_weight_sum += weight
            far_weighted_error_sum += weight * column_error
            far_found_row_count += 1

        if config.DEBUG_DRAW:
            image.draw_circle(lane_center, row, 2, color=0, fill=True)

    if weight_sum == 0:
        return 0, 0, 0, bar_row_count, 0, corner_signal   # 全部丢线

    error = weighted_error_sum // weight_sum          # 加权平均，整除（后面要发整数）
    far_error = far_weighted_error_sum // far_weight_sum if far_weight_sum > 0 else error  # 远端没数据就用整体顶替
    return error, far_error, found_row_count, bar_row_count, far_found_row_count, corner_signal
