# -*- coding: utf-8 -*-
"""config.py —— 所有可调参数集中在这里，本文件没有任何逻辑。

改完要重新保存到板子上，再运行 main.py 才生效。
"""

# ---------- 一、摄像头安装（上车前量准） ----------
CAMERA_HEIGHT_CM = 20.0     # 镜头离地高度，厘米
CAMERA_PITCH_DEG = 30.0     # 镜头低头角度，度
CAMERA_HFOV_DEG = 90.0      # 镜头水平视场角，度

SCAN_ROW_OFFSET = 0         # 整组扫描行上下平移（应急微调，正常 0）
SCAN_ROW_COUNT = 5          # 扫几行

# 合格扫描行的半间距范围（外侧线到车道中心的像素数）
LINE_GAP_MIN_PX = 22        # 小于它 = 太远，两线挤成一团
LINE_GAP_MAX_PX = 62        # 大于它 = 太近，车道撑出画面

# ---------- 二、找线 ----------
LINE_RUN_MIN_PX = 3         # 黑段最短几像素才算线（滤噪点）
LINE_RUN_MAX_PX = 26        # 黑段最宽几像素（再宽就是糊成一片）

LINE_GAP_TOLERANCE_PX = 14  # 实测间距和理论值差多少以内还算数
SIDE_DEAD_ZONE_PX = 12      # 单线时离画面中心多近就分不清内外线

CONTRAST_MIN_LEVEL = 70     # 本行最亮减最暗至少多少才认（否则没线）
BLACK_THRESHOLD_RATIO = 0.40  # 阈值 = 最暗 + (最亮-最暗)*0.40

SCAN_COLUMN_FIRST = 4       # 每行从左扫到第几列（避开暗角）
SCAN_COLUMN_LAST = 155      # 每行扫到第几列

# ---------- 三、判断 ----------
CALIBRATION_OFFSET_PX = 0   # 车摆正时读出多少偏差就填多少
ERROR_GAIN = 1.0            # 误差整体放大/缩小

CURVE_ERROR_MEDIUM_PX = 10  # 远处偏差 > 10 像素 -> 状态 1（中速）
CURVE_ERROR_LARGE_PX = 22   # 远处偏差 > 22 像素 -> 状态 2（慢速）

CORNER_TURN_BIAS_PX = 40    # 确认右弯时把误差直接顶到这个数
CORNER_TRIGGER_BELOW_PX = 8  # 只有当前误差小于它才算"刚进弯"
CORNER_EDGE_COLUMN = 50     # 外侧线推出的中心在这个列号左边 = 右弯

SMOOTH_FILTER_SHIFT = 0     # 指数平滑：0=不滤波，1/2=越来越平滑但迟钝

LOST_FRAMES_KEEP = 10       # 丢线这么多帧以内照上次结果继续发
LOST_USE_LAST_ERROR = 1     # 超过之后 1=沿用上次误差，0=发 0

FINISH_LINE_ENABLE = False  # 终点线检测，默认关
FINISH_LINE_ROWS = 4        # 几行整行变黑才算横线
FINISH_LINE_FRAMES = 3      # 连续几帧才认

# ---------- 四、串口 ----------
UART_BAUD = 115200          # 必须和 STM32 一致
UART_TX_PIN = 24            # K210 IO24 -> STM32
UART_RX_PIN = 25            # 用不到，但要先占住

# ---------- 五、调试 ----------
VERTICAL_FLIP = False       # 画面上下颠倒改 True
HORIZONTAL_MIRROR = False   # 画面左右镜像改 True（误差自动取反）
DEBUG_DRAW = True           # 在预览画面上画标记
PRINT_EVERY_FRAMES = 30     # 每多少帧打印一行
PRINT_ROW_TABLE = True      # 启动时打印扫描行表
