# -*- coding: utf-8 -*-
"""config.py —— 所有可调参数集中在这里，本文件没有任何逻辑。

改完要重新保存到板子上，再运行 main.py 才生效。
"""

# ---------- 一、摄像头安装（上车前量准） ----------
CAMERA_HEIGHT_CM = 20.0      # 镜头离地高度，厘米
CAMERA_PITCH_DEG = 30.0      # 镜头低头角：光轴比水平低多少度
CAMERA_HFOV_DEG = 93.0       # 镜头水平视场角：镜头左右能张开多少度
LANE_HALF_WIDTH_M = 0.20     # 车道半宽，米：两黑线【中心】距 40 厘米 ÷ 2（线宽 5，白道 35）

SCAN_ROW_SHIFT = 0           # 整组扫描行上下平移（应急微调，正常填 0）
SCAN_ROW_COUNT = 5           # 扫几行

# 合格扫描行的半车道宽范围（外侧线中心到车道中心，占多少像素）
# 2026-11-18：车道半宽 27.5→20 厘米，窗口按 20/27.5 等比缩放（22→16，62→45），扫的行号不变
HALF_LANE_WIDTH_MIN_PX = 16  # 小于它 = 太远，两条线挤成一团分不开
HALF_LANE_WIDTH_MAX_PX = 45  # 大于它 = 太近，车道撑出画面

# ---------- 二、找线（扫一行时用） ----------
BLACK_RUN_MIN_PX = 3         # 连续黑段最短几像素才算一条线（滤掉噪点）
BLACK_RUN_MAX_PX = 26        # 连续黑段最宽几像素（再宽就是糊成一片，不是线）

HALF_WIDTH_TOLERANCE_PX = 14  # 实测半车道宽和理论值差多少以内还算配对成功
SINGLE_LINE_DEAD_ZONE_PX = 12  # 只剩一条线时，离画面中心多近就分不清是内线还是外线

CONTRAST_MIN_LEVEL = 70      # 本行最亮减最暗至少差多少才认（否则当作这行没线）
BLACK_THRESHOLD_RATIO = 0.40  # 分界线 = 最暗 + (最亮 − 最暗) × 这个比例

SCAN_COLUMN_FIRST = 4        # 每行从第几列开始读（跳过左边暗角）
SCAN_COLUMN_LAST = 155       # 每行读到第几列（跳过右边暗角）

# ---------- 三、判断（拍板时用） ----------
ERROR_CALIBRATION_PX = 0     # 车摆正时读出来是多少偏差，就填多少，用来自动扣掉
ERROR_GAIN = 1.0             # 偏差整体放大或缩小

CURVE_ERROR_MEDIUM_PX = 10   # 远处偏差 > 10 像素 → 状态 1（中速）
CURVE_ERROR_LARGE_PX = 22    # 远处偏差 > 22 像素 → 状态 2（慢速）

CORNER_TURN_BIAS_PX = 40     # 确认要拐弯时，把偏差直接顶到这个数
CORNER_TRIGGER_BELOW_PX = 8  # 只有当前偏差小于它，才算"刚进弯"，才允许顶
CORNER_LEFT_LINE_COLUMN = 50  # 左线的列号小于它 → 判定为右弯

SMOOTH_FILTER_SHIFT = 0      # 指数平滑：0 = 不滤波；1、2 越大越平滑但越迟钝

LOST_FRAMES_KEEP = 10        # 一帧全丢时，这么多帧以内照上次结果继续发
LOST_USE_LAST_ERROR = 1      # 超过之后：1 = 沿用上次偏差，0 = 发 0

FINISH_LINE_ENABLE = False   # 终点线检测，默认关
FINISH_LINE_ROWS = 4         # 有几行整行变黑才算横线
FINISH_LINE_FRAMES = 3       # 连续几帧都够数才认

# ---------- 四、串口 ----------
UART_BAUD = 115200           # 必须和 STM32 一致
UART_TX_PIN = 24             # K210 IO24 → STM32 的接收脚
UART_RX_PIN = 25             # 用不到，但引脚要先占住

# ---------- 五、调试（只影响显示和打印，不影响行为） ----------
VERTICAL_FLIP = False        # 画面上下颠倒就改 True
HORIZONTAL_MIRROR = False    # 画面左右镜像就改 True（偏差会自动取反）
DEBUG_DRAW = True            # 在预览画面上画标记
PRINT_EVERY_FRAMES = 30      # 每多少帧打印一行
PRINT_ROW_TABLE = True       # 启动时打印扫描行表
