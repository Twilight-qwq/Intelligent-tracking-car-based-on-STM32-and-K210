# -*- coding: utf-8 -*-
"""judge.py —— 把"差多少像素"变成要发给 STM32 的两个数：(误差, 状态)。

用类是因为要记住上一帧的情况（丢了几帧、上次误差多少、连续几帧看到横线）。
vision.py 负责测量，judge.py 负责拍板。
"""

import config


class Judge(object):

    def __init__(self):
        self.last_error = 0             # 上一帧发出去的误差
        self.last_state = 0             # 上一帧发出去的状态
        self.lost_frame_count = 0       # 连续丢了几帧
        self.bar_frame_count = 0        # 连续几帧看到横线

    def decide(self, raw_error, far_raw_error, found_rows, bar_rows, far_found_rows, corner_signal):
        """每帧调一次，参数就是 find_lane() 的 6 个返回值。"""

        # ---- 终点线：连续几帧都够数才认，防止一块阴影误判 ----
        if config.FINISH_LINE_ENABLE and bar_rows >= config.FINISH_LINE_ROWS:
            self.bar_frame_count += 1
            if self.bar_frame_count >= config.FINISH_LINE_FRAMES:
                return 0, 3
        else:
            self.bar_frame_count = 0

        # ---- 丢线：短暂丢线照上次走，丢太久降速盲开 ----
        if found_rows == 0:
            self.lost_frame_count += 1
            if self.lost_frame_count <= config.LOST_FRAMES_KEEP:
                return self.last_error, self.last_state
            # 注意绝不返回 3：3 代表到达终点，丢线不能冒充终点
            return (self.last_error if config.LOST_USE_LAST_ERROR else 0), 1

        self.lost_frame_count = 0

        # ---- 误差：先校准，再放大 ----
        err = int((raw_error - config.CALIBRATION_OFFSET_PX) * config.ERROR_GAIN)

        # ---- 提前打方向 ----
        # corner_signal=1 说明看到内圈线拐走了，但车本身可能还正对着，等偏了再打就晚了。
        # 加 err < CORNER_TRIGGER_BELOW_PX 是防止已经在弯里时反复触发、把方向打死。
        is_corner_turn = corner_signal and err < config.CORNER_TRIGGER_BELOW_PX
        if is_corner_turn:
            err = config.CORNER_TURN_BIAS_PX

        # 画面镜像时把正负号翻过来
        if config.HORIZONTAL_MIRROR:
            err = -err

        # ---- 指数平滑：从旧值往新值挪一部分 ----
        if config.SMOOTH_FILTER_SHIFT > 0:
            err = self.last_error + ((err - self.last_error) >> config.SMOOTH_FILTER_SHIFT)

        # ---- 必须夹到 int8 范围 ----
        # 超出去会回绕（200 变 -56），方向直接反，所以先夹好再发
        if err > 127:
            err = 127
        elif err < -128:
            err = -128

        # ---- 状态：远处偏得越多，弯越急，档位越低 ----
        far_error_abs = far_raw_error if far_raw_error >= 0 else -far_raw_error
        if far_error_abs < config.CURVE_ERROR_MEDIUM_PX:
            state = 0
        elif far_error_abs < config.CURVE_ERROR_LARGE_PX:
            state = 1
        else:
            state = 2

        if far_found_rows == 0 and state < 1:
            state = 1               # 远处的路看不见了 = 前面到头了
        if is_corner_turn:
            state = 2               # 确认要拐了，直接降到最慢

        # 记住这一帧，供下一帧丢线时使用
        self.last_error = err
        self.last_state = state
        return err, state
