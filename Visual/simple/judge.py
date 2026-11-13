# -*- coding: utf-8 -*-
"""judge.py —— 把"差多少像素"变成要发给 STM32 的两个数：(误差, 状态)。

用类是因为要记住上一帧的情况（丢了几帧、上次误差多少、连续几帧看到横线）。
vision.py 负责测量，judge.py 负责拍板。
"""

import config as C


class Judge(object):

    def __init__(self):
        self.last_err = 0       # 上一帧发出去的误差
        self.last_state = 0     # 上一帧发出去的状态
        self.lost = 0           # 连续丢了几帧
        self.bar_cnt = 0        # 连续几帧看到横线

    def step(self, err_raw, far_raw, hits, bars, far_hits, corner):
        """每帧调一次，参数就是 find_lane() 的 6 个返回值。"""

        # ---- 终点线：连续几帧都够数才认，防止一块阴影误判 ----
        if C.FINISH_ENABLE and bars >= C.FINISH_ROWS:
            self.bar_cnt += 1
            if self.bar_cnt >= C.FINISH_FRAMES:
                return 0, 3
        else:
            self.bar_cnt = 0

        # ---- 丢线：短暂丢线照上次走，丢太久降速盲开 ----
        if hits == 0:
            self.lost += 1
            if self.lost <= C.LOST_KEEP:
                return self.last_err, self.last_state
            # 注意绝不返回 3：3 代表到达终点，丢线不能冒充终点
            return (self.last_err if C.LOST_ERR_HOLD else 0), 1

        self.lost = 0

        # ---- 误差：先校准，再放大 ----
        err = int((err_raw - C.CALIB_OFFSET) * C.ERROR_GAIN)

        # ---- 提前打方向 ----
        # corner=1 说明看到内圈线拐走了，但车本身可能还正对着，等偏了再打就晚了。
        # 加 err < CORNER_TRIGGER 是防止已经在弯里时反复触发、把方向打死。
        turning = corner and err < C.CORNER_TRIGGER
        if turning:
            err = C.CORNER_BIAS

        # 画面镜像时把正负号翻过来
        if C.HMIRROR:
            err = -err

        # ---- 指数平滑：从旧值往新值挪一部分 ----
        if C.SMOOTH_SHIFT > 0:
            err = self.last_err + ((err - self.last_err) >> C.SMOOTH_SHIFT)

        # ---- 必须夹到 int8 范围 ----
        # 超出去会回绕（200 变 -56），方向直接反，所以先夹好再发
        if err > 127:
            err = 127
        elif err < -128:
            err = -128

        # ---- 状态：远处偏得越多，弯越急，档位越低 ----
        curv = far_raw if far_raw >= 0 else -far_raw
        if curv < C.CURV_T1:
            state = 0
        elif curv < C.CURV_T2:
            state = 1
        else:
            state = 2

        if far_hits == 0 and state < 1:
            state = 1           # 远处的路看不见了 = 前面到头了
        if turning:
            state = 2           # 确认要拐了，直接降到最慢

        # 记住这一帧，供下一帧丢线时使用
        self.last_err = err
        self.last_state = state
        return err, state
