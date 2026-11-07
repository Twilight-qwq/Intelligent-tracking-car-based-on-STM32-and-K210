# -*- coding: utf-8 -*-
"""
============================================================
 judge.py  ·  把"差多少像素"翻译成 STM32 要的两个数
------------------------------------------------------------
 输入：vision.find_lane() 吐出来的那一堆
 输出：(误差, 状态)   —— 正好是串口一帧里的那两个字节

 为什么要有这个类：
 丢线要记"丢了几帧"、滤波要记"上次误差"，这些得一直存着。
 把它们和操作它们的函数捆在一起，就叫"对象"。
 用的时候：j = Judge()   然后每帧  j.step(...)
============================================================
"""

import config as C


class Judge(object):

    def __init__(self):
        self.last_err = 0       # 上一帧发出去的误差
        self.last_state = 0     # 上一帧发出去的状态
        self.lost = 0           # 连续丢了几帧
        self.bar_cnt = 0        # 连续几帧看到横线

    def step(self, err_raw, far_raw, hits, bars, far_hits, corner):
        """每帧调一次，返回 (要发出去的误差, 要发出去的状态)。"""

        # ---------------- 终点线 ----------------
        if C.FINISH_ENABLE and bars >= C.FINISH_ROWS:
            self.bar_cnt += 1
            if self.bar_cnt >= C.FINISH_FRAMES:
                return 0, 3
        else:
            self.bar_cnt = 0

        # ---------------- 丢线 ----------------
        if hits == 0:
            self.lost += 1
            if self.lost <= C.LOST_KEEP:
                return self.last_err, self.last_state   # 短暂丢线：照上次的走
            return (self.last_err if C.LOST_ERR_HOLD else 0), 1   # 低速盲开

        self.lost = 0

        # ---------------- 误差 ----------------
        err = int((err_raw - C.CALIB_OFFSET) * C.ERROR_GAIN)

        # 内圈线拐走了、而且此刻本来是正的 = 刚进弯，提前把方向打过去
        turning = corner and err < C.CORNER_TRIGGER
        if turning:
            err = C.CORNER_BIAS

        if C.HMIRROR:
            err = -err
        if C.SMOOTH_SHIFT > 0:
            err = self.last_err + ((err - self.last_err) >> C.SMOOTH_SHIFT)

        if err > 127:
            err = 127
        elif err < -128:
            err = -128

        # ---------------- 状态 ----------------
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

        self.last_err = err
        self.last_state = state
        return err, state
