# -*- coding: utf-8 -*-
"""
============================================================
 judge.py  ·  把"差多少像素"翻译成 STM32 要的两个数
------------------------------------------------------------
 输入：vision.find_lane() 吐出来的那一堆
 输出：(误差, 状态)   —— 正好是串口一帧里的那两个字节

 【为什么要有这个类？】
 有些东西必须"记住上一帧的情况"才能判断：
     已经连续丢了几帧？
     上一次发出去的误差是多少？
     连续几帧看到横线了？
 这些不能放在函数的局部变量里（每次调用都会清零），得找地方一直存着。
 Python 里最方便的做法就是放进一个对象（class），
 用的时候先 j = Judge() 造一个，之后每帧 j.step(...)。

 【它和 vision.py 的分工】
 vision.py 只回答"画面上线在哪"，是纯客观的测量；
 judge.py 负责"拍板"：要不要平滑、丢线怎么办、前面是不是弯、
 该给 STM32 多大的数。以后想改策略，基本只改这个文件。
============================================================
"""

import config as C


class Judge(object):

    def __init__(self):
        """开机时执行一次，把"记忆"初始化。

        self.xxx 就是"这个对象自己的变量"（叫属性），
        它在两次 step() 调用之间一直存在，不会消失。
        """
        self.last_err = 0       # 上一帧发出去的误差
        self.last_state = 0     # 上一帧发出去的状态
        self.lost = 0           # 连续丢了几帧
        self.bar_cnt = 0        # 连续几帧看到横线

    def step(self, err_raw, far_raw, hits, bars, far_hits, corner):
        """每帧调一次，返回 (要发出去的误差, 要发出去的状态)。

        参数就是 vision.find_lane() 的 6 个返回值，一个不多一个不少。
        """

        # ---------------- 终点线 ----------------
        # bars：这一帧有几行整行变黑。
        # 连续 FINISH_FRAMES 帧都够数才认终点 —— 防止正好压过一块阴影就误判。
        if C.FINISH_ENABLE and bars >= C.FINISH_ROWS:
            self.bar_cnt += 1
            if self.bar_cnt >= C.FINISH_FRAMES:
                return 0, 3
        else:
            self.bar_cnt = 0

        # ---------------- 丢线 ----------------
        # 【丢线是常态，不是故障】
        # 过弯、反光、压线的时候，经常会有几帧什么都找不到。
        # 这时候如果立刻停车，车会一路顿挫着走。所以分两种情况：
        if hits == 0:
            self.lost += 1
            # 丢线不超过 LOST_KEEP 帧：假装什么都没发生，照上次的结果继续走
            if self.lost <= C.LOST_KEEP:
                return self.last_err, self.last_state   # 短暂丢线：照上次的走
            # 丢太久：降到状态 1（低速）盲开过去，赌一把。
            # 注意这里绝不返回 3 —— 状态 3 是"到达终点"，丢线不能冒充终点。
            return (self.last_err if C.LOST_ERR_HOLD else 0), 1   # 低速盲开

        self.lost = 0       # 这一帧找到了，丢线计数清零

        # ---------------- 误差 ----------------
        # 校准：把车摆正时读到的那点偏差减掉，让它正好变成 0
        # 增益：整体放大或缩小，一般不用动
        err = int((err_raw - C.CALIB_OFFSET) * C.ERROR_GAIN)

        # 【提前打方向】
        # corner = 1 说明"看到内圈线拐走了"，但此刻车本身可能还正对着
        # （err 接近 0）。等真的偏了再打方向就晚了，会冲出弯道。
        # 所以一旦确认前面是弯，直接把误差顶到 CORNER_BIAS，抢先打过去。
        #
        # 条件 err < CORNER_TRIGGER 是为了避免"已经在弯里"时反复触发：
        # 如果误差已经很大了还往同一个方向顶，反而会把方向打死。
        turning = corner and err < C.CORNER_TRIGGER
        if turning:
            err = C.CORNER_BIAS

        # 摄像头装反 / 画面开了镜像时，把正负号翻过来
        if C.HMIRROR:
            err = -err

        # 【指数平滑】
        # 不直接用新误差，而是"从旧值往新值挪一部分"：
        #     err = 旧值 + (新值 - 旧值) ÷ 2^SMOOTH_SHIFT
        # SMOOTH_SHIFT = 0 时不滤波；
        # 画面抖得厉害就设 1 或 2，越大越平滑，但也越迟钝（反应变慢）。
        # >> 是右移，相当于除以 2 的幂，比浮点除法快很多。
        if C.SMOOTH_SHIFT > 0:
            err = self.last_err + ((err - self.last_err) >> C.SMOOTH_SHIFT)

        # 【必须夹紧到 int8 范围】
        # 串口那一帧里误差只占 1 个字节，有符号的话就是 -128 ~ 127。
        # 超出去的数到了 C 语言的 int8_t 里会发生"回绕"——
        # 比如 200 会变成 -56，方向直接反了，车会往反方向打。
        # 所以在 Python 这边就先夹好，绝不能让超范围的数发出去。
        if err > 127:
            err = 127
        elif err < -128:
            err = -128

        # ---------------- 状态 ----------------
        # 状态只由"远处偏得厉害不厉害"决定，越弯越慢。
        # 取绝对值是因为往左弯、往右弯都要减速。
        curv = far_raw if far_raw >= 0 else -far_raw
        if curv < C.CURV_T1:
            state = 0
        elif curv < C.CURV_T2:
            state = 1
        else:
            state = 2

        # 远处的路一行都看不见了 —— 说明前面到头了（或者马上要急转）。
        # 这时候按直道速度冲过去必翻车，强制降到 1。
        if far_hits == 0 and state < 1:
            state = 1           # 远处的路看不见了 = 前面到头了
        # 已经确认要拐弯了，直接降到最慢的 2
        if turning:
            state = 2           # 确认要拐了，直接降到最慢

        # 记住这一帧的结果，供下一帧丢线时使用
        self.last_err = err
        self.last_state = state
        return err, state
