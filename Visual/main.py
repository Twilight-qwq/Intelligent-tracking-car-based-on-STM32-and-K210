# -*- coding: utf-8 -*-
"""
============================================================
 main.py  ·  程序入口
------------------------------------------------------------
 三件事：开串口、开摄像头、然后死循环。
 真正的活都交给 vision.py 和 judge.py，这个文件只有 60 行。

 发给 STM32 的一帧是 4 个字节（和他的 Protocol.c 完全对应）：
     0xAA | 误差(int8) | 状态(uint8) | 0x55

 【整个程序的节奏】
     每秒钟循环 25 ~ 30 次，每次都是同一个流程：
         拍一张照片 → 找线 → 判断 → 发 4 个字节给 STM32
     STM32 那边负责把"偏差"变成"左右轮各转多快"，
     K210 完全不碰电机，只管报告"我看见了什么"。
============================================================
"""

import sensor
import time
from machine import UART
from fpioa_manager import fm

import config as C
import vision
from judge import Judge


def send_frame(uart, err, state):
    """把一帧 4 个字节通过串口发给 STM32。

    【err & 0xFF 是什么意思？】
    err 是 Python 的整数，可能是负数（比如 -40）。
    串口一次只能发 0~255 的一个字节，负数发不出去。
    在计算机里 -40 存成 8 位二进制是 1101 1000，也就是 216。
    所以 -40 & 0xFF 就等于 216，发出去是 216，
    STM32 那边按"有符号 8 位整数"(int8_t) 来读，216 又被解释回 -40。
    两边约定好用同一个规则，数据就对上了。

    state 是 0~3 的正数，& 0xFF 只是顺手写的，实际不会变。
    """
    # err 是负数时 & 0xFF 变成 128~255，STM32 收下来当 int8_t 正好对上
    uart.write(bytes((0xAA, err & 0xFF, state & 0xFF, 0x55)))


def main():
    """程序入口。跑起来就永远不会退出，除非拔电。"""

    # 调试开关：启动时把"扫哪几行、每行在多远"打印出来，
    # 用来检查 config.py 里的高度/角度填得对不对。
    if C.PRINT_TABLE:
        vision.print_table()

    # ---------------- 串口 ----------------
    # 【fpioa 是什么？】
    # K210 的引脚是"可编程"的：芯片内部有个交叉开关，
    # 可以把任意外设（串口、SPI、I2C…）接到任意物理引脚上。
    # 所以必须先登记：我要把 IO24 当作 UART1 的发送脚用。
    # force=True 表示"就算这个脚之前被别的功能占了，也强行抢过来"。
    fm.register(C.PIN_TX, fm.fpioa.UART1_TX, force=True)
    fm.register(C.PIN_RX, fm.fpioa.UART1_RX, force=True)
    # 波特率必须和 STM32 那边 USART1 一模一样，否则收到的全是乱码
    uart = UART(UART.UART1, C.BAUD)

    # ---------------- 摄像头 ----------------
    sensor.reset()                              # 复位摄像头，回到已知状态

    # 只要黑白（灰度）不要彩色。
    # 为什么？灰度图每个像素占 1 个字节，彩色要 2 个字节（RGB565），
    # 处理量少一半，帧率能翻倍。而找黑线本来也用不上颜色。
    sensor.set_pixformat(sensor.GRAYSCALE)      # 只要黑白，速度快一倍

    # 分辨率用 QQVGA = 160×120。
    # 分辨率越高看得越远越清楚，但也越慢。
    # 160×120 是"够用且够快"的折中，vision.py 里的几何算法也是按这个尺寸写的。
    sensor.set_framesize(sensor.QQVGA)          # 160 x 120

    # 画面上下颠倒 / 左右镜像的开关（摄像头可能是倒着装上去的）
    if C.HMIRROR:
        sensor.set_hmirror(True)
    if C.VFLIP:
        sensor.set_vflip(True)

    # 【为什么要 skip_frames？】
    # 摄像头刚上电时自动曝光还没调好，前几十帧会一会儿全黑一会儿全白。
    # 等 2 秒把这些废帧扔掉，等画面亮度稳定了再开始干活。
    sensor.skip_frames(time=2000)               # 等自动曝光稳定

    j = Judge()             # 造一个"判断器"，它会记住丢线帧数等状态
    clock = time.clock()    # 用来测帧率（每秒能处理几张）
    k = 0                   # 帧计数器，控制"每 30 帧打印一次"

    # ---------------- 主循环 ----------------
    while True:
        clock.tick()                # 记下这一帧的开始时间
        img = sensor.snapshot()     # 拍一张照片（这一步最耗时）

        # 第一步：在照片里找线，得到 6 个原始数据
        err_raw, far_raw, hits, bars, far_hits, corner = vision.find_lane(img)
        # 第二步：把这些原始数据翻译成"要发给 STM32 的两个数"
        err, state = j.step(err_raw, far_raw, hits, bars, far_hits, corner)
        # 第三步：发出去
        send_frame(uart, err, state)

        if C.DEBUG_DRAW:
            # 在画面正中间画个十字，方便肉眼判断"车道中心是不是偏向一边"。
            # 这只影响显示，不影响发出去的数据。
            img.draw_cross(vision.CENTER_X, vision.IMG_H // 2, color=0, size=6)

        k += 1
        # 每 PRINT_EVERY 帧打印一行，串口监视器里能实时看到状态。
        # 调参时全靠这行输出判断车"看到"了什么。
        if k % C.PRINT_EVERY == 0:
            print("err=%d far=%d st=%d hits=%d corner=%d fps=%.1f"
                  % (err, far_raw, state, hits, corner, clock.fps()))


if __name__ == "__main__":
    main()
