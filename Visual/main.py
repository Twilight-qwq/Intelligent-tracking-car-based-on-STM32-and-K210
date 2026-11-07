# -*- coding: utf-8 -*-
"""
============================================================
 main.py  ·  程序入口
------------------------------------------------------------
 三件事：开串口、开摄像头、然后死循环。
 真正的活都交给 vision.py 和 judge.py，这个文件只有 60 行。

 发给 STM32 的一帧是 4 个字节（和他的 Protocol.c 完全对应）：
     0xAA | 误差(int8) | 状态(uint8) | 0x55
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
    # err 是负数时 & 0xFF 变成 128~255，STM32 收下来当 int8_t 正好对上
    uart.write(bytes((0xAA, err & 0xFF, state & 0xFF, 0x55)))


def main():
    if C.PRINT_TABLE:
        vision.print_table()

    # ---------------- 串口 ----------------
    fm.register(C.PIN_TX, fm.fpioa.UART1_TX, force=True)
    fm.register(C.PIN_RX, fm.fpioa.UART1_RX, force=True)
    uart = UART(UART.UART1, C.BAUD)

    # ---------------- 摄像头 ----------------
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)      # 只要黑白，速度快一倍
    sensor.set_framesize(sensor.QQVGA)          # 160 x 120
    if C.HMIRROR:
        sensor.set_hmirror(True)
    if C.VFLIP:
        sensor.set_vflip(True)
    sensor.skip_frames(time=2000)               # 等自动曝光稳定

    j = Judge()
    clock = time.clock()
    k = 0

    # ---------------- 主循环 ----------------
    while True:
        clock.tick()
        img = sensor.snapshot()

        err_raw, far_raw, hits, bars, far_hits, corner = vision.find_lane(img)
        err, state = j.step(err_raw, far_raw, hits, bars, far_hits, corner)
        send_frame(uart, err, state)

        if C.DEBUG_DRAW:
            img.draw_cross(vision.CENTER_X, vision.IMG_H // 2, color=0, size=6)

        k += 1
        if k % C.PRINT_EVERY == 0:
            print("err=%d far=%d st=%d hits=%d corner=%d fps=%.1f"
                  % (err, far_raw, state, hits, corner, clock.fps()))


if __name__ == "__main__":
    main()
