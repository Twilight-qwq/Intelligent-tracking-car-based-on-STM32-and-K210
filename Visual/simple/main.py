# -*- coding: utf-8 -*-
"""main.py —— 程序入口。

流程：拍一张 → find_lane() → Judge.step() → 发 4 字节给 STM32。
一帧格式：0xAA | 误差(int8) | 状态(uint8) | 0x55
K210 只管"看见了什么"，把偏差变成轮速是 STM32 的事。
"""

import sensor
import time
from machine import UART
from fpioa_manager import fm

import config as C
import vision
from judge import Judge


def send_frame(uart, err, state):
    """发一帧 4 字节。

    err 可能是负数（如 -40），串口一次只能发 0~255。
    -40 的 8 位补码是 1101 1000 = 216，err & 0xFF 就得到 216，
    STM32 按 int8_t 读回来又是 -40，两边就对齐了。
    """
    uart.write(bytes((0xAA, err & 0xFF, state & 0xFF, 0x55)))


def main():

    if C.PRINT_TABLE:
        vision.print_table()

    # ---- 串口 ----
    # K210 引脚可映射，先把 IO24/IO25 登记成 UART1 的收发脚
    fm.register(C.PIN_TX, fm.fpioa.UART1_TX, force=True)
    fm.register(C.PIN_RX, fm.fpioa.UART1_RX, force=True)
    uart = UART(UART.UART1, C.BAUD)      # 波特率必须和 STM32 一致

    # ---- 摄像头 ----
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)   # 只要黑白，速度快一倍
    sensor.set_framesize(sensor.QQVGA)       # 160 x 120
    if C.HMIRROR:
        sensor.set_hmirror(True)
    if C.VFLIP:
        sensor.set_vflip(True)
    sensor.skip_frames(time=2000)            # 等自动曝光稳定

    j = Judge()             # 判断器，负责记住丢线帧数等状态
    clock = time.clock()    # 测帧率
    k = 0                   # 帧计数，控制打印频率

    while True:
        clock.tick()
        img = sensor.snapshot()                                  # 1. 拍照

        err_raw, far_raw, hits, bars, far_hits, corner = vision.find_lane(img)  # 2. 找线
        err, state = j.step(err_raw, far_raw, hits, bars, far_hits, corner)     # 3. 拍板
        send_frame(uart, err, state)                                            # 4. 发送

        if C.DEBUG_DRAW:
            # 画个十字，方便肉眼判断中心偏没偏；只影响显示
            img.draw_cross(vision.CENTER_X, vision.IMG_H // 2, color=0, size=6)

        k += 1
        if k % C.PRINT_EVERY == 0:
            print("err=%d far=%d st=%d hits=%d corner=%d fps=%.1f"
                  % (err, far_raw, state, hits, corner, clock.fps()))


if __name__ == "__main__":
    main()
