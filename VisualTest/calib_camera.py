# -*- coding: utf-8 -*-
"""摄像头参数标定：把现场抄下来的 9 个「实测半宽」变成新的镜头参数。

怎么用
  1. 把下面 MEASURED 里 9 个数换成板子打印的「实」那一列（顺序：行129 → 行62）
  2. 运行这个文件
  3. 把打印出来的 CAMERA_HEIGHT_CM 和 CAMERA_PITCH_DEG 抄回 live_scan_test.py
  4. 重新烧，再读一块，看「实/理」是不是都接近 1.00

原理
  一行的半宽（像素）= 焦距 × 车道半宽 × (cos低头角 × 这一行的正切 + sin低头角) ÷ 镜头高度
  所以「半宽」对「这一行的正切」是**一条直线**：
      半宽 = 斜率 × 正切 + 截距
  用 9 个点拟合出这条直线，斜率和截距就把低头角和高度算出来了。

  焦距不参与拟合（它吸收进高度里）—— 这三个量本来就分不开，
  只能定下两个，第三个跟着走。这和现在的 config 是一致的。
"""
import math

# ================== 现场抄的 9 个数 ==================
MEASURED = [91, 78, 64, 47, 41, 37, 34, 32, 29]   # 「实」那一列，行123 → 行55（现场改成实测值）
# =====================================================

ROWS = [123, 108, 93, 75, 68, 64, 60, 58, 55]
IMAGE_HEIGHT = 240
LANE_HALF_WIDTH_M = 0.20      # 车道半宽 20 厘米（两条胶带中心距 40 厘米 ÷ 2）
LINE_WIDTH_CM = 5.0           # 胶带宽度
HFOV_DEG = 93.0               # 现在的估计值，只用来定焦距
FOCAL = (320 / 2.0) / math.tan(math.radians(HFOV_DEG) / 2.0)


def fit(rows, measured):
    """最小二乘拟合 半宽 = 斜率 × 正切 + 截距。"""
    xs = [(r - IMAGE_HEIGHT / 2.0) / FOCAL for r in rows]
    ys = list(measured)
    n = len(xs)
    sx = sum(xs)
    sy = sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None, None
    slope = (n * sxy - sx * sy) / den
    inter = (sy - slope * sx) / n
    return slope, inter


def half_width_of_row(row, height_cm, pitch_deg):
    tan_a = (row - IMAGE_HEIGHT / 2.0) / FOCAL
    sin_t = math.sin(math.radians(pitch_deg))
    cos_t = math.cos(math.radians(pitch_deg))
    return FOCAL * LANE_HALF_WIDTH_M * (cos_t * tan_a + sin_t) / (height_cm / 100.0)


def main():
    if len(MEASURED) != len(ROWS):
        print("!! MEASURED 必须是 %d 个数，现在是 %d 个" % (len(ROWS), len(MEASURED)))
        return
    if min(MEASURED) <= 0:
        print("!! 有 0 或负数，说明那一行没读出实测半宽，先去现场重读")
        return
    if MEASURED[0] <= MEASURED[-1]:
        print("!! 第一个（最近的行）应该比最后一个（最远的行）大，顺序可能抄反了")
        return

    slope, inter = fit(ROWS, MEASURED)
    if slope is None:
        print("!! 拟合失败")
        return

    pitch = math.degrees(math.atan2(inter, slope))
    # 板子上的 MicroPython（MaixPy）没有 math.hypot，只能用 sqrt 自己拼。
    # 别改回 hypot，一改板子上就跑不了。
    scale = math.sqrt(slope * slope + inter * inter)
    height_cm = FOCAL * LANE_HALF_WIDTH_M / scale * 100.0

    print("=" * 62)
    print("拟合结果")
    print("  斜率  %+8.3f    截距  %+8.3f" % (slope, inter))
    print("")
    print("  CAMERA_PITCH_DEG = %.1f        （现在写的是多少，对比一下）" % pitch)
    print("  CAMERA_HEIGHT_CM = %.1f        （这个不是真实长度，是等效值）" % height_cm)
    print("=" * 62)
    print("")
    print("逐行验证（新参数下理论值应该贴近实测值）")
    print("  行号    实测    新理论   比值    拟合残差")
    worst = 0.0
    for r, m in zip(ROWS, MEASURED):
        th = half_width_of_row(r, height_cm, pitch)
        ratio = m / th if th else 0
        res = m - (slope * (r - IMAGE_HEIGHT / 2.0) / FOCAL + inter)
        worst = max(worst, abs(ratio - 1.0))
        print("  %4d   %5.1f   %6.1f   %.3f   %+6.2f" % (r, m, th, ratio, res))
    print("")
    print("  最大比值偏差  %.1f%%" % (worst * 100))
    if worst < 0.05:
        print("  -> 九个点很贴合，说明镜头没歪、赛道也是直的，可以采信")
    elif worst < 0.12:
        print("  -> 有点散，先看看是不是某个点抄错了")
    else:
        print("  -> 散得厉害：可能是镜头侧倾、车头没摆正、或者那个位置不是直道")
    print("")
    print("顺带算出来的线宽（应该是 5 厘米左右）")
    for r, m in zip(ROWS, MEASURED):
        print("  行%4d  理论线宽 %.1f 像素" % (r, m * LINE_WIDTH_CM / (LANE_HALF_WIDTH_M * 100)))
    print("")
    print("除了上面两个数，如果还想动 HFOV，先别动 —— 它和高度是绑在一起的。")


if __name__ == "__main__":
    main()
