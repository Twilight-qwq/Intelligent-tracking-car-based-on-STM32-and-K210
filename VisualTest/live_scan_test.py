# -*- coding: utf-8 -*-
"""live_scan_test.py v3 —— 现场扫线记录（320x240，9 行，单文件）。

粘进 MaixPy IDE 直接跑。要改的只有下面 GROUP_NAME 那一行。

和 v2（160x120、5 行）的区别：
    1. 画面 320x240，帧率从 19.2 提到 38.5（实测）
    2. 扫 9 行：近 4 行算偏差，远 5 行判状态（远的行不进偏差）
    3. 每行的最小线宽、配对容差都按距离自动算，不用手填
    4. 发出去的偏差先除以 2，回到 160 尺度 —— STM32 那边一个数都不用改

每 2.5 秒打印一块。一块就是一个完整采样，拍一张截图就能带走。
"""

# ================== 换组只改这一行 ==================
GROUP_NAME = "3 锐角弯"      # 例："2 左偏10cm" "7 反光"；每换一组改一次再运行
PRINT_EVERY_MS = 2500          # 每多少毫秒打印一块（2500 = 2.5 秒）
DEBUG_DRAW = False             # 在画面上画扫描线和圆心：IDE 预览是关的就别开，白花时间
PATTERN_LOG = True             # 九行图案一变就打一行：采弯道数据必须开
PATTERN_LOG_MIN_MS = 100       # 但最少隔这么多毫秒才打一行（防刷屏拖垮帧率）
# 为什么需要这个限速：图案要是每帧都在抖（直道上出现过 ######### 和 ######R## 来回跳），
# 一行 6 毫秒，一帧一打就把 26 毫秒的帧期撑到 30 多毫秒，帧率直接腰斩。
# 限速之后最多每秒 10 行，一个弯走 1 秒还能留 10 个采样点，够看。


# ================== 画面尺寸 ==================
IMAGE_WIDTH = 320              # 320x240 = QVGA，实测 38.46 帧/秒
IMAGE_HEIGHT = 240
SCALE = IMAGE_WIDTH / 160.0    # = 2.0。所有像素阈值都乘它，别手填
CENTER_COLUMN = IMAGE_WIDTH // 2


# ================== 安装参数（必须和 config.py 一致） ==================
CAMERA_HFOV_DEG = 93.0         # 镜头水平视场角
CAMERA_PITCH_DEG = 32.5        # 镜头低头角（2026-11-19 现场标定：用前 6 行的实测半宽反推）
CAMERA_HEIGHT_CM = 18.4        # 镜头离地高度（同上，等效值；不是用卷尺量的真实长度）
LANE_HALF_WIDTH_M = 0.20       # 车道半宽 20 厘米（现场实测：两黑线中心距 40 厘米 ÷ 2）
LINE_WIDTH_CM = 5.0            # 一条黑线宽 5 厘米（现场实测，白道 35 厘米），用来算每行的最小/最大线宽


# ================== 扫哪 9 行：按【车前距离】指定 ==================
# 近 4 行算偏差（线够粗、够稳），远 5 行判状态（看得远、线细）
ROW_TARGET_CM = (
    27.5, 34.6, 45.0, 63.9,                  # 近 4 行 → 偏差
    76.0, 85.0, 94.0, 103.0, 112.0,          # 远 5 行 → 远偏差 / 状态（都挤在能测准的区间）
)
NEAR_ROW_COUNT = 4                 # 前几行算偏差
SCAN_ROW_COUNT = 9                 # 总共几行（126/137 厘米那两行实测认不出，挪到 70~110 了）
NEAR_WEIGHT = (3, 3, 2, 2)         # 近 4 行的权重：越近越可信
FAR_WEIGHT = (3, 3, 2, 2, 1)       # 远 5 行的权重：越近的远行越可信

# ---- 扫描窗口：每行只扫"车道可能出现的那一段"，不再全宽 8~311 ----
# 好处一：省算力（远行能省一半以上）
# 好处二：画面外的 logo、墙、箱子根本进不了窗口
ROI_MARGIN_PX = 15 * 2             # 预期中心两侧各留多少余量（320 尺度，= 160 尺度的 15 像素）
# 2026-11-19 现场：原来留 40（=80 像素），结果把场地右侧的墙根（240~311 那一大片）
# 一起罩进来了，远处 5 行要么被当成"太宽"丢掉，要么和右边的线错配。
# 收到 15 之后窗口只比两条线多出 30 个像素，背景进不来，总像素也少了三分之一。


# ================== 认线的参数（160 尺度，乘 SCALE 使用） ==================
BLACK_RUN_MAX_FACTOR = 3.0             # 黑段最宽 = 理论线宽 × 这个倍数（再宽就是阴影/横线）
BLACK_RUN_MAX_FLOOR = 6                # 但最宽不小于 6 像素
ADAPT_HALF_WIDTH = False               # 测试脚本里关掉：只报告「实 / 理」，不让理论值被实测带跑
HALF_WIDTH_TOLERANCE_RATIO = 0.40      # 配对容差 = 这一行半宽的 40%（不用绝对数）
SINGLE_LINE_DEAD_ZONE_PX = int(12 * SCALE)   # 只剩一条线时，离中心多近就分不清是哪条
CONTRAST_MIN_LEVEL = 70                # 本行最亮减最暗至少差多少才认
BLACK_THRESHOLD_RATIO = 0.55           # 分界线 = 最暗 + (最亮-最暗) × 这个比例
# 2026-11-19 现场：0.40 太保守，远处的胶带只露出中间一条芯（实测线宽只有理论的 60~77%），
# 到 76 厘米处左线只剩 2 个像素、84 厘米以后一个都不剩。
# 提到 0.55 让胶带的身子整条算进来。再往上调（>0.65）地面会开始变黑，别超。
SCAN_COLUMN_FIRST = int(4 * SCALE)     # 从第几列开始读（跳过左边暗角）
SCAN_COLUMN_LAST = IMAGE_WIDTH - 1 - SCAN_COLUMN_FIRST   # 读到第几列
RUN_MIN_FACTOR = 0.45                  # 最小线宽 = 理论线宽 × 这个比例
# 2026-11-19 现场：0.6 太严。实测胶带只露出理论的 60~80%（边缘被分界线切掉），
# 远处几行的线正好 4~6 像素，而要求 5~6 像素 —— 图案就会在「好行 9」和「好行 5」之间抖。
# 降到 0.45 之后远处 3~4 像素也算线，实测 4~8 像素全都过得去。
RUN_MIN_FLOOR = 3                      # 但最小不小于 3 像素
# 并段：相邻两个黑段之间只隔了几个白像素，就当成一整段。
# 反光会把线打出一个白点，线被切成两截、两截都太窄被丢掉，并段就是把这个洞填回去。
# 太强的反光（整段冲白）填不回来 —— 那一段根本不"黑"，没有洞可填。
# 洞能有多大，按这一行线宽的比例定：线越细，洞就得越小，否则把噪声也一起吞进去。
MERGE_GAP_RATIO = 0.20                 # 能填的洞 ≤ 这一行理论线宽的 20%
MERGE_GAP_FLOOR = 2                    # 但至少给 2 个像素

# ---- 2026-11-20 查出来的两个坑，开关留着，现场不对时两秒钟能关回去 ----
USE_TRIM_BRIGHTEST = True              # 分界线不用"最亮"，用"去掉高光后的最亮"
TRIM_BRIGHT_PERCENT = 97               # 从暗往亮累加到这一比例就停（剩下的算高光）
# 为什么：2026-11-19 串口原样数据里，76 厘米那行 最暗 52 最亮 124，
# 分界线 = 52 + (124-52) × 0.55 = 92，而远处昏暗的地面只有 60~85 ——
# 整片地面被算成黑，扫出 39、58、82、156 像素宽的巨块。
# 那个 124 是灯打在纸上的一点反光，不是地面。把最亮的 3% 扔掉再算，分界线就落回地面以上。

REJECT_ROW_WITH_WIDE_RUN = True        # 一行里出现"太宽的黑块" → 这一行整个作废
# 为什么：巨块本身会被线宽上限挡掉，但挡掉之后剩下的"合格段"全是巨块的边缘，
# 或者白纸外面那块深色地板（列 8~26、290~311）。实测这一条能造出中 54 / 259 / 268
# 三个错 100 像素的中心，远偏差 −52 就是它。
REJECT_ONLY_WHEN_SINGLE = True         # 只在"这一行只剩一条线"时才作废；配对成功的不管

BORDER_GUARD_PX = 30                   # 兜底用全宽重扫时，画面最外侧这么多列一律不看
# 为什么：窄窗口一条线都没找到时才退回全宽（兜底）。全宽会把白纸外面的深色地板、
# 墙根、暗角一起罩进来。这些地方本来就该在窗口外，兜底也不该看。


# ================== 判断那一侧（judge.py）的参数，一律 160 尺度 ==================
ERROR_CALIBRATION_PX = 0
ERROR_GAIN = 1.0
CURVE_ERROR_MEDIUM_PX = 10             # 远偏差 > 10 像素 → 状态 1（中速）
CURVE_ERROR_LARGE_PX = 22              # 远偏差 > 22 像素 → 状态 2（慢速）
CORNER_TURN_BIAS_PX = 40               # 确认拐弯时把偏差顶到这个数
CORNER_TRIGGER_BELOW_PX = 8            # 只有当前偏差小于它才允许顶
CORNER_LEFT_LINE_COLUMN = 50           # 左线列号小于它 → 右弯
LOST_FRAMES_KEEP = 10
LOST_USE_LAST_ERROR = 1


# ================== 弯道状态机（只做右弯） ==================
# 打开就由状态机接管弯道；关掉就回到原来「远偏差判档 + corner_signal 顶方向」的老逻辑。
USE_CORNER_SM = True

# 特征只有一个：从最远那行往回数，连续有多少行「不是两条都在」。
#     0        → 直道
#     1 ~ 2    → 预警：降一档
#     3 以上   → 确认进弯：降到最低档，并且强行顶方向
#
# 只做右弯：进弯时直接发正的 SM_BIAS_PX，不去分辨是 L 还是 R。
# 赛道是凸的房子形，顺时针跑每个弯都是右弯，所以不需要左弯。
#
# 下面带【待定】的数都是拍的，等弯道数据回来再填。
SM_TAIL_WARN = 3                       # 连续丢这么多行 → 预警（降一档）。丢 1~2 行还留在 30 档
SM_TAIL_CORNER = 7                     # 旧门槛：连续丢这么多行 → 进弯（好行 ≤ 2）。现在只当参考
# 2026-11-20 实车：原来 5 行（好行 4）太早 —— 丢 5 行发生在弯前 60~80 厘米
# （最远那几行先看到弯），于是车还在直道上就被判成「进弯」，
# 拿到固定的 +100 一直右转 → 原地转圈。改成 7 行，那时车已到弯口附近。
# （原来「9-5=4，正好近 4 行还全在」的说法作废）
#
# ---- 2026-11-20 下午：判弯改成只看【最近 3 行】----
# 远处那几行受光照影响最大：线又细又远，一反光就认不出来，
# 换个场地参数就得重调。而最近 3 行只有车真的走到弯口才会一起丢，
# 位置稳得多，也不怕远处反光。
# 远处 6 行不废弃 —— 它们继续负责「提前降速」（SM_TAIL_WARN）。
# 最近 3 行刚好是 27.7 / 34.6 / 44.6 厘米这三行。
SM_NEAR_ROWS = 3                       # 判弯只看最近这几行
SM_NEAR_CORNER = 3                     # 这几行全丢，才算车真的到弯口了
SM_CONFIRM_FRAMES = 2                  # 【待定】连续几帧指向新状态才认进弯
SM_RELEASE_FRAMES = 4                  # 【待定】连续几帧指向新状态才认出来
SM_BIAS_PX = 0                         # 接管时强行发出去的偏差（160 尺度，正数 = 往右）
# 2026-11-20 实车：原来 30 会「一点偏差就打死方向」，直道画龙、直角弯提前掉头，改成 0 关掉。
CORNER_ERR_FIXED = 50                  # 状态 2（真进弯）时固定发出去的值（160 尺度）
SEND_CORNER_FIXED = False              # 状态2时是否改发上面这个固定值
# 2026-11-20 下午改成 False：STM32 转弯期间不看偏差（按编码器角度转），
# 那个假值已经没用；而且它会把"转完之后用视觉修航向"这条路堵死。
SM_LATE_FRAMES = 3                     # 【待定】弯中偏差连续这么多帧还是负的 → 怀疑来不及回中
# ---- 防局部误判：必须先"预警"够久，才允许"进弯" ----
# 真弯是从远到近一路丢过来的，一定会先触发状态 1；
# 褶皱、反光是局部随机的，经常直接砸在近处，不经过状态 1。
# 所以要求：进弯之前，状态 1 至少要持续这么多帧。
# 4 帧 ≈ 110 毫秒（一帧 27.7 毫秒），车几乎感觉不到。
SM_PRE_FRAMES = 4
# 2026-11-19 现场弯道数据：车一接近弯顶，只剩左线，程序用「左线 + 半宽」推中心，
# 那条线越靠左，推出来的中心就越小 → 偏差反而变成负的（实测 −10 → −52）。
# 也就是说最该往右打死的那一段，偏差在喊"往左"。
# 原来定 10 帧（260 毫秒）才强制，实测太慢：15 档下 260 毫秒已经走了 11 厘米。
# 降到 3 帧（78 毫秒）抢先接管。
SM_GOOD_WINDOW = 5                     # 好行个数滤波：取最近几帧的最小值（抗 ±1 回退）
SM_GOOD_MAX_DROP = 2                   # 好行个数一帧最多允许降几行（抗反光暴降）
SM_MIN_SINGLE = 0                      # 坏掉的行里至少要有几个「只剩一条线」才算真弯。0 = 不要求，只要最近几行丢了就算弯
                                       #
                                       # 2026-11-22 从 1 改成 0 —— 现场日志证明这道门在挡真弯：
                                       #   翻 11-20 的八份日志，锐角弯那一处「最近3行丢掉」的记录共 5728 条，
                                       #   其中只有 12.1% 是「只剩左线 L」，15.4% 是「只剩任意一条线」，
                                       #   84.6% 是「近处几行全是 x（一条都没认出来）」。
                                       #   为什么弯口给的是 x 而不是 L：内圈线拐走了之后，
                                       #   外侧线也跑到了扫描窗口边上（或那条黑段被判太宽/太窄作废），
                                       #   于是那一行"一条都没结果"，不是"只剩一条"。
                                       #   所以当初「真弯→只剩一条线；反光→两条都没」这个假设不成立。
                                       # 改了之后：宁可多报，不能漏。误报靠 STM32 的里程门禁挡
                                       #   （Service.c 里 TURN_DIST_TOLERANCE_CM 已从 55 收到 40 厘米）。
# ---- 降速和打方向用两个不同的门槛 ----
# 降速：早一点安全（晚了刹不住），所以丢 SM_TAIL_CORNER=5 行就降。
# 打方向：晚一点安全（早了会切内线），所以丢 SM_DIR_LOST=7 行才接管。
# 钝角弯的路只是轻轻折一下，内圈那条线一直没离开画面，丢不到 7 行 ——
# 所以钝角弯永远是「只降速、不打方向」，偏差自己就能把车带过去。
# 只有锐角（77°）和直角才丢得这么深，才真的需要顶那一脚。
SM_DIR_LOST = 7                        # 【待定】丢到这么多行才接管方向（好行 ≤ 2）
SM_DIR_CONFIRM = 2                     # 【待定】连续几帧都丢够深才允许打方向（抗单帧噪声）

# ---- 保险丝：按【走了多远】算，不按时间算 ----
# 状态机不会说"我错了"，只好靠它自己累计走过的路程兜底。
# 为什么不能用时间：同样 1.2 米的路，15 档要 2.86 秒，30 档只要 1.43 秒。
# 为什么不用问 STM32：K210 知道自己发的是几档，档位速度是定死的，自己乘一下就行。
SM_FUSE_M = 2.5                        # 实车用：状态 2 期间累计走 2.5 米还没出来 → 判错，强退
# 【历史记录】手推测弯道时必须临时改成 99（等于关掉）。
# 原因：保险丝假设车按 15 档（0.4216 米/秒）在跑，2.5 ÷ 0.4216 = 5.93 秒；
# 手推远没那么快，甚至推推停停，于是"状态 2 撑了 5.93 秒"被当成"走了 2.5 米"，
# 假烧 → 强制退回状态 1 → 把方向接管提前掐断（实测 19.6 秒接管，23.9 秒就被掐了）。
# 2026-11-20 上午手推测完，已按用户要求改回 2.5，留给实车。
# 三档的标称速度（米/秒）。出处：实测电机上限 37 个计数/5 毫秒 = 1.04 米/秒，
# 所以 1 个计数 = 1.04/37 米/秒，乘 30 / 20 / 15 就是这三档。
# 【注意】这里的 30/20/15 必须和 Service.c 的 Speed_Straight / SmallAngle / LargeAngle 一致。
# 那边一降速，这里必须跟着改，否则保险丝算出来的路程是错的。
SM_GEAR_SPEED = (1.04 / 37.0 * 30, 1.04 / 37.0 * 20, 1.04 / 37.0 * 15)


# ================== 几何 ==================
import math

FOCAL_LENGTH_PX = (IMAGE_WIDTH / 2.0) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2.0)
PITCH_SIN = math.sin(math.radians(CAMERA_PITCH_DEG))
PITCH_COS = math.cos(math.radians(CAMERA_PITCH_DEG))
CAMERA_HEIGHT_M = CAMERA_HEIGHT_CM / 100.0
LANE_HALF_WIDTH_M_LOCAL = LANE_HALF_WIDTH_M
LINE_WIDTH_M = LINE_WIDTH_CM / 100.0


def axial_distance_of_row(row):
    """第 row 行：沿光轴量的轴向距离（米）。打不到地面返回 0。"""
    tan_angle = (row - IMAGE_HEIGHT / 2.0) / FOCAL_LENGTH_PX
    vertical = -PITCH_COS * tan_angle - PITCH_SIN      # 这条光线朝下的分量
    if vertical > -1e-6:
        return 0.0                                      # 朝上或水平 = 打不到地面
    return -CAMERA_HEIGHT_M / vertical                  # 镜头高度 ÷ 朝下分量


def horizontal_distance_cm(row):
    """第 row 行正中央对应的车前水平距离（厘米）。打不到地面返回负数。"""
    axial = axial_distance_of_row(row)
    if axial <= 1e-6:
        return -1.0
    tan_angle = (row - IMAGE_HEIGHT / 2.0) / FOCAL_LENGTH_PX
    forward = -PITCH_SIN * tan_angle + PITCH_COS        # 这条光线朝前的分量
    return forward * axial * 100.0


def half_lane_width_of_row(row):
    """第 row 行：半车道宽占几个像素。"""
    axial = axial_distance_of_row(row)
    if axial <= 1e-6:
        return 0
    return int(FOCAL_LENGTH_PX * LANE_HALF_WIDTH_M_LOCAL / axial + 0.5)


def line_width_px_of_row(row):
    """第 row 行：一条黑线占几个像素。越远越细，最小线宽靠它算。"""
    axial = axial_distance_of_row(row)
    if axial <= 1e-6:
        return 0.0
    return FOCAL_LENGTH_PX * LINE_WIDTH_M / axial


def row_for_distance_cm(target_cm):
    """反推：车前 target_cm 那么远，落在第几行。"""
    best_row = -1
    best_diff = 1e9
    for row in range(IMAGE_HEIGHT):
        distance = horizontal_distance_cm(row)
        if distance <= 0:
            continue                                    # 地平线以上，跳过
        diff = distance - target_cm
        if diff < 0:
            diff = -diff
        if diff < best_diff:
            best_diff = diff
            best_row = row
    return best_row


def choose_scan_rows():
    """按 ROW_TARGET_CM 里的距离反推 9 个行号，由近到远返回。"""
    picked = []
    for target in ROW_TARGET_CM:
        picked.append(row_for_distance_cm(target))
    return picked


# 开机算一次，之后全程序共用
SCAN_ROWS = choose_scan_rows()
ROW_HALF_LANE_WIDTH_PX = [half_lane_width_of_row(row) for row in SCAN_ROWS]
ROW_LINE_WIDTH_PX = [line_width_px_of_row(row) for row in SCAN_ROWS]
ROW_MIN_RUN_PX = []          # 这一行最小几像素才算线
ROW_MAX_RUN_PX = []          # 这一行最宽几像素还算线
ROW_TOLERANCE_PX = []        # 这一行配对容差
ROW_MERGE_GAP_PX = []        # 这一行相邻黑段之间最多能填几个白像素
for _i in range(SCAN_ROW_COUNT):
    _min_run = int(ROW_LINE_WIDTH_PX[_i] * RUN_MIN_FACTOR + 0.5)
    if _min_run < RUN_MIN_FLOOR:
        _min_run = RUN_MIN_FLOOR
    ROW_MIN_RUN_PX.append(_min_run)
    _max_run = int(ROW_LINE_WIDTH_PX[_i] * BLACK_RUN_MAX_FACTOR + 0.5)
    if _max_run < BLACK_RUN_MAX_FLOOR:
        _max_run = BLACK_RUN_MAX_FLOOR
    ROW_MAX_RUN_PX.append(_max_run)
    ROW_TOLERANCE_PX.append(int(ROW_HALF_LANE_WIDTH_PX[_i] * HALF_WIDTH_TOLERANCE_RATIO))
    _gap = int(ROW_LINE_WIDTH_PX[_i] * MERGE_GAP_RATIO)   # 直接截断，不要四舍五入
    if _gap < MERGE_GAP_FLOOR:
        _gap = MERGE_GAP_FLOOR
    ROW_MERGE_GAP_PX.append(_gap)

ROW_THRESHOLD = [0] * SCAN_ROW_COUNT   # 诊断用：这一行最后用的分界线，print_block 直接读
ROW_WIDE_COUNT = [0] * SCAN_ROW_COUNT  # 诊断用：这一行出现了几个"太宽的黑块"


# ================== 扫一行 ==================
def scan_one_row(img, row, row_index, column_lo, column_hi, use_dead_zone=True):
    """扫第 row 行【从 column_lo 到 column_hi 这一段】，返回 8 个东西（顺序固定）：

    0 中心列     算不出来是 -1
    1 单线标记   0=两条都在 1=只剩外侧(左) 2=只剩内圈(右) -1=没线
    2 合格段数
    3 实测半宽   合格段 >= 2 时才有意义
    4 暗点总数
    5 最暗灰度
    6 最亮灰度
    7 段列表     [(起列, 止列, 是否合格), ...]，最多记 6 段

    use_dead_zone：只有近 4 行传 True。近 4 行的中心要拿去算偏差，
    所以"分不清左右"时宁可作废；远 5 行只用来数找到了几条线，不设死区。
    """
    ROW_THRESHOLD[row_index] = 0      # 诊断用：先清零，提前返回时不会打出上一次的旧值
    ROW_WIDE_COUNT[row_index] = 0
    get_pixel = img.get_pixel
    values = []
    append = values.append
    # 灰度直方图（32 格，格 = 灰度 >> 3）。不单独再扫一遍 —— 直接和读像素同一遍步，
    # 否则每帧多出 2500 次循环，实测 16 毫秒的扫线会被拖到 21 毫秒。
    hist = [0] * 32 if USE_TRIM_BRIGHTEST else None
    if hist is not None:
        for x in range(column_lo, column_hi + 1):   # 只读窗口内这一段
            _v = get_pixel(x, row)
            append(_v)
            hist[_v >> 3] += 1
    else:
        for x in range(column_lo, column_hi + 1):
            append(get_pixel(x, row))

    darkest = min(values)
    brightest = max(values)
    if brightest - darkest < CONTRAST_MIN_LEVEL:
        return -1, -1, 0, 0, 0, darkest, brightest, []
    pixel_count = len(values)

    # ---- 分界线：上界不用"最亮"，用"去掉最亮那百分之几之后的最亮" ----
    # 最亮经常是灯打在纸上的一点反光，比地面高 40 级；拿它当上界会把分界线整体抬高，
    # 整片地面就变成黑的。从暗往亮累加到 TRIM_BRIGHT_PERCENT% 就停，取那一格的上边。
    # 不排序，开销很小。
    if hist is not None:
        need = pixel_count * TRIM_BRIGHT_PERCENT // 100
        if need < 1:
            need = 1
        acc = 0
        hi_bin = 31
        for b in range(32):
            acc += hist[b]
            if acc >= need:
                hi_bin = b
                break
        upper = (hi_bin << 3) + 7          # 这一格最亮的值
        if upper > brightest:
            upper = brightest
        if upper < darkest:
            upper = darkest
    else:
        upper = brightest
    threshold = darkest + int((upper - darkest) * BLACK_THRESHOLD_RATIO)
    ROW_THRESHOLD[row_index] = threshold   # 诊断用

    min_run = ROW_MIN_RUN_PX[row_index]         # 这一行自己的最小线宽
    max_run = ROW_MAX_RUN_PX[row_index]         # 这一行自己的最大线宽
    merge_gap = ROW_MERGE_GAP_PX[row_index]     # 这一行最多能填几个白像素
    runs = []
    wide_count = 0                              # 这一行出现几个"太宽的黑块"
    line_centers = []                           # 所有合格线的中心列
    left_column = -1
    right_column = -1
    line_count = 0
    dark_pixel_count = 0
    measured = 0
    # ---- 第一步：先找出所有连续黑段（还没做判断）----
    raw_runs = []
    index = 0
    while index < pixel_count:
        if values[index] < threshold:
            end = index
            while end < pixel_count and values[end] < threshold:
                end += 1
            raw_runs.append((index, end - 1))
            index = end
        else:
            index += 1

    # ---- 第二步：并段（填反光白点）----
    # 相邻两段之间只隔了 merge_gap 个白像素，就当成一整段。
    merged = []
    for (run_start, run_end) in raw_runs:
        if merged and run_start - merged[-1][1] - 1 <= merge_gap:
            merged[-1] = (merged[-1][0], run_end)
        else:
            merged.append((run_start, run_end))

    # ---- 第三步：逐段量长度，判断是不是车道线 ----
    for (run_start, run_end) in merged:
        run_length = run_end - run_start + 1
        dark_pixel_count += run_length
        keep = min_run <= run_length <= max_run
        if run_length > max_run:
            wide_count += 1                     # 太宽 = 地面被分界线吃了 / 白纸外面的深色地板
        if len(runs) < 6:
            runs.append((run_start + column_lo, run_end + column_lo, keep))
        if keep:
            center_column = (run_start + run_end) // 2 + column_lo
            line_centers.append(center_column)
            if line_count == 0:
                left_column = center_column
            right_column = center_column
            line_count += 1

    ROW_WIDE_COUNT[row_index] = wide_count      # 诊断用

    if line_count == 0:
        return -1, -1, 0, 0, dark_pixel_count, darkest, brightest, runs

    # ---- 出现太宽的黑块，而且这一行只剩一条线 → 这条线不可信，整行作废 ----
    # 巨块本身已经被线宽上限挡掉了，但挡掉之后剩下的"合格段"是巨块的边缘，
    # 或者白纸外面那块深色地板。实测这一条造出过中 54 / 259 / 268，错 100 个像素。
    if REJECT_ROW_WITH_WIDE_RUN and wide_count > 0:
        if (not REJECT_ONLY_WHEN_SINGLE) or line_count <= 1:
            return -1, -1, line_count, 0, dark_pixel_count, darkest, brightest, runs

    half_width = ROW_HALF_LANE_WIDTH_PX[row_index]
    tolerance = ROW_TOLERANCE_PX[row_index]
    if line_count >= 2:
        # 关键：不能取"最左 + 最右" —— 画面里有杂物（墙、箱子）时会被当成线。
        # 改成在所有这些线里，挑"间距最接近 2×理半宽"的一对。
        target_gap = half_width * 2
        pair_left = line_centers[0]
        pair_right = line_centers[-1]
        best_diff = -1
        count = len(line_centers)
        for i in range(count - 1):
            for j in range(i + 1, count):
                gap = line_centers[j] - line_centers[i]
                diff = gap - target_gap
                if diff < 0:
                    diff = -diff
                if best_diff < 0 or diff < best_diff:
                    best_diff = diff
                    pair_left = line_centers[i]
                    pair_right = line_centers[j]
        measured = (pair_right - pair_left) // 2
        if abs(measured - half_width) <= tolerance:
            if ADAPT_HALF_WIDTH:
                # 实测和理论差得不多就挪 1/16，和 vision.py 一样
                ROW_HALF_LANE_WIDTH_PX[row_index] = (half_width * 15 + measured) >> 4
            return ((pair_left + pair_right) // 2, 0, line_count, measured,
                    dark_pixel_count, darkest, brightest, runs)
        # 找到两条以上的线却配不上 —— 说明画面里有杂物，不是"只剩一条线"。
        # 这种情况硬推中心会推出半幅画面的错值（实测出现过 39 和 245 同时出现），
        # 所以直接判这一行作废，让加权平均跳过它。
        return -1, -1, line_count, measured, dark_pixel_count, darkest, brightest, runs
    if use_dead_zone:
        # 近 4 行：算不出中心就整行作废，因为它们要用中心去算偏差。
        # 分不清左右的时候硬猜，中心会错 2 倍半宽（近行 186 像素）= 一个满舵，
        # 所以中间那段死区里不猜，直接把这一行扔掉。
        if left_column < CENTER_COLUMN - SINGLE_LINE_DEAD_ZONE_PX:
            return (left_column + half_width, 1, line_count, measured,
                    dark_pixel_count, darkest, brightest, runs)
        if right_column > CENTER_COLUMN + SINGLE_LINE_DEAD_ZONE_PX:
            return (right_column - half_width, 2, line_count, measured,
                    dark_pixel_count, darkest, brightest, runs)
        return -1, -1, line_count, measured, dark_pixel_count, darkest, brightest, runs

    # 远 5 行：它们只用来回答"这一行找到了几条线"，中心不进偏差。
    # 所以中间没有"分不清"这回事 —— 一条线就按它在画面哪一半来标，不设死区。
    # （原来用同一个 ±24 的死区，最远那行半宽才 29，车偏 10 个像素就把 L 判成 x 了。）
    if left_column < CENTER_COLUMN:
        return (left_column + half_width, 1, line_count, measured,
                dark_pixel_count, darkest, brightest, runs)
    return (right_column - half_width, 2, line_count, measured,
            dark_pixel_count, darkest, brightest, runs)


# ================== 一帧：扫 9 行 + 合成偏差 + 算实际发出 ==================
NOTE_NONE = 0
NOTE_CORNER = 1
NOTE_LOST_KEEP = 2
NOTE_LOST_BLIND = 3
NOTE_LATE = 4            # 弯中偏差一直回不到正数：可能来不及回中
NOTE_SLOW = 5            # 降到最慢档了，但还没到打方向的门槛 —— 只减速，方向没动

last_error = 0          # 上一帧实际发出去的误差（160 尺度），丢线时要用
last_state = 0
lost_frame_count = 0
last_center = CENTER_COLUMN   # 上一帧算出的车道中心列，给下一帧当扫描窗口的基准


def to_sent_scale(value):
    """把 320 尺度的偏差换回 160 尺度（四舍五入），STM32 那边就不用改。"""
    if value >= 0:
        return int(value / SCALE + 0.5)
    return -int(-value / SCALE + 0.5)


# ================== 弯道状态机 ==================
sm_state = 0          # 0 直道 / 1 预警 / 2 弯中
sm_pending = 0        # 连续多少帧「想变但还没变」
sm_enter_ms = 0       # 进弯的那一刻
sm_blocked = False    # 超时强退之后先锁住，免得马上又进弯
sm_neg_frames = 0     # 弯中偏差连续为负的帧数
sm_late = False       # 这一趟弯已经打了「可能来不及回中」的标记
sm_bias_hold = 0      # 方向接管的保持帧数
sm_now_ms = 0         # 主循环每帧更新；离线测试里一直是 0
sm_travel_m = 0.0     # 状态 2 期间累计走了多少米（保险丝用）
sm_last_ms = 0        # 上一帧的时间戳，用来算这一帧过了多久
sm_dir_frames = 0     # 连续多少帧「丢得够深」，够 SM_DIR_CONFIRM 才允许打方向
sm_warn_frames = 0    # 状态 1 已经连续持续了多少帧（防局部误判用）


def good_rows(pattern):
    """从最近那行往远数：连续有几个「两条都在」。

    pattern 下标 0 = 最近的行(129)，越往后越远(62)。

    ---- 为什么数「好的行」而不是数「丢的行」----
    直觉是数丢了几行。但实测下来，远端的行会从 1（只剩左线）
    变成 -1（一条都没有），丢的行数并不增加，只是往近处挪。
    所以数丢的行会原地打转。

    数「从近端开始还有几个好的」，才是单调的：
        直道            9
        最远开始丢      8
        丢到第 3 行     6
        丢到第 6 行     3
        近处也丢        0

    ---- 它抗光照 ----
    光照变差坏的是远处的行，近处那几行还是好的，
    所以这个数不受影响。
    """
    count = 0
    for marker in pattern:
        if marker != 0:                 # 0 = 两条都在
            break
        count += 1
    return count


def count_unknown(pattern):
    """九行里有多少行是「一条线都没找到」—— 光照 / 反光的指标，不进状态机。"""
    n = 0
    for marker in pattern:
        if marker < 0:
            n += 1
    return n


# 好行个数的滤波器状态
sm_good = SCAN_ROW_COUNT               # 第一层（限速后）的值
sm_good_buf = [SCAN_ROW_COUNT] * SM_GOOD_WINDOW   # 第二层：最近几帧
sm_good_head = 0


def filter_good(raw):
    """好行个数 -> 滤波后的好行个数。两层，代价约 10 微秒。

    第 1 层  下降限速：一帧最多降 SM_GOOD_MAX_DROP 行
             挡「反光打在近处，9 一次掉到 0」
             真弯是一帧降 1 行，限速完全不影响它

    第 2 层  最近 SM_GOOD_WINDOW 帧取最小值
             挡「某一帧又跳回 9」这种 ±1 回退
             出弯时窗口滑过去就自动恢复，不会卡住
    """
    global sm_good, sm_good_buf, sm_good_head

    if raw < sm_good:
        sm_good = max(raw, sm_good - SM_GOOD_MAX_DROP)   # 只能慢慢降
    else:
        sm_good = raw                                     # 升回来不限制

    sm_good_buf[sm_good_head] = sm_good
    sm_good_head = (sm_good_head + 1) % SM_GOOD_WINDOW

    lowest = sm_good_buf[0]
    for v in sm_good_buf:
        if v < lowest:
            lowest = v
    return lowest


def run_corner_sm(pattern, error):
    """九行图案 + 当前偏差 →（car_state, 实际发出去的偏差）。

    car_state：0 快 / 1 中 / 2 慢（STM32 那边对应 30 / 20 / 15）

    分工：
        速度档  -> 用【滤波后】的好行个数判，要稳
        方向    -> 弯中把偏差补到一个下限，偏差本来就是负的不动
                   证据一消失就松手，不硬顶
    """
    global sm_state, sm_pending, sm_enter_ms, sm_blocked
    global sm_neg_frames, sm_late, sm_bias_hold
    global sm_travel_m, sm_last_ms, sm_dir_frames, sm_warn_frames

    # ---- 预警帧计数：状态 1 已经连续多少帧了 ----
    # 只统计「上一帧就已经在状态 1 或 2」的帧数，用来要求"先预警、再进弯"。
    if sm_state >= 1:
        if sm_warn_frames < 1000:
            sm_warn_frames += 1
    else:
        sm_warn_frames = 0

    # ---- 路程累计（保险丝用）----
    # 只在状态 2 的时候加，用【上一帧】的状态（这一帧还没决定新状态）。
    # 档位速度是定死的，K210 发的是几档自己知道，所以不需要 STM32 回传。
    if sm_now_ms:
        if sm_last_ms:
            dt_ms = sm_now_ms - sm_last_ms
            if 0 < dt_ms < 200 and sm_state == 2:   # 超过 200 毫秒说明卡了一下，不计
                sm_travel_m += SM_GEAR_SPEED[2] * (dt_ms / 1000.0)
        sm_last_ms = sm_now_ms

    # 好行个数 -> 滤波 -> 换算成「从近端数，有多少行已经不是好的」
    good = filter_good(good_rows(pattern))
    lost = SCAN_ROW_COUNT - good
    # 方向接管用【没滤波】的那份：滤波为了稳，速度要稳，
    # 但方向要跟得快，不然出弯之后还会多顶一百多毫秒。
    raw_lost = SCAN_ROW_COUNT - good_rows(pattern)

    # ---- 判弯改成只看最近 3 行（2026-11-20 下午）----
    # good_rows() 返回的是「从最近那行往外，连续有几个两条都在」，
    # 直接夹到 3 就是最近 3 行的情况，不用另外扫。
    near_run = good_rows(pattern)
    if near_run > SM_NEAR_ROWS:
        near_run = SM_NEAR_ROWS
    lost_near = SM_NEAR_ROWS - near_run

    # 真假：最近这几行坏掉时，是不是「只剩一条线」
    #   真弯 -> 内圈线拐走了，外圈还在（marker == 1）
    #   反光 -> 两条都认不出来（marker == -1）
    if lost_near > 0 and sm_state < 2:
        single_count = 0
        for i in range(SM_NEAR_ROWS):
            if pattern[i] == 1:
                single_count += 1
        if single_count < SM_MIN_SINGLE:
            # 近处全是"根本没认出线" —— 这更像光照/反光，不像弯。
            # 这种情况只降速，不判弯。
            lost_near = 0

    if lost_near >= SM_NEAR_CORNER:
        want = 2                            # 最近 3 行全丢 = 车真的到弯口了
    elif lost >= SM_TAIL_WARN:
        want = 1                            # 远处先丢几行 = 提前降速
    else:
        want = 0

    # ---- 必须先「预警」够久，才允许「进弯」----
    # 真弯是从远到近一路丢过来的，一定会先触发状态 1；
    # 褶皱、反光是局部随机的，经常直接砸在近处，不经过状态 1。
    # 所以：想进弯，之前的状态 1 至少要撑够 SM_PRE_FRAMES 帧。
    if want == 2 and not (sm_state >= 1 and sm_warn_frames >= SM_PRE_FRAMES):
        want = 1

    # 超时强退之后先锁住：要等最近的几行线回来，才允许再进弯
    if sm_blocked:
        if lost_near < SM_NEAR_CORNER:
            sm_blocked = False
        elif want == 2:
            want = 1

    # 降档一级一级降，不能从最慢直接跳回最快（跳回去车会猛地窜出去）
    if want < sm_state:
        want = sm_state - 1

    # 防抖：要换状态，得连续 SM_*_FRAMES 帧都指向新状态
    if want != sm_state:
        sm_pending += 1
        need = SM_CONFIRM_FRAMES if want > sm_state else SM_RELEASE_FRAMES
        if sm_pending >= need:
            was = sm_state
            sm_state = want
            sm_pending = 0
            if want == 2:
                sm_enter_ms = sm_now_ms
                sm_travel_m = 0.0                   # 新的一趟弯，路程从 0 开始算
                if was != 2:                        # 新的一趟弯，清掉上一趟的标记
                    sm_neg_frames = 0
                    sm_late = False
            elif was == 2:
                sm_dir_frames = 0                   # 出弯了，方向的确认计数也清掉
    else:
        sm_pending = 0

    # 保险丝：状态 2 期间累计走了 SM_FUSE_M 还没出来，说明这一脚判错了，别一直卡在最慢档。
    # 按路程算而不是按时间：真弯该走多远是定的，慢的时候只是要走更久。
    if sm_state == 2 and sm_travel_m > SM_FUSE_M:
        sm_state = 1
        sm_pending = 0
        sm_blocked = True
        sm_travel_m = 0.0
        sm_dir_frames = 0

    if sm_state == 2:
        # 方向接管有两个门槛：
        #   ① 丢的行数要够深（SM_DIR_LOST = 7）—— 钝角弯丢不到这么深，天生不接管
        #   ② 而且要连续 SM_DIR_CONFIRM 帧都够深 —— 单帧噪声顶不出方向
        if raw_lost >= SM_DIR_LOST:
            if sm_dir_frames < 100:
                sm_dir_frames += 1
        else:
            sm_dir_frames = 0
        if sm_dir_frames < SM_DIR_CONFIRM:
            sm_bias_hold = 0
            sm_neg_frames = 0
            return 2, error                     # 只降速，不打方向
        # 方向接管：看【原始】好行数，证据一消失就松开，
        # 只多保持 2 帧防止闪跳。
        if raw_lost >= SM_TAIL_WARN:
            sm_bias_hold = 2
        elif sm_bias_hold > 0:
            sm_bias_hold -= 1
        if sm_bias_hold == 0:
            sm_neg_frames = 0
            return 2, error                     # 只降速，不再硬顶方向
        # ---- 方案 A：只有「偏差是正的但不够」才补这一脚 ----
        # 偏差 = 均值 - 中心列。偏差 < 0 说明车道中心在画面左边，也就是车偏右，
        # 这时候该往【左】打把车拉回中间，硬顶 +40 会把它顶出赛道。
        # 所以负偏差先不动，让偏差自己把车拉回中线，等它变正了再补。
        #
        # 但是：如果一直回不到正数，说明已经来不及回中了，
        # 再等下去车就是一直往左打、直直冲出弯道外侧。
        # 所以超过 SM_LATE_FRAMES 帧就放弃回中，强制补方向。
        if error < 0:
            sm_neg_frames += 1
            if sm_neg_frames >= SM_LATE_FRAMES:
                sm_late = True                      # 标记：这一趟来不及回中
        else:
            sm_neg_frames = 0
        bias = SM_BIAS_PX * int(SCALE)
        if sm_late:
            if error < bias:
                error = bias                        # 来不及了，至少给到 40
        elif 0 <= error < bias:
            error = bias
        return 2, error

    sm_neg_frames = 0
    if sm_state == 1:
        return 1, error
    return 0, error


def scan_frame(image):
    """扫一帧，返回 11 个东西：
    0 九行结果   1 偏差   2 远偏差   3 找到行数   4 远处找到行数
    5 弯道信号   6 触发弯道的行下标   7 状态   8 实际发出的误差（160 尺度）
    9 备注       10 九行的单线标记图案
    """
    global last_error, last_state, lost_frame_count, last_center

    rows_info = []
    pattern = []
    near_sum = 0
    near_weight_sum = 0
    far_sum = 0
    far_weight_sum = 0
    found_row_count = 0
    far_found_row_count = 0
    corner_signal = 0
    corner_row = -1

    for row_index in range(SCAN_ROW_COUNT):
        row = SCAN_ROWS[row_index]

        # 扫描窗口：以【上一帧算出的车道中心】为基准，两侧各留 半宽 + 余量。
        # 这样远行只扫一百多个像素，画面外的 logo / 墙 / 箱子根本进不来。
        span = ROW_HALF_LANE_WIDTH_PX[row_index] + ROI_MARGIN_PX
        column_lo = last_center - span
        column_hi = last_center + span
        if column_lo < SCAN_COLUMN_FIRST:
            column_lo = SCAN_COLUMN_FIRST
        if column_hi > SCAN_COLUMN_LAST:
            column_hi = SCAN_COLUMN_LAST

        info = scan_one_row(image, row, row_index, column_lo, column_hi,
                            row_index < NEAR_ROW_COUNT)

        # 兜底：窗口里一条线都没找到，说明预期中心偏了（起步、丢线、被撞），
        # 这一行退回全宽再扫一次，宁可慢也不要瞎。
        # 【2026-11-20】兜底也不能看画面最外侧 BORDER_GUARD_PX 列：
        # 白纸外面的深色地板、墙根、镜头暗角都在那里，宽度刚好能冒充车道线。
        if info[0] < 0 and (column_lo > SCAN_COLUMN_FIRST or column_hi < SCAN_COLUMN_LAST):
            guard_lo = SCAN_COLUMN_FIRST + BORDER_GUARD_PX
            guard_hi = SCAN_COLUMN_LAST - BORDER_GUARD_PX
            info = scan_one_row(image, row, row_index, guard_lo, guard_hi,
                                row_index < NEAR_ROW_COUNT)

        rows_info.append(info)
        pattern.append(info[1])                     # 单线标记，用来判弯

        if DEBUG_DRAW:
            image.draw_line(SCAN_COLUMN_FIRST, row, SCAN_COLUMN_LAST, row, color=160)
            if info[0] >= 0:
                image.draw_circle(info[0], row, 4, color=0, fill=True)

        # 弯道信号：一期保持原样 —— 只看最近两行、只剩外侧、左线在左边界内
        if info[1] == 1 and row_index <= 1:
            left_column = info[0] - ROW_HALF_LANE_WIDTH_PX[row_index]
            if 0 <= left_column < CORNER_LEFT_LINE_COLUMN * SCALE:
                corner_signal = 1
                corner_row = row_index

        if info[0] < 0:
            continue                                # 这行没算出中心
        column_error = info[0] - CENTER_COLUMN
        found_row_count += 1
        if row_index < NEAR_ROW_COUNT:
            weight = NEAR_WEIGHT[row_index]
            near_sum += weight * column_error
            near_weight_sum += weight
        else:
            weight = FAR_WEIGHT[row_index - NEAR_ROW_COUNT]
            far_sum += weight * column_error
            far_weight_sum += weight
            far_found_row_count += 1

    raw_error = near_sum // near_weight_sum if near_weight_sum > 0 else 0
    far_raw_error = (far_sum // far_weight_sum
                     if far_weight_sum > 0 else raw_error)

    # ---- 一行都没找到：照 judge.py 的做法，先沿用上次，超过 10 帧降档 ----
    if found_row_count == 0:
        lost_frame_count += 1
        if lost_frame_count <= LOST_FRAMES_KEEP:
            return (rows_info, raw_error, far_raw_error, 0, far_found_row_count,
                    corner_signal, corner_row, last_state, last_error,
                    NOTE_LOST_KEEP, pattern)
        # 2026-11-20 实车：这里原来是 2（最慢档）。但 STM32 现在把「状态2」当成
        # 「这里有弯，准备转90度」的触发信号，于是只要相机盲开一秒钟，
        # 车就会原地转一圈 —— 实测在起跑线后几厘米就开始转，就是这一行造成的。
        # 盲开改成报 1（降速），不再冒充弯道。
        return (rows_info, raw_error, far_raw_error, 0, far_found_row_count,
                corner_signal, corner_row, 1,
                last_error if LOST_USE_LAST_ERROR else 0,
                NOTE_LOST_BLIND, pattern)
    lost_frame_count = 0
    last_center = CENTER_COLUMN + raw_error    # 给下一帧的扫描窗口当基准

    # ---- 以下按 judge.py 的规则算"实际会发出什么"，全部在 320 尺度里算 ----
    error = int((raw_error - ERROR_CALIBRATION_PX * SCALE) * ERROR_GAIN)
    if USE_CORNER_SM:
        # 新路子：状态机看「远处连续丢了几行」，弯道直接接管
        error_before_sm = error
        car_state, error = run_corner_sm(pattern, error)
        # 状态机到底动没动方向：偏差被顶上去过才算"接管"
        is_corner_turn = (error != error_before_sm)
    else:
        # 老路子：远偏差判档 + corner_signal 顶方向（留着重写对比用）
        # 注意 abs() —— 少了它，偏差 −11 的时候也会顶，方向会反
        is_corner_turn = corner_signal and abs(error) < CORNER_TRIGGER_BELOW_PX * SCALE
        if is_corner_turn:
            error = CORNER_TURN_BIAS_PX * SCALE
        far_abs = far_raw_error if far_raw_error >= 0 else -far_raw_error
        if far_abs < CURVE_ERROR_MEDIUM_PX * SCALE:
            car_state = 0
        elif far_abs < CURVE_ERROR_LARGE_PX * SCALE:
            car_state = 1
        else:
            car_state = 2
        if far_found_row_count == 0 and car_state < 1:
            car_state = 1                           # 远处的路看不见了 = 前面到头了
        if is_corner_turn:
            car_state = 2

    # ---- 换回 160 尺度，再夹到 int8（顺序不能反，先除后夹）----
    sent_error = to_sent_scale(error)
    if sent_error > 127:
        sent_error = 127
    elif sent_error < -128:
        sent_error = -128

    last_error = sent_error
    last_state = car_state

    if USE_CORNER_SM:
        if sm_late:
            note_out = NOTE_LATE
        elif is_corner_turn:
            note_out = NOTE_CORNER
        elif car_state == 2:
            note_out = NOTE_SLOW          # 最慢档，但方向没动
        else:
            note_out = NOTE_NONE
    elif is_corner_turn:
        note_out = NOTE_CORNER
    else:
        note_out = NOTE_NONE

    return (rows_info, raw_error, far_raw_error, found_row_count, far_found_row_count,
            corner_signal, corner_row, car_state, sent_error,
            note_out, pattern)


# ================== 打印一块 ==================
SINGLE_NAME = {0: "两条", 1: "只剩外侧", 2: "只剩内圈", -1: "没线"}


def format_runs(runs):
    """把段列表变成 '30-41* 275-287*' 这样一串（* = 被当成线）。"""
    if not runs:
        return "--"
    pieces = []
    for (start_column, end_column, keep) in runs:
        pieces.append("%d-%d%s" % (start_column, end_column, "*" if keep else ""))
    return " ".join(pieces)


def format_center(center_column):
    return "---" if center_column < 0 else "%d" % center_column


def format_pattern(pattern):
    """九行的单线标记 -> 9 个字符，左起第 1 个 = 最近的行（SCAN_ROWS 由近到远排）。

        #  两条都在（最健康）。用 # 而不是点，是为了竖着一眼数得出几个
        L  只剩【左】边那条线
        R  只剩【右】边那条线
        x  这一行没结果（没线 / 配不上 / 落在死区里）

    例：#########  直道；####LLLLL  弯要来了；##xxxLLxx  一团糟

    弯道里内圈那条先拐出画面，剩下的总是外侧那条。所以：
        只剩左线（L） -> 往【右】拐
        只剩右线（R） -> 往【左】拐
    """
    pieces = []
    for marker in pattern:
        if marker == 0:
            pieces.append("#")
        elif marker == 1:
            pieces.append("L")
        elif marker == 2:
            pieces.append("R")
        else:
            pieces.append("x")
    return "".join(pieces)


RECORD_SINK = None      # 板子上会把它设成"写文件"的函数；电脑上跑自检时一直是 None


def _emit(text):
    """打印一行，同时（如果板子上开了记录）写进文件。

    为什么要这个出口：现场只有一个人，推车的时候没法看屏幕、更没法拍视频。
    让板子自己把每一行写进存储卡，跑完把卡拿出来在电脑上看，一行都不丢。
    """
    print(text)
    sink = RECORD_SINK
    if sink is not None:
        sink(text)


def print_block(sample_number, result, elapsed_ms, fps, scan_ms):
    """打印一块：块头 + 9 行 + 3 行汇总。"""
    (rows_info, raw_error, far_raw_error, found_row_count, far_found_row_count,
     corner_signal, corner_row, car_state, sent_error, note, pattern) = result

    _emit("===== 采样#%d  组=%s  运行 %.1f 秒  fps %.1f  用时 %.1f ms ====="
          % (sample_number, GROUP_NAME, elapsed_ms / 1000.0, fps, scan_ms))
    for row_index in range(SCAN_ROW_COUNT):
        (center_column, single, line_count, measured,
         dark, darkest, brightest, runs) = rows_info[row_index]
        half_note = ""
        if measured > 0:
            # 只要这一行找到了两条线就报实测半宽，不管配对有没有通过 ——
            # 标定摄像头参数时全靠这个数，配对失败的时候更需要它。
            half_note = " | 实%d 理%d" % (measured, ROW_HALF_LANE_WIDTH_PX[row_index])
        _emit("行%3d 前%5.1f 半%3d | 暗%3d 亮%3d 阈%3d | 段 %s | 中%4s | %s%s%s"
              % (SCAN_ROWS[row_index], horizontal_distance_cm(SCAN_ROWS[row_index]),
                 ROW_HALF_LANE_WIDTH_PX[row_index], darkest, brightest,
                 ROW_THRESHOLD[row_index],
                 format_runs(runs), format_center(center_column),
                 SINGLE_NAME[single], half_note,
                 ("  宽块%d" % ROW_WIDE_COUNT[row_index]) if ROW_WIDE_COUNT[row_index] else ""))
    _emit("图案 %s  (左起第1个=最近的行  # 两条都在  L 只剩左线  R 只剩右线  x 没结果)"
          % format_pattern(pattern))
    _emit("偏差 %+d | 远偏差 %+d | 状态 %d | 找到 %d/%d"
          % (to_sent_scale(raw_error), to_sent_scale(far_raw_error),
             car_state, found_row_count, SCAN_ROW_COUNT))
    tail_text = {NOTE_NONE: "",
                 NOTE_CORNER: "   <<< 方向接管（顶到 %d）" % SM_BIAS_PX,
                 NOTE_SLOW: "   <<< 只降速，方向没动",
                 NOTE_LOST_KEEP: "   <<< 丢线，沿用上次",
                 NOTE_LOST_BLIND: "   <<< 丢线超时，盲开（最慢档，方向沿用上次）",
                 NOTE_LATE: "   <<< 弯中一直偏右没回中，怀疑来不及（考虑降速）"}[note]
    _emit("实际发出 %+d%s" % (sent_error, tail_text))
    _emit("")


# ================== 开机：摄像头 ==================
import sensor
import time

# ================== 发给 STM32（只有实车测试才打开）==================
# 协议和正式程序 main.py 完全一样：0xAA | 误差(int8) | 状态(uint8) | 0x55
# 【重要】打开后，只要电机电源是通的，车就会自己跑。
#   阶段一（架空试方向）必须架空再做；阶段二三才放到地上。
#   做纯手推测试时改回 False，否则一只手推车、一只手躲车，会出事。
SEND_UART = True          # 2026-11-20 上午按用户要求打开，准备实车
UART_TX_PIN = 24          # K210 IO24 → STM32 的接收脚（和 config.py 一致）
UART_RX_PIN = 25
UART_BAUD = 115200

uart = None
if SEND_UART:
    from machine import UART
    from fpioa_manager import fm
    fm.register(UART_TX_PIN, fm.fpioa.UART1_TX, force=True)
    fm.register(UART_RX_PIN, fm.fpioa.UART1_RX, force=True)
    uart = UART(UART.UART1, UART_BAUD)
    print("串口已打开：IO%d → STM32，%d 波特率 —— 电机可能会转，手离开" % (UART_TX_PIN, UART_BAUD))
else:
    print("串口关闭（手推测试模式）：只打印，不发 STM32，电机不会动")

sensor.reset(dual_buff=True)               # 双缓冲：处理的那 20 多毫秒里 DMA 还能接着拍下一帧
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)          # 320x240，空转实测 38.46 帧/秒
sensor.skip_frames(time=2000)
# ---- 锁死曝光和增益（2026-11-20）----
# 为什么：相机默认是【自动曝光】的。换个场地，亮度基准一变，
# 所有按亮度算出来的分界线跟着偏 —— 这就是「参数一换场地就得重调」的根子。
# 做法：开机先让它自己适应这 2 秒（这 2 秒本来就有，不多等），
# 然后把它定下来的曝光值读出来、锁死。跑的时候不再变。
# 跑起来之后不受阴影、顶灯影响，换场地也不用改参数。
LOCK_EXPOSURE = True
if LOCK_EXPOSURE:
    try:
        _exp_us = sensor.get_exposure_us()
        _gain_db = sensor.get_gain_db()
        sensor.set_auto_exposure(False, exposure_us=_exp_us)
        sensor.set_auto_gain(False, gain_db=_gain_db)
        print("曝光已锁死：%d 微秒，增益 %.1f dB" % (_exp_us, _gain_db))
    except Exception as _e:
        print("曝光锁死失败（%s），继续用自动曝光" % _e)
_probe = sensor.snapshot()                 # 先拍一张，确认真的是 320x240 而不是被降回去了
ACTUAL_WIDTH = _probe.width()
ACTUAL_HEIGHT = _probe.height()

print("################ 扫线记录 v3（320x240，9 行）################")
print("组名：%s" % GROUP_NAME)
print("画面 %dx%d 灰度（实际拿到 %dx%d）   SCALE=%.2f   焦距 %.1f 像素"
      % (IMAGE_WIDTH, IMAGE_HEIGHT, ACTUAL_WIDTH, ACTUAL_HEIGHT,
         SCALE, FOCAL_LENGTH_PX))
print("镜头：高 %.1f 厘米，低头 %.1f 度，水平视场 %.1f 度"
      % (CAMERA_HEIGHT_CM, CAMERA_PITCH_DEG, CAMERA_HFOV_DEG))
print("扫描列 %d~%d（每行 %d 个像素）"
      % (SCAN_COLUMN_FIRST, SCAN_COLUMN_LAST, SCAN_COLUMN_LAST - SCAN_COLUMN_FIRST + 1))
print("认线：对比度下限 %d，分界线 = 最暗 + (上界-最暗) x %.2f，最宽 = 线宽 x %.1f"
      % (CONTRAST_MIN_LEVEL, BLACK_THRESHOLD_RATIO, BLACK_RUN_MAX_FACTOR))
print("上界取法：%s | 宽块作废：%s | 兜底边界保护：%d 列"
      % (("去掉最亮 %d%% 之后的亮度" % (100 - TRIM_BRIGHT_PERCENT)) if USE_TRIM_BRIGHTEST
         else "直接用最亮",
         ("开（只剩单线时）" if REJECT_ONLY_WHEN_SINGLE else "开（不看单线双线）")
         if REJECT_ROW_WITH_WIDE_RUN else "关",
         BORDER_GUARD_PX))
print("半宽自适应：%s" % ("开" if ADAPT_HALF_WIDTH else "关（只报告，不让理论值被带跑）"))
print("扫描行（近 -> 远）：")
for i in range(SCAN_ROW_COUNT):
    use = "算偏差" if i < NEAR_ROW_COUNT else "判状态"
    weight = NEAR_WEIGHT[i] if i < NEAR_ROW_COUNT else FAR_WEIGHT[i - NEAR_ROW_COUNT]
    print("  行 %3d | 车前 %6.1f 厘米 | 半宽 %3d | 线宽 %4.1f | 段长 %2d~%2d | 权重 %d | %s"
          % (SCAN_ROWS[i], horizontal_distance_cm(SCAN_ROWS[i]),
             ROW_HALF_LANE_WIDTH_PX[i], ROW_LINE_WIDTH_PX[i],
             ROW_MIN_RUN_PX[i], ROW_MAX_RUN_PX[i], weight, use))
print("判档：远偏差 %d / %d 像素；弯道顶 %d 像素，触发条件 偏差 < %d（都是 160 尺度）"
      % (CURVE_ERROR_MEDIUM_PX, CURVE_ERROR_LARGE_PX,
         CORNER_TURN_BIAS_PX, CORNER_TRIGGER_BELOW_PX))
print("弯道状态机：%s" % (
    "开（丢 %d 行降一档，丢 %d 行接管方向、顶 %d，走满 %.1f 米强退；坏行里至少 %d 个 L 才算真弯）"
    % (SM_TAIL_WARN, SM_TAIL_CORNER, SM_BIAS_PX, SM_FUSE_M, SM_MIN_SINGLE) if USE_CORNER_SM
    else "关（用老的远偏差判档）"))
print("分两个门槛：丢 %d 行 → 降速（20），丢 %d 行且连续 %d 帧 → 才打方向（顶 %d）"
      % (SM_TAIL_CORNER, SM_DIR_LOST, SM_DIR_CONFIRM, SM_BIAS_PX))
print("并段：能填的洞 ≤ 这一行线宽的 %.0f%%（最少 %d 像素）" % (MERGE_GAP_RATIO * 100, MERGE_GAP_FLOOR))
print("每 %d 毫秒打印一块。截屏时板子不要动。" % PRINT_EVERY_MS)
print("图案日志：%s" % ("开 —— 九行图案一变就打一行（采弯道必须开）" if PATTERN_LOG
                    else "关 —— 只打原来的整块"))
print("图案读法：左起第1个=最近的行  # 两条都在  L 只剩左线  R 只剩右线  x 没结果")
print("日志字段：好行 = 从最近那行起连续几个「两条都在」")
print("          偏差 = 近 4 行加权后的偏差（偏差 = 均值 − 中心列）")
print("          状态 = 速度档：0 快(30) / 1 中(20) / 2 慢(15)")
print("          发出 = 真正发给 STM32 的那个数")
print("          已跑 = 从上电到这一行的秒数，用来对位置")
print("##########################################################")
print("")

# ================== 记录到文件（一个人在现场专用）==================
# 现场只有一个人：推车的时候没法看屏幕，更没法拍视频。
# 让板子自己把每一行写进存储卡，跑完把卡取出来在电脑上看，一行都不丢。
RECORD_TO_FILE = True
RECORD_FILE_NAME = "run_log.txt"

record_fh = None
record_path = ""

if RECORD_TO_FILE:
    for _d in ("/sd", "/flash"):
        try:
            record_path = _d + "/" + RECORD_FILE_NAME
            record_fh = open(record_path, "a")
            print("记录到文件：%s" % record_path)
            print("  跑完把卡取出来，在电脑上打开这个文件 = 全部记录，不用拍视频。")
            break
        except Exception as _e:
            record_fh = None
            print("记录：%s 写不进去（%s）" % (_d, _e))
    if record_fh is None:
        print("记录：没有可写的存储卡，只能看屏幕 —— 跑完别让它滚走。")


def record(line):
    """把一行写进文件。写失败不影响主循环。"""
    if record_fh is None:
        return
    try:
        record_fh.write(line + "\n")
        record_fh.flush()
    except Exception:
        pass


RECORD_SINK = record


if record_fh is not None:
    record("")
    record("########## 新的一次运行 ##########")
    record("画面 %dx%d | 镜头 高%.1f 低头%.1f 视场%.1f | 焦距 %.1f 像素"
           % (IMAGE_WIDTH, IMAGE_HEIGHT, CAMERA_HEIGHT_CM, CAMERA_PITCH_DEG,
              CAMERA_HFOV_DEG, FOCAL_LENGTH_PX))
    record("去高光 %s(%d%%) | 宽块作废 %s | 兜底边界保护 %d 列"
           % ("开" if USE_TRIM_BRIGHTEST else "关", 100 - TRIM_BRIGHT_PERCENT,
              "开" if REJECT_ROW_WITH_WIDE_RUN else "关", BORDER_GUARD_PX))
    record("扫描行（近->远）：%s" % " ".join([str(r) for r in SCAN_ROWS]))

last_print_ms = time.ticks_ms()
last_pattern_ms = 0                    # 上一行图案日志是什么时候打的（限速用）
sample_number = 0
frames_in_window = 0
scan_us_sum = 0
last_pattern_str = ""                  # 上一帧的九行图案，用来判断"变了没有"

while True:
    image = sensor.snapshot()

    sm_now_ms = time.ticks_ms()         # 状态机靠它判「进弯多久了」
    t_start = time.ticks_us()
    result = scan_frame(image)
    t_end = time.ticks_us()
    scan_us = t_end - t_start
    if scan_us < 0:                     # 计时器回绕
        scan_us += 1073741824
    scan_us_sum += scan_us
    frames_in_window += 1

    # ---- 发给 STM32（实车测试才开）----
    # result[8] = 实际发出的误差（160 尺度），result[7] = 速度档 0/1/2
    # 两个数的含义和正式程序 main.py 里完全一致，STM32 那边不用改。
    if uart is not None:
        # ---- 弯道放大转向 ----
        # 直道上 Kp=0.1 刚好（不画龙），但弯里偏差只有 5~8，
        # 0.1 × 8 = 0.8 个计数，基速 15 下几乎是 0，车根本不拐。
        # 原来靠 SM_BIAS_PX=30 硬顶，但它不分场合，直道上一点偏差也顶 → 画龙。
        # 所以改成：只在【状态 2（真的进弯了）】把偏差放大 4 倍再发，
        # 相当于弯里 Kp 等效 0.4，直道仍然 0.1。
        _err = result[8]
        # 2026-11-20 下午：默认【不再】改发固定值。
        # 原因：STM32 现在转弯期间根本不看偏差（它按编码器角度转），
        # 那个假的 50 已经没用了；留着反而堵死了"转完之后用视觉修航向"这条路。
        # 想回到老行为，把 SEND_CORNER_FIXED 改回 True 就行。
        if SEND_CORNER_FIXED and result[7] == 2:
            _err = CORNER_ERR_FIXED
        uart.write(bytes((0xAA, _err & 0xFF, result[7] & 0xFF, 0x55)))

    now_ms = time.ticks_ms()

    # ---- 九行图案一变就打一行（有最小间隔，防止图案抖动时刷屏拖垮帧率）----
    # 弯道是个"过程"（远端先丢、一行一行往里推进），2.5 秒打一块会整个漏掉。
    if PATTERN_LOG:
        pattern_str = format_pattern(result[10])
        if (pattern_str != last_pattern_str
                and (now_ms - last_pattern_ms) >= PATTERN_LOG_MIN_MS):
            _wide = 0
            for _w in ROW_WIDE_COUNT:
                _wide += _w
            # 加了「最远行阈」和「宽块合计」两个字段：
            # 前者要落在地面亮度以下（否则地面被整片吃掉），后者应该基本都是 0。
            _emit("[已跑 %5.1f 秒] 图案=%s 好行=%d 偏差=%+d 状态=%d 发出=%+d | 最远行阈%3d 宽块合计%d"
                  % (now_ms / 1000.0, pattern_str, good_rows(result[10]),
                     to_sent_scale(result[1]), result[7], result[8],
                     ROW_THRESHOLD[SCAN_ROW_COUNT - 1], _wide))
            last_pattern_ms = now_ms
            last_pattern_str = pattern_str

    if now_ms < last_print_ms:          # 计时器回绕，重新起算
        last_print_ms = now_ms
    elapsed = now_ms - last_print_ms
    if elapsed >= PRINT_EVERY_MS:
        sample_number += 1
        scan_ms = scan_us_sum / 1000.0 / frames_in_window if frames_in_window else 0.0
        print_block(sample_number, result, now_ms,
                    frames_in_window * 1000.0 / elapsed, scan_ms)
        last_print_ms = now_ms
        frames_in_window = 0
        scan_us_sum = 0
