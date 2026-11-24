# -*- coding: utf-8 -*-
"""
弯道状态机蒙特卡洛仿真（只读分析，不碰任何源码）
复现 app_control.c 的：视觉报弯 -> 里程门禁 -> TURN_ARM -> 转角判定 -> 终点兜底
目的：验证"标定正确 + 光照/里程/温度噪声"下能否稳定完赛，
      并把温度偏差做成可变量，量化"热干扰到什么程度才有影响"。

用法：python 仿真_弯道状态机蒙特卡洛.py [输出文件]
      传输出文件时，报告同时写入该文件（UTF-8），便于在非 UTF-8 终端查看。
"""

import random
import sys

# ===== 代码里的确定参数（来自 app_control.c）=====
THRESH   = [180.0, 325.0, 330.0]   # Turn_ThresholdCm
ANGLES   = [90.0, 103.0, 90.0]     # Turn_AngleDeg
ARM_CM   = 20.0                     # TURN_ARM_CM
EARLY    = 30.0                     # TURN_DIST_EARLY_CM
LATE     = 10.0                     # TURN_DIST_LATE_CM
BACKSTOP = 2400.0                   # TURN_COUNT_BACKSTOP
TIMEOUT  = 800.0                    # TURN_TIMEOUT_MS
FINISH   = 99.0                     # FINISH_STOP_CM
RATE_DPS = 240.0                    # 转弯角速度（度/秒）
DT_MS    = 5.0                      # 控制周期
DIFF_STEP = 23.1                    # 差速计数/5ms（(26.55-3.45)）

# ===== 噪声参数（用户选的"典型"档）=====
LIGHT_JITTER = 8.0   # 视觉报弯里程抖动 σ(cm)
P_MISS       = 0.03  # 视觉整弯漏报概率
ODO_SIG      = 0.01  # 里程读数随机误差比例 σ
TEMP_SIG     = 5.0   # 陀螺仪等效转角偏差 σ(度)——代表温度残留零偏在0.4s内的积分

STEP_CM = 0.6        # 仿真步长（厘米/步）


def sim_lap(temp_sig=TEMP_SIG, p_miss=P_MISS, light_jit=LIGHT_JITTER, odo_sig=ODO_SIG):
    """跑一圈，返回结果统计字典"""
    res = dict(completed=False, ok=0, miss=0, gate_reject=0,
               backstop=0, timeout=0, angle_errs=[], finish_real=None)
    seg = 0.0
    for i in range(3):
        T, A = THRESH[i], ANGLES[i]
        odo_err = random.gauss(0.0, odo_sig)          # 本段里程读数比例误差（标定正确=均值0）
        missed = random.random() < p_miss             # 视觉是否整弯漏报
        report_at = T - ARM_CM + random.gauss(0.0, light_jit)  # 报弯时的真实里程
        bias = random.gauss(0.0, temp_sig)            # 本弯温度等效角度偏差

        armed = active = done = False
        arm_at = 0.0
        real_ang = 0.0
        accum = 0.0
        active_ms = 0.0          # 【修复】转弯计时从 active 开始，不把报弯前的行进时间算进来
        outcome = None

        while not done:
            seg += STEP_CM
            read = seg * (1.0 + odo_err)
            state2 = (not missed) and (seg >= report_at)

            # 里程门禁：收到 state=2 且读数落在 [T-30, T+10] 才认弯
            if state2 and not armed and not active:
                if (T - EARLY) <= read <= (T + LATE):
                    armed = True
                    arm_at = seg
            # Armed 后再走 20cm 开转
            if armed and not active and (seg - arm_at) >= ARM_CM:
                active = True
            # Active：转角累积 + 三种结束条件
            if active:
                active_ms += DT_MS
                real_ang += RATE_DPS * DT_MS / 1000.0
                accum += DIFF_STEP
                gyro_read = real_ang + bias
                if gyro_read >= A:
                    done, outcome = True, 'ok'
                elif accum >= BACKSTOP:
                    done, outcome = True, 'backstop'
                elif active_ms > TIMEOUT:
                    done, outcome = True, 'timeout'
            # 没认弯且冲过弯口很远 -> 该弯错过
            if not done and seg > T + 120:
                done, outcome = True, ('miss' if missed else 'gate_reject')
            if seg > 1200:  # 死循环保护
                done, outcome = True, 'miss'

        if outcome == 'ok':
            res['ok'] += 1
            # 实际转过角度 = A - bias（读数提前/滞后导致少转/多转）
            res['angle_errs'].append(round(real_ang - A, 2))
        elif outcome in ('backstop', 'timeout'):
            res[outcome] += 1
            # 兜底结束时同样按实际转过角度统计
            res['angle_errs'].append(round(real_ang - A, 2))
        elif outcome == 'miss':
            res['miss'] += 1
        elif outcome == 'gate_reject':
            res['gate_reject'] += 1

        seg = 0.0  # 每个弯转完，段内里程清零

    # 终点兜底：只有三个弯都转完，Finish_Armed 才置位，才走 99cm 停车
    if len(res['angle_errs']) == 3:
        err = random.gauss(0.0, odo_sig)
        while seg * (1.0 + err) < FINISH:
            seg += STEP_CM
        res['finish_real'] = round(seg, 1)
        res['completed'] = True
    return res


def run(n=3000, **kw):
    return [sim_lap(**kw) for _ in range(n)]


def statistics_sd(xs):
    m = sum(xs) / len(xs)
    return (sum((x-m)**2 for x in xs) / len(xs)) ** 0.5


_LINES = []
def out(s=""):
    print(s)
    _LINES.append(s)


def report(name, laps):
    n = len(laps)
    completed = sum(l['completed'] for l in laps)
    ok = sum(l['ok'] for l in laps)
    miss = sum(l['miss'] for l in laps)
    gate = sum(l['gate_reject'] for l in laps)
    bs = sum(l['backstop'] for l in laps)
    to = sum(l['timeout'] for l in laps)
    errs = [e for l in laps for e in l['angle_errs']]
    fin = [l['finish_real'] for l in laps if l['finish_real'] is not None]
    out("=" * 56)
    out(name)
    out("  完赛率        : %5.1f%%  (%d/%d)" % (100.0*completed/n, completed, n))
    out("  转角达标(ok)  : %5.1f%%  (%d/%d 个弯)" % (100.0*ok/(n*3), ok, n*3))
    out("  漏报弯(无兜底): %5.1f%%  视觉3%%漏报且 TURN_ALLOW_FORCE=0" % (100.0*miss/(n*3)))
    out("  里程门禁误判  : %5.1f%%  真弯被判假弯" % (100.0*gate/(n*3)))
    out("  兜底触发      : backstop %d / timeout %d" % (bs, to))
    if errs:
        out("  转角误差      : 均值 %+.2f°  标准差 %.2f°  范围 [%+.1f, %+.1f]" %
            (sum(errs)/len(errs), statistics_sd(errs), min(errs), max(errs)))
    if fin:
        out("  终点停车里程  : 均值 %.1fcm  范围 [%.1f, %.1f]cm（目标99）" %
            (sum(fin)/len(fin), min(fin), max(fin)))


if __name__ == '__main__':
    random.seed(20261123)

    out("\n【基准】标定正确 + 典型噪声（光照±8cm/漏3%、里程±1%、温度±5°）")
    report("典型档", run(3000))

    out("\n【温度敏感性对比】只改温度偏差，其余固定——回答'热干扰是否严重'")
    report("温度 ±5°",  run(2000, temp_sig=5.0))
    report("温度 ±15°", run(2000, temp_sig=15.0))
    report("温度 ±30°", run(2000, temp_sig=30.0))

    out("\n【漏报救不救】模拟打开 TURN_ALLOW_FORCE（漏报弯靠里程强转）")
    report("假设零漏报", run(2000, p_miss=0.0))

    if len(sys.argv) > 1:
        with open(sys.argv[1], 'w', encoding='utf-8') as f:
            f.write("\n".join(_LINES) + "\n")
        out("\n[报告已写入] " + sys.argv[1])
