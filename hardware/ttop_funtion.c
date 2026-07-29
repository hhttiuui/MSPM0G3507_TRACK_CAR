/*
 * ttop_funtion.c — 模式执行模块
 *
 * 四个模式函数当前为空壳，后续填充具体逻辑。
 * ModeExecTask() 在 TIMER_0 ISR 的槽 3 中每 5ms 调用一次。
 */

#include "ttop_funtion.h"
#include "menu.h"
#include "speed_control.h"
#include "heading_control.h"
#include "track.h"
#include "param_storage.h"
#include "encoder.h"
#include "IMU660RB/imu660rb.h"
#include "buzzer.h"

/* 循迹传感器数据（gpio_toggle_output.c 中定义） */
extern volatile uint8_t gRxByte;

/* ---- 内部状态 ---- */
static uint8_t s_lastStartFlag = 0U;
static uint8_t s_running       = 0U;  /* 是否处于运行状态 */

void Mode1_Task(void)
{
    /* 10ms 分频计数 */
    static uint8_t s_divider = 0U;

    /* 首次调用时初始化 */
    if (!SpeedControl_IsRunning()) {
        int16_t baseSpd = g_modeParams[g_menuMode - 1U].trackBaseSpeed;
        if (!SpeedControl_SetTargets(baseSpd, baseSpd)) {
            return;
        }
        if (!SpeedControl_Start()) {
            return;
        }
        s_divider = 0U;
        return;
    }

    /* ── 定距停止检测 ── */
    {
        int16_t stopDist = g_modeParams[g_menuMode - 1U].stopDistance;
        int32_t totalM3  = Encoder_GetTotalPulses(&gEncMotor3);
        if (stopDist > 0 && totalM3 >= stopDist) {
            SpeedControl_Stop();
            g_menuStartFlag = 0U;
            return;
        }
    }

    /* 每 2 次调用（10ms）执行一次循迹 PID */
    s_divider++;
    if (s_divider < 2U) {
        return;
    }
    s_divider = 0U;

    {
        int16_t baseSpd = g_modeParams[g_menuMode - 1U].trackBaseSpeed;
        float offset = Track_CalcOffset(gRxByte);
        float diff   = Track_PidUpdate(offset, 0.01f);
        int16_t left  = baseSpd + (int16_t)diff;
        int16_t right = baseSpd - (int16_t)diff;

        if (left  < 0) left  = 0;
        if (right < 0) right = 0;

        SpeedControl_SetTargets(left, right);
    }
}

void Mode2_Task(void)
{
    /* 10ms 分频计数 */
    static uint8_t s_divider = 0U;

    /* 首次调用时初始化 */
    if (!SpeedControl_IsRunning()) {
        int16_t baseSpd = g_modeParams[g_menuMode - 1U].trackBaseSpeed;
        if (!SpeedControl_SetTargets(baseSpd, baseSpd)) {
            return;
        }
        if (!SpeedControl_Start()) {
            return;
        }
        s_divider = 0U;
        return;
    }

    /* ── 定距停止检测 ── */
    {
        int16_t stopDist = g_modeParams[g_menuMode - 1U].stopDistance;
        int32_t totalM3  = Encoder_GetTotalPulses(&gEncMotor3);
        if (stopDist > 0 && totalM3 >= stopDist) {
            SpeedControl_Stop();
            g_menuStartFlag = 0U;
            return;
        }
    }

    /* 每 2 次调用（10ms）执行一次循迹 PID */
    s_divider++;
    if (s_divider < 2U) {
        return;
    }
    s_divider = 0U;

    {
        int16_t baseSpd = g_modeParams[g_menuMode - 1U].trackBaseSpeed;
        float offset = Track_CalcOffset(gRxByte);
        float diff   = Track_PidUpdate(offset, 0.01f);
        int16_t left  = baseSpd + (int16_t)diff;
        int16_t right = baseSpd - (int16_t)diff;

        if (left  < 0) left  = 0;
        if (right < 0) right = 0;

        SpeedControl_SetTargets(left, right);
    }
}

void Mode3_Task(void)
{
    /* TODO: MODE3 具体逻辑 */
}

void Mode4_Task(void)
{
    /* TODO: MODE4 具体逻辑 */
}

/*
 * 在 TIMER_0 ISR 槽 3 中每 5ms 调用一次。
 * 处理 RESET（急停）和 START（启动）标志位。
 */
void ModeExecTask(void)
{
    /* ---- RESET 优先：立即急停 + 陀螺仪复位 + 编码器清零 ---- */
    if (g_menuResetFlag) {
        HeadingControl_Stop();
        SpeedControl_Stop();
        FusionAhrsReset(&ahrs);
        gEncMotor1.totalPulses = 0;
        gEncMotor3.totalPulses = 0;
        g_menuStartFlag = 0U;
        g_menuResetFlag = 0U;
        s_running       = 0U;
        return;
    }

    /* ---- START 上升沿：应用参数并启动 ---- */
    if (g_menuStartFlag && !s_lastStartFlag) {
        if (g_menuMode >= 1U && g_menuMode <= 4U) {
            Param_ApplyMode(g_menuMode);
            s_running = 1U;
        }
    }
    s_lastStartFlag = g_menuStartFlag;

    /* ---- 停止则返回 ---- */
    if (!g_menuStartFlag || !s_running) {
        return;
    }

    /* ---- 按模式分发 ---- */
    switch (g_menuMode) {
        case 1U: Mode1_Task(); break;
        case 2U: Mode2_Task(); break;
        case 3U: Mode3_Task(); break;
        case 4U: Mode4_Task(); break;
        default: break;
    }
}
