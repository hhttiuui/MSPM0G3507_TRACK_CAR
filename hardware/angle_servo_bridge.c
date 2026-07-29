/*
 *  angle_servo_bridge.c  —— 陀螺仪 → 舵机 PID 闭环实现
 *  ============================================================
 *
 *  【控制模型】
 *    IMU 作为"标尺"验证舵机定位精度：
 *      基准IMU角 : 设定目标时锁存的当前 IMU 角度
 *      期望IMU角 = 基准IMU角 + 目标偏移 (g_servoTargetDelta)
 *      当前IMU角 : 陀螺仪实时读数
 *      误差      = 期望IMU角 − 当前IMU角
 *      PID输出   = Kp×误差 + Ki×∫误差dt + Kd×d误差/dt
 *      舵机角度  = 基准舵机角 + 目标偏移 + PID输出
 *
 *  【调用周期】 5ms（与 IMU 读数同频）
 *  【积分时间】 dt = 0.005s
 */

#include "angle_servo_bridge.h"
#include "servo.h"
#include "param_storage.h"
#include "ti_msp_dl_config.h"

/* ---- 控制周期 (秒) ---- */
#define PID_DT  (0.005f)

/* ---- 绝对值 ---- */
static float AbsF(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* ---- 钳位 ---- */
static float ClampF(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* ================================================================
 *  全局变量
 * ================================================================ */
ServoAnglePidCfg g_servoPidCfg = {
    .kp       = 2.0f,
    .ki       = 0.1f,
    .kd       = 0.05f,
    .outMax   = 5.0f,
    .iMax     = 3.0f,
    .deadband = 0.3f,
};

float g_servoTargetDelta = 3.0f;   /* 默认目标偏移 +3° */
float g_servoBaseAngle   = 90.0f;  /* 舵机机械中位 */

/* ---- 内部 PID 状态 ---- */
static float  s_refImuDeg;      /* 基准 IMU 角（设定目标时锁存） */
static float  s_expImuDeg;      /* 期望 IMU 角 */
static float  s_integral;       /* 积分累加 */
static float  s_lastError;      /* 上一次误差（用于微分） */
static uint8_t s_active;        /* PID 是否激活 */
static uint8_t s_firstUpdate;   /* 首次更新标志 */

/* ================================================================
 *  AngleServo_Init
 * ================================================================ */
void AngleServo_Init(void)
{
    s_integral    = 0.0f;
    s_lastError   = 0.0f;
    s_active      = 0U;
    s_firstUpdate = 1U;
    s_refImuDeg   = 0.0f;
    s_expImuDeg   = 0.0f;

    /* 舵机停在中位 */
    Servo_SetAngle((uint8_t)g_servoBaseAngle);
}

/* ================================================================
 *  AngleServo_SetTargetDelta
 * ================================================================ */
void AngleServo_SetTargetDelta(float deltaDeg)
{
    g_servoTargetDelta = deltaDeg;

    /*
     * 不在这里锁存 IMU 基准 — 下次 Update 时才锁存。
     * 这样 SetTarget 和 Update 调用顺序无关。
     */
    s_firstUpdate = 1U;
    s_integral    = 0.0f;
    s_lastError   = 0.0f;
    s_active      = 1U;
}

/* ================================================================
 *  AngleServo_GetTargetDelta
 * ================================================================ */
float AngleServo_GetTargetDelta(void)
{
    return g_servoTargetDelta;
}

/* ================================================================
 *  AngleServo_Stop
 * ================================================================ */
void AngleServo_Stop(void)
{
    s_active = 0U;
    Servo_SetAngle((uint8_t)g_servoBaseAngle);
}

/* ================================================================
 *  AngleServo_IsActive
 * ================================================================ */
uint8_t AngleServo_IsActive(void)
{
    return s_active;
}

/* ================================================================
 *  AngleServo_Update — 核心 PID 计算（每 5ms）
 * ================================================================ */
void AngleServo_Update(float imuAngleDeg)
{
    float error, pOut, iOut, dOut, pidOut;
    float servoAngle;

    if (!s_active) {
        return;  /* PID 未激活，不动作 */
    }

    /* 首次更新：锁存当前 IMU 角作为基准 */
    if (s_firstUpdate) {
        s_refImuDeg   = imuAngleDeg;
        s_expImuDeg   = s_refImuDeg + g_servoTargetDelta;
        s_integral    = 0.0f;
        s_lastError   = 0.0f;
        s_firstUpdate = 0U;

        /* 先开环驱动到目标位置（基准 + 目标偏移） */
        servoAngle = g_servoBaseAngle + g_servoTargetDelta;
        servoAngle = ClampF(servoAngle, 0.0f, 180.0f);
        Servo_SetAngle((uint8_t)servoAngle);
        return;
    }

    /* ---- 误差 = 期望IMU − 当前IMU ---- */
    error = s_expImuDeg - imuAngleDeg;

    /* ---- 死区：误差太小不动作 ---- */
    if (AbsF(error) < g_servoPidCfg.deadband) {
        return;
    }

    /* ---- P: 比例项 ---- */
    pOut = g_servoPidCfg.kp * error;

    /* ---- I: 积分项 + 抗饱和 ---- */
    s_integral += g_servoPidCfg.ki * error * PID_DT;
    s_integral  = ClampF(s_integral,
                     -g_servoPidCfg.iMax, g_servoPidCfg.iMax);
    iOut = s_integral;

    /* ---- D: 微分项 (误差变化率) ---- */
    dOut = g_servoPidCfg.kd * (error - s_lastError) / PID_DT;
    s_lastError = error;

    /* ---- 总输出，钳位 ---- */
    pidOut = pOut + iOut + dOut;
    pidOut = ClampF(pidOut,
                -g_servoPidCfg.outMax, g_servoPidCfg.outMax);

    /* ---- 舵机角度 = 基准 + 目标偏移 + PID微调 ---- */
    servoAngle = g_servoBaseAngle + g_servoTargetDelta + pidOut;
    servoAngle = ClampF(servoAngle, 0.0f, 180.0f);

    Servo_SetAngle((uint8_t)servoAngle);
}

/* ================================================================
 *  AngleServo_ApplyParams — 从 ModeParams 同步到运行时
 * ================================================================
 *
 *  由 Param_ApplyMode() 调用。将 EEPROM/菜单中的参数写入
 *  运行时结构体 g_servoPidCfg 和 g_servoBaseAngle。
 */
void AngleServo_ApplyParams(const ModeParams *p)
{
    g_servoPidCfg.kp       = p->servoKp;
    g_servoPidCfg.ki       = p->servoKi;
    g_servoPidCfg.kd       = p->servoKd;
    g_servoPidCfg.outMax   = p->servoOutMax;
    g_servoPidCfg.iMax     = p->servoIMax;
    g_servoPidCfg.deadband = p->servoDeadband;
    g_servoBaseAngle       = p->servoBaseAngle;
}
