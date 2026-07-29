/*
 *  angle_servo_bridge.h  —— 陀螺仪 → 舵机 PID 闭环桥接
 *  ============================================================
 *
 *  【控制模型】
 *    设定目标偏移 (delta)，IMU 作为"标尺"验证舵机是否真正转到目标：
 *      期望IMU角 = 基准IMU角 + 目标偏移
 *      误差      = 期望IMU角 − 当前IMU角
 *      舵机角    = 基准舵机角 + 目标偏移 + PID微调修正
 *
 *  【后续扩展】
 *    - 菜单可直接读写 g_servoPidCfg 结构体修改 PID 参数
 *    - 菜单可调用 AngleServo_SetTargetDelta() 修改目标偏移
 */

#ifndef __ANGLE_SERVO_BRIDGE_H
#define __ANGLE_SERVO_BRIDGE_H

#include <stdint.h>
#include "param_storage.h"   /* ModeParams 类型定义 */

/* ================================================================
 *  PID 参数结构体（菜单直接读写，不用 #define）
 * ================================================================ */
typedef struct {
    float kp;         /* 比例系数          默认 2.0  */
    float ki;         /* 积分系数          默认 0.1  */
    float kd;         /* 微分系数          默认 0.05 */
    float outMax;     /* PID 修正量上限 (°) 默认 5.0  */
    float iMax;       /* 积分饱和限        默认 3.0  */
    float deadband;   /* 死区 (°)          默认 0.3  */
} ServoAnglePidCfg;

/* ================================================================
 *  全局变量（菜单直接读写）
 * ================================================================ */
extern ServoAnglePidCfg g_servoPidCfg;   /* PID 参数 */
extern float           g_servoTargetDelta; /* 目标偏移 (°) */
extern float           g_servoBaseAngle;   /* 舵机基准角 (默认 90°) */

/* ================================================================
 *  API 函数
 * ================================================================ */

/* 上电初始化：复位 PID 状态 */
void AngleServo_Init(void);

/*
 * 每 5ms 调用一次（在 TIMER_0 ISR 的 5ms 槽中）。
 * imuAngleDeg: 陀螺仪当前角度 (euler.angle.pitch)
 */
void AngleServo_Update(float imuAngleDeg);

/*
 * 运行时修改目标偏移。
 * 调用时会锁存当前 IMU 角作为新基准，重置 PID 积分。
 */
void AngleServo_SetTargetDelta(float deltaDeg);

/* 查询当前目标偏移 */
float AngleServo_GetTargetDelta(void);

/* 停止 PID，舵机回到基准角度 */
void AngleServo_Stop(void);

/* 查询 PID 是否处于激活状态 */
uint8_t AngleServo_IsActive(void);

/*
 * 将 ModeParams 中的舵机 PID 参数同步到运行时结构体 g_servoPidCfg。
 * 由 Param_ApplyMode() 在切换模式时调用。
 */
void AngleServo_ApplyParams(const ModeParams *p);

#endif /* __ANGLE_SERVO_BRIDGE_H */
