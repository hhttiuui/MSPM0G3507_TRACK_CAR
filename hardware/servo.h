/*
 *  servo.h  —— 舵机 PWM 驱动（TIMG7 CCP0, PA28）
 *
 *  参数：
 *    频率: 300Hz (周期 3333μs)
 *    脉宽范围: 1000μs ~ 2000μs
 *    中点: 1500μs (对应 90°)
 *
 *  时钟配置：
 *    TIMG7, BUSCLK=32MHz, 预分频=8 → 4MHz tick
 *    周期 = 13333 ticks, 脉宽 4000~8000 ticks
 */

#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/* ---- 角度范围 ---- */
#define SERVO_ANGLE_MIN     0
#define SERVO_ANGLE_MAX     180
#define SERVO_ANGLE_CENTER  90

/* ---- API ---- */

/* 初始化（SYSCFG_DL_PWM_SERVO_init 已自动设置定时器，此处设置初始占空比） */
void Servo_Init(void);

/* 设置角度 0~180°，内部自动限幅，线性映射到 1000~2000μs */
void Servo_SetAngle(uint8_t angle);

/* 直接设置脉宽 tick 值（4000~8000，每个 tick=0.25μs），内部自动限幅 */
void Servo_SetPulse(uint16_t pulse);

/* 停止 PWM 输出（0% 占空比，舵机松弛） */
void Servo_Stop(void);

#endif /* __SERVO_H */
