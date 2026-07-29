/*
 *  servo.h  —— 舵机 PWM 驱动（TIMG7 CCP0, PA28）
 *  ============================================================
 *
 *  【硬件】
 *    定时器: TIMG7      引脚: PA28 (CCP0)
 *    时钟:   32MHz ÷ 8 = 4MHz (每 tick = 0.25μs)
 *
 *  【PWM 信号】
 *    频率:   200Hz (周期 5000μs = 20000 ticks)
 *
 *  【舵机角度 → 脉宽映射 (180° 舵机)】
 *      0° →  500μs (2000 ticks)
 *     90° → 1500μs (6000 ticks)  ← 中点
 *    180° → 2500μs (10000 ticks)
 *
 *  【SysConfig 配置 (gpio_toggle_output.syscfg)】
 *    PWM_SERVO: clockPrescale=8, timerCount=20000, CCP0=PA28
 */

#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/* ---- 角度常量 ---- */
#define SERVO_ANGLE_MIN     0    /* 最小角度 */
#define SERVO_ANGLE_MAX     180  /* 最大角度 */
#define SERVO_ANGLE_CENTER  90   /* 中点角度 */

/* ================================================================
 *  API 函数
 * ================================================================ */

/*
 * 上电初始化，执行自检扫掠: 0°→90°→180°→90°→0°
 * 调用时机: main() 中 SYSCFG_DL_init() 之后
 * 注意: 此函数阻塞约 4 秒
 */
void Servo_Init(void);

/*
 * 设置舵机角度
 *   angle: 0~180°，超限自动钳位
 *   内部线性映射: angle → 500~2500μs 脉宽
 */
void Servo_SetAngle(uint8_t angle);

/*
 * 直接设置脉宽（高级接口）
 *   pulse: 2000~10000 (单位: 0.25μs tick)，超限自动钳位
 *   2000=500μs, 6000=1500μs, 10000=2500μs
 */
void Servo_SetPulse(uint16_t pulse);

/*
 * 紧急停止: PWM 输出常低，舵机完全松弛
 */
void Servo_Stop(void);

#endif /* __SERVO_H */
