/*
 *  servo.c  —— 舵机 PWM 驱动实现
 *
 *  TIMG7, PA28 (CCP0), 200Hz 舵机信号
 *
 *  预分频 = 8 → 每 tick = 250ns
 *  周期   = 20000 ticks → 5000μs = 200Hz
 *  脉宽范围: 4000 (1000μs) ~ 8000 (2000μs)
 *  中点:     6000 (1500μs)
 *
 *  时钟和周期由 SysConfig (gpio_toggle_output.syscfg) 统一管理，
 *  本模块不再重复配置定时器，避免与 SYSCFG_DL_PWM_SERVO_init() 冲突。
 */

#include "servo.h"
#include "ti_msp_dl_config.h"
#include "delay.h"

/* ---- 脉宽 tick 常量（预分频=8，每 tick=0.25μs）---- */
#define PULSE_MIN    4000U   /* 1000μs */
#define PULSE_MAX    8000U   /* 2000μs */
#define PULSE_CENTER 6000U   /* 1500μs */

/* ---- 公开 API ---- */

void Servo_Init(void)
{
    /*
     * SYSCFG_DL_PWM_SERVO_init() 已在 SYSCFG_DL_init() 中完成：
     *   - 引脚复用 (IOMUX): PA28 → TIMG7 CCP0
     *   - 时钟配置: 32MHz ÷ 8 = 4MHz
     *   - PWM 模式: 边沿对齐, 周期 20000 (200Hz)
     *   - 定时器已启动
     *
     * 这里只需确保上电初始状态安全，然后执行自检扫掠。
     */

    /* 上电自检扫掠: 0° → 90° → 180° → 90° → 0° */
    Servo_SetAngle(0);
    Delay_ms(1000);
    Servo_SetAngle(90);
    Delay_ms(1000);
    Servo_SetAngle(180);
    Delay_ms(1000);
    Servo_SetAngle(90);
    Delay_ms(1000);
    Servo_SetAngle(0);
}

void Servo_SetAngle(uint8_t angle)
{
    uint16_t pulse;

    if (angle > 180U) {
        angle = 180U;
    }

    /* 线性映射: angle [0, 180] → pulse [4000, 8000] */
    pulse = PULSE_MIN + (uint16_t)(((uint32_t)angle * (PULSE_MAX - PULSE_MIN)) / 180U);

    Servo_SetPulse(pulse);
}

void Servo_SetPulse(uint16_t pulse)
{
    if (pulse < PULSE_MIN) {
        pulse = PULSE_MIN;
    }
    if (pulse > PULSE_MAX) {
        pulse = PULSE_MAX;
    }

    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST, (uint32_t)pulse, GPIO_PWM_SERVO_C0_IDX);
}

void Servo_Stop(void)
{
    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST, 0, GPIO_PWM_SERVO_C0_IDX);
}
