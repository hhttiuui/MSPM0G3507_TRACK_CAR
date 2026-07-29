/*
 *  servo.c  —— 舵机 PWM 驱动实现
 *  ============================================================
 *
 *  【硬件资源】
 *    定时器: TIMG7 (通用定时器 G7)
 *    引脚:   PA28 (CCP0 通道输出)
 *    时钟:   32MHz BUSCLK ÷ 8 预分频 = 4MHz
 *
 *  【PWM 参数】
 *    频率:   200Hz (周期 = 5000μs = 20000 ticks)
 *    精度:   每 tick = 0.25μs
 *
 *  【舵机脉宽映射 (180° 舵机)】
 *    PULSE_MIN    = 2000 ticks =  500μs →  0°
 *    PULSE_CENTER = 6000 ticks = 1500μs → 90°
 *    PULSE_MAX    =10000 ticks = 2500μs → 180°
 *
 *  【配置管理】
 *    定时器初始化由 SysConfig 生成的 SYSCFG_DL_PWM_SERVO_init()
 *    在 SYSCFG_DL_init() 中统一完成，本模块不重复配置，
 *    仅负责运行时的脉宽控制。
 *
 *  【安全设计】
 *    - 上电时 PWM 占空比为 0，舵机处于松弛状态
 *    - Servo_SetAngle() / Servo_SetPulse() 输入自动限幅
 *    - Servo_Stop() 可紧急切断 PWM 输出
 */

#include "servo.h"              /* 本模块头文件 */
#include "ti_msp_dl_config.h"   /* SysConfig 生成的硬件宏 (PWM_SERVO_INST 等) */

/* ================================================================
 *  脉宽常量（所有值单位: 0.25μs tick）
 * ================================================================ */
#define PULSE_MIN    2000U   /*  500μs — 舵机 0°  位置 */
#define PULSE_MAX    10000U  /* 2500μs — 舵机 180° 位置 */
#define PULSE_CENTER 6000U   /* 1500μs — 舵机 90° 中点 */


/* ================================================================
 *  Servo_Init — 上电初始化
 * ================================================================
 *
 *  调用时机: main() 中 SYSCFG_DL_init() 之后调用。
 *  SysConfig 已完成 TIMG7 的 200Hz PWM 初始化，
 *  本函数仅设置舵机到安全初始位置。
 */
void Servo_Init(void)
{
    /* 上电时舵机松弛，占空比 0，不施加力矩 */
    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST, 0, GPIO_PWM_SERVO_C0_IDX);
}


/* ================================================================
 *  Servo_SetAngle — 角度控制（主要用户接口）
 * ================================================================
 *
 *  参数:  angle — 0~180°，自动限幅
 *
 *  映射公式:
 *    pulse = PULSE_MIN + angle × (PULSE_MAX - PULSE_MIN) ÷ 180
 *          = 2000      + angle × 8000 ÷ 180
 *
 *  示例:
 *    0°  → pulse = 2000 ( 500μs)
 *    45° → pulse = 4000 (1000μs)
 *    90° → pulse = 6000 (1500μs)
 *    135°→ pulse = 8000 (2000μs)
 *    180°→ pulse =10000 (2500μs)
 */
void Servo_SetAngle(uint8_t angle)
{
    uint16_t pulse;

    /* 上限钳位: 超过 180° 按 180° 处理 */
    if (angle > 180U) {
        angle = 180U;
    }

    /* 线性插值: angle [0,180] → pulse [PULSE_MIN, PULSE_MAX] */
    pulse = PULSE_MIN
          + (uint16_t)(((uint32_t)angle * (PULSE_MAX - PULSE_MIN)) / 180U);

    /* 写入硬件比较寄存器 */
    Servo_SetPulse(pulse);
}


/* ================================================================
 *  Servo_SetPulse — 直接脉宽控制（高级接口）
 * ================================================================
 *
 *  参数:  pulse — tick 值 (2000~10000)，自动限幅到安全范围
 *
 *  直接操作 TIMG7 的 CCP0 比较寄存器 (CCR0)。
 *  计数器从 0 递增，当计数值 == CCR0 时输出翻转，
 *  从而产生指定宽度的脉冲。
 */
void Servo_SetPulse(uint16_t pulse)
{
    /* 下限钳位: 防止脉宽过窄导致舵机抖动或失控 */
    if (pulse < PULSE_MIN) {
        pulse = PULSE_MIN;
    }
    /* 上限钳位: 防止脉宽超过周期 (5000μs) */
    if (pulse > PULSE_MAX) {
        pulse = PULSE_MAX;
    }

    /* 写入 CCP0 比较值，立即生效 (IMMEDIATE 更新模式) */
    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST,           /* TIMG7                          */
        (uint32_t)pulse,          /* 比较值 (tick)                  */
        GPIO_PWM_SERVO_C0_IDX);   /* CCP0 通道 (DL_TIMER_CC_0_INDEX) */
}


/* ================================================================
 *  Servo_Stop — 紧急停止
 * ================================================================
 *
 *  将 CCP0 比较值设为 0，PWM 输出常低，舵机完全松弛（无力矩）。
 *  适用于: 紧急停机、节能待机、切换控制模式前。
 */
void Servo_Stop(void)
{
    DL_TimerG_setCaptureCompareValue(
        PWM_SERVO_INST, 0, GPIO_PWM_SERVO_C0_IDX);
}
