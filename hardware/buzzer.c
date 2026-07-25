/*
 * buzzer.c — 蜂鸣器驱动实现（低电平触发）
 *
 * PB11 输出: 0=蜂鸣响, 1=停止
 * 自动关闭基于全局 tick_ms（clock.h），无需额外硬件定时器。
 */

#include "buzzer.h"
#include "ti_msp_dl_config.h"
#include "clock.h"

/* 自动关闭计时 */
static volatile uint16_t s_beepRemainMs = 0U;
static volatile unsigned long s_beepStartTick = 0U;

/* ---- 公开 API ---- */

void Buzzer_Init(void)
{
    /* 引脚由 SYSCFG_DL_init() 初始化；高电平确保低电平触发蜂鸣器保持关闭。 */
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);

    s_beepRemainMs  = 0U;
    s_beepStartTick = 0U;
}

void Buzzer_On(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
}

void Buzzer_Off(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
    s_beepRemainMs  = 0U;
    s_beepStartTick = 0U;
}

void Buzzer_BeepMs(uint16_t ms)
{
    if (ms == 0U) {
        Buzzer_Off();
        return;
    }

    Buzzer_On();
    s_beepRemainMs  = ms;
    s_beepStartTick = tick_ms;
}

uint8_t Buzzer_IsBeeping(void)
{
    return (s_beepRemainMs > 0U) ? 1U : 0U;
}

void Buzzer_Task(void)
{
    if (s_beepRemainMs == 0U) {
        return;
    }

    unsigned long elapsed = tick_ms - s_beepStartTick;
    if (elapsed >= s_beepRemainMs) {
        Buzzer_Off();
    }
}
