/*
 * buzzer.c — 蜂鸣器驱动实现（高电平触发）
 *
 * PB11 输出: 1=蜂鸣响, 0=停止
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
    /* 引脚由 SYSCFG_DL_init() 初始化；低电平确保高电平触发蜂鸣器保持关闭。 */
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);

    s_beepRemainMs  = 0U;
    s_beepStartTick = 0U;
}

void Buzzer_On(void)
{
    DL_GPIO_setPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
}

void Buzzer_Off(void)
{
    DL_GPIO_clearPins(BUZZER_PORT, BUZZER_BUZZER_PIN_PIN);
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

/* ---- 磁铁控制（CITIE: PA10，高电平有磁力） ---- */

void Magnet_On(void)
{
    DL_GPIO_setPins(CITIE_PORT, CITIE_PIN_0_PIN);
}

void Magnet_Off(void)
{
    DL_GPIO_clearPins(CITIE_PORT, CITIE_PIN_0_PIN);
}

/* ---- LED 控制（LED: PA7，高电平亮） ---- */

void LED_On(void)
{
    DL_GPIO_setPins(LED_PORT, LED_PIN_1_PIN);
}

void LED_Off(void)
{
    DL_GPIO_clearPins(LED_PORT, LED_PIN_1_PIN);
}
