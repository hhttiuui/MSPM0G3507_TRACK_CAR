/*
 * buzzer.c — 磁铁 / LED 驱动实现
 *
 * CITIE — PA10  高电平磁铁有磁力
 * LED   — PA7   高电平 LED 亮
 */

#include "buzzer.h"
#include "ti_msp_dl_config.h"

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
