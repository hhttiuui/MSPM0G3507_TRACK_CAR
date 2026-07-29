/*
 * buzzer.h — 磁铁 / LED 驱动
 *
 * 引脚（SysConfig）：
 *   CITIE  — PA10  高电平磁铁有磁力
 *   LED    — PA7   高电平 LED 亮
 */

#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

/* ========== 磁铁 ========== */
void Magnet_On(void);
void Magnet_Off(void);

/* ========== LED ========== */
void LED_On(void);
void LED_Off(void);

#endif /* __BUZZER_H */
