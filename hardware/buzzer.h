/*
 * buzzer.h — 蜂鸣器 / 磁铁 / LED 驱动
 *
 * 引脚（SysConfig）：
 *   BUZZER — PB11  高电平触发蜂鸣
 *   CITIE  — PA10  高电平磁铁有磁力
 *   LED    — PA7   高电平 LED 亮
 *
 * Buzzer_Task() 需在主循环或定时中断中周期调用以管理自动关闭计时。
 */

#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

/* ========== 蜂鸣器 ========== */

/* ---- 初始化 ---- */
void Buzzer_Init(void);

/* 立即控制 */
void Buzzer_On(void);
void Buzzer_Off(void);

/*
 * 触发响指定毫秒（非阻塞，复用 tick_ms 计时）。
 * 重复调用以最后一次为准，剩余时间会被覆盖。
 */
void Buzzer_BeepMs(uint16_t ms);

/* 返回蜂鸣器当前是否正在响。 */
uint8_t Buzzer_IsBeeping(void);

/*
 * 每帧在主循环中调用一次，管理自动关闭。
 * 直接使用全局 tick_ms，无需专用定时器。
 */
void Buzzer_Task(void);

/* ========== 磁铁 ========== */
void Magnet_On(void);
void Magnet_Off(void);

/* ========== LED ========== */
void LED_On(void);
void LED_Off(void);

#endif /* __BUZZER_H */
