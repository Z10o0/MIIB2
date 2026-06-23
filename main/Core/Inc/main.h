/* Core/Inc/main.h */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* ── LL-заголовки STM32H7 ─────────────────────────────────────────── */
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif

/* ─────────────────────────────────────────────────────────────────────
 *  Кольцевой буфер UART
 *  256 слотов × 36 байт = 9 КБ SRAM — укладывается в STM32H723
 * ──────────────────────────────────────────────────────────────────── */
#define TX_BUF_SIZE     256U    /* количество слотов в кольцевом буфере */
#define PKT_SLOT_SIZE    36U    /* >= max(sizeof RAW=20, sizeof CALIB=34) */

/* ─────────────────────────────────────────────────────────────────────
 *  Команды управления режимом (один ASCII-байт по UART RX)
 *
 *  'R' (0x52) → MODE_RAW   — выдавать сырые int16 пакеты
 *  'C' (0x43) → MODE_CALIB — выдавать калиброванные float пакеты
 *  'S' (0x53) → MODE_IDLE  — остановить выдачу
 * ──────────────────────────────────────────────────────────────────── */
#define CMD_RAW    ((uint8_t)'R')
#define CMD_CALIB  ((uint8_t)'C')
#define CMD_STOP   ((uint8_t)'S')

/* ─────────────────────────────────────────────────────────────────────
 *  Режимы работы прибора
 * ──────────────────────────────────────────────────────────────────── */
typedef enum
{
    MODE_IDLE  = 0U,   /* ожидание команды, UART TX молчит     */
    MODE_RAW   = 1U,   /* пакет 0x55AA, 20 байт, int16 raw     */
    MODE_CALIB = 2U,   /* пакет 0x55BB, 34 байта, float calib  */
} device_mode_t;

/* ─────────────────────────────────────────────────────────────────────
 *  Индекс IMU в таблице калибровок
 *  Если датчик один — оставить 0.
 * ──────────────────────────────────────────────────────────────────── */
#define ICM_CALIB_IMU_IDX   0U

/* ─────────────────────────────────────────────────────────────────────
 *  NVIC Priority Groups
 * ──────────────────────────────────────────────────────────────────── */
#ifndef NVIC_PRIORITYGROUP_4
#define NVIC_PRIORITYGROUP_0  ((uint32_t)0x00000007)
#define NVIC_PRIORITYGROUP_1  ((uint32_t)0x00000006)
#define NVIC_PRIORITYGROUP_2  ((uint32_t)0x00000005)
#define NVIC_PRIORITYGROUP_3  ((uint32_t)0x00000004)
#define NVIC_PRIORITYGROUP_4  ((uint32_t)0x00000003)
#endif

/* ─────────────────────────────────────────────────────────────────────
 *  Экспортируемые функции
 * ──────────────────────────────────────────────────────────────────── */
void Error_Handler(void);
void UART4_TryStartDMA(void);

#ifdef __cplusplus
}
#endif
#endif /* __MAIN_H */
