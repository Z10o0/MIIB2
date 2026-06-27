/**
  ******************************************************************************
  * @file    stm32h7xx_ll_bdma.h
  * @brief   Header file of BDMA LL module for STM32H7xx.
  *
  * BDMA (Basic DMA) обслуживает периферию домена D3:
  *   SPI6, I2C4, LPUART1 и др.
  * Буферы памяти ДОЛЖНЫ находиться в SRAM4 (0x38000000).
  ******************************************************************************
  */

#ifndef STM32H7xx_LL_BDMA_H
#define STM32H7xx_LL_BDMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx.h"

/* ── Каналы BDMA ──────────────────────────────────────────────────── */
#define LL_BDMA_CHANNEL_0    0U
#define LL_BDMA_CHANNEL_1    1U
#define LL_BDMA_CHANNEL_2    2U
#define LL_BDMA_CHANNEL_3    3U
#define LL_BDMA_CHANNEL_4    4U
#define LL_BDMA_CHANNEL_5    5U
#define LL_BDMA_CHANNEL_6    6U
#define LL_BDMA_CHANNEL_7    7U

/* ── Направление передачи ─────────────────────────────────────────── */
#define LL_BDMA_DIRECTION_PERIPH_TO_MEMORY   0x00000000U
#define LL_BDMA_DIRECTION_MEMORY_TO_PERIPH   BDMA_CCR_DIR
#define LL_BDMA_DIRECTION_MEMORY_TO_MEMORY   BDMA_CCR_MEM2MEM

/* ── Режим ────────────────────────────────────────────────────────── */
#define LL_BDMA_MODE_NORMAL     0x00000000U
#define LL_BDMA_MODE_CIRCULAR   BDMA_CCR_CIRC

/* ── Инкремент ────────────────────────────────────────────────────── */
#define LL_BDMA_PERIPH_NOINCREMENT   0x00000000U
#define LL_BDMA_PERIPH_INCREMENT     BDMA_CCR_PINC
#define LL_BDMA_MEMORY_NOINCREMENT   0x00000000U
#define LL_BDMA_MEMORY_INCREMENT     BDMA_CCR_MINC

/* ── Размер данных ────────────────────────────────────────────────── */
#define LL_BDMA_PDATAALIGN_BYTE       0x00000000U
#define LL_BDMA_PDATAALIGN_HALFWORD   BDMA_CCR_PSIZE_0
#define LL_BDMA_PDATAALIGN_WORD       BDMA_CCR_PSIZE_1
#define LL_BDMA_MDATAALIGN_BYTE       0x00000000U
#define LL_BDMA_MDATAALIGN_HALFWORD   BDMA_CCR_MSIZE_0
#define LL_BDMA_MDATAALIGN_WORD       BDMA_CCR_MSIZE_1

/* ── Приоритет ────────────────────────────────────────────────────── */
#define LL_BDMA_PRIORITY_LOW        0x00000000U
#define LL_BDMA_PRIORITY_MEDIUM     BDMA_CCR_PL_0
#define LL_BDMA_PRIORITY_HIGH       BDMA_CCR_PL_1
#define LL_BDMA_PRIORITY_VERYHIGH   BDMA_CCR_PL

/* ════════════════════════════════════════════════════════════════════
 *  Вспомогательный макрос: получить указатель на регистры канала
 * ════════════════════════════════════════════════════════════════════ */
#define __LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, ch) \
    ((BDMA_Channel_TypeDef *)((uint32_t)(BDMAx) + 0x08U + ((ch) * 0x14U)))

/* ════════════════════════════════════════════════════════════════════
 *  Управление каналом
 * ════════════════════════════════════════════════════════════════════ */
__STATIC_INLINE void LL_BDMA_EnableChannel(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    SET_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_EN);
}

__STATIC_INLINE void LL_BDMA_DisableChannel(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    CLEAR_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_EN);
}

__STATIC_INLINE uint32_t LL_BDMA_IsEnabledChannel(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    return (READ_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_EN) == BDMA_CCR_EN) ? 1UL : 0UL;
}

/* ════════════════════════════════════════════════════════════════════
 *  Конфигурация
 * ════════════════════════════════════════════════════════════════════ */
__STATIC_INLINE void LL_BDMA_SetPeriphRequest(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t Request)
{
    /* DMAMUX2 каналы 0..7 = BDMA каналы 0..7 */
    MODIFY_REG(DMAMUX2_Channel0[Channel].CCR, DMAMUX_CxCR_DMAREQ_ID, Request);
}

__STATIC_INLINE void LL_BDMA_SetDataTransferDirection(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t Direction)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR,
               BDMA_CCR_DIR | BDMA_CCR_MEM2MEM, Direction);
}

__STATIC_INLINE void LL_BDMA_SetChannelPriorityLevel(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t Priority)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_PL, Priority);
}

__STATIC_INLINE void LL_BDMA_SetMode(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t Mode)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_CIRC, Mode);
}

__STATIC_INLINE void LL_BDMA_SetPeriphIncMode(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t PeriphOrM2MSrcIncMode)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_PINC, PeriphOrM2MSrcIncMode);
}

__STATIC_INLINE void LL_BDMA_SetMemoryIncMode(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t MemoryOrM2MDstIncMode)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_MINC, MemoryOrM2MDstIncMode);
}

__STATIC_INLINE void LL_BDMA_SetPeriphSize(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t PeriphOrM2MSrcDataSize)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_PSIZE, PeriphOrM2MSrcDataSize);
}

__STATIC_INLINE void LL_BDMA_SetMemorySize(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t MemoryOrM2MDstDataSize)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_MSIZE, MemoryOrM2MDstDataSize);
}

__STATIC_INLINE void LL_BDMA_SetPeriphAddress(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t PeriphOrM2MSrcAddress)
{
    WRITE_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CPAR, PeriphOrM2MSrcAddress);
}

__STATIC_INLINE void LL_BDMA_SetMemoryAddress(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t MemoryOrM2MDstAddress)
{
    WRITE_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CM0AR, MemoryOrM2MDstAddress);
}

__STATIC_INLINE void LL_BDMA_SetDataLength(BDMA_TypeDef *BDMAx, uint32_t Channel, uint32_t NbData)
{
    MODIFY_REG(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CNDTR, BDMA_CNDTR_NDT, NbData);
}

/* ════════════════════════════════════════════════════════════════════
 *  Прерывания
 * ════════════════════════════════════════════════════════════════════ */
__STATIC_INLINE void LL_BDMA_EnableIT_TC(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    SET_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_TCIE);
}

__STATIC_INLINE void LL_BDMA_DisableIT_TC(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    CLEAR_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_TCIE);
}

__STATIC_INLINE void LL_BDMA_EnableIT_TE(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    SET_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_TEIE);
}

__STATIC_INLINE void LL_BDMA_DisableIT_TE(BDMA_TypeDef *BDMAx, uint32_t Channel)
{
    CLEAR_BIT(__LL_BDMA_GET_CHANNEL_INSTANCE(BDMAx, Channel)->CCR, BDMA_CCR_TEIE);
}

/* ════════════════════════════════════════════════════════════════════
 *  Флаги статуса (ISR регистр BDMA)
 * ════════════════════════════════════════════════════════════════════ */
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TC0(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TCIF0)  == BDMA_ISR_TCIF0)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TC1(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TCIF1)  == BDMA_ISR_TCIF1)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TC2(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TCIF2)  == BDMA_ISR_TCIF2)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TC3(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TCIF3)  == BDMA_ISR_TCIF3)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TE0(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TEIF0)  == BDMA_ISR_TEIF0)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TE1(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TEIF1)  == BDMA_ISR_TEIF1)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TE2(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TEIF2)  == BDMA_ISR_TEIF2)  ? 1UL : 0UL; }
__STATIC_INLINE uint32_t LL_BDMA_IsActiveFlag_TE3(BDMA_TypeDef *BDMAx)  { return (READ_BIT(BDMAx->ISR, BDMA_ISR_TEIF3)  == BDMA_ISR_TEIF3)  ? 1UL : 0UL; }

/* ── Сброс флагов (через IFCR) ────────────────────────────────────── */
__STATIC_INLINE void LL_BDMA_ClearFlag_TC0(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTCIF0); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TC1(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTCIF1); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TC2(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTCIF2); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TC3(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTCIF3); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TE0(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTEIF0); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TE1(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTEIF1); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TE2(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTEIF2); }
__STATIC_INLINE void LL_BDMA_ClearFlag_TE3(BDMA_TypeDef *BDMAx) { WRITE_REG(BDMAx->IFCR, BDMA_IFCR_CTEIF3); }

#ifdef __cplusplus
}
#endif
#endif /* STM32H7xx_LL_BDMA_H */
