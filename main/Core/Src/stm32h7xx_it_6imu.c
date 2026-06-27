/* Core/Src/stm32h7xx_it_6imu.c
 *
 * Исправления:
 *   - UART4_IRQHandler: заменён LL_USART_IsActiveFlag_RXNE /
 *     LL_USART_IsEnabledIT_RXNE на _RXNE_RXFNE версии.
 *     На STM32H7 (SPIv2/USARTv3) флаг RXNE объединён с RXFNE
 *     в единый макрос RXNE_RXFNE. Старое имя RXNE не работает
 *     при отключённом FIFO — условие никогда не выполнялось и
 *     команды с ПЛИС игнорировались.
 *   - Аналогично EnableIT_RXNE → EnableIT_RXNE_RXFNE в main_6imu.c
 */

#include "main.h"
#include "spi6_imu_port.h"

extern volatile uint32_t tx_head;
extern volatile uint32_t tx_tail;
extern volatile uint8_t  uart_tx_busy;
extern volatile uint32_t dma_tc_cnt;
extern volatile device_mode_t g_device_mode;

extern void UART4_TryStartDMA(void);

/* ── UART4 TX DMA ────────────────────────────────────────────────── */
void DMA1_Stream0_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC0(DMA1))
    {
        LL_DMA_ClearFlag_TC0(DMA1);
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
        dma_tc_cnt++;
        tx_tail = (tx_tail + 1U) % 256U;
        uart_tx_busy = 0U;
        UART4_TryStartDMA();
    }
    if (LL_DMA_IsActiveFlag_TE0(DMA1))
    {
        LL_DMA_ClearFlag_TE0(DMA1);
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
        uart_tx_busy = 0U;
    }
}

/* ── UART4 RX ────────────────────────────────────────────────────── */
void UART4_IRQHandler(void)
{
    /* ИСПРАВЛЕНО: _RXNE_RXFNE вместо _RXNE
     * На STM32H7 при DisableFIFO флаг RXNE называется RXNE_RXFNE.
     * Проверка IsEnabledIT убрана — она была лишней и проверяла
     * неправильный бит CR1_RXNEIE вместо CR1_RXFNEIE. */
    if (LL_USART_IsActiveFlag_RXNE_RXFNE(UART4))
    {
        uint8_t cmd = LL_USART_ReceiveData8(UART4);
        switch (cmd)
        {
        case CMD_RAW:   g_device_mode = MODE_RAW;   break;
        case CMD_CALIB: g_device_mode = MODE_CALIB; break;
        case CMD_STOP:  g_device_mode = MODE_IDLE;  break;
        default: break;
        }
    }
    if (LL_USART_IsActiveFlag_ORE(UART4)) LL_USART_ClearFlag_ORE(UART4);
    if (LL_USART_IsActiveFlag_FE(UART4))  LL_USART_ClearFlag_FE(UART4);
    if (LL_USART_IsActiveFlag_NE(UART4))  LL_USART_ClearFlag_NE(UART4);
}

/* ── SPI6 BDMA ───────────────────────────────────────────────────── */
void BDMA_Channel1_IRQHandler(void) { SPI6_BDMA_RX_IRQHandler(); }
void BDMA_Channel0_IRQHandler(void) { SPI6_BDMA_TX_IRQHandler(); }
