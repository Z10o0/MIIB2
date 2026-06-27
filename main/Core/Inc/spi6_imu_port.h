/* Core/Inc/spi6_imu_port.h
 *
 * Аппаратный адаптер для 6 датчиков ICM-45686 на SPI6, STM32H723.
 *
 * Распиновка SPI6:
 *   SCK  = PC12  (SPI6, AF5)
 *   MISO = PG12  (SPI6, AF5)
 *   MOSI = PG14  (SPI6, AF5)
 *
 * CS (software GPIO, активный LOW):
 *   CS1 = PC9
 *   CS2 = PA8
 *   CS3 = PD2
 *   CS4 = PD3
 *   CS5 = PG9
 *   CS6 = PG15
 *
 * ВАЖНО: SPI6 на STM32H723 находится в домене D3 (APB4).
 * DMA для SPI6 — только BDMA + DMAMUX2. DMA1/DMA2 НЕ работают с SPI6.
 * DMA-буферы ОБЯЗАНЫ находиться в SRAM4 (0x38000000).
 */

#ifndef SPI6_IMU_PORT_H
#define SPI6_IMU_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_dmamux.h"

/* stm32h7xx_ll_bdma.h входит в стандартный STM32H7 LL пакет.
 * Если файл не найден — убедись что в Drivers/STM32H7xx_HAL_Driver/Inc
 * есть stm32h7xx_ll_bdma.h. При необходимости скопируй из STM32CubeH7. */
#include "stm32h7xx_ll_bdma.h"

#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_advanced.h"

/* ── Константы ──────────────────────────────────────────────────── */
#define IMU_COUNT          6U
#define SPI_DMA_BUF_SIZE   256U   /* байт, максимальная длина одной транзакции */

/* ── Состояние DMA-машины ───────────────────────────────────────── */
typedef enum {
    SPI_DMA_IDLE = 0,
    SPI_DMA_BUSY,
} spi_dma_state_t;

/* ── Дескриптор транзакции ──────────────────────────────────────── */
typedef struct {
    uint8_t   imu_idx;                    /* индекс датчика [0..5]   */
    uint8_t  *tx_buf;                     /* буфер для отправки       */
    uint8_t  *rx_buf;                     /* буфер для приёма         */
    uint16_t  len;                        /* длина в байтах           */
    void    (*done_cb)(void *ctx);        /* коллбэк по завершению    */
    void     *done_ctx;                   /* аргумент коллбэка        */
} spi_dma_xfer_t;

/* ── Публичный API ──────────────────────────────────────────────── */

/* Инициализировать GPIO + SPI6 + BDMA. Вызывать ОДИН РАЗ до ICM_Init_All(). */
void SPI6_IMU_Port_Init(inv_imu_device_t dev[IMU_COUNT]);

/* Запустить BDMA-транзакцию (или поставить в очередь если шина занята). */
void SPI6_DMA_Transfer(const spi_dma_xfer_t *xfer);

/* Polling-чтение FIFO одного датчика (блокирующее, для отладки). */
void SPI6_PollSensor(uint8_t imu_idx);

/* IRQ-обработчики — вызываются из stm32h7xx_it_6imu.c */
void SPI6_BDMA_RX_IRQHandler(void);   /* → BDMA_Channel1_IRQHandler */
void SPI6_BDMA_TX_IRQHandler(void);   /* → BDMA_Channel0_IRQHandler */

#ifdef __cplusplus
}
#endif
#endif /* SPI6_IMU_PORT_H */
