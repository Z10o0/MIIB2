/* Core/Inc/spi6_imu_port.h
 *
 * Аппаратный адаптер: 6 × ICM-45686 на SPI6, STM32H723.
 *
 * Пины SPI6:
 *   SCK  = PC12  (AF5)
 *   MISO = PG12  (AF5)
 *   MOSI = PG14  (AF5)
 *
 * CS (активный LOW):
 *   CS1=PC9, CS2=PA8, CS3=PD2, CS4=PD3, CS5=PG9, CS6=PG15
 *
 * BDMA:
 *   Channel0 = SPI6_TX (DMAMUX2 REQ 12)
 *   Channel1 = SPI6_RX (DMAMUX2 REQ 11)
 *   Буферы ОБЯЗАНЫ быть в SRAM4 (0x38000000).
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
#include "stm32h7xx_ll_bdma.h"
#include "stm32h7xx_ll_dmamux.h"
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_advanced.h"

#define IMU_COUNT        6U
#define SPI_DMA_BUF_SIZE 256U

/* Индекс датчика текущей обработки (устанавливается в SPI6_PollSensor) */
extern volatile uint8_t cb_current_imu;

/* Инициализация GPIO + SPI6 + BDMA. Вызывать до ICM_Init_All(). */
void SPI6_IMU_Port_Init(inv_imu_device_t dev[IMU_COUNT]);

/* Читает FIFO одного датчика через BDMA (синхронно).
 * Внутри: FIFO_COUNT → FIFO_DATA → parse → sensor_event_cb. */
void SPI6_PollSensor(uint8_t imu_idx);

/* IRQ-обработчики — вызываются из stm32h7xx_it_6imu.c */
void SPI6_BDMA_RX_IRQHandler(void);   /* BDMA_Channel1_IRQn */
void SPI6_BDMA_TX_IRQHandler(void);   /* BDMA_Channel0_IRQn */

#ifdef __cplusplus
}
#endif
#endif /* SPI6_IMU_PORT_H */
