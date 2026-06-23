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
 * Каждый датчик получает свою inv_imu_device_t и свои коллбэки.
 * Индекс датчика [0..5] передаётся через поле context транспорта.
 */

#ifndef SPI6_IMU_PORT_H
#define SPI6_IMU_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_driver.h"

/* Число датчиков на шине */
#define IMU_COUNT   6U

/* Инициализировать GPIO + SPI6, затем заполнить транспортные
 * структуры для всех 6 устройств.
 * Вызывать ОДИН РАЗ до ICM_Init_All(). */
void SPI6_IMU_Port_Init(inv_imu_device_t dev[IMU_COUNT]);

#ifdef __cplusplus
}
#endif
#endif /* SPI6_IMU_PORT_H */
