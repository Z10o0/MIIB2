/* Core/Inc/icm45686_port.h
 *
 * Адаптер ICM-45686 <-> STM32H723 LL SPI2.
 * Заполняет поля transport и регистрирует callback в imu_dev.
 */
#ifndef ICM45686_PORT_H
#define ICM45686_PORT_H

#include "imu/inv_imu_transport.h"   /* inv_imu_transport_t */
#include "imu/inv_imu_driver.h"      /* inv_imu_device_t    */
#include <stdint.h>

/**
 * @brief  Заполнить transport-поля в imu_dev под SPI2 STM32H723.
 *         Вызвать ПЕРЕД inv_imu_adv_init().
 * @param  dev  Указатель на inv_imu_device_t.
 */
void ICM_Port_Init(inv_imu_device_t *dev);

#endif /* ICM45686_PORT_H */
