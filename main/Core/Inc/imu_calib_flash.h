/**
 * @file    imu_calib_flash.h
 * @brief   Таблицы калибровки 36 IMU (ICM-45686), хранимые во Flash как const.
 *
 * Сетка температур:  36 узлов, −40…+85 °C (равномерная).
 * Сетка скоростей:   33 узла, −2000…+2000 °/с (знаковая, неравномерная).
 */

#ifndef IMU_CALIB_FLASH_H
#define IMU_CALIB_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ─── РАЗМЕРНОСТИ ──────────────────────────────────────────────────────────── */

#define IMU_COUNT_MAX    36U   /**< Число датчиков в системе                  */
#define CALIB_TEMP_NODES 36U   /**< Узлы по температуре: −40…+85 °C           */
#define CALIB_RATE_NODES 33U   /**< Узлы по скорости: −2000…+2000 °/с (знак) */

/* ─── ПЕРЕЧИСЛЕНИЯ ─────────────────────────────────────────────────────────── */

typedef enum
{
    IMU_AXIS_X     = 0,
    IMU_AXIS_Y     = 1,
    IMU_AXIS_Z     = 2,
    IMU_AXIS_COUNT = 3
} imu_axis_t;

/* ─── LUT-СТРУКТУРЫ ────────────────────────────────────────────────────────── */

/** 1D-таблица: значение по оси температуры */
typedef struct { float y[CALIB_TEMP_NODES]; } lut1d_t;

/** 2D-таблица: значение по осям (температура × скорость) */
typedef struct { float z[CALIB_TEMP_NODES][CALIB_RATE_NODES]; } lut2d_t;

/* ─── КАЛИБРОВКА ОДНОЙ ОСИ ─────────────────────────────────────────────────── */

typedef struct
{
    lut1d_t acc_bias_vs_temp;        /**< Bias акселерометра  [м/с²] vs T     */
    lut1d_t acc_scale_vs_temp;       /**< Scale акселерометра [б/р]  vs T     */
    lut1d_t gyr_bias_vs_temp;        /**< Bias гироскопа      [°/с]  vs T     */
    lut2d_t gyr_scale_vs_temp_rate;  /**< МК гироскопа        [б/р]  vs T,ω  */
} imu_axis_calib_t;

typedef struct { imu_axis_calib_t axis[IMU_AXIS_COUNT]; } imu_one_calib_t;
typedef struct { imu_one_calib_t  imu [IMU_COUNT_MAX];  } imu_calib_flash_t;

/* ─── ВНЕШНИЕ ПЕРЕМЕННЫЕ (определены в imu_calib_tables.c) ────────────────── */

extern const float             g_temp_nodes[CALIB_TEMP_NODES];
extern const float             g_rate_nodes[CALIB_RATE_NODES];
extern const imu_calib_flash_t g_imu_calib_flash;

/* ─── ПУБЛИЧНЫЙ API ─────────────────────────────────────────────────────────── */

/** @pre  imu_id < IMU_COUNT_MAX
 *  @pre  axis   < IMU_AXIS_COUNT */

/** @return  Bias акселерометра [м/с²] при данной температуре */
float imu_get_acc_bias (uint8_t imu_id, imu_axis_t axis, float temp_c);

/** @return  Scale акселерометра [б/р] при данной температуре */
float imu_get_acc_scale(uint8_t imu_id, imu_axis_t axis, float temp_c);

/** @return  Bias гироскопа [°/с] при данной температуре */
float imu_get_gyr_bias (uint8_t imu_id, imu_axis_t axis, float temp_c);

/**
 * @brief   МК гироскопа из 2D-таблицы (T, ω).
 * @param   gyr_rate  Угловая скорость [°/с], знаковая (−2000…+2000)
 * @return  Масштабный коэффициент [б/р]
 */
float imu_get_gyr_scale(uint8_t imu_id, imu_axis_t axis,
                        float temp_c, float gyr_rate);

/**
 * @brief   Корректирует сырое значение акселерометра.
 * Формула:  corr = (acc_raw − bias(T)) × scale(T)
 */
float imu_correct_acc(uint8_t imu_id, imu_axis_t axis,
                      float acc_raw, float temp_c);

/**
 * @brief   Корректирует сырое значение гироскопа.
 * Формула:  corr = (gyr_raw − bias(T)) × scale(T, gyr_raw)
 *
 * @note    scale вычисляется по НЕскорректированному gyr_raw.
 *          При необходимости итерации — вызови imu_get_gyr_scale() напрямую.
 */
float imu_correct_gyr(uint8_t imu_id, imu_axis_t axis,
                      float gyr_raw, float temp_c);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIB_FLASH_H */
