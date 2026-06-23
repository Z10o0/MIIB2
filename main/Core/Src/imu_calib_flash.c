/**
 * @file    imu_calib_flash.c
 * @brief   Интерполяция калибровочных таблиц из Flash.
 *          Сетка: 36T × 33R (−2000..+2000 °/с, знаковая).
 */

#include "imu_calib_flash.h"
#include <assert.h>

/* ─── Бинарный поиск нижнего индекса ──────────────────────────────────────── */

static uint16_t find_lower_idx(const float *nodes, uint16_t n, float xq)
{
    uint16_t lo = 0U, hi = n - 1U;
    while ((uint16_t)(hi - lo) > 1U)
    {
        uint16_t mid = (lo + hi) >> 1U;
        if (nodes[mid] <= xq) lo = mid; else hi = mid;
    }
    return lo;
}

/* ─── 1D линейная интерполяция ─────────────────────────────────────────────── */

static float lut1d_interp(const float *nodes, const float *vals,
                           uint16_t n, float xq)
{
    if (xq <= nodes[0U])      return vals[0U];
    if (xq >= nodes[n - 1U])  return vals[n - 1U];
    uint16_t i = find_lower_idx(nodes, n, xq);
    float t = (xq - nodes[i]) / (nodes[i + 1U] - nodes[i]);
    return vals[i] + t * (vals[i + 1U] - vals[i]);
}

/* ─── 2D билинейная интерполяция ───────────────────────────────────────────── */

static float lut2d_interp(const float *xnodes, const float *ynodes,
                           const float *zvals,
                           uint16_t nx, uint16_t ny,
                           float xq, float yq)
{
    if (xq < xnodes[0U])      xq = xnodes[0U];
    if (xq > xnodes[nx - 1U]) xq = xnodes[nx - 1U];
    if (yq < ynodes[0U])      yq = ynodes[0U];
    if (yq > ynodes[ny - 1U]) yq = ynodes[ny - 1U];

    uint16_t ix = find_lower_idx(xnodes, nx, xq);
    uint16_t iy = find_lower_idx(ynodes, ny, yq);

    float tx = (xq - xnodes[ix]) / (xnodes[ix + 1U] - xnodes[ix]);
    float ty = (yq - ynodes[iy]) / (ynodes[iy + 1U] - ynodes[iy]);

    float z00 = zvals[ ix      * ny + iy      ];
    float z10 = zvals[(ix + 1U)* ny + iy      ];
    float z01 = zvals[ ix      * ny +(iy + 1U)];
    float z11 = zvals[(ix + 1U)* ny +(iy + 1U)];

    return z00*(1.0f-tx)*(1.0f-ty)
         + z10*      tx *(1.0f-ty)
         + z01*(1.0f-tx)*      ty
         + z11*      tx *      ty;
}

#define CHECK_PARAMS(id, ax) \
    do { assert((id)<IMU_COUNT_MAX); assert((uint8_t)(ax)<IMU_AXIS_COUNT); } while(0)

/* ─── ПУБЛИЧНЫЙ API ─────────────────────────────────────────────────────────── */

float imu_get_acc_bias(uint8_t imu_id, imu_axis_t axis, float temp_c)
{
    CHECK_PARAMS(imu_id, axis);
    const float *y = g_imu_calib_flash.imu[imu_id].axis[axis].acc_bias_vs_temp.y;
    return lut1d_interp(g_temp_nodes, y, CALIB_TEMP_NODES, temp_c);
}

float imu_get_acc_scale(uint8_t imu_id, imu_axis_t axis, float temp_c)
{
    CHECK_PARAMS(imu_id, axis);
    const float *y = g_imu_calib_flash.imu[imu_id].axis[axis].acc_scale_vs_temp.y;
    return lut1d_interp(g_temp_nodes, y, CALIB_TEMP_NODES, temp_c);
}

float imu_get_gyr_bias(uint8_t imu_id, imu_axis_t axis, float temp_c)
{
    CHECK_PARAMS(imu_id, axis);
    const float *y = g_imu_calib_flash.imu[imu_id].axis[axis].gyr_bias_vs_temp.y;
    return lut1d_interp(g_temp_nodes, y, CALIB_TEMP_NODES, temp_c);
}

/*
 * ИЗМЕНЕНИЕ 6:
 *   - Параметр rate_abs переименован в gyr_rate
 *   - fabsf() УДАЛЁН — знак передаётся в интерполятор как есть
 *   - Это позволяет иметь разные МК для +ω и -ω (реальная асимметрия МЕМС)
 */
float imu_get_gyr_scale(uint8_t imu_id, imu_axis_t axis,
                        float temp_c, float gyr_rate)
{
    CHECK_PARAMS(imu_id, axis);
    const float *z = &g_imu_calib_flash.imu[imu_id].axis[axis]
                        .gyr_scale_vs_temp_rate.z[0][0];
    return lut2d_interp(g_temp_nodes, g_rate_nodes,
                        z,
                        CALIB_TEMP_NODES, CALIB_RATE_NODES,
                        temp_c, gyr_rate);     /* ← знак сохранён */
}

float imu_correct_acc(uint8_t imu_id, imu_axis_t axis,
                      float acc_raw, float temp_c)
{
    return (acc_raw - imu_get_acc_bias (imu_id, axis, temp_c))
                    * imu_get_acc_scale(imu_id, axis, temp_c);
}

/*
 * ИЗМЕНЕНИЕ 7:
 *   - fabsf(gyr_raw) УДАЛЁН — gyr_raw передаётся со знаком
 *   - scale теперь учитывает направление вращения
 */
float imu_correct_gyr(uint8_t imu_id, imu_axis_t axis,
                      float gyr_raw, float temp_c)
{
    float bias  = imu_get_gyr_bias (imu_id, axis, temp_c);
    float scale = imu_get_gyr_scale(imu_id, axis, temp_c, gyr_raw); /* ← знак */
    return (gyr_raw - bias) * scale;
}
