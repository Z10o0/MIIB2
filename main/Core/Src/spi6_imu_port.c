/* Core/Src/spi6_imu_port.c
 *
 * Аппаратный адаптер: 6 × ICM-45686 на SPI6, STM32H723, LL-драйверы.
 *
 * Архитектура:
 *   - Один SPI6 в режиме polling (без DMA, без прерываний).
 *   - Каждый датчик имеет собственный CS-пин.
 *   - Индекс датчика [0..5] передаётся через поле context транспорта.
 *   - spi_byte() защищена от зависания: таймаут + сброс EOT флага H7.
 *
 * Распиновка:
 *   SPI6 SCK  = PC12  (AF5)
 *   SPI6 MISO = PG12  (AF5)
 *   SPI6 MOSI = PG14  (AF5)
 *
 *   CS1 = PC9   CS2 = PA8   CS3 = PD2
 *   CS4 = PD3   CS5 = PG9   CS6 = PG15
 *
 * Тактирование SPI6:
 * Источник: PLL3Q (PLL3 VCO = 160 МГц, Q‑делитель задаётся в SystemClock_Config).
 * Для текущей конфигурации PLL3Q = 40 МГц, BaudRate = DIV2 → SCK = 20 МГц.
 */

#include "spi6_imu_port.h"
#include "main.h"
#include "stm32h7xx_ll_utils.h"

typedef struct {
    GPIO_TypeDef *port;
    uint32_t      pin;
} cs_pin_t;

static const cs_pin_t cs_table[IMU_COUNT] = {
    { GPIOC, LL_GPIO_PIN_9  },   /* CS1 — PC9  */
    { GPIOA, LL_GPIO_PIN_8  },   /* CS2 — PA8  */
    { GPIOD, LL_GPIO_PIN_2  },   /* CS3 — PD2  */
    { GPIOD, LL_GPIO_PIN_3  },   /* CS4 — PD3  */
    { GPIOG, LL_GPIO_PIN_9  },   /* CS5 — PG9  */
    { GPIOG, LL_GPIO_PIN_15 },   /* CS6 — PG15 */
};

static inline void cs_select(uint8_t idx)
{
    LL_GPIO_ResetOutputPin(cs_table[idx].port, cs_table[idx].pin);
}

static inline void cs_deselect(uint8_t idx)
{
    LL_GPIO_SetOutputPin(cs_table[idx].port, cs_table[idx].pin);
}

static uint8_t spi_byte(uint8_t b)
{
    uint32_t timeout;

    timeout = 10000U;
    while (!LL_SPI_IsActiveFlag_TXP(SPI6))
    {
        if (--timeout == 0U) return 0xFFU;
        if (LL_SPI_IsActiveFlag_EOT(SPI6))
        {
            LL_SPI_ClearFlag_EOT(SPI6);
            LL_SPI_ClearFlag_TXTF(SPI6);
            LL_SPI_StartMasterTransfer(SPI6);
        }
    }

    LL_SPI_TransmitData8(SPI6, b);

    timeout = 10000U;
    while (!LL_SPI_IsActiveFlag_RXP(SPI6))
    {
        if (--timeout == 0U) return 0xFFU;
    }

    return LL_SPI_ReceiveData8(SPI6);
}

static int imu_read_reg(void *context, uint8_t reg,
                        uint8_t *buf, uint32_t len)
{
    uint8_t idx = *((const uint8_t *)context);
    cs_select(idx);
    spi_byte(reg | 0x80U);
    for (uint32_t i = 0; i < len; i++)
        buf[i] = spi_byte(0x00U);
    cs_deselect(idx);
    return 0;
}

static int imu_write_reg(void *context, uint8_t reg,
                         const uint8_t *buf, uint32_t len)
{
    uint8_t idx = *((const uint8_t *)context);
    cs_select(idx);
    spi_byte(reg & 0x7FU);
    for (uint32_t i = 0; i < len; i++)
        spi_byte(buf[i]);
    cs_deselect(idx);
    return 0;
}

static void imu_sleep_us(uint32_t us)
{
    uint32_t ms = (us + 999U) / 1000U;
    if (ms == 0U) ms = 1U;
    LL_mDelay(ms);
}

static const uint8_t imu_idx[IMU_COUNT] = {0, 1, 2, 3, 4, 5};

void SPI6_IMU_Port_Init(inv_imu_device_t dev[IMU_COUNT])
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);

    /* SPI6 тактируется от PLL3R = 160 МГц */
    LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PLL3Q);
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);

    /* PC12 — SCK (AF5)
     * Скорость VERY_HIGH обязательна при 20 МГц */
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PG12 — MISO, PG14 — MOSI (AF5) */
    GPIO_InitStruct.Pin       = LL_GPIO_PIN_12 | LL_GPIO_PIN_14;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* CS1 = PC9 — HIGH до инициализации датчика */
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_9;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = 0U;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* CS2 = PA8 */
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CS3 = PD2, CS4 = PD3 */
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_2 | LL_GPIO_PIN_3);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CS5 = PG9, CS6 = PG15 */
    LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_9 | LL_GPIO_PIN_15);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_15;
    LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* SPI6 периферия
     * CPOL=1, CPHA=2 (MODE3) — как в одно-датчиковом проекте
     * DIV8: 160 МГц / 8 = 20 МГц */
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode              = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth         = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity	 = LL_SPI_POLARITY_LOW;   // CPOL=0
    SPI_InitStruct.ClockPhase   	 = LL_SPI_PHASE_1EDGE;    // CPHA=0
    SPI_InitStruct.NSS               = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate 		 = LL_SPI_BAUDRATEPRESCALER_DIV2;
    SPI_InitStruct.BitOrder          = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly           = 0x0U;
    LL_SPI_Init(SPI6, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI6, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_DisableNSSPulseMgt(SPI6);
    LL_SPI_SetTransferSize(SPI6, 0U);
    LL_SPI_Enable(SPI6);
    LL_SPI_StartMasterTransfer(SPI6);

    for (uint8_t i = 0U; i < IMU_COUNT; i++)
    {
        dev[i].transport.context    = (void *)&imu_idx[i];
        dev[i].transport.read_reg   = imu_read_reg;
        dev[i].transport.write_reg  = imu_write_reg;
        dev[i].transport.sleep_us   = imu_sleep_us;
        dev[i].transport.serif_type = UI_SPI4;
    }
}
