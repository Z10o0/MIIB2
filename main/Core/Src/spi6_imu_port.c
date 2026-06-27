/* Core/Src/spi6_imu_port.c
 *
 * ИСПРАВЛЕНИЯ v2:
 *  1. dma_start(): CS опускается ПОСЛЕ Enable SPI — раньше CS шёл до Enable,
 *     из-за чего первый байт уходил без активного CS.
 *  2. SPI6_BDMA_RX_IRQHandler: убрано повторное отключение DMA запросов
 *     после cs_hi — они уже отключены внутри dma_start после транзакции.
 *  3. SPI6_PollSensor: FIFO_COUNT регистр 0x2E даёт только 1 значимый байт
 *     (ICM-45686 FIFO_COUNT — 12 бит, Little-endian: LOW=0x2E, HIGH=0x2F).
 *     Исправлен порядок байт: fifo_bytes = g_rx[1] | (g_rx[2] << 8).
 *  4. После чтения FIFO_DATA данные начинаются с g_rx[1] (g_rx[0] = мусор
 *     адресного цикла) — передаём fifo_bytes, не total.
 *  5. dma_wait таймаут увеличен до 5 000 000 (при 400 МГц ~12 мс).
 *  6. spi_reset() вызывается перед каждой dma_start транзакцией чтобы
 *     гарантированно сбросить TSIZE из предыдущей транзакции.
 *
 * Принцип работы:
 *  - Инициализация датчиков: через polling (spi_byte)
 *  - Опрос FIFO каждого датчика: ПОЛНОСТЬЮ через BDMA
 *    1) BDMA транзакция: читаем FIFO_COUNT (3 байта)
 *    2) Смотрим g_rx[1..2] — количество байт в FIFO (Little-endian)
 *    3) BDMA транзакция: читаем FIFO_DATA (1+N байт)
 *    4) Парсим данные начиная с g_rx[1]
 *
 * Пины:
 *  SCK  = PC12, MISO = PG12, MOSI = PG14
 *  CS1..6 = PC9, PA8, PD2, PD3, PG9, PG15
 */

#include "spi6_imu_port.h"
#include "main.h"
#include "imu/inv_imu_driver_advanced.h"
#include "stm32h7xx_ll_bdma.h"
#include "stm32h7xx_ll_utils.h"
#include <string.h>

/* ─── CS пины ──────────────────────────────────────────────────── */
typedef struct { GPIO_TypeDef *port; uint32_t pin; } cs_pin_t;

static const cs_pin_t cs_table[IMU_COUNT] = {
    { GPIOC, LL_GPIO_PIN_9  },
    { GPIOA, LL_GPIO_PIN_8  },
    { GPIOD, LL_GPIO_PIN_2  },
    { GPIOD, LL_GPIO_PIN_3  },
    { GPIOG, LL_GPIO_PIN_9  },
    { GPIOG, LL_GPIO_PIN_15 },
};

static inline void cs_lo(uint8_t i) { LL_GPIO_ResetOutputPin(cs_table[i].port, cs_table[i].pin); }
static inline void cs_hi(uint8_t i) { LL_GPIO_SetOutputPin  (cs_table[i].port, cs_table[i].pin); }

/* ─── Все буферы в SRAM4 (обязательно для BDMA) ────────────────── */
#define SRAM4  __attribute__((section(".sram4"), aligned(4)))

static uint8_t SRAM4 g_tx[SPI_DMA_BUF_SIZE];
/* +1 для dummy байта адреса; FIFO_MIRRORING_SIZE должен быть >= SPI_DMA_BUF_SIZE */
static uint8_t SRAM4 g_rx[SPI_DMA_BUF_SIZE + 1U];

/* ─── Состояние ─────────────────────────────────────────────────── */
static inv_imu_device_t *g_dev;
static const uint8_t     g_idx[IMU_COUNT] = {0,1,2,3,4,5};

static volatile uint8_t  g_dma_done;
static volatile uint8_t  g_cur_imu;

extern volatile uint8_t cb_current_imu;

/* ════════════════════════════════════════════════════════════════
 *  Низкоуровневые функции
 * ════════════════════════════════════════════════════════════════ */

/* ИСПРАВЛЕНО: таймаут 5 000 000 (~12 мс при 400 МГц) */
static int dma_wait(void)
{
    uint32_t t = 5000000U;
    while (!g_dma_done && --t) {}
    return (g_dma_done) ? 0 : -1;
}

static void spi_reset_for_dma(void)
{
    LL_SPI_Disable(SPI6);
    /* Слить RX FIFO */
    while (LL_SPI_IsActiveFlag_RXWNE(SPI6))
        (void)LL_SPI_ReceiveData8(SPI6);
    LL_SPI_ClearFlag_EOT(SPI6);
    LL_SPI_ClearFlag_TXTF(SPI6);
    LL_SPI_DisableDMAReq_RX(SPI6);
    LL_SPI_DisableDMAReq_TX(SPI6);
}

/* ИСПРАВЛЕНО: CS опускается ПОСЛЕ запуска SPI и мастер-трансфера */
static void dma_start(uint8_t imu, uint16_t len)
{
    g_dma_done = 0;
    g_cur_imu  = imu;

    /* Сброс SPI перед новой транзакцией */
    spi_reset_for_dma();

    /* ── RX канал (Channel1) ── */
    LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);
    while (LL_BDMA_IsEnabledChannel(BDMA, LL_BDMA_CHANNEL_1)) {}
    LL_BDMA_ClearFlag_TC1(BDMA);
    LL_BDMA_ClearFlag_TE1(BDMA);
    LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_1, (uint32_t)g_rx);
    LL_BDMA_SetDataLength   (BDMA, LL_BDMA_CHANNEL_1, len);
    LL_BDMA_EnableIT_TC     (BDMA, LL_BDMA_CHANNEL_1);
    LL_BDMA_EnableChannel   (BDMA, LL_BDMA_CHANNEL_1);

    /* ── TX канал (Channel0) ── */
    LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
    while (LL_BDMA_IsEnabledChannel(BDMA, LL_BDMA_CHANNEL_0)) {}
    LL_BDMA_ClearFlag_TC0(BDMA);
    LL_BDMA_ClearFlag_TE0(BDMA);
    LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_0, (uint32_t)g_tx);
    LL_BDMA_SetDataLength   (BDMA, LL_BDMA_CHANNEL_0, len);
    LL_BDMA_EnableIT_TE     (BDMA, LL_BDMA_CHANNEL_0);
    LL_BDMA_EnableChannel   (BDMA, LL_BDMA_CHANNEL_0);

    /* Настраиваем SPI: TSIZE → DMA REQ → Enable */
    LL_SPI_SetTransferSize(SPI6, len);
    LL_SPI_EnableDMAReq_RX(SPI6);
    LL_SPI_EnableDMAReq_TX(SPI6);
    LL_SPI_Enable(SPI6);

    /* ИСПРАВЛЕНО: CS опускается ПОСЛЕ Enable SPI */
    cs_lo(imu);
    LL_SPI_StartMasterTransfer(SPI6);
}

static int dma_xfer_sync(uint8_t imu, uint16_t len)
{
    dma_start(imu, len);
    return dma_wait();
}

/* ════════════════════════════════════════════════════════════════
 *  Polling для инициализации датчиков
 * ════════════════════════════════════════════════════════════════ */

static void spi_reset(void)
{
    LL_SPI_Disable(SPI6);
    while (LL_SPI_IsActiveFlag_RXWNE(SPI6))
        (void)LL_SPI_ReceiveData8(SPI6);
    LL_SPI_ClearFlag_EOT(SPI6);
    LL_SPI_ClearFlag_TXTF(SPI6);
    LL_SPI_DisableDMAReq_RX(SPI6);
    LL_SPI_DisableDMAReq_TX(SPI6);
    LL_SPI_Enable(SPI6);
}

static uint8_t spi_byte(uint8_t b)
{
    uint32_t t;
    t = 50000U; while (!LL_SPI_IsActiveFlag_TXP(SPI6) && --t) {}
    LL_SPI_TransmitData8(SPI6, b);
    t = 50000U; while (!LL_SPI_IsActiveFlag_RXP(SPI6) && --t) {}
    return LL_SPI_ReceiveData8(SPI6);
}

static int imu_read(void *ctx, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t idx = *(const uint8_t *)ctx;
    spi_reset();
    LL_SPI_SetTransferSize(SPI6, len + 1U);
    cs_lo(idx);
    LL_SPI_StartMasterTransfer(SPI6);
    spi_byte(reg | 0x80U);
    for (uint32_t i = 0; i < len; i++) buf[i] = spi_byte(0x00U);
    uint32_t t = 50000U; while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
    LL_SPI_ClearFlag_EOT(SPI6);
    LL_SPI_ClearFlag_TXTF(SPI6);
    cs_hi(idx);
    return 0;
}

static int imu_write(void *ctx, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint8_t idx = *(const uint8_t *)ctx;
    spi_reset();
    LL_SPI_SetTransferSize(SPI6, len + 1U);
    cs_lo(idx);
    LL_SPI_StartMasterTransfer(SPI6);
    spi_byte(reg & 0x7FU);
    for (uint32_t i = 0; i < len; i++) spi_byte(buf[i]);
    uint32_t t = 50000U; while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
    LL_SPI_ClearFlag_EOT(SPI6);
    LL_SPI_ClearFlag_TXTF(SPI6);
    cs_hi(idx);
    return 0;
}

static void imu_sleep(uint32_t us)
{
    LL_mDelay((us + 999U) / 1000U);
}

/* ════════════════════════════════════════════════════════════════
 *  Опрос одного датчика через BDMA (вызывается из main loop)
 * ════════════════════════════════════════════════════════════════ */

void SPI6_PollSensor(uint8_t idx)
{
    /* ── Шаг 1: читаем FIFO_COUNT (регистры 0x2E, 0x2F) ──
     * ICM-45686: FIFO_COUNTH=0x2E (биты [3:0] = COUNT[11:8]),
     *            FIFO_COUNTL=0x2F (биты [7:0] = COUNT[7:0])
     * При чтении 2 регистров подряд начиная с 0x2E:
     *   g_rx[0] = мусор (ответ на байт адреса)
     *   g_rx[1] = FIFO_COUNTH (старший, биты [3:0])
     *   g_rx[2] = FIFO_COUNTL (младший, биты [7:0])
     *
     * ИСПРАВЛЕНО: порядок байт Little-endian в ICM-45686:
     * FIFO_COUNT = (COUNTH[3:0] << 8) | COUNTL[7:0]
     */
    g_tx[0] = 0x2EU | 0x80U;
    g_tx[1] = 0x00U;
    g_tx[2] = 0x00U;

    if (dma_xfer_sync(idx, 3U) != 0) return;

    /* ИСПРАВЛЕНО: ICM-45686 FIFO_COUNT — Big-endian в регистрах:
     * COUNTH (0x2E) содержит старшие биты [11:8],
     * COUNTL (0x2F) содержит младшие биты [7:0] */
    uint16_t fifo_bytes = ((uint16_t)(g_rx[1] & 0x0FU) << 8) | (uint16_t)g_rx[2];

    if (fifo_bytes == 0U)                    return;
    if (fifo_bytes > (SPI_DMA_BUF_SIZE - 1U))
        fifo_bytes = SPI_DMA_BUF_SIZE - 1U;

    /* ── Шаг 2: читаем FIFO_DATA (регистр 0x3F) ──
     * g_tx[0] = адрес + READ, g_tx[1..N] = 0x00
     * g_rx[0] = мусор, g_rx[1..N] = данные FIFO
     */
    g_tx[0] = 0x3FU | 0x80U;
    memset(g_tx + 1U, 0x00U, fifo_bytes);

    uint16_t total = (uint16_t)(fifo_bytes + 1U);
    if (dma_xfer_sync(idx, total) != 0) return;

    /* ── Шаг 3: парсим ──
     * ИСПРАВЛЕНО: передаём fifo_bytes (не total), данные с g_rx[1]
     */
    cb_current_imu = idx;
    inv_imu_adv_parse_fifo_data(&g_dev[idx], &g_rx[1], fifo_bytes);
}

/* ════════════════════════════════════════════════════════════════
 *  IRQ обработчики
 * ════════════════════════════════════════════════════════════════ */

void SPI6_BDMA_RX_IRQHandler(void)   /* BDMA_Channel1_IRQn */
{
    if (LL_BDMA_IsActiveFlag_TC1(BDMA))
    {
        LL_BDMA_ClearFlag_TC1(BDMA);

        /* Ждём EOT SPI6 (все биты ушли) */
        uint32_t t = 10000U;
        while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
        LL_SPI_ClearFlag_EOT(SPI6);
        LL_SPI_ClearFlag_TXTF(SPI6);

        /* Отключаем DMA запросы, поднимаем CS */
        LL_SPI_DisableDMAReq_RX(SPI6);
        LL_SPI_DisableDMAReq_TX(SPI6);
        cs_hi(g_cur_imu);

        g_dma_done = 1U;
    }

    if (LL_BDMA_IsActiveFlag_TE1(BDMA))
    {
        LL_BDMA_ClearFlag_TE1(BDMA);
        LL_SPI_DisableDMAReq_RX(SPI6);
        LL_SPI_DisableDMAReq_TX(SPI6);
        cs_hi(g_cur_imu);
        g_dma_done = 1U;
    }
}

void SPI6_BDMA_TX_IRQHandler(void)   /* BDMA_Channel0_IRQn */
{
    if (LL_BDMA_IsActiveFlag_TE0(BDMA))
    {
        LL_BDMA_ClearFlag_TE0(BDMA);
        LL_SPI_DisableDMAReq_RX(SPI6);
        LL_SPI_DisableDMAReq_TX(SPI6);
        cs_hi(g_cur_imu);
        g_dma_done = 1U;
    }
}

/* ════════════════════════════════════════════════════════════════
 *  Инициализация
 * ════════════════════════════════════════════════════════════════ */

void SPI6_IMU_Port_Init(inv_imu_device_t dev[IMU_COUNT])
{
    g_dev = dev;

    /* ── GPIO ─────────────────────────────────────────────────── */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);

    LL_GPIO_InitTypeDef gp = {0};
    gp.Mode       = LL_GPIO_MODE_ALTERNATE;
    gp.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gp.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gp.Pull       = LL_GPIO_PULL_NO;
    gp.Alternate  = LL_GPIO_AF_5;

    gp.Pin = LL_GPIO_PIN_12; LL_GPIO_Init(GPIOC, &gp);  /* SCK  */
    gp.Pin = LL_GPIO_PIN_12; LL_GPIO_Init(GPIOG, &gp);  /* MISO */
    gp.Pin = LL_GPIO_PIN_14; LL_GPIO_Init(GPIOG, &gp);  /* MOSI */

    /* CS пины: OUTPUT PP HIGH (неактивный), PULL_UP */
    gp.Mode       = LL_GPIO_MODE_OUTPUT;
    gp.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    gp.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gp.Pull       = LL_GPIO_PULL_UP;
    gp.Alternate  = 0;

    gp.Pin = LL_GPIO_PIN_9;  LL_GPIO_Init(GPIOC, &gp); LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
    gp.Pin = LL_GPIO_PIN_8;  LL_GPIO_Init(GPIOA, &gp); LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
    gp.Pin = LL_GPIO_PIN_2;  LL_GPIO_Init(GPIOD, &gp); LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_2);
    gp.Pin = LL_GPIO_PIN_3;  LL_GPIO_Init(GPIOD, &gp); LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_3);
    gp.Pin = LL_GPIO_PIN_9;  LL_GPIO_Init(GPIOG, &gp); LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_9);
    gp.Pin = LL_GPIO_PIN_15; LL_GPIO_Init(GPIOG, &gp); LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_15);

    /* ── SPI6 ─────────────────────────────────────────────────── */
    /* SPI6 источник тактирования: по умолчанию APB4.
     * APB4 = SYSCLK/2/2 = 400/2/2 = 100 МГц.
     * DIV8 → 100/8 = 12.5 МГц. ICM-45686 max = 24 МГц — OK. */
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);

    LL_SPI_InitTypeDef sp = {0};
    sp.TransferDirection = LL_SPI_FULL_DUPLEX;
    sp.Mode              = LL_SPI_MODE_MASTER;
    sp.DataWidth         = LL_SPI_DATAWIDTH_8BIT;
    sp.ClockPolarity     = LL_SPI_POLARITY_HIGH;
    sp.ClockPhase        = LL_SPI_PHASE_2EDGE;
    sp.NSS               = LL_SPI_NSS_SOFT;
    sp.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV8;  /* 12.5 МГц */
    sp.BitOrder          = LL_SPI_MSB_FIRST;
    sp.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
    LL_SPI_Init(SPI6, &sp);
    LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_DisableNSSPulseMgt(SPI6);
    /* НЕ включаем SPI здесь — он включается внутри dma_start / imu_read / imu_write */

    /* ── BDMA ─────────────────────────────────────────────────── */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_BDMA);

    /* TX: BDMA Channel0 ← DMAMUX2 REQ_SPI6_TX */
    LL_BDMA_SetPeriphRequest        (BDMA, LL_BDMA_CHANNEL_0, LL_DMAMUX2_REQ_SPI6_TX);
    LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_BDMA_SetChannelPriorityLevel (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PRIORITY_HIGH);
    LL_BDMA_SetMode                 (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MODE_NORMAL);
    LL_BDMA_SetPeriphIncMode        (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PERIPH_NOINCREMENT);
    LL_BDMA_SetMemoryIncMode        (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MEMORY_INCREMENT);
    LL_BDMA_SetPeriphSize           (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PDATAALIGN_BYTE);
    LL_BDMA_SetMemorySize           (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MDATAALIGN_BYTE);
    LL_BDMA_SetPeriphAddress        (BDMA, LL_BDMA_CHANNEL_0, LL_SPI_DMA_GetTxRegAddr(SPI6));

    /* RX: BDMA Channel1 ← DMAMUX2 REQ_SPI6_RX */
    LL_BDMA_SetPeriphRequest        (BDMA, LL_BDMA_CHANNEL_1, LL_DMAMUX2_REQ_SPI6_RX);
    LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_BDMA_SetChannelPriorityLevel (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PRIORITY_VERYHIGH);
    LL_BDMA_SetMode                 (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MODE_NORMAL);
    LL_BDMA_SetPeriphIncMode        (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PERIPH_NOINCREMENT);
    LL_BDMA_SetMemoryIncMode        (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MEMORY_INCREMENT);
    LL_BDMA_SetPeriphSize           (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PDATAALIGN_BYTE);
    LL_BDMA_SetMemorySize           (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MDATAALIGN_BYTE);
    LL_BDMA_SetPeriphAddress        (BDMA, LL_BDMA_CHANNEL_1, LL_SPI_DMA_GetRxRegAddr(SPI6));

    /* NVIC */
    NVIC_SetPriority(BDMA_Channel0_IRQn, 5U);
    NVIC_EnableIRQ  (BDMA_Channel0_IRQn);
    NVIC_SetPriority(BDMA_Channel1_IRQn, 5U);
    NVIC_EnableIRQ  (BDMA_Channel1_IRQn);

    /* ── transport для SDK ────────────────────────────────────── */
    for (uint8_t i = 0; i < IMU_COUNT; i++) {
        dev[i].transport.context    = (void *)&g_idx[i];
        dev[i].transport.read_reg   = imu_read;
        dev[i].transport.write_reg  = imu_write;
        dev[i].transport.sleep_us   = imu_sleep;
        dev[i].transport.serif_type = UI_SPI4;
    }

    /* Первый spi_reset чтобы SPI был в чистом состоянии для polling */
    spi_reset();
}
