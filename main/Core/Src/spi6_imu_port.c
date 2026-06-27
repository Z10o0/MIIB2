/* Core/Src/spi6_imu_port.c
 *
 * SPI6 + BDMA DMA-транспорт: 6 × ICM-45686, STM32H723, LL-драйверы.
 *
 * ВАЖНО: SPI6 на STM32H7 находится в домене D3 (APB4).
 *   Он обслуживается только через BDMA (Basic DMA) + DMAMUX2.
 *   DMA1/DMA2 к SPI6 подключить НЕЛЬЗЯ.
 *
 * Ресурсы:
 *   BDMA Channel0  —  SPI6_TX  (DMAMUX2 REQ = 12)
 *   BDMA Channel1  —  SPI6_RX  (DMAMUX2 REQ = 11)
 *
 * Буферы ОБЯЗАТЕЛЬНО в SRAM4 (домен D3, некешируемая память).
 * BDMA может обращаться только к SRAM4 и к периферии D3.
 * Адрес SRAM4: 0x38000000, размер 16 КБ.
 */

#include "spi6_imu_port.h"
#include "main.h"
#include "imu/inv_imu_driver_advanced.h"
#include "stm32h7xx_ll_bdma.h"
#include "stm32h7xx_ll_utils.h"
#include <string.h>

/* ── CS-пины ─────────────────────────────────────────────────────── */
typedef struct { GPIO_TypeDef *port; uint32_t pin; } cs_pin_t;
static const cs_pin_t cs_table[IMU_COUNT] = {
    { GPIOC, LL_GPIO_PIN_9  },
    { GPIOA, LL_GPIO_PIN_8  },
    { GPIOD, LL_GPIO_PIN_2  },
    { GPIOD, LL_GPIO_PIN_3  },
    { GPIOG, LL_GPIO_PIN_9  },
    { GPIOG, LL_GPIO_PIN_15 },
};
static inline void cs_select  (uint8_t i) { LL_GPIO_ResetOutputPin(cs_table[i].port, cs_table[i].pin); }
static inline void cs_deselect (uint8_t i) { LL_GPIO_SetOutputPin  (cs_table[i].port, cs_table[i].pin); }

/* ── DMA-буферы в SRAM4 (обязательное условие для BDMA) ──────────── */
#define SRAM4_ATTR  __attribute__((section(".sram4"), aligned(4)))
static uint8_t tx_dma_buf[SPI_DMA_BUF_SIZE] SRAM4_ATTR;
static uint8_t rx_dma_buf[SPI_DMA_BUF_SIZE] SRAM4_ATTR;

/* ── Состояние и текущая транзакция ──────────────────────────────── */
static volatile spi_dma_state_t dma_state = SPI_DMA_IDLE;
static spi_dma_xfer_t           cur_xfer;
static volatile uint8_t         poll_done_flag[IMU_COUNT];
static inv_imu_device_t        *g_imu_dev;
static const uint8_t            imu_idx[IMU_COUNT] = {0,1,2,3,4,5};

/* ── Очередь транзакций ──────────────────────────────────────────── */
#define XFER_QUEUE_DEPTH  32U
static spi_dma_xfer_t    xfer_queue[XFER_QUEUE_DEPTH];
static volatile uint32_t xq_head = 0U;
static volatile uint32_t xq_tail = 0U;

/* ════════════════════════════════════════════════════════════════════
 *  Polling SPI для инициализации датчиков (write_reg / read_reg)
 * ════════════════════════════════════════════════════════════════════ */
static uint8_t spi_byte_poll(uint8_t b)
{
    uint32_t t;
    t = 10000U; while (!LL_SPI_IsActiveFlag_TXP(SPI6) && --t) {}
    LL_SPI_TransmitData8(SPI6, b);
    t = 10000U; while (!LL_SPI_IsActiveFlag_RXP(SPI6) && --t) {}
    return LL_SPI_ReceiveData8(SPI6);
}

static int imu_read_reg_polling(void *ctx, uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t idx = *(const uint8_t *)ctx;
    LL_SPI_SetTransferSize(SPI6, (uint32_t)(len + 1U));
    LL_SPI_StartMasterTransfer(SPI6);
    cs_select(idx);
    spi_byte_poll(reg | 0x80U);
    for (uint32_t i = 0; i < len; i++) buf[i] = spi_byte_poll(0x00U);
    uint32_t t = 10000U; while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
    LL_SPI_ClearFlag_EOT(SPI6); LL_SPI_ClearFlag_TXTF(SPI6);
    cs_deselect(idx);
    return 0;
}

static int imu_write_reg_polling(void *ctx, uint8_t reg, const uint8_t *buf, uint32_t len)
{
    uint8_t idx = *(const uint8_t *)ctx;
    LL_SPI_SetTransferSize(SPI6, (uint32_t)(len + 1U));
    LL_SPI_StartMasterTransfer(SPI6);
    cs_select(idx);
    spi_byte_poll(reg & 0x7FU);
    for (uint32_t i = 0; i < len; i++) spi_byte_poll(buf[i]);
    uint32_t t = 10000U; while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
    LL_SPI_ClearFlag_EOT(SPI6); LL_SPI_ClearFlag_TXTF(SPI6);
    cs_deselect(idx);
    return 0;
}

static void imu_sleep_us(uint32_t us)
{
    /* Для инициализации точность не критична */
    uint32_t ms = (us + 999U) / 1000U;
    if (ms == 0U) ms = 1U;
    LL_mDelay(ms);
}

/* ════════════════════════════════════════════════════════════════════
 *  BDMA-транзакция
 * ════════════════════════════════════════════════════════════════════ */
static void bdma_start_transfer(const spi_dma_xfer_t *x)
{
    cur_xfer  = *x;
    dma_state = SPI_DMA_BUSY;

    memcpy(tx_dma_buf, x->tx_buf, x->len);
    cs_select(x->imu_idx);

    /* RX: BDMA Channel1 ─────────────────────────────────────────── */
    LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_1);
    while (LL_BDMA_IsEnabledChannel(BDMA, LL_BDMA_CHANNEL_1)) {}
    LL_BDMA_ClearFlag_TC1(BDMA);
    LL_BDMA_ClearFlag_TE1(BDMA);
    LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_1, (uint32_t)rx_dma_buf);
    LL_BDMA_SetDataLength   (BDMA, LL_BDMA_CHANNEL_1, x->len);
    LL_BDMA_EnableIT_TC     (BDMA, LL_BDMA_CHANNEL_1);
    LL_BDMA_EnableChannel   (BDMA, LL_BDMA_CHANNEL_1);

    /* TX: BDMA Channel0 ─────────────────────────────────────────── */
    LL_BDMA_DisableChannel(BDMA, LL_BDMA_CHANNEL_0);
    while (LL_BDMA_IsEnabledChannel(BDMA, LL_BDMA_CHANNEL_0)) {}
    LL_BDMA_ClearFlag_TC0(BDMA);
    LL_BDMA_ClearFlag_TE0(BDMA);
    LL_BDMA_SetMemoryAddress(BDMA, LL_BDMA_CHANNEL_0, (uint32_t)tx_dma_buf);
    LL_BDMA_SetDataLength   (BDMA, LL_BDMA_CHANNEL_0, x->len);
    LL_BDMA_EnableIT_TE     (BDMA, LL_BDMA_CHANNEL_0);
    LL_BDMA_EnableChannel   (BDMA, LL_BDMA_CHANNEL_0);

    LL_SPI_SetTransferSize(SPI6, x->len);
    LL_SPI_EnableDMAReq_RX(SPI6);
    LL_SPI_EnableDMAReq_TX(SPI6);
    LL_SPI_StartMasterTransfer(SPI6);
}

static void bdma_try_next(void)
{
    if (xq_tail == xq_head) { dma_state = SPI_DMA_IDLE; return; }
    spi_dma_xfer_t next = xfer_queue[xq_tail];
    xq_tail = (xq_tail + 1U) % XFER_QUEUE_DEPTH;
    bdma_start_transfer(&next);
}

/* ── Коллбэк завершения: парсинг FIFO ───────────────────────────── */
static void poll_done_cb(void *ctx)
{
    uint8_t idx = *(const uint8_t *)ctx;
    uint16_t data_len = cur_xfer.len - 1U;
    if (data_len > 0U) {
    	uint8_t local_rx[FIFO_MIRRORING_SIZE];
        memcpy(local_rx, rx_dma_buf + 1U, data_len);
        inv_imu_adv_parse_fifo_data(&g_imu_dev[idx], local_rx, data_len);
    }
    poll_done_flag[idx] = 1U;
}

/* ════════════════════════════════════════════════════════════════════
 *  Публичный API
 * ════════════════════════════════════════════════════════════════════ */
void SPI6_DMA_Transfer(const spi_dma_xfer_t *xfer)
{
    __disable_irq();
    if (dma_state == SPI_DMA_IDLE) {
        __enable_irq();
        bdma_start_transfer(xfer);
        return;
    }
    uint32_t next = (xq_head + 1U) % XFER_QUEUE_DEPTH;
    if (next != xq_tail) { xfer_queue[xq_head] = *xfer; xq_head = next; }
    __enable_irq();
}

void SPI6_PollSensor(uint8_t idx)
{
    /* Шаг 1: прочитать FIFO_COUNT (polling, 2 байта) */
    uint8_t rx2[2] = {0, 0};
    LL_SPI_SetTransferSize(SPI6, 2U);
    LL_SPI_StartMasterTransfer(SPI6);
    cs_select(idx);
    spi_byte_poll(0x2EU | 0x80U);
    rx2[1] = spi_byte_poll(0x00U);
    uint32_t t = 10000U; while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
    LL_SPI_ClearFlag_EOT(SPI6); LL_SPI_ClearFlag_TXTF(SPI6);
    cs_deselect(idx);

    uint16_t fifo_bytes = rx2[1];
    if (fifo_bytes == 0U) return;
    if (fifo_bytes > (SPI_DMA_BUF_SIZE - 1U))
        fifo_bytes = (uint16_t)(SPI_DMA_BUF_SIZE - 1U);

    /* Шаг 2: читать FIFO DATA через BDMA */
    static uint8_t tx_fifo_data[SPI_DMA_BUF_SIZE];
    tx_fifo_data[0] = 0x3FU | 0x80U;  /* FIFO_DATA */
    memset(tx_fifo_data + 1U, 0x00U, fifo_bytes);

    poll_done_flag[idx] = 0U;
    spi_dma_xfer_t xd = {
        .imu_idx  = idx,
        .tx_buf   = tx_fifo_data,
        .rx_buf   = rx_dma_buf,
        .len      = (uint16_t)(fifo_bytes + 1U),
        .done_cb  = poll_done_cb,
        .done_ctx = (void *)&imu_idx[idx],
    };
    SPI6_DMA_Transfer(&xd);

    t = 300000U;
    while (!poll_done_flag[idx] && --t) {}
}

/* ════════════════════════════════════════════════════════════════════
 *  IRQ-обработчики (реализации — см. stm32h7xx_it_6imu.c)
 * ════════════════════════════════════════════════════════════════════ */
void SPI6_BDMA_RX_IRQHandler(void)  /* BDMA_Channel1_IRQn */
{
    if (LL_BDMA_IsActiveFlag_TC1(BDMA))
    {
        LL_BDMA_ClearFlag_TC1(BDMA);
        uint32_t t = 10000U;
        while (!LL_SPI_IsActiveFlag_EOT(SPI6) && --t) {}
        LL_SPI_ClearFlag_EOT(SPI6);
        LL_SPI_ClearFlag_TXTF(SPI6);
        LL_SPI_DisableDMAReq_RX(SPI6);
        LL_SPI_DisableDMAReq_TX(SPI6);
        cs_deselect(cur_xfer.imu_idx);
        if (cur_xfer.done_cb) cur_xfer.done_cb(cur_xfer.done_ctx);
        bdma_try_next();
    }
    if (LL_BDMA_IsActiveFlag_TE1(BDMA))
    {
        LL_BDMA_ClearFlag_TE1(BDMA);
        cs_deselect(cur_xfer.imu_idx);
        LL_SPI_DisableDMAReq_RX(SPI6);
        LL_SPI_DisableDMAReq_TX(SPI6);
        dma_state = SPI_DMA_IDLE;
    }
}

void SPI6_BDMA_TX_IRQHandler(void)  /* BDMA_Channel0_IRQn */
{
    if (LL_BDMA_IsActiveFlag_TE0(BDMA))
    {
        LL_BDMA_ClearFlag_TE0(BDMA);
        cs_deselect(cur_xfer.imu_idx);
        LL_SPI_DisableDMAReq_RX(SPI6);
        LL_SPI_DisableDMAReq_TX(SPI6);
        dma_state = SPI_DMA_IDLE;
    }
}

/* ════════════════════════════════════════════════════════════════════
 *  Инициализация
 * ════════════════════════════════════════════════════════════════════ */
void SPI6_IMU_Port_Init(inv_imu_device_t dev[IMU_COUNT])
{
    g_imu_dev = dev;
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);

    LL_RCC_SetSPIClockSource(LL_RCC_SPI6_CLKSOURCE_PLL3Q);
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SPI6);

    /* SCK = PC12, AF5 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12; GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH; GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO; GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* MISO=PG12, MOSI=PG14 */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12 | LL_GPIO_PIN_14; GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
    LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* CS GPIO */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = 0U;
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);  GPIO_InitStruct.Pin = LL_GPIO_PIN_9; LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);  GPIO_InitStruct.Pin = LL_GPIO_PIN_8; LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_2 | LL_GPIO_PIN_3); GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3; LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    LL_GPIO_SetOutputPin(GPIOG, LL_GPIO_PIN_9 | LL_GPIO_PIN_15); GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_15; LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* SPI6: MODE0, 20 МГц */
    LL_SPI_InitTypeDef SPI_Init = {0};
    SPI_Init.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_Init.Mode              = LL_SPI_MODE_MASTER;
    SPI_Init.DataWidth         = LL_SPI_DATAWIDTH_8BIT;
    SPI_Init.ClockPolarity     = LL_SPI_POLARITY_LOW;
    SPI_Init.ClockPhase        = LL_SPI_PHASE_1EDGE;
    SPI_Init.NSS               = LL_SPI_NSS_SOFT;
    SPI_Init.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV2;
    SPI_Init.BitOrder          = LL_SPI_MSB_FIRST;
    SPI_Init.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
    LL_SPI_Init(SPI6, &SPI_Init);
    LL_SPI_SetStandard(SPI6, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_SetFIFOThreshold(SPI6, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_DisableNSSPulseMgt(SPI6);
    LL_SPI_Enable(SPI6);

    /* BDMA + DMAMUX2 ─────────────────────────────────────────────── */
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_BDMA);

    /* TX: BDMA Channel0, DMAMUX2 REQ=12 (SPI6_TX) */
    LL_BDMA_SetPeriphRequest       (BDMA, LL_BDMA_CHANNEL_0, LL_DMAMUX2_REQ_SPI6_TX);
    LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_BDMA_SetChannelPriorityLevel (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PRIORITY_HIGH);
    LL_BDMA_SetMode                 (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MODE_NORMAL);
    LL_BDMA_SetPeriphIncMode        (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PERIPH_NOINCREMENT);
    LL_BDMA_SetMemoryIncMode        (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MEMORY_INCREMENT);
    LL_BDMA_SetPeriphSize           (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_PDATAALIGN_BYTE);
    LL_BDMA_SetMemorySize           (BDMA, LL_BDMA_CHANNEL_0, LL_BDMA_MDATAALIGN_BYTE);
    LL_BDMA_SetPeriphAddress        (BDMA, LL_BDMA_CHANNEL_0,
        LL_SPI_DMA_GetTxRegAddr(SPI6));

    NVIC_SetPriority(BDMA_Channel0_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    NVIC_EnableIRQ(BDMA_Channel0_IRQn);

    /* RX: BDMA Channel1, DMAMUX2 REQ=11 (SPI6_RX) */
    LL_BDMA_SetPeriphRequest       (BDMA, LL_BDMA_CHANNEL_1, LL_DMAMUX2_REQ_SPI6_RX);
    LL_BDMA_SetDataTransferDirection(BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_BDMA_SetChannelPriorityLevel (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PRIORITY_VERYHIGH);
    LL_BDMA_SetMode                 (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MODE_NORMAL);
    LL_BDMA_SetPeriphIncMode        (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PERIPH_NOINCREMENT);
    LL_BDMA_SetMemoryIncMode        (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MEMORY_INCREMENT);
    LL_BDMA_SetPeriphSize           (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_PDATAALIGN_BYTE);
    LL_BDMA_SetMemorySize           (BDMA, LL_BDMA_CHANNEL_1, LL_BDMA_MDATAALIGN_BYTE);
    LL_BDMA_SetPeriphAddress        (BDMA, LL_BDMA_CHANNEL_1,
        LL_SPI_DMA_GetRxRegAddr(SPI6));

    NVIC_SetPriority(BDMA_Channel1_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(BDMA_Channel1_IRQn);

    /* Транспорт для INV SDK — polling для init */
    for (uint8_t i = 0U; i < IMU_COUNT; i++) {
        dev[i].transport.context    = (void *)&imu_idx[i];
        dev[i].transport.read_reg   = imu_read_reg_polling;
        dev[i].transport.write_reg  = imu_write_reg_polling;
        dev[i].transport.sleep_us   = imu_sleep_us;
        dev[i].transport.serif_type = UI_SPI4;
    }
}
