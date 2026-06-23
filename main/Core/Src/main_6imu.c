/* Core/Src/main_6imu.c
 *
 * 6 × ICM-45686 на SPI6, STM32H723, LL-драйверы.
 *
 * Ключевые отличия от одно-датчикового main.c:
 *   1. Массив из 6 inv_imu_device_t — каждый со своим CS (через context).
 *   2. Один общий sensor_event_cb + cb_current_imu для идентификации датчика.
 *   3. imu_buf[IMU_COUNT][IMU_BUF_SIZE] — двумерный буфер сэмплов.
 *   4. В main loop FIFO опрашивается последовательно для каждого датчика.
 *   5. UART-пакет содержит поле imu_id (uint8_t).
 *   6. SPI2 / старый порт удалены, заменены SPI6.
 *
 * Формат пакетов:
 *   RAW   0x55AA  22 байта  (header+idx+id+pad + 6×int16 raw + temp+pad)
 *   CALIB 0x55BB  36 байт   (header+idx+id+pad + 7×float)
 *
 * ODR: 1600 Гц (акселерометр + гироскоп).
 *   Бюджет UART (CALIB, 10 Мбит/с приёмопередатчик):
 *   6 × 1600 × 36 × 10 = 3.456 Мбит/с → 34.6% нагрузки при 10 Мбит/с.
 *
 * SPI6: тактируется от PLL3R = 20 МГц (DIV8 нет, DIV1 = 20 МГц напрямую).
 *   PLL3: HSE=25 МГц, M=5, N=32, R=1 → PLL3R = 160 МГц
 *   SPI6 источник = PLL3R, BaudRate = DIV8 → 160/8 = 20 МГц.
 *   ICM-45686 поддерживает SPI до 24 МГц — запас есть.
 */

#include "main.h"
#include "spi6_imu_port.h"
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_advanced.h"
#include "imu_calib_flash.h"
#include <string.h>

#define ICM_ACC_FSR_G       2.0f
#define ICM_GYR_FSR_DPS     15.625f
#define ICM_LSB             32768.0f
#define ICM_ACC_SCALE       (ICM_ACC_FSR_G   * 9.80665f / ICM_LSB)
#define ICM_GYR_SCALE       (ICM_GYR_FSR_DPS / ICM_LSB)
#define ICM_TEMP_SCALE      0.5f
#define ICM_TEMP_OFFSET_C   25.0f

/* ODR = 1600 Гц — максимум при 10 Мбит/с приёмопередатчике.
 * Нагрузка: 6 × 1600 × 36 × 10 = 3.456 Мбит/с = 34.6%. */
#define ICM_ODR_ACCEL   ACCEL_CONFIG0_ACCEL_ODR_1600_HZ
#define ICM_ODR_GYRO    GYRO_CONFIG0_GYRO_ODR_1600_HZ

typedef struct __attribute__((packed)) {
    uint16_t header;
    uint32_t sample_idx;
    uint8_t  imu_id;
    uint8_t  _pad0;
    int16_t  acc_x_raw;
    int16_t  acc_y_raw;
    int16_t  acc_z_raw;
    int16_t  gyr_x_raw;
    int16_t  gyr_y_raw;
    int16_t  gyr_z_raw;
    int8_t   temp_raw;
    uint8_t  _pad1;
} icm6_uart_pkt_raw_t;  /* 22 байта */

typedef struct __attribute__((packed)) {
    uint16_t header;
    uint32_t sample_idx;
    uint8_t  imu_id;
    uint8_t  _pad0;
    float    acc_x_ms2;
    float    acc_y_ms2;
    float    acc_z_ms2;
    float    gyr_x_dps;
    float    gyr_y_dps;
    float    gyr_z_dps;
    float    temp_c;
} icm6_uart_pkt_cal_t;  /* 36 байт */

#define PKT_SLOT_SZ6    36U
#define TX_BUF_SZ6      512U

static uint8_t   tx_buf[TX_BUF_SZ6][PKT_SLOT_SZ6];
static uint16_t  tx_len[TX_BUF_SZ6];

volatile uint32_t tx_head      = 0U;
volatile uint32_t tx_tail      = 0U;
volatile uint8_t  uart_tx_busy = 0U;

/* IMU_BUF_SIZE увеличен до 2048 под 1600 Гц:
 * запас ~1280 мс на датчик при polling-задержках. */
#define IMU_BUF_SIZE    2048U

typedef struct __attribute__((packed)) {
    uint32_t sample_idx;
    int16_t  acc_x_raw;
    int16_t  acc_y_raw;
    int16_t  acc_z_raw;
    int16_t  gyr_x_raw;
    int16_t  gyr_y_raw;
    int16_t  gyr_z_raw;
    int8_t   temp_raw;
    uint8_t  imu_id;
} icm6_raw_sample_t;

static icm6_raw_sample_t imu_buf[IMU_COUNT][IMU_BUF_SIZE];
static volatile uint32_t imu_head[IMU_COUNT];
static volatile uint32_t imu_tail[IMU_COUNT];
static volatile uint32_t imu_sample_cnt[IMU_COUNT];

static inv_imu_device_t  imu_dev[IMU_COUNT];
static uint8_t           fifo_buf[IMU_COUNT][FIFO_MIRRORING_SIZE];

volatile device_mode_t g_device_mode = MODE_IDLE;
static volatile uint8_t cb_current_imu = 0U;

volatile uint32_t dma_start_cnt = 0U;
volatile uint32_t dma_tc_cnt    = 0U;

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static int  ICM_Init_All(void);
static void UART4_DMA_Send(uint8_t *data, uint16_t len);
void        UART4_TryStartDMA(void);
static void tx_enqueue(const void *data, uint16_t len);
static void process_sample_raw(const icm6_raw_sample_t *s);
static void process_sample_cal(const icm6_raw_sample_t *s);

static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
    uint8_t id = cb_current_imu;
    if (!(event->sensor_mask & (1 << INV_SENSOR_ACCEL))) return;
    if (!(event->sensor_mask & (1 << INV_SENSOR_GYRO)))  return;

    uint32_t next = (imu_head[id] + 1U) % IMU_BUF_SIZE;
    if (next == imu_tail[id])
        imu_tail[id] = (imu_tail[id] + 1U) % IMU_BUF_SIZE;

    icm6_raw_sample_t *s = &imu_buf[id][imu_head[id]];
    s->sample_idx = imu_sample_cnt[id]++;
    s->imu_id     = id;
    s->acc_x_raw  = event->accel[0];
    s->acc_y_raw  = event->accel[1];
    s->acc_z_raw  = event->accel[2];
    s->gyr_x_raw  = event->gyro[0];
    s->gyr_y_raw  = event->gyro[1];
    s->gyr_z_raw  = event->gyro[2];
    s->temp_raw   = (int8_t)(event->temperature & 0xFF);
    imu_head[id] = next;
}

static int ICM_Init_All(void)
{
    int rc = 0;
    inv_imu_adv_fifo_config_t fifo_cfg;

    SPI6_IMU_Port_Init(imu_dev);

    for (uint8_t i = 0U; i < IMU_COUNT; i++)
    {
        inv_imu_adv_var_t *e = (inv_imu_adv_var_t *)imu_dev[i].adv_var;
        e->sensor_event_cb = sensor_event_cb;

        rc = inv_imu_adv_init(&imu_dev[i]);
        if (rc != 0) return -(int)(i + 1U);

        uint8_t who = 0U;
        rc = inv_imu_get_who_am_i(&imu_dev[i], &who);
        if (rc != 0 || who != INV_IMU_WHOAMI) return -(int)(i + 1U);

        rc |= inv_imu_set_accel_fsr(&imu_dev[i], ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G);
        rc |= inv_imu_set_gyro_fsr(&imu_dev[i],  GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS);
        rc |= inv_imu_set_accel_frequency(&imu_dev[i], ICM_ODR_ACCEL);
        rc |= inv_imu_set_gyro_frequency(&imu_dev[i],  ICM_ODR_GYRO);

        rc |= inv_imu_adv_get_fifo_config(&imu_dev[i], &fifo_cfg);
        fifo_cfg.base_conf.accel_en   = INV_IMU_ENABLE;
        fifo_cfg.base_conf.gyro_en    = INV_IMU_ENABLE;
        fifo_cfg.base_conf.hires_en   = INV_IMU_DISABLE;
        /* При 1600 Гц watermark=16 снижает частоту опросов FIFO
         * с 1600/с до ~100/с на датчик. */
        fifo_cfg.base_conf.fifo_wm_th = 16U;
        fifo_cfg.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_STREAM;
        fifo_cfg.tmst_fsync_en        = INV_IMU_ENABLE;
        fifo_cfg.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
        fifo_cfg.comp_en              = INV_IMU_DISABLE;
        rc |= inv_imu_adv_set_fifo_config(&imu_dev[i], &fifo_cfg);
        rc |= inv_imu_adv_enable_accel_ln(&imu_dev[i]);
        rc |= inv_imu_adv_enable_gyro_ln(&imu_dev[i]);

        if (rc != 0) return -(int)(i + 1U);
        LL_mDelay(10U);
    }
    LL_mDelay(100U);
    return 0;
}

static void tx_enqueue(const void *data, uint16_t len)
{
    uint32_t next = (tx_head + 1U) % TX_BUF_SZ6;
    if (next == tx_tail)
        tx_tail = (tx_tail + 1U) % TX_BUF_SZ6;
    memcpy(tx_buf[tx_head], data, len);
    tx_len[tx_head] = len;
    tx_head = next;
}

static void UART4_DMA_Send(uint8_t *data, uint16_t len)
{
    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    while (LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_0)) {}
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0,
        LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)data);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, len);
    LL_DMA_ClearFlag_TC0(DMA1);
    LL_DMA_ClearFlag_TE0(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
}

void UART4_TryStartDMA(void)
{
    if (uart_tx_busy)       return;
    if (tx_head == tx_tail) return;
    uart_tx_busy = 1U;
    dma_start_cnt++;
    UART4_DMA_Send(tx_buf[tx_tail], tx_len[tx_tail]);
}

static void process_sample_raw(const icm6_raw_sample_t *s)
{
    icm6_uart_pkt_raw_t p;
    p.header     = 0x55AAU;
    p.sample_idx = s->sample_idx;
    p.imu_id     = s->imu_id;
    p._pad0      = 0U;
    p.acc_x_raw  = s->acc_x_raw;
    p.acc_y_raw  = s->acc_y_raw;
    p.acc_z_raw  = s->acc_z_raw;
    p.gyr_x_raw  = s->gyr_x_raw;
    p.gyr_y_raw  = s->gyr_y_raw;
    p.gyr_z_raw  = s->gyr_z_raw;
    p.temp_raw   = s->temp_raw;
    p._pad1      = 0U;
    tx_enqueue(&p, sizeof(icm6_uart_pkt_raw_t));
}

static void process_sample_cal(const icm6_raw_sample_t *s)
{
    icm6_uart_pkt_cal_t p;
    float temp_c = (float)s->temp_raw * ICM_TEMP_SCALE + ICM_TEMP_OFFSET_C;
    float acc_x = (float)s->acc_x_raw * ICM_ACC_SCALE;
    float acc_y = (float)s->acc_y_raw * ICM_ACC_SCALE;
    float acc_z = (float)s->acc_z_raw * ICM_ACC_SCALE;
    float gyr_x = (float)s->gyr_x_raw * ICM_GYR_SCALE;
    float gyr_y = (float)s->gyr_y_raw * ICM_GYR_SCALE;
    float gyr_z = (float)s->gyr_z_raw * ICM_GYR_SCALE;
    uint8_t id = s->imu_id;
    p.acc_x_ms2 = imu_correct_acc(id, IMU_AXIS_X, acc_x, temp_c);
    p.acc_y_ms2 = imu_correct_acc(id, IMU_AXIS_Y, acc_y, temp_c);
    p.acc_z_ms2 = imu_correct_acc(id, IMU_AXIS_Z, acc_z, temp_c);
    p.gyr_x_dps = imu_correct_gyr(id, IMU_AXIS_X, gyr_x, temp_c);
    p.gyr_y_dps = imu_correct_gyr(id, IMU_AXIS_Y, gyr_y, temp_c);
    p.gyr_z_dps = imu_correct_gyr(id, IMU_AXIS_Z, gyr_z, temp_c);
    p.temp_c     = temp_c;
    p.header     = 0x55BBU;
    p.sample_idx = s->sample_idx;
    p.imu_id     = id;
    p._pad0      = 0U;
    tx_enqueue(&p, sizeof(icm6_uart_pkt_cal_t));
}

int main(void)
{
    MPU_Config();
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    NVIC_SetPriority(SysTick_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    SystemClock_Config();
    g_device_mode = MODE_IDLE;
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_UART4_Init();
    volatile int rc = ICM_Init_All();
    (void)rc;

    while (1)
    {
        for (uint8_t i = 0U; i < IMU_COUNT; i++)
        {
            uint16_t fifo_count = 0U;
            cb_current_imu = i;
            if (inv_imu_adv_get_data_from_fifo(&imu_dev[i],
                    fifo_buf[i], &fifo_count) == 0)
            {
                if (fifo_count > 0U)
                    inv_imu_adv_parse_fifo_data(&imu_dev[i],
                        fifo_buf[i], fifo_count);
            }
        }

        for (uint8_t i = 0U; i < IMU_COUNT; i++)
        {
            while (imu_tail[i] != imu_head[i])
            {
                icm6_raw_sample_t s = imu_buf[i][imu_tail[i]];
                imu_tail[i] = (imu_tail[i] + 1U) % IMU_BUF_SIZE;
                switch (g_device_mode)
                {
                case MODE_IDLE:  break;
                case MODE_RAW:   process_sample_raw(&s);  break;
                case MODE_CALIB: process_sample_cal(&s);  break;
                default:         break;
                }
            }
        }
        UART4_TryStartDMA();
    }
}

/* ─────────────────────────────────────────────────────────────────
 *  SystemClock_Config
 *
 *  PLL1: SYSCLK = 400 МГц (HSE=25 МГц, M=4, N=128, P=2)
 *  PLL2: PLL2Q  = 48 МГц  (M=25, N=192, Q=4) — источник UART4
 *  PLL3: PLL3R  = 160 МГц (M=5, N=32, R=1)   — источник SPI6
 *
 *  SPI6: DIV8 → 160/8 = 20 МГц.
 *  UART4: PLL2Q=48 МГц / USARTDIV=1 → 3 МБод (старый UART).
 *  ВНИМАНИЕ: если приёмопередатчик 10 Мбит/с — смени BaudRate
 *  в MX_UART4_Init() на нужное значение и пересчитай PLL2Q.
 * ───────────────────────────────────────────────────────────────── */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {}

    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    while (LL_PWR_IsActiveFlag_VOS() == 0) {}

    LL_RCC_HSE_Enable();
    while (LL_RCC_HSE_IsReady() != 1) {}

    /* ── PLL1: 400 МГц ─────────────────────────────────────────── */
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1Q_Enable();
    LL_RCC_PLL1R_Enable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(4);
    LL_RCC_PLL1_SetN(128);
    LL_RCC_PLL1_SetP(2);
    LL_RCC_PLL1_SetQ(80);
    LL_RCC_PLL1_SetR(2);
    LL_RCC_PLL1_Enable();
    while (LL_RCC_PLL1_IsReady() != 1) {}

    /* ── PLL2: 48 МГц для UART4 ────────────────────────────────
     *  VCO input = 25/25 = 1 МГц
     *  VCO       = 1 × 192 = 192 МГц
     *  PLL2Q     = 192 / 4 = 48 МГц → UART4 @ 3 МБод
     * ────────────────────────────────────────────────────────── */
    LL_RCC_PLL2Q_Enable();
    LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_1_2);
    LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_MEDIUM);
    LL_RCC_PLL2_SetM(25);
    LL_RCC_PLL2_SetN(192);
    LL_RCC_PLL2_SetQ(4);
    LL_RCC_PLL2_Enable();
    while (LL_RCC_PLL2_IsReady() != 1) {}

    /*  PLL3: HSE=25 МГц, M=5, N=32, Q=4 → PLL3Q = 40 МГц
     *  SPI6 источник = PLL3Q, BaudRate = DIV2 → 40/2 = 20 МГц.
     */

    LL_RCC_PLL3Q_Enable();
    LL_RCC_PLL3_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
    LL_RCC_PLL3_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL3_SetM(5);
    LL_RCC_PLL3_SetN(32);
    LL_RCC_PLL3_SetQ(4);    // 160 / 4 = 40 МГц
    LL_RCC_PLL3_Enable();
    while (LL_RCC_PLL3_IsReady() != 1) {}

    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1) {}

    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    LL_Init1msTick(400000000);
    LL_SetSystemCoreClock(400000000);
}

static void MX_UART4_Init(void)
{
    LL_USART_InitTypeDef UART_InitStruct = {0};
    LL_GPIO_InitTypeDef  GPIO_InitStruct = {0};
    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PLL2Q);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_UART4_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);
    UART_InitStruct.PrescalerValue      = LL_USART_PRESCALER_DIV1;
    UART_InitStruct.BaudRate            = 3000000;
    UART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_8B;
    UART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
    UART_InitStruct.Parity              = LL_USART_PARITY_NONE;
    UART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
    UART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    UART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART4, &UART_InitStruct);
    LL_USART_DisableFIFO(UART4);
    LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_ConfigAsyncMode(UART4);
    LL_USART_EnableDMAReq_TX(UART4);
    LL_USART_EnableIT_RXNE(UART4);
    LL_USART_Enable(UART4);
    while ((!LL_USART_IsActiveFlag_TEACK(UART4)) ||
           (!LL_USART_IsActiveFlag_REACK(UART4))) {}
}

static void MX_DMA_Init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    NVIC_SetPriority(DMA1_Stream0_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_SetPriority(UART4_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
    NVIC_EnableIRQ(UART4_IRQn);
}

static void MX_GPIO_Init(void)
{
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOH);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOG);
}

void MPU_Config(void)
{
    LL_MPU_Disable();
    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER0, 0x87, 0x0,
        LL_MPU_REGION_SIZE_4GB | LL_MPU_TEX_LEVEL0 |
        LL_MPU_REGION_NO_ACCESS | LL_MPU_INSTRUCTION_ACCESS_DISABLE |
        LL_MPU_ACCESS_SHAREABLE | LL_MPU_ACCESS_NOT_CACHEABLE |
        LL_MPU_ACCESS_NOT_BUFFERABLE);
    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
