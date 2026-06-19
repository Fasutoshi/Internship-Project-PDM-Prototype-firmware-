/**
 * @file    can_adc_latch.c
 * @brief   TMS570LS3137 — CAN ADC MUX + Latch library implementation
 */

#include "can_mux_latch.h"

/* ── CMD / frame type ────────────────────────────────────────── */
#define CMD_ADC             0x01U
#define CMD_LATCH           0x02U
#define FRAME_ADC           0xADU
#define FRAME_VOLT          0xBBU
#define FRAME_LATCH         0x02U
#define FRAME_ERR           0xEEU
#define LATCH_CLEAR_ALL     10U
#define LATCH_SET_ALL       11U
#define ADC_MAX             4095UL

/* ── Private state ───────────────────────────────────────────── */
static CAL_Config_t s_cfg;
static QueueHandle_t s_adcQueue;
static QueueHandle_t s_latchQueue;
static volatile uint8_t s_latchState = 0U;

typedef struct
{
    uint8_t cmd;
    uint8_t param;
} prv_Cmd_t;

/* ================================================================
 * Private: Latch driver (74HC259)
 * ================================================================ */
static void prv_LatchWrite(uint8_t addr, uint8_t data)
{
    uint32_t dout = gioPORTA->DOUT;
    dout &= ~((1u << s_cfg.latchA0Pin) | (1u << s_cfg.latchA1Pin)
            | (1u << s_cfg.latchA2Pin) | (1u << s_cfg.latchDPin)
            | (1u << s_cfg.latchEnPin));
    dout |= ((addr & 1u) << s_cfg.latchA0Pin)
            | (((addr >> 1u) & 1u) << s_cfg.latchA1Pin)
            | (((addr >> 2u) & 1u) << s_cfg.latchA2Pin)
            | ((data & 1u) << s_cfg.latchDPin);
    gioPORTA->DOUT = dout;
    gioPORTA->DSET = (1u << s_cfg.latchEnPin); /* EN pulse HIGH */
    gioPORTA->DCLR = (1u << s_cfg.latchEnPin); /* EN pulse LOW  */
}

static void prv_LatchSet(uint8_t bit)
{
    s_latchState |= (1u << bit);
    prv_LatchWrite(bit, 1u);
}

static void prv_LatchClear(uint8_t bit)
{
    s_latchState &= ~(1u << bit);
    prv_LatchWrite(bit, 0u);
}

static void prv_LatchToggle(uint8_t bit)
{
    if ((s_latchState >> bit) & 1u)
        prv_LatchClear(bit);
    else
        prv_LatchSet(bit);
}

/* ================================================================
 * Private: MUX + ADC driver
 * ================================================================ */
static void prv_SelectMuxChannel(uint8_t ch)
{
    gioSetBit(s_cfg.muxPort, s_cfg.muxA0Pin, (ch >> 0u) & 1u);
    gioSetBit(s_cfg.muxPort, s_cfg.muxA1Pin, (ch >> 1u) & 1u);
    gioSetBit(s_cfg.muxPort, s_cfg.muxA2Pin, (ch >> 2u) & 1u);
    gioSetBit(s_cfg.muxPort, s_cfg.muxEnPin, 0u); /* EN# LOW = enable */
}

static uint16_t prv_ReadADC(void)
{
    adcData_t adcData;
    adcStartConversion(adcREG1, adcGROUP1);
    volatile uint32_t timeout = 200000UL;
    while (!adcIsConversionComplete(adcREG1, adcGROUP1))
    {
        if (--timeout == 0UL)
            return 0xFFFFU;
    }
    adcGetData(adcREG1, adcGROUP1, &adcData);
    return (uint16_t) adcData.value;
}

/* ================================================================
 * Private: FreeRTOS Tasks
 * ================================================================ */
static void prv_TaskCANRx(void *pvParameters)
{
    uint8_t rx[8];
    prv_Cmd_t cmd;
    (void) pvParameters;

    for (;;)
    {
        if (canIsRxMessageArrived(s_cfg.canReg, s_cfg.canRxBox))
        {
            canGetData(s_cfg.canReg, s_cfg.canRxBox, rx);
            cmd.cmd = rx[0];
            cmd.param = rx[1];

            if (cmd.cmd == CMD_ADC && cmd.param <= 7U)
            {
                xQueueSend(s_adcQueue, &cmd, 0);
            }
            else if (cmd.cmd == CMD_LATCH
                    && (cmd.param <= 7U || cmd.param == LATCH_CLEAR_ALL
                            || cmd.param == LATCH_SET_ALL))
            {
                xQueueSend(s_latchQueue, &cmd, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void prv_TaskADC(void *pvParameters)
{
    prv_Cmd_t cmd;
    uint8_t tx[8];
    uint16_t adc;
    uint32_t mv;
    (void) pvParameters;

    for (;;)
    {
        if (xQueueReceive(s_adcQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {

            prv_SelectMuxChannel(cmd.param);
            vTaskDelay(pdMS_TO_TICKS(1));

            adc = prv_ReadADC();
            mv = (uint32_t) adc * s_cfg.vrefMv / ADC_MAX;

            /* Frame 1: ADC raw (0xAD) */
            tx[0] = FRAME_ADC;
            tx[1] = cmd.param;
            tx[2] = (adc >> 8u) & 0x0Fu;
            tx[3] = adc & 0xFFu;
            tx[4] = (adc == 0xFFFFU) ? FRAME_ERR : 0x00U;
            tx[5] = tx[6] = tx[7] = 0U;
            canTransmit(s_cfg.canReg, s_cfg.canTxBox, tx);

            vTaskDelay(pdMS_TO_TICKS(2));

            /* Frame 2: Voltage mV (0xBB) */
            tx[0] = FRAME_VOLT;
            tx[1] = cmd.param;
            tx[2] = (mv >> 8u) & 0xFFu;
            tx[3] = mv & 0xFFu;
            tx[4] = tx[5] = tx[6] = tx[7] = 0U;
            canTransmit(s_cfg.canReg, s_cfg.canTxBox, tx);
        }
    }
}

static void prv_TaskLatch(void *pvParameters)
{
    prv_Cmd_t cmd;
    uint8_t tx[8];
    uint8_t i;
    (void) pvParameters;

    for (;;)
    {
        if (xQueueReceive(s_latchQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {

            tx[4] = tx[5] = tx[6] = tx[7] = 0U;

            if (cmd.param <= 7U)
            {
                prv_LatchToggle(cmd.param);
                tx[0] = FRAME_LATCH;
                tx[1] = cmd.param;
                tx[2] = (s_latchState >> cmd.param) & 1u;
                tx[3] = s_latchState;
            }
            else if (cmd.param == LATCH_CLEAR_ALL)
            {
                for (i = 0U; i <= 7U; i++)
                    prv_LatchClear(i);
                tx[0] = FRAME_LATCH;
                tx[1] = LATCH_CLEAR_ALL;
                tx[2] = 0xFFU;
                tx[3] = s_latchState; /* = 0x00 */
            }
            else if (cmd.param == LATCH_SET_ALL)
            {
                for (i = 0U; i <= 7U; i++)
                    prv_LatchSet(i);
                tx[0] = FRAME_LATCH;
                tx[1] = LATCH_SET_ALL;
                tx[2] = 0xFFU;
                tx[3] = s_latchState; /* = 0xFF */
            }
            canTransmit(s_cfg.canReg, s_cfg.canTxBox, tx);
        }
    }
}

/* ================================================================
 * Public API
 * ================================================================ */
void CAL_DefaultConfig(CAL_Config_t *cfg)
{
    /* ADC */
    cfg->vrefMv = 3000UL;

    /* MUX — gioPORTB pin 0-3 */
    cfg->muxPort = gioPORTB;
    cfg->muxA0Pin = 0U;
    cfg->muxA1Pin = 1U;
    cfg->muxA2Pin = 2U;
    cfg->muxEnPin = 3U;

    /* Latch — gioPORTA pin 0,1,2,3,5 */
    cfg->latchA0Pin = 0U;
    cfg->latchA1Pin = 1U;
    cfg->latchA2Pin = 2U;
    cfg->latchEnPin = 3U;
    cfg->latchDPin = 5U;

    /* CAN */
    cfg->canReg = canREG1;
    cfg->canTxBox = canMESSAGE_BOX1;
    cfg->canRxBox = canMESSAGE_BOX2;

    /* FreeRTOS */
    cfg->queueDepth = 8U;
    cfg->stackSize = (uint32_t) configMINIMAL_STACK_SIZE;
}

void CAL_Init(const CAL_Config_t *cfg)
{
    uint8_t i;

    /* บันทึก config */
    s_cfg = *cfg;

    /* MUX GPIO → output */
    uint32_t muxMask = (1u << s_cfg.muxA0Pin) | (1u << s_cfg.muxA1Pin)
            | (1u << s_cfg.muxA2Pin) | (1u << s_cfg.muxEnPin);
    gioSetDirection(s_cfg.muxPort, muxMask);

    /* Latch GPIO → output */
    uint32_t latchMask = (1u << s_cfg.latchA0Pin) | (1u << s_cfg.latchA1Pin)
            | (1u << s_cfg.latchA2Pin) | (1u << s_cfg.latchEnPin)
            | (1u << s_cfg.latchDPin);
    gioSetDirection(gioPORTA, latchMask);
    gioSetBit(gioPORTA, s_cfg.latchEnPin, 0u);

    /* Clear latch ทุก output */
    s_latchState = 0U;
    for (i = 0U; i <= 7U; i++)
        prv_LatchClear(i);

    /* FreeRTOS Queue */
    s_adcQueue = xQueueCreate(s_cfg.queueDepth, sizeof(prv_Cmd_t));
    s_latchQueue = xQueueCreate(s_cfg.queueDepth, sizeof(prv_Cmd_t));

    /* FreeRTOS Tasks */
    xTaskCreate(prv_TaskCANRx, "CANRx", s_cfg.stackSize, NULL, 3, NULL);
    xTaskCreate(prv_TaskADC, "ADC", s_cfg.stackSize, NULL, 1, NULL);
    xTaskCreate(prv_TaskLatch, "Latch", s_cfg.stackSize, NULL, 2, NULL);
}

uint8_t CAL_GetLatchState(void)
{
    return s_latchState;
}

uint16_t CAL_ReadADC(uint8_t ch)
{
    if (ch > 7U)
        return 0xFFFFU;
    prv_SelectMuxChannel(ch);
    vTaskDelay(pdMS_TO_TICKS(1));
    return prv_ReadADC();
}

uint32_t CAL_RawToMv(uint16_t raw)
{
    if (raw == 0xFFFFU)
        return 0xFFFFFFFFUL;
    return (uint32_t) raw * s_cfg.vrefMv / ADC_MAX;
}
