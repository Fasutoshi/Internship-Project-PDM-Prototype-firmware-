/* USER CODE BEGIN (0) */
#include "string.h"
#include "adc.h"
#include "can.h"
#include "gio.h"
#include "het.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"
/* USER CODE END */

#include "sys_common.h"

/* USER CODE BEGIN (2) */

/* ── MUX (gioPORTB) ── */
#define MUX_PORT   gioPORTB
#define MUX_A0     0U
#define MUX_A1     1U
#define MUX_A2     2U
#define MUX_EN     3U
#define VREF_MV    3000UL

/* ── Latch 74HC259 (gioPORTA) ── */
#define LATCH_A0   0
#define LATCH_A1   1
#define LATCH_A2   2
#define LATCH_EN   3
#define LATCH_D    5

/* ── CMD ── */
#define CMD_ADC    0x01
#define CMD_LATCH  0x02

typedef struct {
    uint8_t cmd;
    uint8_t param;
} CanCmd_t;

static QueueHandle_t xAdcQueue;
static QueueHandle_t xLatchQueue;
volatile uint8_t     latch_state = 0;

/* ================================================================
 * Latch driver
 * ================================================================ */
static void latch_write(uint8_t addr, uint8_t data)
{
    uint32_t dout = gioPORTA->DOUT;
    dout &= ~((1u<<LATCH_A0)|(1u<<LATCH_A1)|(1u<<LATCH_A2)
             |(1u<<LATCH_D) |(1u<<LATCH_EN));
    dout |= (( addr       & 1u) << LATCH_A0)
          | (((addr >> 1) & 1u) << LATCH_A1)
          | (((addr >> 2) & 1u) << LATCH_A2)
          | (( data       & 1u) << LATCH_D);
    gioPORTA->DOUT = dout;
    gioPORTA->DSET = (1u << LATCH_EN);
    gioPORTA->DCLR = (1u << LATCH_EN);
}

static void latch_set   (uint8_t b) { latch_state |=  (1u<<b); latch_write(b,1); }
static void latch_clear (uint8_t b) { latch_state &= ~(1u<<b); latch_write(b,0); }
static void latch_toggle(uint8_t b) {
    if ((latch_state>>b)&1u) latch_clear(b); else latch_set(b);
}

/* ================================================================
 * ADC + MUX driver  ← ใช้โค้ดที่ใช้งานได้จริง
 * ================================================================ */
static void selectMuxChannel(uint8_t ch)
{
    gioSetBit(MUX_PORT, MUX_A0, (ch >> 0) & 1);
    gioSetBit(MUX_PORT, MUX_A1, (ch >> 1) & 1);
    gioSetBit(MUX_PORT, MUX_A2, (ch >> 2) & 1);
    gioSetBit(MUX_PORT, MUX_EN, 0);
}

static uint16_t read_adc(void)
{
    adcData_t adcData;
    adcStartConversion(adcREG1, adcGROUP1);
    volatile uint32_t timeout = 200000;
    while (!adcIsConversionComplete(adcREG1, adcGROUP1)) {
        if (--timeout == 0) return 0xFFFF;
    }
    adcGetData(adcREG1, adcGROUP1, &adcData);
    return (uint16_t)adcData.value;
}

/* ================================================================
 * Task 1: CAN RX — แยก cmd ส่งเข้า Queue
 * ================================================================ */
static void vTaskCANRx(void *pvParameters)
{
    uint8_t  rx[8];
    CanCmd_t cmd;

    for (;;) {
        if (canIsRxMessageArrived(canREG1, canMESSAGE_BOX2)) {
            canGetData(canREG1, canMESSAGE_BOX2, rx);
            cmd.cmd   = rx[0];
            cmd.param = rx[1];

            if (cmd.cmd == CMD_ADC && cmd.param <= 7)
                xQueueSend(xAdcQueue, &cmd, 0);
            else if (cmd.cmd == CMD_LATCH && (cmd.param <= 7 || cmd.param == 10 || cmd.param == 11))
                xQueueSend(xLatchQueue, &cmd, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================================================================
 * Task 2: ADC ← ใช้ logic เดียวกับ Task1 ที่ใช้งานได้
 * ================================================================ */
static void vTaskADC(void *pvParameters)
{
    CanCmd_t cmd;
    uint8_t  tx[8];
    uint16_t adc_val;
    uint32_t mv;

    for (;;) {
        if (xQueueReceive(xAdcQueue, &cmd, portMAX_DELAY) == pdTRUE) {

            selectMuxChannel(cmd.param);
            vTaskDelay(pdMS_TO_TICKS(1));

            adc_val = read_adc();
            mv      = (uint32_t)adc_val * VREF_MV / 4095UL;

            /* Frame ADC raw */
            tx[0] = 0xAD;
            tx[1] = cmd.param;
            tx[2] = (adc_val >> 8) & 0x0F;
            tx[3] =  adc_val & 0xFF;
            tx[4] = (adc_val == 0xFFFF) ? 0xEE : 0x00;
            tx[5] = tx[6] = tx[7] = 0;
            canTransmit(canREG1, canMESSAGE_BOX1, tx);

            vTaskDelay(pdMS_TO_TICKS(2));

            /* Frame Voltage */
            tx[0] = 0xBB;
            tx[1] = cmd.param;
            tx[2] = (mv >> 8) & 0xFF;
            tx[3] =  mv & 0xFF;
            tx[4] = tx[5] = tx[6] = tx[7] = 0;
            canTransmit(canREG1, canMESSAGE_BOX1, tx);
        }
    }
}

/* ================================================================
 * Task 3: Latch
 * ================================================================ */
static void vTaskLatch(void *pvParameters)
{
    CanCmd_t cmd;
    uint8_t  tx[8];
    uint8_t  i;

    for (;;) {
        if (xQueueReceive(xLatchQueue, &cmd, portMAX_DELAY) == pdTRUE) {

            if (cmd.param <= 7) {
                latch_toggle(cmd.param);
                tx[0] = 0x02;
                tx[1] = cmd.param;
                tx[2] = (latch_state >> cmd.param) & 1u;
                tx[3] = latch_state;
                tx[4] = tx[5] = tx[6] = tx[7] = 0;
            } else if (cmd.param == 10) {
                for (i = 0; i <= 7; i++) latch_clear(i);
                tx[0] = 0x02; tx[1] = 10;
                tx[2] = 0;    tx[3] = 0;
                tx[4] = tx[5] = tx[6] = tx[7] = 0;
            } else if (cmd.param == 11) {
                for (i = 0; i <= 7; i++) latch_set(i);
                tx[0] = 0x02; tx[1] = 0x0B;
                tx[2] = 0;    tx[3] = 0;
                tx[4] = tx[5] = tx[6] = tx[7] = 0;
            }
            canTransmit(canREG1, canMESSAGE_BOX1, tx);
        }
    }
}
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    adcInit();
    canInit();
    gioInit();
    hetInit();

    /* MUX pins → output */
    uint32_t muxMask = (1u<<MUX_A0)|(1u<<MUX_A1)|(1u<<MUX_A2)|(1u<<MUX_EN);
    gioSetDirection(MUX_PORT, muxMask);

    /* Latch pins → output */
    uint32_t latchMask = (1u<<LATCH_A0)|(1u<<LATCH_A1)|(1u<<LATCH_A2)
                        |(1u<<LATCH_EN)|(1u<<LATCH_D);
    gioSetDirection(gioPORTA, latchMask);
    gioSetBit(gioPORTA, LATCH_EN, 0);

    /* clear latch ทุก output */
    uint8_t i;
    for (i = 0; i <= 7; i++) latch_clear(i);

    /* Queue */
    xAdcQueue   = xQueueCreate(8, sizeof(CanCmd_t));
    xLatchQueue = xQueueCreate(8, sizeof(CanCmd_t));

    /* Tasks */
    xTaskCreate(vTaskCANRx, "CANRx", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskADC,   "ADC",   configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vTaskLatch, "Latch", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    vTaskStartScheduler();
    for (;;);
/* USER CODE END */
    return 0;
}
