/* USER CODE BEGIN (0) */
#include "string.h"
#include "adc.h"
#include "can.h"
#include "gio.h"
#include "FreeRTOS.h"
#include "os_task.h"
/* USER CODE END */

#include "sys_common.h"

/* USER CODE BEGIN (2) */
#define MUX_PORT   gioPORTB
#define MUX_A0     0U
#define MUX_A1     1U
#define MUX_A2     2U
#define MUX_EN     3U
#define VREF_MV    3300UL

void selectMuxChannel(uint8_t ch) {
    gioSetBit(MUX_PORT, MUX_A0, (ch >> 0) & 1);
    gioSetBit(MUX_PORT, MUX_A1, (ch >> 1) & 1);
    gioSetBit(MUX_PORT, MUX_A2, (ch >> 2) & 1);
    gioSetBit(MUX_PORT, MUX_EN, 0);
}

uint16_t read_adc(void) {
    adcData_t adcData;
    adcStartConversion(adcREG1, adcGROUP1);
    volatile uint32_t timeout = 200000;
    while (!adcIsConversionComplete(adcREG1, adcGROUP1)) {
        if (--timeout == 0) return 0xFFFF;
    }
    adcGetData(adcREG1, adcGROUP1, &adcData);
    return (uint16_t)adcData.value;
}

/* ✅ ประกาศ signature ให้ถูกต้อง */
void Task1(void *pvParameters);
/* USER CODE END */

int main(void) {
/* USER CODE BEGIN (3) */
    adcInit();
    canInit();
    gioInit();

    /* ✅ Setup GPIO ก่อน scheduler */
    uint32_t pinMask = (1u<<MUX_A0)|(1u<<MUX_A1)
                     |(1u<<MUX_A2)|(1u<<MUX_EN);
    gioSetDirection(MUX_PORT, pinMask);

    xTaskCreate(Task1, "ADC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    /* ✅ เรียกสุดท้าย — ไม่ return */
    vTaskStartScheduler();

    for(;;);   /* ไม่ควรมาถึงตรงนี้ */
/* USER CODE END */
    return 0;
}

/* USER CODE BEGIN (4) */
void Task1(void *pvParameters) {
    uint8_t  rx_data[8];
    uint8_t  tx_frame[8];
    uint16_t adc_val;
    uint32_t mv;

    for (;;) {
        if (canIsRxMessageArrived(canREG1, canMESSAGE_BOX2)) {
            canGetData(canREG1, canMESSAGE_BOX2, rx_data);

            uint8_t cmd = rx_data[0];
            uint8_t ch  = rx_data[1];

            if (cmd == 0x01 && ch <= 7) {
                selectMuxChannel(ch);
                vTaskDelay(pdMS_TO_TICKS(1));   /*  ใช้ FreeRTOS delay */

                adc_val = read_adc();
                mv      = (uint32_t)adc_val * VREF_MV / 4095;

                /* Frame ADC raw */
                tx_frame[0] = 0xAD;
                tx_frame[1] = ch;
                tx_frame[2] = (adc_val >> 8) & 0x0F;
                tx_frame[3] =  adc_val & 0xFF;
                tx_frame[4] = (adc_val == 0xFFFF) ? 0xEE : 0x00;
                tx_frame[5] = tx_frame[6] = tx_frame[7] = 0;
                canTransmit(canREG1, canMESSAGE_BOX1, tx_frame);

                vTaskDelay(pdMS_TO_TICKS(2));

                /* Frame Voltage (mV) — BOX เดิม แยกด้วย type byte */
                tx_frame[0] = 0xBB;
                tx_frame[1] = ch;
                tx_frame[2] = (mv >> 8) & 0xFF;
                tx_frame[3] =  mv & 0xFF;
                tx_frame[4] = tx_frame[5] = tx_frame[6] = tx_frame[7] = 0;
                canTransmit(canREG1, canMESSAGE_BOX1, tx_frame);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));   /*  yield ให้ task อื่นทำงาน */
    }
}
/* USER CODE END */
