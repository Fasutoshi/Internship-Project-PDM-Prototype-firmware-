/** @file sys_main.c 
 *   @brief Application main file
 *   @date 11-Dec-2018
 *   @version 04.07.01
 *
 *   This file contains an empty main function,
 *   which can be used for the application.
 */

/* 
 * Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* USER CODE BEGIN (0) */
#include "can.h"
#include "het.h"         // เพิ่ม Header สำหรับ nHET GPIO
#include "string.h"
#include "stdio.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_semphr.h"
#include "gio.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
/* USER CODE END */

/** @fn void main(void)
 *   @brief Application main function
 *   @note This function is empty by default.
 *
 *   This function is called after startup.
 *   The user can use this function to implement the application.
 */

/* USER CODE BEGIN (2) */
uint8_t myMsg[8];
volatile int value;
adcData_t adcData;

void lightSensorTask(void *pvParameters)
{
    {
        while (1)
        {

            adcStartConversion(adcREG1, adcGROUP1);
            while (!adcIsConversionComplete(adcREG1, adcGROUP1))
                ;
            adcGetData(adcREG1, adcGROUP1, &adcData);
            value = (unsigned int) adcData.value;
            myMsg[0] = (value >> 8U);
            myMsg[1] = value;
            myMsg[2] = myMsg[3] = myMsg[4] = myMsg[5] = myMsg[6] = myMsg[7] = 0;
            canTransmit(canREG1, canMESSAGE_BOX1, myMsg);

            vTaskDelay(pdMS_TO_TICKS(50));
            myMsg[0] = 0xA1;
            myMsg[1] = (value >> 8U);
            myMsg[2] = value;
//        canTransmit(canREG1, canMESSAGE_BOX1, myMsg);
            vTaskDelay(pdMS_TO_TICKS(50));

        }
    }
}

void gauge(void *pvParameters)
{

    while (1)
    {
        adcGetData(adcREG1, adcGROUP1, &adcData);
        if ((unsigned int) adcData.value > 0)
        {
            hetPORT1->DOUT ^= (1U << 0);
            if ((unsigned int) adcData.value > 1000)
            {
                hetPORT1->DOUT ^= (1U << 5);
                if ((unsigned int) adcData.value > 1500)
                {
                    hetPORT1->DOUT ^= (1U << 17);
                    if ((unsigned int) adcData.value > 2000)
                    {
                        hetPORT1->DOUT ^= (1U << 18);
                        if ((unsigned int) adcData.value > 2500)
                        {
                            hetPORT1->DOUT ^= (1U << 25);
                            if ((unsigned int) adcData.value > 3000)
                            {
                                hetPORT1->DOUT ^= (1U << 27);
                                if ((unsigned int) adcData.value > 3500)
                                {
                                    hetPORT1->DOUT ^= (1U << 29);
                                    if ((unsigned int) adcData.value > 4000)
                                    {
                                        hetPORT1->DOUT ^= (1U << 31);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else
                continue;
        }
    }
}

/* USER CODE END */

uint8 emacAddress[6U] = { 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU };
uint32 emacPhyAddress = 0U;

int main(void)
{
    /* USER CODE BEGIN (3) */
    canInit();
    adcInit();
    hetInit();
    gioSetDirection(hetPORT1, 0xFFFFFFFF);

//    mutex = xSemaphoreHandle
    xTaskCreate(lightSensorTask, "lightsensor", configMINIMAL_STACK_SIZE,
    NULL,
                1,
                NULL);
    xTaskCreate(gauge, "demoLEDgauge", configMINIMAL_STACK_SIZE, NULL, 1,
    NULL);
    vTaskStartScheduler();
    while (1)
    {
        ;

    }

    /* USER CODE END */

    return 0;
}

/* USER CODE BEGIN (4) */
/* USER CODE END */
