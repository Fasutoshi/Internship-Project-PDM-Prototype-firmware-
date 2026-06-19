//Latch Claude completed
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
#include "gio.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
/* ── 74HC259 pin mapping (gioPORTA) ──────────────────────────── */
#define PIN_A0  0
#define PIN_A1  1
#define PIN_A2  2
#define PIN_EN  3
#define PIN_MEM 4 //RST#
#define PIN_D   5

/* ── Latch state ─────────────────────────────────────────────── */
volatile uint8_t latch_state = 0;   /* bit 0–7 = สถานะ output Q0–Q7 */

/* ── 74HC259 driver ──────────────────────────────────────────── */

void latch_write(uint8_t addr, uint8_t data)
{
    uint32_t dout = gioPORTA->DOUT; // DOUT อ่านสถานะทุกตัวของ gioPORTA //DOUT is register 32-bit store the outputs

    /* เซ็ต address + data ขณะ EN=0 (latch ไม่ฟัง) */
    dout &= ~((1u<<PIN_A0)|(1u<<PIN_A1)|(1u<<PIN_A2)|(1u<<PIN_D)|(1u<<PIN_EN));
    dout |= (( addr       & 1u) << PIN_A0)
          | (((addr >> 1) & 1u) << PIN_A1)
          | (((addr >> 2) & 1u) << PIN_A2)
          | (( data       & 1u) << PIN_D );

    gioPORTA->DOUT = dout;             /* address+data พร้อม EN=0 */

    /* Pulse EN: HIGH → latch จำ → LOW */
    gioPORTA->DSET = (1u << PIN_EN);   /* EN=1 */ //DSET และ DCLR เป็น register พิเศษของ TMS570
    gioPORTA->DCLR = (1u << PIN_EN);   /* EN=0 */ //ที่ออกแบบมาให้ toggle แค่ bit ที่ระบุ โดยไม่ต้อง read-modify-write
}

//
//void latch_write(uint8_t addr, uint8_t data)
//{
//    /* อ่านค่า port ปัจจุบัน */
//    uint32_t port_val = gioGetPort(gioPORTA);
//
//    /* เคลียร์ bit A0, A1, A2, D ก่อน แล้วเซ็ตใหม่ทีเดียว */
//        port_val &= ~((1u<<PIN_A0)|(1u<<PIN_A1)|(1u<<PIN_A2)|(1u<<PIN_D));
//        port_val |= (( addr       & 1u) << PIN_A0)
//                  | (((addr >> 1) & 1u) << PIN_A1)
//                  | (((addr >> 2) & 1u) << PIN_A2)
//                  | ((data        & 1u) << PIN_D);
//
//    /* เขียน port ทีเดียว — ไม่มี intermediate address */
//    gioSetPort(gioPORTA, port_val);
//
//    /* 3. เซ็ต address */
////    gioSetBit(gioPORTA, PIN_A2, (addr >> 2) & 1);
////    gioSetBit(gioPORTA, PIN_A1, (addr >> 1) & 1);
////    gioSetBit(gioPORTA, PIN_A0,  addr       & 1);
//
//    /* 4. เซ็ต D ค่าจริง */
//    gioSetBit(gioPORTA, PIN_D, data);
//    /* 5. Pulse EN: low → latch ฟัง, high → latch จำ */
//    gioSetBit(gioPORTA, PIN_EN, 0);
//    gioSetBit(gioPORTA, PIN_EN, 1);
//}

void latch_set(uint8_t bit)
{
    if (bit > 7) return;
    latch_state |=  (1u << bit);
    latch_write(bit, 1);
}

void latch_clear(uint8_t bit)
{
    if (bit > 7) return;
    latch_state &= ~(1u << bit);
    latch_write(bit, 0);
}

void latch_toggle(uint8_t bit)
{
    if (bit > 7) return;
    if ((latch_state >> bit) & 1u)
        latch_clear(bit);
    else
        latch_set(bit);
}

/* ── Delay ───────────────────────────────────────────────────── */
void delay_ms(unsigned int ms)
{
    volatile unsigned int count, j;
    for (j = 0; j < ms; j++)
        for (count = 0; count < 12727U; count++) {}
}

/* ── CAN RX handler ──────────────────────────────────────────────
 *  Byte[0] = latch bit index (0–7) → toggle output นั้น
 *  Echo กลับ: Byte[0] = index, Byte[1] = สถานะใหม่ (0 หรือ 1)
 * ──────────────────────────────────────────────────────────────*/
void check_can_rx(void)
{
    uint8_t rx_data[8] = {0};

    if (!canIsRxMessageArrived(canREG1, canMESSAGE_BOX2))
        return;

    canGetData(canREG1, canMESSAGE_BOX2, rx_data);

    uint8_t idx = rx_data[0];

    if (idx <= 7)
    {
        latch_toggle(idx);

        /* Echo: index + สถานะหลัง toggle */
        uint8_t echo[8] = {0};
        echo[0] = idx;
        echo[1] = (latch_state >> idx) & 1u;
        canTransmit(canREG1, canMESSAGE_BOX1, echo);
    }
    else if (idx == 10)
        {
            /* Clear ทุก output */
            uint8_t i;
            for (i = 0; i <= 7; i++)
                latch_clear(i);

            uint8_t echo[8] = {0};
            echo[0] = 10;
            echo[1] = 0;   /* latch_state = 0 หมด */
            canTransmit(canREG1, canMESSAGE_BOX1, echo);
        }
}
/* USER CODE END */

uint8   emacAddress[6U] =   {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
uint32  emacPhyAddress  =   0U;

int main(void)
{
/* USER CODE BEGIN (3) */
    canInit();
    gioInit();
    hetInit();

    /* ตั้ง A0, A1, A2, EN, D เป็น output */
        uint32_t pinMask = (1u<<PIN_A0)|(1u<<PIN_A1)|(1u<<PIN_A2)
                          |(1u<<PIN_EN)|(1u<<PIN_D);
        gioSetDirection(gioPORTA, pinMask);
        gioSetBit(gioPORTA, PIN_EN, 0);

        /* เคลียร์ latch ทุก output ตอนเริ่ม */
        uint8_t i;
        for (i = 0; i <= 7; i++)
            latch_clear(i);

        while (1)
        {
            check_can_rx();

//            for (i = 0; i <= 7; i++){
//                latch_toggle(i);
//                delay_ms(500);
//            }
//            for (i = 7; i != 0; i--){
//                latch_toggle(i);
//                delay_ms(500);
//                if (i <= 0)break;
//                        }

            delay_ms(50);
        }
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */

/* USER CODE END */
