/**
 * @file    can_adc_latch.h
 * @brief   TMS570LS3137 — CAN ADC MUX + Latch library
 *
 * วิธีใช้งาน:
 *   1. CAL_DefaultConfig(&cfg)   โหลดค่า default
 *   2. แก้ cfg.xxx เฉพาะที่ต่างจาก default
 *   3. CAL_Init(&cfg)            เริ่มต้น library
 *   4. vTaskStartScheduler()
 *
 * CAN Protocol (ID ขึ้นกับ HALCoGen config):
 *   PC → MCU  byte[0]=cmd  byte[1]=param
 *   MCU → PC  byte[0]=type frame
 *
 * CMD:
 *   0x01  ADC    param = channel 0-7
 *   0x02  Latch  param = 0-7 toggle | 10 clear all | 11 set all
 *
 * Response frame type:
 *   0xAD  ADC raw   [1]=ch [2]=hi [3]=lo [4]=0xEE if error
 *   0xBB  Voltage   [1]=ch [2]=mV_hi [3]=mV_lo
 *   0x02  Latch ack [1]=bit [2]=bit_state [3]=latch_state
 */

#ifndef CAN_ADC_LATCH_H
#define CAN_ADC_LATCH_H

#include "sys_common.h"
#include "gio.h"
#include "can.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "os_task.h"
#include "os_queue.h"

/* ================================================================
 * Config struct — user กำหนดค่าทั้งหมดตรงนี้
 * ================================================================ */
typedef struct {
    /* ── ADC ───────────────────────────────────── */
    uint32_t    vrefMv;         /* Vref ในหน่วย mV  เช่น 3000, 3300 */

    /* ── MUX 74HC4051 ───────────────────────────── */
    gioPORT_t  *muxPort;        /* port เช่น gioPORTB                */
    uint8_t     muxA0Pin;       /* pin ของสัญญาณ A0                  */
    uint8_t     muxA1Pin;       /* pin ของสัญญาณ A1                  */
    uint8_t     muxA2Pin;       /* pin ของสัญญาณ A2                  */
    uint8_t     muxEnPin;       /* pin ของสัญญาณ EN# (active LOW)    */

    /* ── Latch 74HC259 ──────────────────────────── */
    uint8_t     latchA0Pin;     /* pin ของสัญญาณ A0                  */
    uint8_t     latchA1Pin;     /* pin ของสัญญาณ A1                  */
    uint8_t     latchA2Pin;     /* pin ของสัญญาณ A2                  */
    uint8_t     latchEnPin;     /* pin ของสัญญาณ EN (active HIGH)    */
    uint8_t     latchDPin;      /* pin ของสัญญาณ D (data)            */

    /* ── CAN ────────────────────────────────────── */
    canBASE_t  *canReg;         /* register เช่น canREG1             */
    uint32_t    canTxBox;       /* message box TX เช่น canMESSAGE_BOX1 */
    uint32_t    canRxBox;       /* message box RX เช่น canMESSAGE_BOX2 */

    /* ── FreeRTOS ───────────────────────────────── */
    uint32_t    queueDepth;     /* ความลึก queue (default 8)         */
    uint32_t    stackSize;      /* stack size แต่ละ task (words)     */
} CAL_Config_t;

/* ================================================================
 * Public API
 * ================================================================ */

/**
 * @brief  โหลดค่า default เข้า config struct
 *         เรียกก่อน CAL_Init() เสมอ แล้วค่อยแก้เฉพาะที่ต้องการ
 * @param  cfg  pointer ไปยัง CAL_Config_t ที่ต้องการโหลด
 */
void CAL_DefaultConfig(CAL_Config_t *cfg);

/**
 * @brief  เริ่มต้น library ทั้งหมด
 *         - ตั้ง GPIO direction
 *         - Clear latch Q0-Q7
 *         - สร้าง FreeRTOS Queue และ Task
 *         เรียก 1 ครั้งใน main() ก่อน vTaskStartScheduler()
 * @param  cfg  pointer ไปยัง config (ใช้ค่าจาก CAL_DefaultConfig + แก้เพิ่ม)
 */
void CAL_Init(const CAL_Config_t *cfg);

/**
 * @brief  อ่านสถานะ latch ปัจจุบัน
 * @return bit mask Q0-Q7  (bit0=Q0 ... bit7=Q7)
 */
uint8_t CAL_GetLatchState(void);

/**
 * @brief  อ่านค่า ADC จาก MUX channel โดยตรง (ไม่ผ่าน CAN)
 *         ใช้สำหรับ local read หรือ debug
 * @param  ch  channel 0-7
 * @return ADC raw 0-4095 หรือ 0xFFFF ถ้า timeout
 */
uint16_t CAL_ReadADC(uint8_t ch);

/**
 * @brief  แปลง ADC raw เป็น mV ตาม Vref ที่ตั้งไว้
 * @param  raw  ค่า ADC 0-4095
 * @return แรงดันในหน่วย mV
 */
uint32_t CAL_RawToMv(uint16_t raw);

#endif /* CAN_ADC_LATCH_H */
