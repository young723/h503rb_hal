/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively Software is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#ifndef _INV_ICM4X6XX_H_
#define _INV_ICM4X6XX_H_

#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <stdbool.h>

#include "bsp_hardware.h"

#define INV_SLAVE							0x69

#define INV_USE_SPI

#define FIFO_WM_MODE_EN                      0 //1:fifo mode 0:dri mode
#if defined(INV_USE_SPI)
#define SPI_MODE_EN                          1 //1:spi 0:i2c
#else
#define SPI_MODE_EN                          0 //1:spi 0:i2c
#endif
#define INT_LATCH_EN                         0 //1:latch mode 0:pulse mode
#define INT_ACTIVE_HIGH_EN                   1 //1:active high 0:active low
//#define ICM4X6XX_USE_INT2

#define SUPPORT_PEDOMETER                    0
#define SUPPORT_WOM                          0

#define MAX_RECV_PACKET                      4	//16	//100


#define ICM4X6XX_DEBUG

#ifdef ICM4X6XX_DEBUG
#define icm_log		qst_logi
#else
#define icm_log(...)
#endif

typedef enum {
    ACC = 0,
    GYR,
    TEMP,
    #if SUPPORT_WOM
    WOM,
    #endif
    NUM_OF_SENSOR,
} SensorType_t;

struct accGyroData {
    uint8_t sensType;
    float x, y, z;
    uint64_t timeStamp;
};

struct accGyroDataPacket {
    uint8_t accOutSize;
    uint8_t gyroOutSize;
    uint64_t timeStamp;
    float temperature;
    struct accGyroData outBuf[MAX_RECV_PACKET];
    uint32_t magicNum;
};

int inv_icm4x6xx_initialize(void);
void inv_icm4x6xx_set_serif(int (*read)(void *, uint8_t, uint8_t *, uint8_t),
                            int (*write)(void *, uint8_t, uint8_t *, uint8_t));
void inv_icm4x6xx_set_delay(void (*delay_ms)(uint32_t), void (*delay_us)(uint32_t));
int inv_icm4x6xx_acc_enable(void);
int inv_icm4x6xx_gyro_enable(void);

int inv_icm4x6xx_acc_disable(void);
int inv_icm4x6xx_gyro_disable(void);

int inv_icm4x6xx_acc_set_rate(float odr_hz, uint16_t watermark);
int inv_icm4x6xx_gyro_set_rate(float odr_hz, uint16_t watermark);

int inv_icm4x6xx_get_rawdata(struct accGyroDataPacket *dataPacket);

bool sensor_icm4x6xx_get_raw_gyro_acc(struct accGyroDataPacket *dataPacket);

#if SUPPORT_SELFTEST
int inv_icm4x6xx_acc_selftest(bool *result);
int inv_icm4x6xx_gyro_selftest(bool *result);
#endif

#if SUPPORT_WOM
int inv_icm4x6xx_wom_enable(void);
int inv_icm4x6xx_wom_disable(void);
int inv_icm4x6xx_wom_get_event(bool *detect);
#endif

void inv_icm4x6xx_read_data(float acc[3], float gyr[3]);

//void inv_icm4x6xx_dumpRegs(void);

#endif
