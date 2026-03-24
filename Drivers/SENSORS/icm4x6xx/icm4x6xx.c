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

#include "icm4x6xx.h"
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define GYRO_STALL_PATCH_ENABLE       1

#define NUM_TODISCARD                 0
#define SELFTEST_SAMPLES_NUMBER       200
#define ICM4X6XX_MAX_FIFO_SIZE        2000  // Max FIFO count size

#define G                             9.80665
#define PI                            3.141592
#define KSCALE_ACC_8G_RANGE           0.002394202f  // ACC_range * 9.81f / 65536.0f;
#define KSCALE_GYRO_2000_RANGE        0.001065264f  // GYR_range * M_PI / (180.0f * 65536.0f);   弧度/秒
#define KSCALE_GYRO_2000_RANGE_DPS    0.061035156f  // GYR_range * 4000.0f / 65536.0f;     度/秒
#define TEMP_SENSITIVITY_FIFO         0.4831f
#define TEMP_SENSITIVITY_REG          0.007548f
#define ROOM_TEMP_OFFSET              25 //Celsius degree
#define max(x, y)                     (x > y ? x : y)
#define ARRAY_SIZE(a)                 (sizeof((a)) / sizeof((a)[0]))

#ifndef abs
#define abs(a)                        ((a) > 0 ? (a) : -(a)) /*!< Absolute value */
#endif

/* ICM4X6XX register BANK */
/* bank 0 */
#define REG_CHIP_CONFIG             0x11
#define BIT_SOFT_RESET_CONFIG       0x01
#define REG_DRIVE_CONFIG            0x13
#define BIT_SPI_SLEW_RATE_MASK      0x07
#define BIT_I2C_SLEW_RATE_MASK      0x38
#define BIT_I2C_SLEW_RATE_SHIFT     3
#define REG_INT_CONFIG              0x14
#define BIT_INT1_POLARITY_MASK      0x01
#define BIT_INT1_ACTIVE_HIGH        0x01
#define BIT_INT1_DRIVE_CIRCUIT_MASK 0x02
#define BIT_INT1_PUSH_PULL          0x02
#define BIT_INT1_MODE_MASK          0x04
#define BIT_INT1_LATCHED_MODE       0x04
#define BIT_INT2_POLARITY_MASK      0x08
#define BIT_INT2_ACTIVE_HIGH        0x08
#define BIT_INT2_DRIVE_CIRCUIT_MASK 0x10
#define BIT_INT2_PUSH_PULL          0x10
#define BIT_INT2_MODE_MASK          0x20
#define BIT_INT2_LATCHED_MODE       0x20
#define REG_FIFO_CONFIG             0x16
#define BIT_FIFO_MODE_SHIFT         6
#define BIT_FIFO_MODE_CTRL_MASK     (3 << BIT_FIFO_MODE_SHIFT)
#define BIT_FIFO_MODE_CTRL_BYPASS   (0 << BIT_FIFO_MODE_SHIFT)
#define BIT_FIFO_MODE_CTRL_STREAM   (1 << BIT_FIFO_MODE_SHIFT)
#define BIT_FIFO_MODE_CTRL_SNAPSHOT (2 << BIT_FIFO_MODE_SHIFT)
#define REG_TEMP_DATA1              0x1D
#define REG_ACCEL_DATA_X1           0x1F
#define REG_GYRO_DATA_X1            0x25
#define REG_INT_STATUS              0x2D
#define BIT_INT_FIFO_FULL           0x02
#define BIT_INT_FIFO_WM             0x04
#define BIT_INT_DRDY                0x08
#define BIT_RESET_DONE              0x10
#define REG_FIFO_COUNT_H            0x2E
#define REG_FIFO_DATA               0x30
#define REG_APEX_DATA0              0x31
#define REG_APEX_DATA2              0x33
#define REG_APEX_DATA3              0x34
#define BIT_DMP_IDLE                0x04
#define BIT_ACTIVITY_CLASS_MASK     0x03
#define BIT_ACTIVITY_CLASS_UNKNOWN  0x00
#define BIT_ACTIVITY_CLASS_WALK     0x01
#define BIT_ACTIVITY_CLASS_RUN      0x02
#define REG_INT_STATUS2             0x37
#define BIT_SMD_INT                 0x08
#define BIT_INT_WOM_MASK            0x07
#define BIT_INT_WOM_Z               0x04
#define BIT_INT_WOM_Y               0x02
#define BIT_INT_WOM_X               0x01
#define REG_INT_STATUS3             0x38
#define BIT_INT_STEP_DET            0x20
#define BIT_INT_STEP_CNT_OVF        0x10
#define BIT_INT_TILT_DET            0x08
#define BIT_INT_WAKE_DET            0x04
#define BIT_INT_LOW_G_DET           0x04
#define BIT_INT_SLEEP_DET           0x02
#define BIT_INT_HIGH_G_DET          0x02
#define BIT_INT_TAP_DET             0x01
#define REG_SIGNAL_PATH_RESET       0x4B
#define BIT_DMP_INIT_EN             0x40
#define BIT_DMP_MEM_RESET_EN        0x20
#define BIT_ABORT_AND_RESET         0x08
#define BIT_FIFO_FLUSH              0x02
#define REG_INTF_CONFIG0            0x4C
#define BIT_FIFO_HOLD_LAST_DATA_EN  0x80
#define BIT_FIFO_COUNT_REC_MASK     0x40
#define BIT_FIFO_COUNT_RECORD_MODE  0x40
#define BIT_FIFO_COUNT_ENDIAN_MASK  0x20
#define BIT_FIFO_COUNT_BIG_ENDIAN   0x20
#define BIT_SENSOR_DATA_ENDIAN_MASK 0x10
#define BIT_SENSOR_DATA_BIG_ENDIAN  0x10
#define BIT_UI_SIFS_CFG_MASK        0x03
#define BIT_UI_SIFS_CFG_I2C_DIS     0x03
#define BIT_UI_SIFS_CFG_SPI_DIS     0x02
#define REG_INTF_CONFIG1            0x4D
#define BIT_ACCEL_LP_CLK_SEL_MASK   0x08
#define BIT_ACCEL_LP_CLK_SEL_RC     0x08
#define BIT_CLKSEL_MASK             0x03
#define BIT_CLKSEL_RC               0x00
#define BIT_CLKSEL_PLL              0x01
#define BIT_CLKSEL_DIS              0x03
#define REG_PWR_MGMT0               0x4E
#define BIT_TEMP_DIS                0x20
#define BIT_IDLE_MASK               0x10
#define BIT_GYRO_MODE_MASK          0x0C
#define BIT_GYRO_MODE_STANDBY       0x04
#define BIT_GYRO_MODE_LN            0x0C
#define BIT_ACCEL_MODE_MASK         0x03
#define BIT_ACCEL_MODE_LP           0x02
#define BIT_ACCEL_MODE_LN           0x03
#define REG_GYRO_CONFIG0            0x4F
#define BIT_GYRO_FSR_MASK           0xE0
#define BIT_GYRO_FSR_SHIFT          5
#define BIT_GYRO_ODR_MASK           0x0F
#define REG_ACCEL_CONFIG0           0x50
#define BIT_ACCEL_FSR_MASK          0xE0
#define BIT_ACCEL_FSR_SHIFT         5
#define BIT_ACCEL_ODR_MASK          0x0F
#define REG_GYRO_CONFIG1            0x51
#define BIT_GYRO_AVG_FILT_8K_HZ     0x10
#define BIT_GYRO_FILT_ORD_SHIFT     0x02
#define BIT_GYRO_FILT_ORD_MASK      0x0C
#define REG_GYRO_ACCEL_CONFIG0      0x52
#define BIT_ACCEL_FILT_BW_MASK      0xF0
#define BIT_ACCEL_FILT_BW_SHIFT     4
#define BIT_GYRO_FILT_BW_MASK       0x0F
#define REG_ACCEL_CONFIG1           0x53
#define BIT_ACCEL_AVG_FILT_8K_HZ    0x01
#define BIT_ACCEL_FILT_ORD_SHIFT    0x03
#define BIT_ACCEL_FILT_ORD_MASK     0x18
#define REG_APEX_CONFIG0            0x56
#define BIT_DMP_POWER_SAVE_EN       0x80
#define BIT_PED_ENABLE_EN           0x20
#define BIT_R2W_EN_EN               0x08
#define BIT_LOW_G_EN                0x08
#define BIT_HIGH_G_EN               0x04
#define BIT_DMP_ODR_MASK            0x03
#define BIT_DMP_ODR_25HZ            0x00
#define BIT_DMP_ODR_50HZ            0x02
#define BIT_DMP_ODR_100HZ           0x03
#define REG_SMD_CONFIG              0x57
#define BIT_WOM_INT_MODE_MASK       0x08
#define BIT_WOM_INT_MODE_AND        0x08
#define BIT_WOM_MODE_MASK           0x04
#define BIT_WOM_MODE_COMPARE_PRE    0x04
#define BIT_SMD_MODE_MASK           0x03
#define BIT_SMD_MODE_WOM            0x01
#define BIT_SMD_MODE_SHORT          0x02
#define BIT_SMD_MODE_LONG           0x03
#define REG_FIFO_CONFIG_1           0x5F
#define BIT_FIFO_RESUME_PARTIAL_RD_MASK 0x40
#define BIT_FIFO_WM_GT_TH_MASK          0x20
#define BIT_FIFO_TMST_FSYNC_EN_MASK     0x08
#define BIT_FIFO_TEMP_EN_MASK           0x04
#define BIT_FIFO_TEMP_EN_EN             0x04
#define BIT_FIFO_GYRO_EN_MASK           0x02
#define BIT_FIFO_GYRO_EN_EN             0x02
#define BIT_FIFO_ACCEL_EN_MASK          0x01
#define BIT_FIFO_ACCEL_EN_EN            0x01
#define REG_FIFO_WM_TH_L            0x60
#define REG_FIFO_WM_TH_H            0x61
#define REG_INT_CONFIG0             0x63
#define BIT_INT_FIFO_THS_CLR_MASK   0x0C
#define BIT_INT_FIFO_THS_CLR_SHIFT  2
#define BIT_INT_FIFO_FULL_CLR_MASK  0x03
#define REG_INT_CONFIG1             0x64
#define BIT_INT_ASY_RST_DIS         0x00
#define REG_INT_SOURCE0             0x65
#define BIT_INT1_DRDY_EN            0x08
#define BIT_INT1_WM_EN              0x04
#define BIT_INT1_FIFO_FULL_EN       0x02
#define REG_INT_SOURCE1             0x66
#define BIT_INT1_WOM_EN_MASK        0x07
#define BIT_INT1_SMD_EN             0x08
#define REG_INT_SOURCE3             0x68
#define BIT_INT2_DRDY_EN            0x08
#define BIT_INT2_WM_EN              0x04
#define BIT_INT2_FIFO_FULL_EN       0x02
#define REG_INT_SOURCE4             0x69
#define BIT_INT2_WOM_EN_MASK        0x07
#define BIT_INT2_SMD_EN             0x08
#define REG_SELF_TEST_CONFIG        0x70
#define BIT_ACCEL_ST_POWER_EN       0x40
#define BIT_ACCEL_ST_EN_MASK        0x38
#define BIT_ACCEL_Z_ST_EN           0x20
#define BIT_ACCEL_Y_ST_EN           0x10
#define BIT_ACCEL_X_ST_EN           0x08
#define BIT_GYRO_ST_EN_MASK         0x07
#define BIT_GYRO_Z_ST_EN            0x04
#define BIT_GYRO_Y_ST_EN            0x02
#define BIT_GYRO_X_ST_EN            0x01
#define REG_SCAN0                   0x71
#define BIT_DMP_MEM_ACCESS_EN       0x08
#define BIT_MEM_OTP_ACCESS_EN       0x04
#define BIT_FIFO_MEM_RD_SYS         0x02
#define BIT_FIFO_MEM_WR_SER         0x01
#define REG_MEM_BANK_SEL            0x72
#define REG_MEM_START_ADDR          0x73
#define REG_MEM_R_W                 0x74
#define REG_WHO_AM_I                0x75
#define REG_BANK_SEL                0x76

/* Bank 1 */
#define REG_XG_ST_DATA              0x5F
#define REG_YG_ST_DATA              0x60
#define REG_ZG_ST_DATA              0x61

#define REG_TMSTVAL0                0x62
#define REG_TMSTVAL1                0x63
#define REG_TMSTVAL2                0x64
#define REG_INTF_CONFIG4            0x7A
#define BIT_SPI_AP_4WIRE_EN         0x02
#define REG_INTF_CONFIG5            0x7B
#define BIT_PIN9_FUNC_INT2          0x00
#define BIT_PIN9_FUNC_FSYNC         0x02
#define BIT_PIN9_FUNC_CLKIN         0x04
#define BIT_PIN9_FUNC_MASK          0x06
#define REG_INTF_CONFIG6            0x7C
#define BIT_I3C_EN                  0x10
#define BIT_I3C_SDR_EN              0x01
#define BIT_I3C_DDR_EN              0x02

/* Bank 2 */
#define REG_XA_ST_DATA              0x3B
#define REG_YA_ST_DATA              0x3C
#define REG_ZA_ST_DATA              0x3D

/* Bank 4 */
#define REG_APEX_CONFIG1            0x40
#define BIT_LOW_ENERGY_AMP_TH_SEL_MASK   0xF0
#define BIT_LOW_ENERGY_AMP_TH_SEL_SHIFT  4
#define BIT_DMP_POWER_SAVE_TIME_SEL_MASK 0x0F
#define REG_APEX_CONFIG2            0x41
#define BIT_PED_AMP_TH_SEL_MASK          0xF0
#define BIT_PED_AMP_TH_SEL_SHIFT         4
#define BIT_PED_STEP_CNT_TH_SEL_MASK     0x0F
#define REG_APEX_CONFIG3            0x42
#define BIT_PED_STEP_DET_TH_SEL_MASK     0xE0
#define BIT_PED_STEP_DET_TH_SEL_SHIFT    5
#define BIT_PED_SB_TIMER_TH_SEL_MASK     0x1C
#define BIT_PED_SB_TIMER_TH_SEL_SHIFT    2
#define BIT_PED_HI_EN_TH_SEL_MASK        0x03
#define REG_APEX_CONFIG4            0x43
#define BIT_TILT_WAIT_TIME_SEL_MASK      0xC0
#define BIT_TILT_WAIT_TIME_SEL_SHIFT     6
#define BIT_SLEEP_TIME_OUT_MASK          0x38
#define BIT_SLEEP_TIME_OUT_SHIFT         3
#define BIT_LOWG_PEAK_TH_HYST_MASK       0x38
#define BIT_LOWG_PEAK_TH_HYST_SHIFT      3
#define REG_APEX_CONFIG5            0x44
#define BIT_LOWG_PEAK_TH_SHIFT           3
#define BIT_LOWG_PEAK_TH_MASK            0xF8
#define BIT_LOWG_TIME_TH_MASK            0x07
#define REG_APEX_CONFIG6            0x45
#define BIT_HIGHG_PEAK_TH_SHIFT              3
#define BIT_HIGHG_PEAK_TH_MASK               0xF8
#define REG_APEX_CONFIG9            0x48
#define BIT_SENSITIVITY_MODE_MASK        0x01
#define REG_ACCEL_WOM_X_THR         0x4A
#define REG_ACCEL_WOM_Y_THR         0x4B
#define REG_ACCEL_WOM_Z_THR         0x4C
#define REG_INT_SOURCE6             0x4D
#define BIT_INT1_STEP_DET_EN             0x20
#define BIT_INT1_STEP_CNT_OFL_EN         0x10
#define BIT_INT1_TILT_DET_EN             0x08
#define BIT_INT1_WAKE_DET_EN             0x04
#define BIT_INT1_LOW_G_DET_EN            0x04
#define BIT_INT1_SLEEP_DET_EN            0x02
#define BIT_INT1_HIGH_G_DET_EN           0x02
#define BIT_INT1_TAP_DET_EN              0x01
#define REG_INT_SOURCE7             0x4E
#define BIT_INT2_STEP_DET_EN             0x20
#define BIT_INT2_STEP_CNT_OFL_EN         0x10
#define BIT_INT2_TILT_DET_EN             0x08
#define BIT_INT2_WAKE_DET_EN             0x04
#define BIT_INT2_LOW_G_DET_EN            0x04
#define BIT_INT2_SLEEP_DET_EN            0x02
#define BIT_INT2_HIGH_G_DET_EN           0x02
#define BIT_INT2_TAP_DET_EN              0x01

/* chip id config */
#define ICM40607_WHO_AM_I           0x38
#define ICM40607I_WHO_AM_I          0x3C
#define ICM42602_WHO_AM_I           0x41
#define ICM42605_WHO_AM_I           0x42
#define ICM42688_WHO_AM_I           0x47
#define ICM42686_WHO_AM_I           0x44

/* wom config */
/** Default value for the WOM threshold
 *  Resolution of the threshold is 1g/256 ~= 3.9mg
 */

/* Error List */
#define SENSOR_WHOAMI_INVALID_ERROR     -1
#define SENSOR_CONVERT_INVALID_ERROR    -2
#define SENSOR_INIT_INVALID_ERROR       -3
#define INT_STATUS_READ_ERROR           -4
#define DRDY_UNEXPECT_INT_ERROR         -5
#define DRDY_DATA_READ_ERROR            -6
#define DRDY_DATA_CONVERT_ERROR         -7
#define FIFO_COUNT_INVALID_ERROR        -8
#define FIFO_OVERFLOW_ERROR             -9
#define FIFO_UNEXPECT_WM_ERROR          -10
#define FIFO_DATA_READ_ERROR            -11
#define FIFO_DATA_CONVERT_ERROR         -12
#define FIFO_DATA_FULL_ERROR            -13

#define DMP_TIMEOUT_ERROR               -23
#define DMP_IDLE_ERROR                  -24
#define DMP_SIZE_ERROR                  -25
#define DMP_SRAM_ERROR                  -26
#define DMP_STATUS_ERROR                -27
#define DMP_LOAD_ERROR                  -28

#define AXIS_X          0
#define AXIS_Y          1
#define AXIS_Z          2
#define AXES_NUM        3

#define SENSOR_HZ(_hz)  ((uint32_t)((_hz) * 1024.0f))

#define ACCEL_DATA_SIZE               6
#define GYRO_DATA_SIZE                6
#define TEMP_DATA_SIZE                2

#define FIFO_HEADER_SIZE              1
#define FIFO_ACCEL_DATA_SHIFT         1
#define FIFO_ACCEL_DATA_SIZE          ACCEL_DATA_SIZE
#define FIFO_GYRO_DATA_SHIFT          7
#define FIFO_GYRO_DATA_SIZE           GYRO_DATA_SIZE
#define FIFO_TEMP_DATA_SHIFT          13
#define FIFO_TEMP_DATA_SIZE           1
#define FIFO_TIMESTAMP_SIZE           2
#define FIFO_TEMP_HIGH_RES_SIZE       1
#define FIFO_ACCEL_GYRO_HIGH_RES_SIZE 3

#define FIFO_EMPTY_PACKAGE_SIZE       0
#define FIFO_8BYTES_PACKET_SIZE       (FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_TEMP_DATA_SIZE)
#define FIFO_16BYTES_PACKET_SIZE      (FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TIMESTAMP_SIZE)
#define FIFO_20BYTES_PACKET_SIZE      (FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TIMESTAMP_SIZE +\
                                       FIFO_TEMP_HIGH_RES_SIZE + FIFO_ACCEL_GYRO_HIGH_RES_SIZE)
#define FIFO_HEADER_EMPTY_BIT         0x80
#define FIFO_HEADER_A_BIT             0x40
#define FIFO_HEADER_G_BIT             0x20
#define FIFO_HEADER_20_BIT            0x10

#define DRI_ACCEL_DATA_SHIFT          2
#define DRI_GYRO_DATA_SHIFT           8
#define DRI_14BYTES_PACKET_SIZE       (TEMP_DATA_SIZE + ACCEL_DATA_SIZE + GYRO_DATA_SIZE)

/* wom config */
/** Default value for the WOM threshold
 *  Resolution of the threshold is 1g/256 ~= 3.9mg
 */
#if SUPPORT_WOM
#define DEFAULT_WOM_THS_MG 60 >> 2
#endif

#if SUPPORT_WOM
#define WOM_ENABLE_ERROR                -32
#endif

typedef enum 
{
	ODR_8KHZ    = 3,
	ODR_4KHZ    = 4,
	ODR_2KHZ    = 5,
	ODR_1KHZ    = 6,
	ODR_200HZ   = 7,
	ODR_100HZ   = 8,
	ODR_50HZ    = 9,
	ODR_25HZ    = 10,
	ODR_12_5HZ  = 11,
	ODR_500HZ   = 15,
} icm4x6xx_sensor_odr_t;

typedef enum 
{
	ACCEL_RANGE_16G = (0 << BIT_ACCEL_FSR_SHIFT),
	ACCEL_RANGE_8G  = (1 << BIT_ACCEL_FSR_SHIFT),
	ACCEL_RANGE_4G  = (2 << BIT_ACCEL_FSR_SHIFT),
	ACCEL_RANGE_2G  = (3 << BIT_ACCEL_FSR_SHIFT),
} icm4x6xx_accel_fsr_t;

typedef enum
{
	GYRO_RANGE_2000DPS   = (0 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_1000DPS   = (1 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_500DPS    = (2 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_250DPS    = (3 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_125DPS    = (4 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_62_5DPS   = (5 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_31_25DPS  = (6 << BIT_GYRO_FSR_SHIFT),
	GYRO_RANGE_15_625DPS = (7 << BIT_GYRO_FSR_SHIFT),
} icm4x6xx_gyro_fsr_t;

/* Filter order */
typedef enum 
{
	FIRST_ORDER = 0,    // 1st order
	SEC_ORDER   = 1,    // 2nd order
	THIRD_ORDER = 2,    // 3rd order
} icm4x6xx_filter_order_t;

typedef enum 
{
	I2C_SLEW_RATE_20_60NS = 0 << BIT_I2C_SLEW_RATE_SHIFT,
	I2C_SLEW_RATE_12_36NS = 1 << BIT_I2C_SLEW_RATE_SHIFT,
	I2C_SLEW_RATE_6_18NS  = 2 << BIT_I2C_SLEW_RATE_SHIFT,
	I2C_SLEW_RATE_4_12NS  = 3 << BIT_I2C_SLEW_RATE_SHIFT,
	I2C_SLEW_RATE_2_6NS   = 4 << BIT_I2C_SLEW_RATE_SHIFT,
	I2C_SLEW_RATE_2NS     = 5 << BIT_I2C_SLEW_RATE_SHIFT,
	SPI_SLEW_RATE_20_60NS = 0,
	SPI_SLEW_RATE_12_36NS = 1,
	SPI_SLEW_RATE_6_18NS  = 2,
	SPI_SLEW_RATE_4_12NS  = 3,
	SPI_SLEW_RATE_2_6NS   = 4,
	SPI_SLEW_RATE_2NS     = 5,
} icm4x6xx_slew_rate_t;

/* sensor bandwidth */
typedef enum 
{
	BW_ODR_DIV_2  = 0,
	BW_ODR_DIV_4  = 1,
	BW_ODR_DIV_5  = 2,
	BW_ODR_DIV_8  = 3,
	BW_ODR_DIV_10 = 4,
	BW_ODR_DIV_16 = 5,
	BW_ODR_DIV_20 = 6,
	BW_ODR_DIV_40 = 7,
} icm4x6xx_bandwidth_t;

static uint8_t ICM4X6XX_ODR_MAPPING[] = {
	ODR_12_5HZ,
	ODR_25HZ,
	ODR_50HZ,
	ODR_100HZ,
	ODR_200HZ,
	ODR_500HZ
};

/* Support odr range 25HZ - 500HZ */
static uint32_t ICM4X6XXHWRates[] = {
	SENSOR_HZ(12.5f),
	SENSOR_HZ(25.0f),
	SENSOR_HZ(50.0f),
	SENSOR_HZ(100.0f),
	SENSOR_HZ(200.0f),
	SENSOR_HZ(500.0f),
};

/* chip type */
typedef enum 
{
	UNVALID_TYPE = 0,
	ICM40607,
	ICM40607I,
	ICM40608,
	ICM42602,
	ICM42605,
	ICM42688,
	ICM42686,
} chip_type_t;

struct sensorConvert 
{
	int8_t    sign[3];
	uint8_t   axis[3];
};

#define SENSOR_DIRECTION                 2 //0~7 sensorConvert map index below
#define AC10_IS_IN_ITRON

struct sensorConvert map[] = {
	{ { 1, 1, 1},    {0, 1, 2} },
	{ { -1, 1, 1},   {1, 0, 2} },
	{ { -1, -1, 1},  {0, 1, 2} },
	{ { 1, -1, 1},   {1, 0, 2} },

	{ { -1, 1, -1},  {0, 1, 2} },
	{ { 1, 1, -1},   {1, 0, 2} },
	{ { 1, -1, -1},  {0, 1, 2} },
	{ { -1, -1, -1}, {1, 0, 2} },

	// // 自行车应用坐标系(E50)
	// { {1, -1, 1}, {2, 1, 0} },

#if defined(AC10_IS_IN_ITRON)
	// 自行车应用坐标系(M10--竖直)
	{ {1, 1, -1}, {0, 2, 1} },
#elif defined(AC10_IS_IN_FRAME)
	// 自行车应用坐标系(M10--车筐)
	{ {-1, -1, 1}, {0, 1, 2} },
#elif defined(AC10_IS_IN_TAIL_LED)
	// 自行车应用坐标系(M10--车尾灯处)
	{ {-1, 1, -1}, {0, 1, 2} },
#endif
};

struct accGyroDataPacket icm_data;


typedef struct 
{
	uint32_t rate; //the rate from up layer want
	uint32_t hwRate; // the rate currently set
	uint32_t preRealRate; //the rate from sensor list sync with upper layer want
	uint32_t samplesToDiscard; // depends on acc or gyro start up time to valid data
	uint16_t wm; //the watermark from user
	bool powered;
	bool configed;
	bool needDiscardSample;
} ICM4X6XXSensor_t;

typedef struct inv_icm4x6xx 
{
	ICM4X6XXSensor_t sensors[NUM_OF_SENSOR];

#if 0
	int (*read_reg)(ENUM_I2C_TYPE i2c_type, uint8_t reg, uint8_t *buf, uint8_t len);
	int (*write_reg)(ENUM_I2C_TYPE i2c_type, uint8_t reg, uint8_t *buf, uint8_t len);
#endif
	void (*delay_ms)(uint32_t);
	void (*delay_us)(uint32_t);

	chip_type_t product;
	float    chip_temper;

	/* For save reg status */
	uint8_t  int_cfg;
	uint8_t  int_src0;
	uint8_t  pwr_sta;
	uint8_t  acc_cfg0;
	uint8_t  gyro_cfg0;

	/* For fifo */
	uint16_t watermark;
	uint8_t  dataBuf[ICM4X6XX_MAX_FIFO_SIZE];
	uint32_t fifoDataToRead;
	bool     fifo_mode_en;
	uint16_t fifo_package_size;
	uint16_t dri_package_size;

	bool     init_cfg;
	/* For sensor oriention convert, sync to Andriod coordinate */
	struct sensorConvert cvt;

#if (SUPPORT_PEDOMETER | SUPPORT_WOM)
	bool     dmp_power_save;
	bool     apex_enable;
#endif
} inv_icm4x6xx_t;

inv_icm4x6xx_t icm_dev;

static int inv_icm4x6xx_convert_rawdata(struct accGyroDataPacket *packet);

#if 0
void inv_icm4x6xx_set_serif(int (*read)(void *, uint8_t, uint8_t *, uint8_t),
                            int (*write)(void *, uint8_t, uint8_t *, uint8_t))
{
	if(read == NULL || write == NULL) 
	{
		printf("Read/Write API NULL POINTER!!\r\n");
		// exit(-1);
		return;
	}
	icm_dev.read_reg = read;
	icm_dev.write_reg = write;
}
#endif

static int inv_read(uint8_t addr, uint32_t len, uint8_t *buf)
{
	int ret = 0;
   //return icm_dev.read_reg(ENUM_I2C_TYPE_GYRO, addr, buf, len);

#if defined(INV_USE_SPI)
	ret = bsp_spi_read_reg(addr, buf, len);
#else
	ret = bsp_i2c_read_reg(INV_SLAVE, addr, buf, len);
#endif

	return ret;
}

static int inv_write(uint8_t addr, uint8_t buf)
{
	int ret = 0;

#if defined(INV_USE_SPI)
	ret = bsp_spi_write_reg(addr, buf);
#else
	ret = bsp_i2c_write_reg(INV_SLAVE, addr, buf);
#endif

	return ret;
}

void inv_icm4x6xx_set_delay(void (*delay_ms)(uint32_t), void (*delay_us)(uint32_t))
{
	if(delay_ms == NULL || delay_us == NULL) 
	{
		icm_log("DELAY API NULL POINTER!!\r\n");
		return;
	}
	icm_dev.delay_ms = delay_ms;
	icm_dev.delay_us = delay_us;
}

static void inv_delay_ms(uint16_t ms)
{
	extern void qst_delay_ms(unsigned int n_ms);

	qst_delay_ms(ms);
}


static void inv_delay_us(uint16_t us)
{
	extern void qst_delay_us(unsigned int n_ms);

	qst_delay_us(us);

}

static void inv_icm4x6xx_get_whoami(void)
{
	int ret = 0;
	uint8_t data;

	for(int j = 0; j < 3; j++) 
	{
		ret = inv_read(REG_WHO_AM_I, 1, &data);
		icm_log("Chip Id is 0x%x ret is %d\r\n", data, ret);
		if(ret)
		{
			break;
		}
	}

	switch(data) 
	{
		case ICM42602_WHO_AM_I: 
			{
				icm_dev.product = ICM42602;
				icm_log("ICM42602 detected\r\n");
			}
			break;
				
		case ICM40607_WHO_AM_I: 
			{
				icm_dev.product = ICM40607;
				icm_log("ICM40607 detected\r\n");
			}
			break;
		
		case ICM40607I_WHO_AM_I: 
			{
				icm_dev.product = ICM40607I;
				icm_log("ICM40607I detected\r\n");
			}
			break;
				
		case ICM42605_WHO_AM_I: 
			{
				icm_dev.product = ICM42605;
				icm_log("ICM42605 detected\r\n");
			}
			break;
		
		case ICM42688_WHO_AM_I: 
			{
				icm_dev.product = ICM42688;
				icm_log("ICM42688 detected\r\n");
			}
			break;
		
		case ICM42686_WHO_AM_I: 
			{
				icm_dev.product = ICM42686;
				icm_log("ICM42686 detected\r\n");
			}
			break;
		
		default: 
		{
			icm_dev.product = UNVALID_TYPE;
			icm_log("Chip not supported\r\n");
		}
		return;
	}
}

// 重启芯片 true & false
static int inv_icm4x6xx_reset_check(void)
{
	int ret = 0;
	uint8_t data = 0;

	ret = inv_read(REG_CHIP_CONFIG, 1, &data);		//0x11   
	icm_log("read REG_CHIP_CONFIG[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_CHIP_CONFIG, ret, data);

	//data |= (uint8_t)BIT_SOFT_RESET_CONFIG;
#if 0//SPI_MODE_EN
	data = (uint8_t)(data | BIT_SPI_SLEW_RATE_MASK | BIT_SOFT_RESET_CONFIG);
#else
	data = (uint8_t)(data | BIT_SOFT_RESET_CONFIG);
#endif
	ret = inv_write(REG_CHIP_CONFIG, data);				//0x11  0x01
	icm_log("write REG_CHIP_CONFIG[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_CHIP_CONFIG, ret, data);

	inv_delay_ms(100);
	data = 0;
	ret = inv_read(REG_INT_STATUS, 1, &data);			//0x2D   0x10
	icm_log("read REG_INT_STATUS[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_STATUS, ret, data);
	if(ret == 0)
	{
		return false;
	}

	if(data & BIT_RESET_DONE) 
	{
		icm_log("ResetCheck Done 0x%x\r\n", data);
		return true;
	} 
	else 
	{
		icm_log("ResetCheck Fail 0x%x\r\n", data);
		return false;
	}
}



static int inv_icm4x6xx_get_convert(int direction, struct sensorConvert *cvt)
{
	struct sensorConvert *src;
	
	if(NULL == cvt) 
	{
		return -1;
	} 
	else if((direction >= sizeof(map) / sizeof(map[0])) || (direction < 0)) 
	{
		return -1;
	}

	//*cvt = map[direction];
	src = &map[direction];
	memcpy(cvt, src, sizeof(struct sensorConvert));

	return 0;
}

//
static int inv_icm4x6xx_init_config(void)
{
	int ret = 0;
	uint8_t data;

	icm_dev.fifo_mode_en = FIFO_WM_MODE_EN;

	ret = inv_icm4x6xx_reset_check();
	if(!ret) 
	{
		icm_log("reset icm40607i failed\r\n");
		return false;
	}

	/* en byte mode & little endian mode & disable spi or i2c interface */
	if(SPI_MODE_EN) 
	{
		/*
		icm_log("SPI BUS\r\n");
		data = (uint8_t)(I2C_SLEW_RATE_20_60NS | SPI_SLEW_RATE_2NS);
		ret = inv_write(REG_DRIVE_CONFIG, data);

		ret = inv_write(REG_BANK_SEL, 1);
		data = inv_read(REG_INTF_CONFIG6, 1, &data);
		data |= (uint8_t)(BIT_I3C_SDR_EN | BIT_I3C_DDR_EN);
		ret = inv_write(REG_INTF_CONFIG6, data);
		//ret += inv_write(REG_INTF_CONFIG4, BIT_SPI_AP_4WIRE_EN);
		ret = inv_write(REG_BANK_SEL, 0);

		data = (uint8_t)(BIT_UI_SIFS_CFG_I2C_DIS | BIT_FIFO_HOLD_LAST_DATA_EN);
		*/
//		icm_log("SPI BUS\r\n");
//		data = (uint8_t)(SPI_SLEW_RATE_2NS);
//		ret = inv_write(REG_DRIVE_CONFIG, data);

		icm_log("SPI BUS\r\n");
		data = (uint8_t)(I2C_SLEW_RATE_20_60NS | SPI_SLEW_RATE_2NS);
		ret = inv_write(REG_DRIVE_CONFIG, data);
		
		data = 0;
		ret = inv_write(REG_BANK_SEL, data);
		icm_log("write REG_BANK_SEL[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_BANK_SEL, ret, data);
		if(!ret) 
		{
			return false;
		}
		
		/*
		data = inv_read(REG_INTF_CONFIG6, 1, &data);
		data |= (uint8_t)(BIT_I3C_SDR_EN | BIT_I3C_DDR_EN);
		ret = inv_write(REG_INTF_CONFIG6, data);
		//ret += inv_write(REG_INTF_CONFIG4, BIT_SPI_AP_4WIRE_EN);
		ret = inv_write(REG_BANK_SEL, 0);
		*/
		data = 0;
		ret = inv_read(REG_INTF_CONFIG0, 1, &data);
		icm_log("write REG_INTF_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INTF_CONFIG0, ret, data);
		if(!ret) 
		{
			return false;
		}
		data |= (uint8_t)(BIT_UI_SIFS_CFG_I2C_DIS | BIT_FIFO_HOLD_LAST_DATA_EN);
	} 
	else 
	{
		icm_log("I2C BUS\r\n");
		// ret += inv_write(REG_BANK_SEL, 1);
		// data = inv_read(REG_INTF_CONFIG6, 1, &data);
		// data &= (uint8_t)~(BIT_I3C_SDR_EN | BIT_I3C_DDR_EN);
		// ret += inv_write(REG_INTF_CONFIG6, data);
		data = 0;
		ret = inv_write(REG_BANK_SEL, data);
		icm_log("write REG_BANK_SEL[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_BANK_SEL, ret, data);
		if(!ret) 
		{
			return false;
		}

		data = 0;
		ret = inv_read(REG_INTF_CONFIG0, 1, &data);
		icm_log("write REG_INTF_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INTF_CONFIG0, ret, data);
		if(!ret) 
		{
			return false;
		}

		data |= (uint8_t)(BIT_UI_SIFS_CFG_SPI_DIS | BIT_FIFO_HOLD_LAST_DATA_EN);
	}
	ret = inv_write(REG_INTF_CONFIG0, data);
	icm_log("write REG_INTF_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INTF_CONFIG0, ret, data);
	if(!ret) 
	{
		return false;
	}

	data = 0;
	ret = inv_read(REG_INT_SOURCE1, 1, &data);
	data &= (uint8_t)~BIT_INT1_WOM_EN_MASK;
	ret = inv_write(REG_INT_SOURCE1, data);
	icm_log("read REG_INT_SOURCE1[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_SOURCE1, ret, data);

	data = 0;
	ret = inv_read(REG_SMD_CONFIG, 1, &data);
	data &= (uint8_t)~BIT_SMD_MODE_MASK;
	ret = inv_write(REG_SMD_CONFIG, data);

	data = 0;
	ret = inv_read(REG_SMD_CONFIG, 1, &data);
	icm_log("read REG_SMD_CONFIG[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_SMD_CONFIG, ret, data);

	/* INT setting.  pulse mode active high by default */
	icm_dev.int_cfg = (uint8_t)BIT_INT1_PUSH_PULL;
#if INT_LATCH_EN
	icm_dev.int_cfg |= (uint8_t)BIT_INT1_LATCHED_MODE;
#endif
#if INT_ACTIVE_HIGH_EN
	icm_dev.int_cfg |= (uint8_t)BIT_INT1_ACTIVE_HIGH;
#endif
	ret = inv_write(REG_INT_CONFIG, icm_dev.int_cfg);
	icm_log("write REG_INT_CONFIG[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_CONFIG, ret, icm_dev.int_cfg);

	/* gyr odr & fs */
	icm_dev.gyro_cfg0 = ((uint8_t)ODR_100HZ) | ((uint8_t)GYRO_RANGE_2000DPS);
	ret = inv_write(REG_GYRO_CONFIG0, icm_dev.gyro_cfg0);
	icm_log("write REG_GYRO_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_GYRO_CONFIG0, ret, icm_dev.gyro_cfg0);

	/* acc odr & fs */
	icm_dev.acc_cfg0 = ((uint8_t)ODR_100HZ) | ((uint8_t)ACCEL_RANGE_8G);
	ret = inv_write(REG_ACCEL_CONFIG0, icm_dev.acc_cfg0);
	icm_log("write REG_ACCEL_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_ACCEL_CONFIG0, ret, icm_dev.acc_cfg0);

	/* acc & gyro BW */
	data = (uint8_t)((BW_ODR_DIV_2 << BIT_ACCEL_FILT_BW_SHIFT) | BW_ODR_DIV_4);
	ret = inv_write(REG_GYRO_ACCEL_CONFIG0, data);
	icm_log("write REG_GYRO_ACCEL_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_GYRO_ACCEL_CONFIG0, ret, data);

	/* acc & gyro UI filter @ 3th order */
	data = (uint8_t)(BIT_GYRO_AVG_FILT_8K_HZ | (THIRD_ORDER << BIT_GYRO_FILT_ORD_SHIFT));
	ret = inv_write(REG_GYRO_CONFIG1, data);
	icm_log("write REG_GYRO_CONFIG1[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_GYRO_CONFIG1, ret, data);

	data = (uint8_t)(BIT_ACCEL_AVG_FILT_8K_HZ | (THIRD_ORDER << BIT_ACCEL_FILT_ORD_SHIFT));
	ret = inv_write(REG_ACCEL_CONFIG1, data);
	icm_log("write REG_ACCEL_CONFIG1[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_ACCEL_CONFIG1, ret, data);

	if(true == icm_dev.fifo_mode_en) 
	{
		//Fifo mode
		/* en fifo cfg(always 16bytes mode) */
		data = (uint8_t)(BIT_FIFO_WM_GT_TH_MASK | BIT_FIFO_TMST_FSYNC_EN_MASK | \
										 BIT_FIFO_TEMP_EN_EN | BIT_FIFO_GYRO_EN_EN | BIT_FIFO_ACCEL_EN_EN);
		ret += inv_write(REG_FIFO_CONFIG_1, data);
		icm_dev.fifo_package_size = FIFO_16BYTES_PACKET_SIZE;
		/* set wm_th=16 as default for byte mode */
		ret += inv_write(REG_FIFO_WM_TH_L, 0x10);/* byte mode */
		ret += inv_write(REG_FIFO_WM_TH_H, 0x00);
		/* bypass fifo */
		ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_BYPASS);
		/* enable fifo full & WM INT */
		icm_dev.int_src0 |= (uint8_t)(BIT_INT1_FIFO_FULL_EN | BIT_INT1_WM_EN);
#if defined(ICM4X6XX_USE_INT2)
		ret += inv_write(REG_INT_SOURCE3, icm_dev.int_src0);
#else
		ret += inv_write(REG_INT_SOURCE0, icm_dev.int_src0);
#endif
		icm_log("FIFO MODE INT_SRC0 0x%x\r\n", icm_dev.int_src0);
	} 
	else 
	{
		//DRDY mode
		/* enable DRDY INT */
		icm_dev.int_src0 = (uint8_t)BIT_INT1_DRDY_EN;
#if defined(ICM4X6XX_USE_INT2)
		ret = inv_write(REG_INT_SOURCE3, icm_dev.int_src0);
#else
		ret = inv_write(REG_INT_SOURCE0, icm_dev.int_src0);
#endif
		icm_log("write REG_INT_SOURCE0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_SOURCE0, ret, icm_dev.int_src0);

		icm_dev.dri_package_size = DRI_14BYTES_PACKET_SIZE;
		icm_log("DRI MODE INT_SRC0 0x%x\r\n", icm_dev.int_src0);
	}

	/* async reset */
	ret = inv_write(REG_INT_CONFIG1, (uint8_t)BIT_INT_ASY_RST_DIS); //The field int_asy_rst_disable must be 0 for icm4x6xx
	icm_log("write REG_INT_CONFIG1[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_CONFIG1, ret, (uint8_t)BIT_INT_ASY_RST_DIS);

#if SUPPORT_WOM
	inv_icm4x6xx_wom_disable();
#endif

	/* get sensor default setting */
	ret = inv_read(REG_PWR_MGMT0, 1, &icm_dev.pwr_sta);
	icm_log("read REG_PWR_MGMT0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_PWR_MGMT0, ret, icm_dev.pwr_sta);

#if GYRO_STALL_PATCH_ENABLE
	/* fix gyro stall issue */
	ret = inv_write(REG_BANK_SEL, 3);
	icm_log("write REG_BANK_SEL[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_BANK_SEL, ret, 3);

	uint8_t reg_2e, reg_32, reg_37, reg_3c;
	ret = inv_read(0x2e, 1, &reg_2e);
	icm_log("read reg_2e[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x2e, ret, reg_2e);

	ret = inv_read(0x32, 1, &reg_32);
	icm_log("read reg_32[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x32, ret, reg_32);

	ret = inv_read(0x37, 1, &reg_37);
	icm_log("read reg_37[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x37, ret, reg_37);

	ret = inv_read(0x3c, 1, &reg_3c);
	icm_log("read reg_3c[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x3c, ret, reg_3c);

	data = 0xfd & reg_2e;
	ret = inv_write(0x2e, data);
	icm_log("write reg_2e[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x2e, ret, data);

	data = 0x9f & reg_32;
	ret = inv_write(0x32, data);
	icm_log("write reg_32[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x32, ret, data);

	data = 0x9f & reg_37;
	ret = inv_write(0x37, data);
	icm_log("write reg_37[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x37, ret, data);

	data = 0x9f & reg_3c;
	ret = inv_write(0x3c, data);
	icm_log("write reg_3c[%02xh] -- ret[%02xh]--data[%02xh]\r\n", 0x3c, ret, data);

	ret = inv_write(REG_BANK_SEL, 0);
	icm_log("write REG_BANK_SEL[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_BANK_SEL, ret, data);
#endif

	return ret;
}

int inv_icm4x6xx_initialize(void)
{
	int ret = 0;

	icm_dev.product = UNVALID_TYPE;
	icm_dev.int_cfg = 0;
	icm_dev.int_src0 = 0;
	icm_dev.pwr_sta = 0;
	icm_dev.acc_cfg0 = 0;
	icm_dev.gyro_cfg0 = 0;
	icm_dev.init_cfg = false;
#if (SUPPORT_PEDOMETER | SUPPORT_WOM)
	icm_dev.dmp_power_save = false;
	icm_dev.apex_enable = true;
#endif
	icm_dev.fifo_package_size = 0;
	icm_dev.dri_package_size = 0;

	inv_icm4x6xx_get_whoami();

	if(icm_dev.product == UNVALID_TYPE) 
	{
		return SENSOR_WHOAMI_INVALID_ERROR;
	}

	ret = inv_icm4x6xx_get_convert(SENSOR_DIRECTION, &icm_dev.cvt);
	if(ret != 0)
	{
		return SENSOR_CONVERT_INVALID_ERROR;
	}

	icm_log("sensor axis[0]:%d, axis[1]:%d, axis[2]:%d, sign[0]:%d, sign[1]:%d, sign[2]:%d\r\n",
               icm_dev.cvt.axis[AXIS_X], icm_dev.cvt.axis[AXIS_Y], icm_dev.cvt.axis[AXIS_Z],
               icm_dev.cvt.sign[AXIS_X], icm_dev.cvt.sign[AXIS_Y], icm_dev.cvt.sign[AXIS_Z]);

	ret = inv_icm4x6xx_init_config();
	if(ret != 0) 
	{
		icm_dev.init_cfg = true;
		icm_log("Initialize Success\r\n");
	} 
	else 
	{
		icm_log("Initialize Failed %d\r\n", ret);
		ret = SENSOR_INIT_INVALID_ERROR;
	}

	return ret;
}

static int inv_icm4x6xx_set_odr(int index)
{
	int ret = 0;
	uint8_t regValue = 0;

	regValue = ICM4X6XX_ODR_MAPPING[index];
	icm_log("Odr Reg value 0x%x\r\n", regValue);
	icm_dev.gyro_cfg0 &= (uint8_t)~BIT_GYRO_ODR_MASK;
	icm_dev.gyro_cfg0 |= regValue;
	ret = inv_write(REG_GYRO_CONFIG0, icm_dev.gyro_cfg0);
	icm_log("write REG_GYRO_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_GYRO_CONFIG0, ret, icm_dev.gyro_cfg0);

	icm_dev.acc_cfg0 &= (uint8_t)~BIT_ACCEL_ODR_MASK;
	icm_dev.acc_cfg0 |= regValue;
	ret = inv_write(REG_ACCEL_CONFIG0, icm_dev.acc_cfg0);
	icm_log("write REG_ACCEL_CONFIG0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_ACCEL_CONFIG0, ret, icm_dev.acc_cfg0);

	return ret;
}

static int inv_icm4x6xx_cal_odr(uint32_t *rate, uint32_t *report_rate)
{
	int i;

	for(i = 0; i < (ARRAY_SIZE(ICM4X6XXHWRates)); i++) 
	{
		if(*rate <= ICM4X6XXHWRates[i]) 
		{
			*report_rate = ICM4X6XXHWRates[i];
			break;
		}
	}

	if(*rate > ICM4X6XXHWRates[(ARRAY_SIZE(ICM4X6XXHWRates) - 1)]) 
	{
		i = (ARRAY_SIZE(ICM4X6XXHWRates) - 1);
		*report_rate = ICM4X6XXHWRates[i];
	}

	return i;
}

static uint16_t inv_icm4x6xx_cal_wm(uint16_t watermark)
{
	uint8_t min_watermark = 1;
	uint8_t max_watermark ;
	uint16_t real_watermark = 0;

	//for yokohama the max fifo size is 2000byte, max fifo pakage is 20bit, so the max fifo pakage is 100
	max_watermark = 70 < (MAX_RECV_PACKET / 2) ? 70 : (MAX_RECV_PACKET / 2); /*60*/

	real_watermark = watermark;
	real_watermark = real_watermark < min_watermark ? min_watermark : real_watermark;
	real_watermark = real_watermark > max_watermark ? max_watermark : real_watermark;

	return real_watermark * icm_dev.fifo_package_size;/* byte mode*/
}
#if 0
static int inv_icm4x6xx_read_fifo(void)
{
	int ret = 0;
	uint8_t count[2];
	ret += inv_read(REG_FIFO_COUNT_H, 2, count);
	icm_dev.fifoDataToRead = (count[1] << 8 | count[0]);

	icm_log("Fifo count is %d\r\n", icm_dev.fifoDataToRead);

	if(icm_dev.fifoDataToRead <= 0 || icm_dev.fifoDataToRead > ICM4X6XX_MAX_FIFO_SIZE)
	{
		return FIFO_COUNT_INVALID_ERROR;
	}
	//icm_dev.fifoDataToRead += 1; // read one more byte to eliminate double interrupt

	ret += inv_read(REG_FIFO_DATA, icm_dev.fifoDataToRead, icm_dev.dataBuf);

	return ret;
}
#endif
static int inv_icm4x6xx_config_fifo(bool enable)
{
	int ret = 0;

	if(true == enable) 
	{
		uint8_t  buffer[2];
		uint16_t watermarkReg;

		watermarkReg = icm_dev.watermark;

#if 1//byte mode
		if(watermarkReg < icm_dev.fifo_package_size)
		{
			watermarkReg = icm_dev.fifo_package_size;
		}
#else
		if(watermarkReg == 0)
		{
			watermarkReg = 1;
		}
#endif
		icm_log("%s watermarkReg %d\r\n", __func__, watermarkReg);

		buffer[0] = watermarkReg & 0x00FF;
		buffer[1] = (watermarkReg & 0xFF00) >> 8;
		/* set bypass mode to reset fifo */
		ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_BYPASS);
		/* set threshold */
		ret += inv_write(REG_FIFO_WM_TH_L, buffer[0]);
		ret += inv_write(REG_FIFO_WM_TH_H, buffer[1]);
		if((true == icm_dev.sensors[ACC].configed) ||
				(true == icm_dev.sensors[GYR].configed)) 
		{
			/* set fifo stream mode */
			ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_STREAM);
			icm_log("Reset, TH_L:0x%x, TH_H:0x%x\r\n", buffer[0], buffer[1]);
		} 
		else 
		{
			/* set fifo bypass mode as no sensor configured */
			icm_log("Fifo to Bypass Mode\r\n");
			ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_BYPASS);
		}
	} 
	else 
	{
		if((false == icm_dev.sensors[ACC].configed) &&
				(false == icm_dev.sensors[GYR].configed)) 
		{
				/* set fifo bypass mode as no sensor configured */
				icm_log("Fifo to Bypass Mode\r\n");
				ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_BYPASS);
		}
	}

	return ret;
}

static int inv_icm4x6xx_config_drdy(bool enable)
{
	int ret = 0;

	if(enable &&
			(true == icm_dev.sensors[ACC].configed ||
			 true == icm_dev.sensors[GYR].configed)) 
	{
		if(!(icm_dev.int_src0 & BIT_INT1_DRDY_EN)) 
		{
			icm_dev.int_src0 |= (uint8_t)BIT_INT1_DRDY_EN;
#if defined(ICM4X6XX_USE_INT2)
			ret = inv_write(REG_INT_SOURCE3, icm_dev.int_src0);
#else
			ret = inv_write(REG_INT_SOURCE0, icm_dev.int_src0);
#endif
			icm_log("write REG_INT_SOURCE0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_SOURCE0, ret, icm_dev.int_src0);
		}
	} 
	else			
	{
		icm_dev.int_src0 &= (uint8_t)~BIT_INT1_DRDY_EN;
#if defined(ICM4X6XX_USE_INT2)
		ret = inv_write(REG_INT_SOURCE3, icm_dev.int_src0);
#else
		ret = inv_write(REG_INT_SOURCE0, icm_dev.int_src0);
#endif
		icm_log("write REG_INT_SOURCE0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_SOURCE0, ret, icm_dev.int_src0);
	}

	return ret;
}

int inv_icm4x6xx_acc_enable(void)
{
	int ret = 0;

	icm_dev.sensors[ACC].powered = true;
	icm_dev.pwr_sta &= ~(uint8_t)BIT_ACCEL_MODE_MASK;
	icm_dev.pwr_sta |= BIT_ACCEL_MODE_LN;
	ret = inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
	icm_log("write REG_PWR_MGMT0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_PWR_MGMT0, ret, icm_dev.pwr_sta);
	//200us here to sync fly changes
	inv_delay_us(200);
	icm_log("ACC PWR 0x%x\r\n", icm_dev.pwr_sta);

	return ret;
}

int inv_icm4x6xx_acc_disable(void)
{
	int ret = 0;
	int odr_index = 0;
	uint32_t sampleRate = 0;
	bool accelOdrChanged = false;
	bool watermarkChanged = false;

	icm_dev.sensors[ACC].preRealRate = 0;
	icm_dev.sensors[ACC].hwRate = 0;
	icm_dev.sensors[ACC].needDiscardSample = false;
	icm_dev.sensors[ACC].samplesToDiscard = 0;
	icm_dev.sensors[ACC].wm = 0;

	if((false == icm_dev.sensors[GYR].powered)
#if SUPPORT_WOM
        && (false == icm_dev.sensors[WOM].powered)
#endif
     ) 
	{
		icm_dev.pwr_sta &= (uint8_t)~(BIT_ACCEL_MODE_MASK | BIT_GYRO_MODE_MASK);
		ret += inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
		inv_delay_us(200); //spec: 200us

		icm_log("acc off pwr: 0x%x\r\n", icm_dev.pwr_sta);
	} 
	else if(true == icm_dev.sensors[GYR].powered) 
	{  //  update gyro old odr
		if(icm_dev.sensors[GYR].hwRate != icm_dev.sensors[GYR].preRealRate) 
		{
			icm_dev.sensors[GYR].hwRate = icm_dev.sensors[GYR].preRealRate;
			odr_index = inv_icm4x6xx_cal_odr(&icm_dev.sensors[GYR].hwRate, &sampleRate);
			ret += inv_icm4x6xx_set_odr(odr_index);
			icm_log("gyro revert rate to preRealRate: %f Hz\r\n",
								 (icm_dev.sensors[GYR].hwRate / 1024.0f));
			accelOdrChanged = true;
		}
		if(icm_dev.fifo_mode_en && (icm_dev.watermark != icm_dev.sensors[GYR].wm)) 
		{
			icm_dev.watermark = icm_dev.sensors[GYR].wm;
			watermarkChanged = true;
			icm_log("watermark revert to: %d\r\n", icm_dev.watermark / icm_dev.fifo_package_size);
		}

		if(true
#if SUPPORT_PEDOMETER
				&& (false == icm_dev.sensors[PEDO].powered)
#endif
#if SUPPORT_WOM
				&& (false == icm_dev.sensors[WOM].powered)
#endif
			) 
		{
			icm_dev.pwr_sta &= (uint8_t)~(BIT_ACCEL_MODE_MASK);
			ret += inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
			inv_delay_us(200); //spec: 200us

			icm_log("gyro on and acc off pwr: 0x%x\r\n", icm_dev.pwr_sta);
		} 
		else 
		{
			icm_log("gyro on and apex on pwr: 0x%x\r\n", icm_dev.pwr_sta);
			//Todo re-set fifo data format as acc disabled
		}
	}

	icm_dev.sensors[ACC].powered = false;
	icm_dev.sensors[ACC].configed = false;

	if(true == icm_dev.fifo_mode_en) 
	{
		inv_icm4x6xx_config_fifo(accelOdrChanged | watermarkChanged);
	} 
	else 
	{
		inv_icm4x6xx_config_drdy(accelOdrChanged);
	}

	return ret;
}

int inv_icm4x6xx_gyro_enable(void)
{
	int ret = 0;

	icm_log("%s\r\n", __func__);

	icm_dev.sensors[GYR].powered = true;
	icm_dev.pwr_sta &= (uint8_t)~BIT_GYRO_MODE_MASK;
	icm_dev.pwr_sta |= BIT_GYRO_MODE_LN;
	ret = inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
	icm_log("write REG_PWR_MGMT0[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_PWR_MGMT0, ret, icm_dev.pwr_sta);
	//200us here to sync fly changes
	inv_delay_us(200);
	icm_log("GYR PWR 0x%x\r\n", icm_dev.pwr_sta);

	return ret;
}

int inv_icm4x6xx_gyro_disable(void)
{
	int ret = 0;
	int odr_index = 0;
	uint32_t sampleRate = 0;
	bool gyroOdrChanged = false;
	bool watermarkChanged = false;

	icm_dev.sensors[GYR].preRealRate = 0;
	icm_dev.sensors[GYR].hwRate = 0;
	icm_dev.sensors[GYR].needDiscardSample = false;
	icm_dev.sensors[GYR].samplesToDiscard = 0;
	icm_dev.sensors[GYR].wm = 0;

	if(true == icm_dev.sensors[ACC].powered) 
	{  //  update ACC old ord
		if(icm_dev.sensors[ACC].hwRate != icm_dev.sensors[ACC].preRealRate) 
		{
			icm_dev.sensors[ACC].hwRate = icm_dev.sensors[ACC].preRealRate;
			odr_index = inv_icm4x6xx_cal_odr(&icm_dev.sensors[ACC].hwRate, &sampleRate);
			ret += inv_icm4x6xx_set_odr(odr_index);
			gyroOdrChanged = true;
			icm_log("acc revert rate to preRealRate: %f Hz\r\n",
								 (icm_dev.sensors[ACC].hwRate / 1024.0f));
			if(icm_dev.fifo_mode_en && (icm_dev.watermark != icm_dev.sensors[ACC].wm)) 
			{
				icm_dev.watermark = icm_dev.sensors[ACC].wm;
				watermarkChanged = true;
				icm_log("watermark revert to: %d\r\n", icm_dev.watermark / icm_dev.fifo_package_size);
			}
		}
		icm_dev.pwr_sta &= (uint8_t)~BIT_GYRO_MODE_MASK; // Gyro OFF  also keep acc in LN Mode
		ret = inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
		inv_delay_ms(150);
		icm_log("acc on and gyro off pwr: 0x%x\r\n", icm_dev.pwr_sta);
	}
	else 
	{
		/* Gyro OFF. In case apex on not ~(BIT_GYRO_MODE_MASK | BIT_ACCEL_MODE_MASK)*/
		icm_dev.pwr_sta &= (uint8_t)~BIT_GYRO_MODE_MASK;
		ret += inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
		inv_delay_ms(150);
		icm_log("gyro off pwr: 0x%x\r\n", icm_dev.pwr_sta);
	}

	icm_dev.sensors[GYR].powered = false;
	icm_dev.sensors[GYR].configed = false;

	if(true == icm_dev.fifo_mode_en) 
	{
		inv_icm4x6xx_config_fifo(gyroOdrChanged | watermarkChanged);
	} 
	else 
	{
		inv_icm4x6xx_config_drdy(gyroOdrChanged);
	}

	return ret;
}

int inv_icm4x6xx_acc_set_rate(float odr_hz, uint16_t watermark)
{
	int ret = 0;
	int odr_index = 0;
	uint32_t sampleRate = 0;
	uint32_t maxRate = 0;
	bool accelOdrChanged = false;
	bool watermarkChanged = false;

	icm_log("odr_hz %f wm %d\r\n", odr_hz, watermark);

	icm_dev.sensors[ACC].rate = SENSOR_HZ(odr_hz);

#if (SUPPORT_PEDOMETER | SUPPORT_WOM)
	if((true == icm_dev.apex_enable) &&
			(icm_dev.sensors[ACC].rate < SENSOR_HZ(50))) 
	{
		icm_log("APEX Enabled, Acc min odr to 50Hz!!\r\n");
		icm_dev.sensors[ACC].rate = SENSOR_HZ(50);
	}
#endif

	if(icm_dev.sensors[ACC].preRealRate == 0) 
	{
		icm_dev.sensors[ACC].needDiscardSample = true;
		icm_dev.sensors[ACC].samplesToDiscard = NUM_TODISCARD;
	}

	odr_index = inv_icm4x6xx_cal_odr(&icm_dev.sensors[ACC].rate, &sampleRate);
	icm_dev.sensors[ACC].preRealRate = sampleRate;

	/* if gyr configed ,compare maxRate with acc and gyr rate */
	if(true == icm_dev.sensors[GYR].configed) 
	{
		maxRate = max(sampleRate, icm_dev.sensors[GYR].preRealRate);// choose with preRealRate
		if(maxRate != icm_dev.sensors[ACC].hwRate ||
				maxRate != icm_dev.sensors[GYR].hwRate) 
		{
			icm_dev.sensors[ACC].hwRate = maxRate;
			icm_dev.sensors[GYR].hwRate = maxRate;
			icm_log("New Acc/Gyro config Rate %f Hz\r\n",
								 (icm_dev.sensors[ACC].hwRate / 1024.0f));
			odr_index = inv_icm4x6xx_cal_odr(&maxRate, &sampleRate);
			ret = inv_icm4x6xx_set_odr(odr_index);
			accelOdrChanged = true;
		} 
		else 
		{
			accelOdrChanged = false;
		}
	} 
	else 
	{
		if((sampleRate != icm_dev.sensors[ACC].hwRate)) 
		{
			icm_dev.sensors[ACC].hwRate = sampleRate;
			icm_log("New Acc config Rate %f Hz\r\n",
								 (icm_dev.sensors[ACC].hwRate / 1024.0f));
			ret = inv_icm4x6xx_set_odr(odr_index);
			accelOdrChanged = true;
		} 
		else 
		{
			accelOdrChanged = false;
		}
	}
	icm_dev.sensors[ACC].configed = true;

	//For fifo mode
	if(true == icm_dev.fifo_mode_en) 
	{
		icm_dev.sensors[ACC].wm = inv_icm4x6xx_cal_wm(watermark);
		if(icm_dev.sensors[ACC].wm != icm_dev.watermark) 
		{
			watermarkChanged = true;
			icm_dev.watermark = icm_dev.sensors[ACC].wm;
			icm_log("New Watermark is %d\r\n", icm_dev.watermark / icm_dev.fifo_package_size);
		} 
		else
		{
			watermarkChanged = false;
		}
		inv_icm4x6xx_config_fifo(accelOdrChanged | watermarkChanged);
	} 
	else 
	{
		inv_icm4x6xx_config_drdy(accelOdrChanged);
	}

	return ret;
}


int inv_icm4x6xx_gyro_set_rate(float odr_hz, uint16_t watermark)
{
	int ret = 0;
	int odr_index = 0;
	uint32_t sampleRate = 0;
	uint32_t maxRate = 0;
	bool gyroOdrChanged = false;
	bool watermarkChanged = false;

	icm_log("odr_hz %f wm %d\r\n", odr_hz, watermark);

	icm_dev.sensors[GYR].rate = SENSOR_HZ(odr_hz);

	if(icm_dev.sensors[GYR].preRealRate == 0) 
	{
		icm_dev.sensors[GYR].needDiscardSample = true;
		icm_dev.sensors[GYR].samplesToDiscard = NUM_TODISCARD;
	}

	/* get hw sample rate */
	odr_index = inv_icm4x6xx_cal_odr(&icm_dev.sensors[GYR].rate, &sampleRate);

	icm_dev.sensors[GYR].preRealRate = sampleRate;

	/* if acc configed ,compare maxRate with acc and gyr rate */
	if(true == icm_dev.sensors[ACC].configed) 
	{
		maxRate = max(sampleRate, icm_dev.sensors[ACC].preRealRate);
		if(maxRate != icm_dev.sensors[ACC].hwRate ||
				maxRate != icm_dev.sensors[GYR].hwRate) 
		{
			icm_dev.sensors[ACC].hwRate = maxRate;
			icm_dev.sensors[GYR].hwRate = maxRate;
			icm_log("New Gyro/Acc config Rate %f Hz\r\n", (icm_dev.sensors[GYR].hwRate / 1024.0f));
			/* update new odr */
			odr_index = inv_icm4x6xx_cal_odr(&maxRate, &sampleRate);
			ret += inv_icm4x6xx_set_odr(odr_index);
			gyroOdrChanged = true;
		} 
		else
		{
			gyroOdrChanged = false;
		}
	} 
	else 
	{
			if((sampleRate != icm_dev.sensors[GYR].hwRate)) 
			{
				icm_dev.sensors[GYR].hwRate = sampleRate;
				icm_log("New Gyro config Rate %f Hz\r\n",
									 (icm_dev.sensors[GYR].hwRate / 1024.0f));
				/* update new odr */
				ret += inv_icm4x6xx_set_odr(odr_index);
				gyroOdrChanged = true;
			} 
			else
			{
				gyroOdrChanged = false;
			}
	}

	icm_dev.sensors[GYR].configed = true;

	//For fifo mode
	if(true == icm_dev.fifo_mode_en) 
	{
		icm_dev.sensors[GYR].wm = inv_icm4x6xx_cal_wm(watermark);
		if(icm_dev.sensors[GYR].wm != icm_dev.watermark) 
		{
			watermarkChanged = true;
			icm_dev.watermark = icm_dev.sensors[GYR].wm;
			icm_log("New Watermark is %d\r\n", icm_dev.watermark / icm_dev.fifo_package_size);
		} 
		else 
		{
			watermarkChanged = false;
		}
		inv_icm4x6xx_config_fifo(gyroOdrChanged | watermarkChanged);
	} 
	else 
	{
		inv_icm4x6xx_config_drdy(gyroOdrChanged);
	}

	return ret;
}

static void inv_icm4x6xx_parse_rawdata(struct accGyroData *data, uint8_t *buf, SensorType_t sensorType)
{
	int16_t raw_data[AXES_NUM] = {0};
	int16_t remap_data[AXES_NUM] = {0};
	int8_t  temper_raw = 0;

	if(sensorType == TEMP) 
	{
		if(true == icm_dev.fifo_mode_en) 
		{
			temper_raw = (int8_t) buf[0];
			/* convert to physical unit: Celsius degree */
			icm_dev.chip_temper = ((float)temper_raw * TEMP_SENSITIVITY_FIFO) + ROOM_TEMP_OFFSET;
		} 
		else 
		{
			/* Use little endian mode */
			temper_raw = (buf[0] | buf[1] << 8);
			/* convert to physical unit: Celsius degree */
			icm_dev.chip_temper = ((float)temper_raw * TEMP_SENSITIVITY_REG) + ROOM_TEMP_OFFSET;
		}
		//icm_log("chip temper %f\r\n", icm_dev.chip_temper);
		return;
	}

	/* Use little endian mode */
	raw_data[AXIS_X] = (buf[0] | buf[1] << 8);
	raw_data[AXIS_Y] = (buf[2] | buf[3] << 8);
	raw_data[AXIS_Z] = (buf[4] | buf[5] << 8);

#if 1
	remap_data[icm_dev.cvt.axis[AXIS_X]] = icm_dev.cvt.sign[AXIS_X] * raw_data[AXIS_X];
	remap_data[icm_dev.cvt.axis[AXIS_Y]] = icm_dev.cvt.sign[AXIS_Y] * raw_data[AXIS_Y];
	remap_data[icm_dev.cvt.axis[AXIS_Z]] = icm_dev.cvt.sign[AXIS_Z] * raw_data[AXIS_Z];
#else
	remap_data[AXIS_X] = raw_data[AXIS_Z];
	remap_data[AXIS_Y] = raw_data[AXIS_Y];
	remap_data[AXIS_Z] = raw_data[AXIS_X] * (-1);
#endif

	if(sensorType == ACC) 
	{
		// 单位：m/s^2
		data->x = (float)remap_data[AXIS_X] * KSCALE_ACC_8G_RANGE;
		data->y = (float)remap_data[AXIS_Y] * KSCALE_ACC_8G_RANGE;
		data->z = (float)remap_data[AXIS_Z] * KSCALE_ACC_8G_RANGE;
		data->sensType = sensorType;
		//icm_log("A: %f %f %f\r\n", (double)data->x, (double)data->y, (double)data->z);
		//icm_log("A: %d %d %d  ", remap_data[AXIS_X], remap_data[AXIS_Y], remap_data[AXIS_Z]);
		// icm_log("A: %f %f %f\r\n", (double)data->x, (double)data->y, (double)data->z);
	} 
	else if(sensorType == GYR) 
	{
		// 单位：弧度/秒
		data->x = (float)remap_data[AXIS_X] * KSCALE_GYRO_2000_RANGE;
		data->y = (float)remap_data[AXIS_Y] * KSCALE_GYRO_2000_RANGE;
		data->z = (float)remap_data[AXIS_Z] * KSCALE_GYRO_2000_RANGE;
		data->sensType = sensorType;
		// icm_log("G: %f %f %f\r\n", (double)data->x, (double)data->y, (double)data->z);
		//icm_log("G: %d %d %d\n", remap_data[AXIS_X], remap_data[AXIS_Y], remap_data[AXIS_Z]);
	}
}

static int inv_icm4x6xx_convert_rawdata(struct accGyroDataPacket *packet)
{
	int ret = 0;
	uint32_t i = 0;
	uint8_t accEventSize = 0;
	uint8_t gyroEventSize = 0;
	uint8_t accEventSize_Discard = 0;
	uint8_t gyroEventSize_Discard = 0;
	uint64_t tick_ts = 0;

	struct accGyroData *data = packet->outBuf;

	if(true == icm_dev.fifo_mode_en) 
	{ //fifo mode
		for(i = 0; i < icm_dev.fifoDataToRead; i += icm_dev.fifo_package_size) 
		{
			//icm_log("Fifo head format is 0x%x\r\n", icm_dev.dataBuf[i]);
			if((accEventSize + gyroEventSize) < MAX_RECV_PACKET) 
			{
				if((true == icm_dev.sensors[ACC].powered) &&
						(true == icm_dev.sensors[ACC].configed)) 
				{
					if(icm_dev.sensors[ACC].samplesToDiscard)
					{
						icm_dev.sensors[ACC].samplesToDiscard--;
						accEventSize_Discard++;
					} 
					else
					{
						inv_icm4x6xx_parse_rawdata(&data[accEventSize + gyroEventSize],
																			 &(icm_dev.dataBuf[i + FIFO_ACCEL_DATA_SHIFT]), ACC);
						data[accEventSize + gyroEventSize].timeStamp = tick_ts;
						accEventSize++;
					}
				}
				if((true == icm_dev.sensors[GYR].powered) &&
						(true == icm_dev.sensors[GYR].configed)) 
				{
					if(icm_dev.sensors[GYR].samplesToDiscard) 
					{
						icm_dev.sensors[GYR].samplesToDiscard--;
						gyroEventSize_Discard++;
					} 
					else 
					{
						inv_icm4x6xx_parse_rawdata(&data[accEventSize + gyroEventSize],
																			 &(icm_dev.dataBuf[i + FIFO_GYRO_DATA_SHIFT]), GYR);
						data[accEventSize + gyroEventSize].timeStamp = tick_ts;
						gyroEventSize++;
					}
				}
				if((true == icm_dev.sensors[ACC].configed) ||
						(true == icm_dev.sensors[GYR].configed))
				{
					inv_icm4x6xx_parse_rawdata(&data[accEventSize + gyroEventSize],
																	 &(icm_dev.dataBuf[i + FIFO_TEMP_DATA_SHIFT]), TEMP);
				}
			} 
			else 
			{
				icm_log("outBuf full, accEventSize = %d, gyroEventSize = %d\r\n", accEventSize, gyroEventSize);
				ret = FIFO_DATA_FULL_ERROR;
			}
		}
	} 
	else 
	{ //dri mode
		if((true == icm_dev.sensors[ACC].configed) &&
				(true == icm_dev.sensors[ACC].powered)) 
		{
			if(icm_dev.sensors[ACC].samplesToDiscard) 
			{
				icm_dev.sensors[ACC].samplesToDiscard--;
				accEventSize_Discard++;
			} 
			else 
			{
				inv_icm4x6xx_parse_rawdata(&data[accEventSize + gyroEventSize],
																	 &(icm_dev.dataBuf[DRI_ACCEL_DATA_SHIFT]), ACC);
				data[accEventSize + gyroEventSize].timeStamp = tick_ts;
				accEventSize++;
			}
		}
		if((true == icm_dev.sensors[GYR].configed) &&
				(true == icm_dev.sensors[GYR].powered)) 
		{
			if(icm_dev.sensors[GYR].samplesToDiscard) 
			{
				icm_dev.sensors[GYR].samplesToDiscard--;
				gyroEventSize_Discard++;
			} 
			else 
			{
				inv_icm4x6xx_parse_rawdata(&data[accEventSize + gyroEventSize],
																	 &(icm_dev.dataBuf[DRI_GYRO_DATA_SHIFT]), GYR);
				data[accEventSize + gyroEventSize].timeStamp = tick_ts;
				gyroEventSize++;
			}
		}
		if((true == icm_dev.sensors[ACC].configed) ||
				(true == icm_dev.sensors[GYR].configed)) 
		{
			inv_icm4x6xx_parse_rawdata(&data[accEventSize + gyroEventSize], &(icm_dev.dataBuf[0]), TEMP);
		}
	}

	packet->accOutSize = accEventSize;
	packet->gyroOutSize = gyroEventSize;
	packet->temperature = icm_dev.chip_temper;
	packet->timeStamp = 0;

	return ret;
}

int inv_icm4x6xx_read_rawdata(void)
{
	int ret = 0;
	int index = 0;
	uint8_t data = 0;

	ret = inv_read(REG_TEMP_DATA1, icm_dev.dri_package_size, icm_dev.dataBuf);
	if(ret) 
	{
		// read data success
		if((true != icm_dev.fifo_mode_en) && (DRI_14BYTES_PACKET_SIZE == icm_dev.dri_package_size))
		{
			// for(index = 0; index < DRI_14BYTES_PACKET_SIZE; index++) {
			//     icm_log("before convert: index[%d] - data[%02xh]\r\n", index, icm_dev.dataBuf[index]);
			// }
			for(index = 0; index < DRI_14BYTES_PACKET_SIZE; index += 2) 
			{
				data = icm_dev.dataBuf[index];
				icm_dev.dataBuf[index] = icm_dev.dataBuf[index + 1];
				icm_dev.dataBuf[index + 1] = data;
			}
			// for(index = 0; index < DRI_14BYTES_PACKET_SIZE; index++) {
			//     icm_log("after convert: index[%d] - data[%02xh]\r\n", index, icm_dev.dataBuf[index]);
			// }
		}
	} 
	else 
	{
		icm_log("read raw data error\r\n");
	}

	return ret;
}

bool sensor_icm4x6xx_get_raw_gyro_acc(struct accGyroDataPacket *dataPacket)
{
	int ret = 0;

	if(dataPacket == NULL) 
	{
		icm_log("DATAPACKET NULL POINTER!!\r\n");
		return false;
	}

	if((false == icm_dev.sensors[ACC].configed) &&
			(false == icm_dev.sensors[GYR].configed)) 
	{
		icm_log("Unexpected DRDY INTR fired\r\n");
		return false;
	}

	ret = inv_icm4x6xx_read_rawdata();
	if(ret == 0) 
	{
		icm_log("Read Raw Data Error %d\r\n", ret);
		return false;
	}

	ret = inv_icm4x6xx_convert_rawdata(dataPacket);
	if(ret != 0) 
	{
		icm_log("Convert Raw Data Error %d\r\n", ret);
		return false;
	}

	return true;
}

int inv_icm4x6xx_get_rawdata(struct accGyroDataPacket *dataPacket)
{
	uint8_t int_status;
	int ret = 0;

	if(dataPacket == NULL)
	{
		icm_log("DATAPACKET NULL POINTER!!\r\n");
		// exit(-1);
		return DRDY_UNEXPECT_INT_ERROR;
	}

	ret = inv_read(REG_INT_STATUS, 1, &int_status);
	// icm_log("INT STATUS 0x%x\r\n", int_status);
	if(ret == 0) 
	{
		icm_log("error for INT STATUS 0x%x\r\n", int_status);
		return INT_STATUS_READ_ERROR;
	}

	if((int_status & BIT_INT_DRDY) && (false == icm_dev.fifo_mode_en)) 
	{
		icm_log("DRDY INT Detected\r\n");
		if((false == icm_dev.sensors[ACC].configed) &&
				(false == icm_dev.sensors[GYR].configed)) 
		{
			icm_log("Unexpected DRDY INTR fired\r\n\r\n");
			//Should enable irq again? Todo
			return DRDY_UNEXPECT_INT_ERROR;
		}
		ret = inv_icm4x6xx_read_rawdata();
		if(ret == 0) 
		{
			icm_log("Read Raw Data Error %d\r\n", ret);
			return DRDY_DATA_READ_ERROR;
		}
		ret = inv_icm4x6xx_convert_rawdata(dataPacket);
		if(ret != 0) 
		{
			icm_log("Convert Raw Data Error %d\r\n", ret);
			return DRDY_DATA_CONVERT_ERROR;
		}

		ret = 0;
	} 
	else 
	{
		icm_log("DRDY INT Detected\r\n");
		ret = 100;
	}

	// if (int_status & BIT_INT_FIFO_FULL) {
	//     icm_log("FIFO Overflow!!!\r\n");
	//     // reset fifo
	//     ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_BYPASS);
	//     //  set fifo stream mode
	//     ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_STREAM);
	//     //Should enable irq again? Todo
	//     return FIFO_OVERFLOW_ERROR;
	// } else if (int_status & BIT_INT_FIFO_WM) {
	//     icm_log("WM INT Detected\r\n");
	//     if ((false == icm_dev.sensors[ACC].configed) &&
	//         (false == icm_dev.sensors[GYR].configed)) {
	//         icm_log("Unexpected FIFO WM INTR fired\r\n");
	//         //reset fifo
	//         ret += inv_write(REG_FIFO_CONFIG, BIT_FIFO_MODE_CTRL_BYPASS);
	//         //Should enable irq again? Todo
	//         return FIFO_UNEXPECT_WM_ERROR;
	//     }
	//     ret += inv_icm4x6xx_read_fifo();
	//     if (ret != 0) {
	//         icm_log("Fifo Data Read Error %d\r\n", ret);
	//         return FIFO_DATA_READ_ERROR;
	//     }
	//     ret += inv_icm4x6xx_convert_rawdata(dataPacket);
	//     if (ret != 0) {
	//         icm_log("Fifo Data Convert Error %d\r\n", ret);
	//         return FIFO_DATA_CONVERT_ERROR;
	//     }
	// } else {
	// icm_log("timewheel read data\r\n");
	// if ((false == icm_dev.sensors[ACC].configed) &&
	//     (false == icm_dev.sensors[GYR].configed)) {
	//     icm_log("Unexpected DRDY INTR fired\r\n");
	//     //Should enable irq again? Todo
	//     return DRDY_UNEXPECT_INT_ERROR;
	// }
	// ret = inv_icm4x6xx_read_rawdata();
	// if (ret == 0) {
	//     icm_log("Read Raw Data Error %d\r\n", ret);
	//     return DRDY_DATA_READ_ERROR;
	// }
	// ret = inv_icm4x6xx_convert_rawdata(dataPacket);
	// if (ret != 0) {
	//     icm_log("Convert Raw Data Error %d\r\n", ret);
	//     return DRDY_DATA_CONVERT_ERROR;
	// }
	// }

	return ret;
}

#if SUPPORT_WOM
int inv_icm4x6xx_wom_enable(void)
{
	int ret = 0;
	uint8_t data = 0;

	// if acc is not in streaming mode enable acc
	if(!icm_dev.sensors[ACC].powered) 
	{
		icm_dev.pwr_sta &= ~BIT_ACCEL_MODE_MASK;
		icm_dev.pwr_sta |= BIT_ACCEL_MODE_LN;
		ret = inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
		inv_delay_us(200);
		icm_log("wom enable, acc on\r\n");
	}

	/* set X,Y,Z threshold */
	ret = inv_write(REG_BANK_SEL, 4);// Bank 4
	ret = inv_write(REG_ACCEL_WOM_X_THR, DEFAULT_WOM_THS_MG);
	ret = inv_write(REG_ACCEL_WOM_Y_THR, DEFAULT_WOM_THS_MG);
	ret = inv_write(REG_ACCEL_WOM_Z_THR, DEFAULT_WOM_THS_MG);
	ret = inv_write(REG_BANK_SEL, 0);// Bank 0

	inv_delay_ms(5);
	ret = inv_read(REG_INT_SOURCE1, 1, &data);
	data |= (uint8_t)BIT_INT1_WOM_EN_MASK;
	ret = inv_write(REG_INT_SOURCE1, data);

	inv_delay_ms(60);

	ret = inv_read(REG_SMD_CONFIG, 1, &data);
	data &= (uint8_t)~BIT_SMD_MODE_MASK;
	data |= (uint8_t)BIT_SMD_MODE_WOM;
	data |= (uint8_t)BIT_WOM_MODE_COMPARE_PRE;
	icm_log("SMD_CFG is 0x%x\r\n", data);
	ret = inv_write(REG_SMD_CONFIG, data);

	// if (ret != 0)
	//     return WOM_ENABLE_ERROR;

	icm_dev.sensors[WOM].powered = true;

	return ret;
}

int inv_icm4x6xx_wom_disable(void)
{
	int ret = 0;
	uint8_t data = 0;

	data = 0;
	ret = inv_read(REG_INT_SOURCE1, 1, &data);
	data &= (uint8_t)~BIT_INT1_WOM_EN_MASK;
	ret = inv_write(REG_INT_SOURCE1, data);
	icm_log("read REG_INT_SOURCE1[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_INT_SOURCE1, ret, data);

	data = 0;
	ret = inv_read(REG_SMD_CONFIG, 1, &data);
	data &= (uint8_t)~BIT_SMD_MODE_MASK;
	ret = inv_write(REG_SMD_CONFIG, data);

	data = 0;
	ret = inv_read(REG_SMD_CONFIG, 1, &data);
	icm_log("read REG_SMD_CONFIG[%02xh] -- ret[%02xh]--data[%02xh]\r\n", REG_SMD_CONFIG, ret, data);

	if(false == icm_dev.sensors[ACC].powered
#if SUPPORT_PEDOMETER
		&& false == icm_dev.sensors[PEDO].powered
#endif
    ) 
	{
		icm_dev.pwr_sta &= ~BIT_ACCEL_MODE_MASK;
		ret = inv_write(REG_PWR_MGMT0, icm_dev.pwr_sta);
		inv_delay_us(200);
		icm_log("wom disable, acc off pwr: 0x%x\r\n", icm_dev.pwr_sta);
	}

	icm_dev.sensors[WOM].powered = false;

	return ret;
}

int inv_icm4x6xx_wom_get_event(bool *event)
{
	if(event == NULL) 
	{
		icm_log("EVENT NULL POINTER!!\r\n");
		// exit(-1);
		return false;
	}

	int ret = 0;
	uint8_t int_status = 0;

	ret += inv_read(REG_INT_STATUS2, 1, &int_status);
	icm_log("INT_STATUS2 0x%x\r\n", int_status);
	if(int_status & BIT_INT_WOM_MASK) 
	{
		icm_log("Motion Detected !!\r\n");
		*event = true;
	}

	return ret;
}
#endif

void inv_icm4x6xx_read_data(float acc[3], float gyr[3])
{
	sensor_icm4x6xx_get_raw_gyro_acc(&icm_data);
	acc[0] = icm_data.outBuf[0].x;
	acc[1] = icm_data.outBuf[0].y;
	acc[2] = icm_data.outBuf[0].z;

	gyr[0] = icm_data.outBuf[1].x;
	gyr[1] = icm_data.outBuf[1].y;
	gyr[2] = icm_data.outBuf[1].z;
}

