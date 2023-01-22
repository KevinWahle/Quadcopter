#include <math.h>
#include "I2Cm/I2Cm.h"

/**
 * @brief chip information definition
 */
#define CHIP_NAME                 "TDK MPU9250"        /**< chip name */
#define MANUFACTURER_NAME         "TDK"                /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        2.4f                 /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        3.6f                 /**< chip max supply voltage */
#define MAX_CURRENT               3.7f                 /**< chip max current */
#define TEMPERATURE_MIN           -40.0f               /**< chip min operating temperature */
#define TEMPERATURE_MAX           85.0f                /**< chip max operating temperature */
#define DRIVER_VERSION            1000                 /**< driver version */

/**
 * @brief chip ak8963 iic address definition
 */
#define AK8963_IIC_ADDRESS              0x0C        /**< ak8963 iic address */

/**
 * @brief chip register definition
 */
#define MPU9250_REG_SELF_TEST_X_GYRO    0x00        /**< gyro self test x register */
#define MPU9250_REG_SELF_TEST_Y_GYRO    0x01        /**< gyro self test y register */
#define MPU9250_REG_SELF_TEST_Z_GYRO    0x02        /**< gyro self test z register */
#define MPU9250_REG_SELF_TEST_X_ACCEL   0x0D        /**< accel self test x register */
#define MPU9250_REG_SELF_TEST_Y_ACCEL   0x0E        /**< accel self test y register */
#define MPU9250_REG_SELF_TEST_Z_ACCEL   0x0F        /**< accel self test z register */
#define MPU9250_REG_XG_OFFSET_H         0x13        /**< gyro offset x high register */
#define MPU9250_REG_XG_OFFSET_L         0x14        /**< gyro offset x low register */
#define MPU9250_REG_YG_OFFSET_H         0x15        /**< gyro offset y high register */
#define MPU9250_REG_YG_OFFSET_L         0x16        /**< gyro offset y low register */
#define MPU9250_REG_ZG_OFFSET_H         0x17        /**< gyro offset z high register */
#define MPU9250_REG_ZG_OFFSET_L         0x18        /**< gyro offset z low register */
#define MPU9250_REG_SMPRT_DIV           0x19        /**< smprt div register */
#define MPU9250_REG_CONFIG              0x1A        /**< configure register */
#define MPU9250_REG_GYRO_CONFIG         0x1B        /**< gyro configure register */
#define MPU9250_REG_ACCEL_CONFIG        0x1C        /**< accel configure register */
#define MPU9250_REG_ACCEL_CONFIG2       0x1D        /**< accel configure 2 register */
#define MPU9250_REG_LP_ACCEL_ODR        0x1E        /**< low power accel odr register */
#define MPU9250_REG_WOM_THR             0x1F        /**< wom threshold register */
#define MPU9250_REG_MOTION_DURATION     0x20        /**< motion duration register */
#define MPU9250_REG_FIFO_EN             0x23        /**< fifo enable register */
#define MPU9250_REG_I2C_MST_CTRL        0x24        /**< i2c master ctrl register */
#define MPU9250_REG_I2C_MST_STATUS      0x36        /**< i2c master status register */
#define MPU9250_REG_I2C_MST_DELAY_CTRL  0x67        /**< i2c master delay ctrl register */
#define MPU9250_REG_I2C_SLV0_ADDR       0x25        /**< iic slave0 address register */
#define MPU9250_REG_I2C_SLV0_REG        0x26        /**< iic slave0 reg register */
#define MPU9250_REG_I2C_SLV0_CTRL       0x27        /**< iic slave0 ctrl register */
#define MPU9250_REG_I2C_SLV0_DO         0x63        /**< iic slave0 do register */
#define MPU9250_REG_I2C_SLV1_ADDR       0x28        /**< iic slave1 address register */
#define MPU9250_REG_I2C_SLV1_REG        0x29        /**< iic slave1 reg register */
#define MPU9250_REG_I2C_SLV1_CTRL       0x2A        /**< iic slave1 ctrl register */
#define MPU9250_REG_I2C_SLV1_DO         0x64        /**< iic slave1 do register */
#define MPU9250_REG_I2C_SLV2_ADDR       0x2B        /**< iic slave2 address register */
#define MPU9250_REG_I2C_SLV2_REG        0x2C        /**< iic slave2 reg register */
#define MPU9250_REG_I2C_SLV2_CTRL       0x2D        /**< iic slave2 ctrl register */
#define MPU9250_REG_I2C_SLV2_DO         0x65        /**< iic slave2 do register */
#define MPU9250_REG_I2C_SLV3_ADDR       0x2E        /**< iic slave3 address register */
#define MPU9250_REG_I2C_SLV3_REG        0x2F        /**< iic slave3 reg register */
#define MPU9250_REG_I2C_SLV3_CTRL       0x30        /**< iic slave3 ctrl register */
#define MPU9250_REG_I2C_SLV3_DO         0x66        /**< iic slave3 do register */
#define MPU9250_REG_I2C_SLV4_ADDR       0x31        /**< iic slave4 address register */
#define MPU9250_REG_I2C_SLV4_REG        0x32        /**< iic slave4 reg register */
#define MPU9250_REG_I2C_SLV4_CTRL       0x34        /**< iic slave4 ctrl register */
#define MPU9250_REG_I2C_SLV4_DO         0x33        /**< iic slave4 do register */
#define MPU9250_REG_I2C_SLV4_DI         0x35        /**< iic slave4 di register */
#define MPU9250_REG_EXT_SENS_DATA_00    0x49        /**< extern sensor data 00 register */
#define MPU9250_REG_INT_PIN_CFG         0x37        /**< interrupt pin configure register */
#define MPU9250_REG_INT_ENABLE          0x38        /**< interrupt enable register */
#define MPU9250_REG_INT_STATUS          0x3A        /**< interrupt status register */
#define MPU9250_REG_ACCEL_XOUT_H        0x3B        /**< accel xout high register */
#define MPU9250_REG_ACCEL_XOUT_L        0x3C        /**< accel xout low register */
#define MPU9250_REG_ACCEL_YOUT_H        0x3D        /**< accel yout high register */
#define MPU9250_REG_ACCEL_YOUT_L        0x3E        /**< accel yout low register */
#define MPU9250_REG_ACCEL_ZOUT_H        0x3F        /**< accel zout high register */
#define MPU9250_REG_ACCEL_ZOUT_L        0x40        /**< accel zout low register */
#define MPU9250_REG_TEMP_OUT_H          0x41        /**< temp high register */
#define MPU9250_REG_TEMP_OUT_L          0x42        /**< temp low register */
#define MPU9250_REG_GYRO_XOUT_H         0x43        /**< gyro xout high register */
#define MPU9250_REG_GYRO_XOUT_L         0x44        /**< gyro xout low register */
#define MPU9250_REG_GYRO_YOUT_H         0x45        /**< gyro yout high register */
#define MPU9250_REG_GYRO_YOUT_L         0x46        /**< gyro yout low register */
#define MPU9250_REG_GYRO_ZOUT_H         0x47        /**< gyro zout high register */
#define MPU9250_REG_GYRO_ZOUT_L         0x48        /**< gyro zout low register */
#define MPU9250_REG_SIGNAL_PATH_RESET   0x68        /**< signal path reset register */
#define MPU9250_REG_MOT_DETECT_CTRL     0x69        /**< motion detect ctrl register */
#define MPU9250_REG_USER_CTRL           0x6A        /**< user ctrl register */
#define MPU9250_REG_PWR_MGMT_1          0x6B        /**< power mangement 1 register */
#define MPU9250_REG_PWR_MGMT_2          0x6C        /**< power mangement 2 register */
#define MPU9250_REG_BANK_SEL            0x6D        /**< bank sel register */
#define MPU9250_REG_MEM                 0x6F        /**< memory register */
#define MPU9250_REG_PROGRAM_START       0x70        /**< program start register */
#define MPU9250_REG_FIFO_COUNTH         0x72        /**< fifo count high threshold register */
#define MPU9250_REG_FIFO_COUNTL         0x73        /**< fifo count low threshold register */
#define MPU9250_REG_R_W                 0x74        /**< fifo read write data register */
#define MPU9250_REG_WHO_AM_I            0x75        /**< who am I register */
#define MPU9250_REG_XA_OFFSET_H         0x77        /**< accel offset x high register */
#define MPU9250_REG_XA_OFFSET_L         0x78        /**< accel offset x low register */
#define MPU9250_REG_YA_OFFSET_H         0x7A        /**< accel offset y high register */
#define MPU9250_REG_YA_OFFSET_L         0x7B        /**< accel offset y low register */
#define MPU9250_REG_ZA_OFFSET_H         0x7D        /**< accel offset z high register */
#define MPU9250_REG_ZA_OFFSET_L         0x7E        /**< accel offset z low register */
#define AK8963_REG_WIA                  0x00        /**< device id register */
#define AK8963_REG_INFO                 0x01        /**< information register */
#define AK8963_REG_ST1                  0x02        /**< status 1 register */
#define AK8963_REG_HXL                  0x03        /**< x axis data high register */
#define AK8963_REG_HXH                  0x04        /**< x axis data low register */
#define AK8963_REG_HYL                  0x05        /**< y axis data high register */
#define AK8963_REG_HYH                  0x06        /**< y axis data low register */
#define AK8963_REG_HZL                  0x07        /**< z axis data high register */
#define AK8963_REG_HZH                  0x08        /**< z axis data low register */
#define AK8963_REG_ST2                  0x09        /**< status 2 register */
#define AK8963_REG_CNTL1                0x0A        /**< control 1 register */
#define AK8963_REG_CNTL2                0x0B        /**< control 2 register */
#define AK8963_REG_ASTC                 0x0C        /**< self test register */
#define AK8963_REG_TS1                  0x0D        /**< test 1 register */
#define AK8963_REG_TS2                  0x0E        /**< test 2 register */
#define AK8963_REG_I2CDIS               0x0F        /**< iic disable register */
#define AK8963_REG_ASAX                 0x10        /**< x axis sensitivity adjustment value register */
#define AK8963_REG_ASAY                 0x11        /**< y axis sensitivity adjustment value register */
#define AK8963_REG_ASAZ                 0x12        /**< z axis sensitivity adjustment value register */

/**
 * @brief mpu9250 dmp code definition
 */
#define MPU9250_DMP_SAMPLE_RATE               200                                                 /**< sample rate */
#define MPU9250_DMP_GYRO_SF                   (46850825LL * 200 / MPU9250_DMP_SAMPLE_RATE)        /**< gyro sf */
#define MPU9250_DMP_D_PEDSTD_TIMECTR          964                                                 /**< walk time mem register */
#define MPU9250_DMP_D_PEDSTD_STEPCTR          (768 + 0x60)                                        /**< step counter mem register */
#define MPU9250_DMP_D_1_36                    (256 + 36)                                          /**< 36 register */
#define MPU9250_DMP_D_1_40                    (256 + 40)                                          /**< 40 register */
#define MPU9250_DMP_D_1_44                    (256 + 44)                                          /**< 44 register */
#define MPU9250_DMP_D_1_72                    (256 + 72)                                          /**< 72 register */
#define MPU9250_DMP_D_1_79                    (256 + 79)                                          /**< 79 register */
#define MPU9250_DMP_D_1_88                    (256 + 88)                                          /**< 88 register */
#define MPU9250_DMP_D_1_90                    (256 + 90)                                          /**< 90 register */
#define MPU9250_DMP_D_1_92                    (256 + 92)                                          /**< 92 register */
#define MPU9250_DMP_D_1_218                   (256 + 218)                                         /**< 218 register */
#define MPU9250_DMP_D_0_22                    (512 + 22)                                          /**< 22 register */
#define MPU9250_DMP_D_0_104                   104                                                 /**< 104 register */
#define MPU9250_DMP_TAPW_MIN                  478                                                 /**< tap time min register */
#define MPU9250_DMP_TAP_THX                   468                                                 /**< tap threshold x register */
#define MPU9250_DMP_TAP_THY                   472                                                 /**< tap threshold y register */
#define MPU9250_DMP_TAP_THZ                   476                                                 /**< tap threshold z register */
#define MPU9250_DMP_CFG_6                     2753                                                /**< cfg 6 register */
#define MPU9250_DMP_CFG_8                     2718                                                /**< cfg 8 register */
#define MPU9250_DMP_CFG_MOTION_BIAS           1208                                                /**< motion bias register */
#define MPU9250_DMP_CFG_LP_QUAT               2712                                                /**< lp quat register */
#define MPU9250_DMP_CFG_FIFO_ON_EVENT         2690                                                /**< fifo on event register */
#define MPU9250_DMP_FCFG_1                    1062                                                /**< fcfg 1 register */
#define MPU9250_DMP_FCFG_2                    1066                                                /**< fcfg 2 register */
#define MPU9250_DMP_FCFG_3                    1088                                                /**< fcfg 3 register */
#define MPU9250_DMP_FCFG_7                    1073                                                /**< fcfg 7 register */
#define MPU9250_DMP_D_EXT_GYRO_BIAS_X         (61 * 16)                                           /**< gyro bias x register */
#define MPU9250_DMP_D_EXT_GYRO_BIAS_Y         (61 * 16 + 4)                                       /**< gyro bias y register */
#define MPU9250_DMP_D_EXT_GYRO_BIAS_Z         (61 * 16 + 8)                                       /**< gyro bias z register */
#define MPU9250_DMP_D_ACCEL_BIAS              660                                                 /**< accel bias register */
#define MPU9250_DMP_FEATURE_SEND_ANY_GYRO     (MPU9250_DMP_FEATURE_SEND_RAW_GYRO | \
                                               MPU9250_DMP_FEATURE_SEND_CAL_GYRO)                 /**< send any gyro register */
#define MPU9250_DMP_CFG_15                    2727                                                /**< cfg 15 register */
#define MPU9250_DMP_CFG_27                    2742                                                /**< cfg 27 register */
#define MPU9250_DMP_CFG_GYRO_RAW_DATA         2722                                                /**< cfg gyro raw data register */
#define MPU9250_DMP_CFG_20                    2224                                                /**< cfg 20 register */
#define MPU9250_DMP_CFG_ORIENT_INT            1853                                                /**< cfg orient int register */
#define MPU9250_DMP_QUAT_ERROR_THRESH         (1L << 24)                                          /**< quat error thresh */
#define MPU9250_DMP_QUAT_MAG_SQ_NORMALIZED    (1L << 28)                                          /**< quat mag sq normalized */
#define MPU9250_DMP_QUAT_MAG_SQ_MIN           (MPU9250_DMP_QUAT_MAG_SQ_NORMALIZED - \
                                               MPU9250_DMP_QUAT_ERROR_THRESH)                     /**< quat mag sq min */
#define MPU9250_DMP_QUAT_MAG_SQ_MAX           (MPU9250_DMP_QUAT_MAG_SQ_NORMALIZED + \
                                               MPU9250_DMP_QUAT_ERROR_THRESH)                     /**< quat mag sq max */
#define MPU9250_DMP_INT_SRC_TAP               0x01                                                /**< int src tap */
#define MPU9250_DMP_INT_SRC_ORIENT            0x08                                                /**< int src orient */
#define MPU9250_DMP_TAP_THRESH                250                                                 /**< 250 mg/ms */
#define MPU9250_DMP_TAP_MIN_TAP_COUNT         1                                                   /**< 1 */
#define MPU9250_DMP_TAP_TIME                  100                                                 /**< 100 ms */
#define MPU9250_DMP_TAP_TIME_MULTI            200                                                 /**< 200 ms */
#define MPU9250_DMP_SHAKE_REJECT_THRESH       200                                                 /**< 200 ms */
#define MPU9250_DMP_SHAKE_REJECT_TIME         40                                                  /**< 40 ms */
#define MPU9250_DMP_SHAKE_REJECT_TIMEOUT      10                                                  /**< 10 ms */

/**
 * @brief inner function definition
 */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))        /**< min function */

/**
 * @brief test st table definition
 */

void initMPU9250(){
	// Initialize MPU9250 device
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x02; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear AFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}




















