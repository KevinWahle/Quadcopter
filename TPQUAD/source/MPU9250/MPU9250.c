#include <math.h>
#include "I2Cm/I2Cm.h"
#include "timer/timer.h"
#include "MPU9250.h"

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C<<1
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
// #define ADO 0
// #if ADO
// #define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
// #else
// #define MPU9250_ADDRESS 0x69 // Device address when ADO = 0
// #endif  
#define MPU9250_ADDRESS 0x69
// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR  
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

uint8_t writeBuffer[4];
uint8_t writeSize;
uint8_t readBuffer[10];
uint8_t readSize;
int32_t gyroPartialSum[3] = {0, 0, 0};
int32_t accelPartialSum[3]={0,0,0};
void initMPU9250(){
	
	// Initialize MPU9250 device
	// wakebool I2CmStartTransaction(I2CPort_t id, uint8_t address, uint8_t* writeBuffer, uint8_t writeSize, uint8_t* readBuffer, uint8_t readSize)
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = PWR_MGMT_1;
	writeBuffer[1] = 0x00;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	//writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	//wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
	timerDelay(TIMER_MS2TICKS(100));

	// get stable time source
	//writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = PWR_MGMT_1;
	writeBuffer[1] = 0x01;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	//writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = CONFIG;
	writeBuffer[1] = 0x03;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	//writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = SMPLRT_DIV;
	writeBuffer[1] = 0x04;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	//uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value  c--->readBuffer[0]
	writeSize = 1;
	readSize = 1;
	writeBuffer[0] = GYRO_CONFIG;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, readBuffer, readSize);
	while(isI2CBusy(I2C_ACC));
	//readBuffer[0] = readBuffer[0] & ~0xE0; // Clear self-test bits [7:5]
	readBuffer[0] = readBuffer[0] & ~0x02; // Clear Fchoice bits [1:0]
	readBuffer[0] = readBuffer[0] & ~0x18; // Clear AFS bits [4:3]
	readBuffer[0] = readBuffer[0] | Gscale << 3; // Set minimun scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	//writeByte(MPU9250_ADDRESS, GYRO_CONFIG, readBuffer[0] ); // Write new GYRO_CONFIG value to register
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = GYRO_CONFIG;
	writeBuffer[1] = readBuffer[0];
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	// Set accelerometer full-scale range configuration
	//c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	writeSize = 1;
	readSize = 1;
	writeBuffer[0] = ACCEL_CONFIG;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, readBuffer, readSize);
	while(isI2CBusy(I2C_ACC));
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	readBuffer[0] = readBuffer[0] & ~0x18;  // Clear AFS bits [4:3]
	readBuffer[0] = readBuffer[0] | Ascale << 3; // Set full scale range for the accelerometer
	//writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, readBuffer[0]); // Write new ACCEL_CONFIG register value
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = ACCEL_CONFIG;
	writeBuffer[1] = readBuffer[0];
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	//c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	writeSize = 1;
	readSize = 1;
	writeBuffer[0] = ACCEL_CONFIG2;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, readBuffer, readSize);
	while(isI2CBusy(I2C_ACC));
	readBuffer[0] = readBuffer[0] & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	readBuffer[0] = readBuffer[0] | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	//writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, readBuffer[0]); // Write new ACCEL_CONFIG2 register value
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = ACCEL_CONFIG2;
	writeBuffer[1] = readBuffer[0];
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	//writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = INT_PIN_CFG;
	writeBuffer[1] = 0x22;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	//writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = INT_ENABLE;
	writeBuffer[1] = 0x00; //Without interrupts.
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
}



void whoAmI(uint8_t * read){
	writeSize = 1;
	readSize = 1;
	writeBuffer[0] = WHO_AM_I_MPU9250;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, read, readSize);
	while(isI2CBusy(I2C_ACC));
}

void readAccelData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	writeSize = 1;
	readSize = 6;
	writeBuffer[0] = ACCEL_XOUT_H;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, rawData, readSize);
	while(isI2CBusy(I2C_ACC));
	//readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}
void readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	writeSize = 1;
	readSize = 6;
	writeBuffer[0] = GYRO_XOUT_H;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, rawData, readSize);
	while(isI2CBusy(I2C_ACC));
	//readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	
	destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
	destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

}

void resetMPU9250() {
  // reset device
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = PWR_MGMT_1;
	writeBuffer[1] = 0x80;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	timerDelay(TIMER_MS2TICKS(100));
  	//writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	//wait(0.1);
  }

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
//   writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
//   wait(0.01);
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = AK8963_CNTL;
	writeBuffer[1] = 0x00;
	I2CmStartTransaction(I2C_ACC, AK8963_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	timerDelay(TIMER_MS2TICKS(100));

//   writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
//   wait(0.01);
  	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = AK8963_CNTL;
	writeBuffer[1] = 0x0F;
	I2CmStartTransaction(I2C_ACC, AK8963_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	timerDelay(TIMER_MS2TICKS(100));

//   readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	writeSize = 3;
	readSize = 1;
	writeBuffer[0] = AK8963_ASAX;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, rawData, readSize);
	while(isI2CBusy(I2C_ACC));

	destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
	destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 

//   writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
//   wait(0.01);
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = AK8963_CNTL;
	writeBuffer[1] = 0x00;
	I2CmStartTransaction(I2C_ACC, AK8963_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	timerDelay(TIMER_MS2TICKS(100));
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  	// writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
 	// wait(0.01);
	writeSize = 2;
	readSize = 0;
	writeBuffer[0] = AK8963_CNTL;
	writeBuffer[1] = Mscale << 4 | Mmode;
	I2CmStartTransaction(I2C_ACC, AK8963_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	while(isI2CBusy(I2C_ACC));
	timerDelay(TIMER_MS2TICKS(100));
  
}
#define AMOUNT_OF_SAMPLES 1000
void calibrateMPU9250()
{
	
	for(uint16_t i=0; i < AMOUNT_OF_SAMPLES; i++)
	{
		int16_t tmpGryro[3];
		readGyroData(tmpGryro);	
		gyroPartialSum[0] += tmpGryro[0];
		gyroPartialSum[1] += tmpGryro[1];
		gyroPartialSum[2] += tmpGryro[2];
	}
	gyroPartialSum[0] /= -AMOUNT_OF_SAMPLES;
	gyroPartialSum[1] /= -AMOUNT_OF_SAMPLES;
	gyroPartialSum[2] /= -AMOUNT_OF_SAMPLES;

	// =========== X adjustment ==================
	// writeSize = 2;
	// readSize = 0;
	// writeBuffer[0] = XG_OFFSET_H;
	// writeBuffer[1] = (uint8_t)(gyroPartialSum[0]>>8);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// writeBuffer[0] = XG_OFFSET_L;
	// writeBuffer[1] = (uint8_t)(gyroPartialSum[0]);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));
	// // =========== Y adjustment ==================
	// writeBuffer[0] = YG_OFFSET_H;
	// writeBuffer[1] = (uint8_t)(gyroPartialSum[1]>>8);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// writeBuffer[0] = YG_OFFSET_L;
	// writeBuffer[1] = (uint8_t)(gyroPartialSum[1]);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// // =========== Z adjustment ==================
	// writeBuffer[0] = ZG_OFFSET_H;
	// writeBuffer[1] = (uint8_t)(gyroPartialSum[2]>>8);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// writeBuffer[0] = ZG_OFFSET_L;
	// writeBuffer[1] = (uint8_t)(gyroPartialSum[2]);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));
	// // ===========================================


	for(uint16_t i=0; i < AMOUNT_OF_SAMPLES; i++)
	{
		int16_t tmpAcc[3];
		readGyroData(tmpAcc);
		accelPartialSum[0] += tmpAcc[0];
		accelPartialSum[1] += tmpAcc[1];
		accelPartialSum[2] += tmpAcc[2];
	}
	accelPartialSum[0] /= -AMOUNT_OF_SAMPLES;
	accelPartialSum[1] /= -AMOUNT_OF_SAMPLES;
	accelPartialSum[2] = (32767/2) - (accelPartialSum[2]/AMOUNT_OF_SAMPLES) ; // Hay que restar la gravedad.

	// =========== X adjustment ==================
	// writeSize = 2;
	// readSize = 0;
	// writeBuffer[0] = XA_OFFSET_H;
	// writeBuffer[1] = (uint8_t)(accelPartialSum[0]>>8);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// writeBuffer[0] = XA_OFFSET_L;
	// writeBuffer[1] = (uint8_t)(accelPartialSum[0]);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));
	// // =========== Y adjustment ==================
	// writeBuffer[0] = YA_OFFSET_H;
	// writeBuffer[1] = (uint8_t)(accelPartialSum[1]>>8);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// writeBuffer[0] = YA_OFFSET_L;
	// writeBuffer[1] = (uint8_t)(accelPartialSum[1]);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// // =========== Z adjustment ==================
	// writeBuffer[0] = ZA_OFFSET_H;
	// writeBuffer[1] = (uint8_t)(accelPartialSum[2]>>8);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));

	// writeBuffer[0] = ZA_OFFSET_L;
	// writeBuffer[1] = (uint8_t)(accelPartialSum[2]);
	// I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, NULL, readSize);
	// while(isI2CBusy(I2C_ACC));
	// ===========================================

}
void int2doubleGyro(double* destination, int16_t* rawData)
{
	destination[0] = ((double)(rawData[0] + gyroPartialSum[0]))/32767.0 * 250;
	destination[1] = ((double)(rawData[1] + gyroPartialSum[1]))/32767.0 * 250;
	destination[2] = ((double)(rawData[2] + gyroPartialSum[2]))/32767.0 * 250;	
}
void int2doubleAcc(double* destination, int16_t* rawData)
{
	destination[0] = ((double)(rawData[0] + accelPartialSum[0]))/32767.0 * 2; // 2 por 2G
	destination[1] = ((double)(rawData[1] + accelPartialSum[1]))/32767.0 * 2;
	destination[2] = ((double)(rawData[2] + accelPartialSum[2]))/32767.0 * 2;
}
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
// void calibrateMPU9250(float * dest1, float * dest2)
// {  
//   uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
//   uint16_t ii, packet_count, fifo_count;
//   int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
// // reset device, reset all registers, clear gyro and accelerometer bias registers
//   writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
//   wait(0.1);  
   
// // get stable time source
// // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
//   writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
//   writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00); 
//   wait(0.2);
  
// // Configure device for bias calculation
//   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
//   writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
//   writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
//   writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
//   writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
//   writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
//   wait(0.015);
  
// // Configure MPU9250 gyro and accelerometer for bias calculation
//   writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
//   writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
//   writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
//   uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
//   uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// // Configure FIFO to capture accelerometer and gyro data for bias calculation
//   writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
//   writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
//   wait(0.04); // accumulate 40 samples in 80 milliseconds = 480 bytes

// // At end of sample accumulation, turn off FIFO sensor read
//   writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
//   readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
//   fifo_count = ((uint16_t)data[0] << 8) | data[1];
//   packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

//   for (ii = 0; ii < packet_count; ii++) {
//     int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
//     readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
//     accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
//     accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
//     accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
//     gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
//     gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
//     gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
//     accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//     accel_bias[1] += (int32_t) accel_temp[1];
//     accel_bias[2] += (int32_t) accel_temp[2];
//     gyro_bias[0]  += (int32_t) gyro_temp[0];
//     gyro_bias[1]  += (int32_t) gyro_temp[1];
//     gyro_bias[2]  += (int32_t) gyro_temp[2];
            
// }
//     accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
//     accel_bias[1] /= (int32_t) packet_count;
//     accel_bias[2] /= (int32_t) packet_count;
//     gyro_bias[0]  /= (int32_t) packet_count;
//     gyro_bias[1]  /= (int32_t) packet_count;
//     gyro_bias[2]  /= (int32_t) packet_count;
    
//   if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
//   else {accel_bias[2] += (int32_t) accelsensitivity;}
 
// // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//   data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//   data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//   data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
//   data[3] = (-gyro_bias[1]/4)       & 0xFF;
//   data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
//   data[5] = (-gyro_bias[2]/4)       & 0xFF;

// /// Push gyro biases to hardware registers
// /*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
//   writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
//   writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
//   writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
//   writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
//   writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
// */
//   dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
//   dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
//   dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// // the accelerometer biases calculated above must be divided by 8.

//   int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//   readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//   accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//   readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//   accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
//   readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//   accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
  
//   uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
//   uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
//   for(ii = 0; ii < 3; ii++) {
//     if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
//   }

//   // Construct total accelerometer bias, including calculated average accelerometer bias from above
//   accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//   accel_bias_reg[1] -= (accel_bias[1]/8);
//   accel_bias_reg[2] -= (accel_bias[2]/8);
 
//   data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//   data[1] = (accel_bias_reg[0])      & 0xFF;
//   data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//   data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//   data[3] = (accel_bias_reg[1])      & 0xFF;
//   data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
//   data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//   data[5] = (accel_bias_reg[2])      & 0xFF;
//   data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// // Apparently this is not working for the acceleration biases in the MPU-9250
// // Are we handling the temperature correction bit properly?
// // Push accelerometer biases to hardware registers
// /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//   writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//   writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//   writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//   writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//   writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
// */
// // Output scaled accelerometer biases for manual subtraction in the main program
//    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
//    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
//    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
// }





















