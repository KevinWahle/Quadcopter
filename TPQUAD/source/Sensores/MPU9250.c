#include <math.h>
#include <Sensores/MPU9250.h>
#include "I2Cm/I2Cm.h"


#define AMOUNT_OF_SAMPLES 1000

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

static const uint16_t DPS_SCALE[] = {250, 500, 1000, 2000};
static const uint16_t G_SCALE[] = {2, 4, 8, 16};

static uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
static uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
static uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
static uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
static double aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

static uint8_t writeBuffer[4];
static uint8_t writeSize;
static uint8_t readBuffer[10];
static uint8_t readSize;
static int32_t gyroAverage[3] = {0, 0, 0};
static int32_t accelAverage[3]={0,0,0};



void initMPU9250(){
	timerInit();
	I2CmInit(I2C_ACC);
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
	writeBuffer[1] = 0x01;
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
	readBuffer[0] = 0x06;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
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

void readAccelData(Acc * accData)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	writeSize = 1;
	readSize = 6;
	writeBuffer[0] = ACCEL_XOUT_H;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, rawData, readSize);
	while(isI2CBusy(I2C_ACC));
	//readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	accData->rawX = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	accData->rawY = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	accData->rawZ = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	raw2trueAcc(accData);
}
void readGyroData(Gyro * gyroData)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	writeSize = 1;
	readSize = 6;
	writeBuffer[0] = GYRO_XOUT_H;
	I2CmStartTransaction(I2C_ACC, MPU9250_ADDRESS, writeBuffer, writeSize, rawData, readSize);
	while(isI2CBusy(I2C_ACC));
	//readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	
	gyroData->rawX = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	gyroData->rawY = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	gyroData->rawZ = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	raw2trueGyro(gyroData);
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

void initAK8963(double * destination)
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

	destination[0] =  (double)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] =  (double)(rawData[1] - 128)/256.0f + 1.0f;
	destination[2] =  (double)(rawData[2] - 128)/256.0f + 1.0f;

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

void calibrateMPU9250()
{
	double gyroPartialSum[3] = {0, 0, 0};
	for(uint16_t i=0; i < AMOUNT_OF_SAMPLES; i++)
	{
		Gyro tmpGyro = {.rawX = 0, .rawY = 0, .rawZ = 0};
		readGyroData(&tmpGyro);
		gyroPartialSum[0] += tmpGyro.rawX;
		gyroPartialSum[1] += tmpGyro.rawY;
		gyroPartialSum[2] += tmpGyro.rawZ;
	}
	gyroAverage[0] = -gyroPartialSum[0]/AMOUNT_OF_SAMPLES;
	gyroAverage[1] = -gyroPartialSum[1]/AMOUNT_OF_SAMPLES;
	gyroAverage[2] = -gyroPartialSum[2]/AMOUNT_OF_SAMPLES;

	double accelPartialSum[3] = {0, 0, 0};
	for(uint16_t i=0; i < AMOUNT_OF_SAMPLES; i++)
	{
		Acc tmpAcc = {.rawX = 0, .rawY = 0, .rawZ = 0};
		readAccelData(&tmpAcc);
		accelPartialSum[0] += tmpAcc.rawX;
		accelPartialSum[1] += tmpAcc.rawY;
		accelPartialSum[2] += tmpAcc.rawZ;
	}
	accelAverage[0] = -accelPartialSum[0]/AMOUNT_OF_SAMPLES;
	accelAverage[1] = -accelPartialSum[1]/AMOUNT_OF_SAMPLES;
	accelAverage[2] = (32767.0/2.0)-(accelPartialSum[2]/AMOUNT_OF_SAMPLES) ; // Hay que restar la gravedad.
}
/*
double calibrateGravity(void){
	double partialSum = 0;
	int16_t tmp[3];
	double tmpG[3];
	for(uint16_t i=0; i < AMOUNT_OF_SAMPLES; i++){
		readAccelData(tmp);
		int2doubleAcc(tmpG, tmp);
		partialSum += tmpG[2]*9.81; // 1G -> 9.81
	}
	return partialSum/AMOUNT_OF_SAMPLES;
}
*/
void raw2trueGyro(Gyro * gyroData)
{
	gyroData->X = ((double)(gyroData->rawX + gyroAverage[0]))/32767.0 * DPS_SCALE[Gscale];
	gyroData->Y = ((double)(gyroData->rawY + gyroAverage[1]))/32767.0 * DPS_SCALE[Gscale];
	gyroData->Z = ((double)(gyroData->rawZ + gyroAverage[2]))/32767.0 * DPS_SCALE[Gscale];
}

void raw2trueAcc(Acc * accData)
{
	accData->X = ((double)(accData->rawX + accelAverage[0]))/32767.0 * G_SCALE[Ascale];
	accData->Y = ((double)(accData->rawY + accelAverage[1]))/32767.0 * G_SCALE[Ascale];
	accData->Z = ((double)(accData->rawZ + accelAverage[2]))/32767.0 * G_SCALE[Ascale];
}



