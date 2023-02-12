#ifndef _MPU9250_
#define _MPU9250_
#include <stdint.h>

#include "MPU_REGISTERS.h"



void initMPU9250();

void readAccelData(Acc * destination);
void readGyroData(Gyro * destination);

void whoAmI(uint8_t * read);

void resetMPU9250(void);

void raw2trueGyro(Gyro * gyroData);
void raw2trueAcc(Acc * accData);


void calibrateMPU9250();  // wait at least 50ms for executing this f
//double calibrateGravity(void);

// SPI



#endif

