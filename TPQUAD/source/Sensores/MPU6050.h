#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "MPU_REGISTERS.h"
#include <stdint.h>

void initMPU6050();

void mpu6050_readGyroData(Gyro * destination);
void mpu6050_readAccelData(Acc * destination);

void mpu6050_readGyroData_async();
void mpu6050_readAccelData_async();

void mpu6050_getLastGyroRead(Gyro * destination);
void mpu6050_getLastAccelRead(Acc * destination);

void mpu6050_whoAmI(uint8_t * read);


void calibrateMPU6050();  // wait at least 50ms for executing this f

#endif // _MPU6050_H_

