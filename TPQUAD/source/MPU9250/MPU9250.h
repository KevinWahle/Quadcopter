#ifndef _MPU9250_
#define _MPU9250_
#include <stdint.h>
void initMPU9250();

void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void whoAmI(uint8_t * read);
void resetMPU9250(void);
void int2doubleGyro(double* destination, int16_t* rawData);
void int2doubleAcc(double* destination, int16_t* rawData);
void calibrateMPU9250();
double calibrateGravity(void);

// SPI



#endif

