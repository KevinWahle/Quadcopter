#include <stdio.h>
#include "MatrixOps.h"

#include "timer/timer.h"
#include "MPU6050.h"
#include "UART/uart.h"
#include "MCAL/gpio.h"
#include <stdio.h>
#include "Fusion/Fusion.h"
#include <math.h>
#include "I2Cm/I2Cm.h"

typedef struct{
	double roll;
	double pitch;
	double yaw;
}EulerAngles;

typedef struct{
	double roll_dot;
	double pitch_dot;
	double yaw_dot;
}EulerAnglesRates;


#define UART_ID			0
#define UART_BAUDRATE	115200

#define PI (3.141592)
#define DEG2RAD (PI/180)
#define RAD2DEG (180/PI)
#define SAMPLE_PERIOD (0.000570f) // replace this with actual sample period

static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, double Ts);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
static void getEulerAnglesRates(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates);
void runControlStep(EulerAngles *Angles, EulerAnglesRates *AnglesRates, double U_PWM[4]);

Acc accelData;
Gyro gyroData;
EulerAngles AnglesIntegrated;
tim_id_t I2CtriggerAcc;

void App_Init (void)
{
	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
	gpioMode (PORTNUM2PIN(PD, 0), OUTPUT);
	gpioWrite(PORTNUM2PIN(PD, 0), LOW);
}
static void startI2CreadingCallBack(){
	mpu6050_readGyroData_async(&gyroData);
	timerStart(I2Ctrigger, TIMER_US2TICKS(20), TIM_MODE_PERIODIC, startI2C_ACCELEROMETER_CallBack);
}
static void startI2C_ACCELEROMETER_CallBack(){
	if(!isI2CBusy(I2C_ACC)){
		mpu6050_readGyroData_async(&accelData);
		timerStop(I2CtriggerAcc);
	}
}

void App_Run (void)
{


}


