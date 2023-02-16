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
//======================================================================
//========================== Madwick init ==============================
//======================================================================
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
//======================================================================
//======================================================================
//======================================================================

	timerInit();
	tim_id_t TS_timer;
	TS_timer = timerGetId();

	I2Ctrigger = timerGetId();

	tim_id_t timerUart;
	timerUart = timerGetId();


	timerDelay(TIMER_MS2TICKS(100));
	initMPU6050();
	timerDelay(TIMER_MS2TICKS(100));
	calibrateMPU6050();

	//======================================================================
	//======================================================================
	//======================================================================
	mpu6050_readGyroData(&gyroData);
	mpu6050_readGyroData(&accelData);

	timerStart(TS_timer, TIMER_US2TICKS(SAMPLE_PERIOD*10e6), TIM_MODE_SINGLESHOT, startI2CreadingCallBack);
	timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);

	while(1){
		gpioWrite(PORTNUM2PIN(PD, 0), HIGH);
		Gyro sampleGyro = gyroData;// Esto es peligroso porque asume que el ultimo timer que se lanzo aun el I2C no le transmitio
		Acc sampleAcc = accelData;// Esto es peligroso porque asume que el ultimo timer que se lanzo aun el I2C no le transmitio

		// ================== Observador de Estados ========================

        const FusionVector gyroscope = {.axis.x =  gyroData.X, .axis.y = gyroData.Y, .axis.z = gyroData.Z}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {.axis.x = accelData.X, .axis.y = accelData.Y, .axis.z = accelData.Z}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        const EulerAngles eulerAngles = {.eulerAngles.pitch = euler.angle.pitch, .eulerAngles.roll = euler.angle.roll, .eulerAngles.yaw = euler.angle.yaw};

        const EulerAnglesRates eulerRates;

        getEulerAnglesRates(&sampleGyro, &eulerAngles, &eulerRates);

        // ==================================================================

		runControlStep();

		gpioWrite(PORTNUM2PIN(PD, 0), LOW);
		while(!timerExpired(TS_timer));
		timerStart(TS_timer, TIMER_US2TICKS(SAMPLE_PERIOD*10e6), TIM_MODE_SINGLESHOT, NULL);
	}
}
const double referenceIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR] = {0, 0};
const double referenceProportional[ROWS_PROPORTIONAL_ERROR_VECTOR] = {0, 0, 0, 0, 0, 0};

void runControlStep(EulerAngles *Angles, EulerAnglesRates *AnglesRates, double U_PWM[4]){
	double statesIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR] = {Angles->roll, Angles->pitch};
	double outputIntError[ROWS_INTEGRATOR_ERROR_VECTOR];
	integrateError(statesIntegrator, referenceIntegrator,
				   SAMPLE_PERIOD, outputInt);

	double outputPropError[ROWS_PROPORTIONAL_ERROR_VECTOR];
	double statesProportional[ROWS_PROPORTIONAL_ERROR_VECTOR] = {Angles->roll, AnglesRates->roll_dot, Angles->pitch, AnglesRates->pitch_dot, Angles->yaw, AnglesRates->yaw_dot};
	proportionalError(statesProportional, referenceProportional,
					  outputPropError);

	double KxU[KX_ROWS];
	double KiU[KI_ROWS];
	denormalized_Kx_U_Values(outputPropError, double Kx[KX_ROWS][KX_COLUMNS], KxU);
	denormalized_Ki_U_Values(outputInt, double Ki[KI_ROWS][KI_COLUMNS], KiU);
	double U[4];
	denormalized_U_total(KxU, KiU, U);
	U2PWM(U, U_PWM);
}


/*
rotacion alrededor del x del sensor (de y a z) -> GyroRates_deg[0] == p
rotacion alrededor del y del sensor (de x a z) -> GyroRates_deg[1] == q
rotacion alrededor del z del sensor (de y a x) -> GyroRates_deg[2] == r

newAngles[0] = roll ~ phi (de y a z) -> p
newAngles[1] = pitch ~ theta (de x a z) -> q
newAngles[2] = yaw ~ psi (de x a y) -> r
*/
static void getEulerAnglesRates(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates){
	rates->roll_dot = GyroRates->X +
					  GyroRates->Y*sin(newAngles->roll*DEG2RAD)*tan(newAngles->pitch*DEG2RAD) +
					  GyroRates->Z*cos(newAngles->roll*DEG2RAD)*tan(newAngles->pitch*DEG2RAD);

	rates->pitch_dot = GyroRates->Y*cos(newAngles->roll*DEG2RAD) -
					   GyroRates->Z*sin(newAngles->roll*DEG2RAD);

	//psidot=(qsin(0)-rcos(0))*sec(1)
	rates->yaw_dot = (GyroRates->Y*sin(newAngles->roll*DEG2RAD) +
					  GyroRates->Z*cos(newAngles->roll*DEG2RAD))/cos(newAngles->pitch*DEG2RAD);
}


static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, double Ts){

	static EulerAngles lastAngle = {.roll = 0, .pitch = 0, .yaw = 0};
	double phi_dot = GyroRates_rad->X +
					 GyroRates_rad->Y*sin(lastAngle.roll*DEG2RAD)*tan(lastAngle.pitch*DEG2RAD) +
					 GyroRates_rad->Z*cos(lastAngle.roll*DEG2RAD)*tan(lastAngle.pitch*DEG2RAD);

	double theta_dot = GyroRates_rad->Y*cos(lastAngle.roll*DEG2RAD) -
					   GyroRates_rad->Z*sin(lastAngle.roll*DEG2RAD);

	//psidot=(qsin(0)-rcos(0))*sec(1)
	double psi_dot = (GyroRates_rad->Y*sin(lastAngle.roll*DEG2RAD) +
					  GyroRates_rad->Z*cos(lastAngle.roll*DEG2RAD))/cos(lastAngle.pitch*DEG2RAD);

	/*double phi_dot = GyroRates_rad->X;
	double theta_dot = GyroRates_rad->Y;
	double psi_dot = GyroRates_rad->Z;*/

	newAngles->roll = lastAngle.roll + Ts * phi_dot;
	newAngles->pitch = lastAngle.pitch + Ts * theta_dot;
	newAngles->yaw = lastAngle.yaw + Ts * psi_dot;

	lastAngle.roll = newAngles->roll;
	lastAngle.pitch = newAngles->pitch;
	lastAngle.yaw = newAngles->yaw;
}



