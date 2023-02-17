#include "timer/timer.h"
#include "UART/uart.h"
#include "MCAL/gpio.h"
#include <stdio.h>
#include "Fusion/Fusion.h"
#include <math.h>
#include "I2Cm/I2Cm.h"
#include <ESCDriver/ESCDriver.h>
#include "MCAL/board.h"
#include "Sensores/MPU6050.h"
#include "Sistema de Control/SistemaDeControl.h"
#include "ESCDriver/ESCDriver.h"

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

#define LED_1 PORTNUM2PIN(PB, 2)


static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, double Ts);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
static void getEulerAnglesRates(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
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

	gpioMode(LED_1, OUTPUT);
	gpioWrite(LED_1, LOW);

	ESCInit();
	gpioMode(PIN_SW2, SW2_INPUT_TYPE);

}
static void startI2CreadingCallBack(){
	mpu6050_readGyroData_async();
	timerStart(I2CtriggerAcc, TIMER_US2TICKS(20), TIM_MODE_PERIODIC, startI2C_ACCELEROMETER_CallBack);
}
static void startI2C_ACCELEROMETER_CallBack(){
	if(!isI2CBusy(I2C_ACC)){
		mpu6050_readGyroData_async();
		timerStop(I2CtriggerAcc);
	}
}

void App_Run (void)
{
	gpioWrite(LED_1, HIGH); // Calibrations Start
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

	I2CtriggerAcc = timerGetId();

	tim_id_t timerUart;
	timerUart = timerGetId();


	timerDelay(TIMER_MS2TICKS(100));
	initMPU6050();
	timerDelay(TIMER_MS2TICKS(100));
	calibrateMPU6050();

	// =====================================================================
	// ========================= ESC =======================================
	// =====================================================================
	ESCCalibrate();
	//======================================================================
	//======================================================================
	//======================================================================
	gpioWrite(LED_1, LOW);   // Quad Starts
	while(gpioRead(PIN_SW2));	// Espero SW
	while(!gpioRead(PIN_SW2));

	mpu6050_readGyroData(&gyroData);
	mpu6050_readAccelData(&accelData);

	timerStart(TS_timer, TIMER_US2TICKS(SAMPLE_PERIOD*10e6), TIM_MODE_SINGLESHOT, startI2CreadingCallBack);
	timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);


	ESCArm();
	double speed[MOTOR_COUNT] = INIT_MOTOR_VALUES;

	while(gpioRead(PIN_SW2)){
		gpioWrite(PORTNUM2PIN(PD, 0), HIGH);
		mpu6050_getLastGyroRead(&gyroData);
		mpu6050_getLastAccelRead(&accelData);
		Gyro sampleGyro = gyroData;// Esto es peligroso porque asume que el ultimo timer que se lanzo aun el I2C no le transmitio
		Acc sampleAcc = accelData;// Esto es peligroso porque asume que el ultimo timer que se lanzo aun el I2C no le transmitio

		// ================== Observador de Estados ========================

        const FusionVector gyroscope = {.axis.x =  sampleGyro.X, .axis.y = sampleGyro.Y, .axis.z = sampleGyro.Z}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {.axis.x = sampleAcc.X, .axis.y = sampleAcc.Y, .axis.z = sampleAcc.Z}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        EulerAngles eulerAngles = {.pitch = euler.angle.pitch, .roll = euler.angle.roll, .yaw = euler.angle.yaw};
        EulerAnglesRates eulerRates;
        getEulerAnglesRates(&sampleGyro, &eulerAngles, &eulerRates);

        // ==================================================================

		runControlStep(&eulerAngles, &eulerRates, speed);
		ESCSetSpeed(speed);

		gpioWrite(PORTNUM2PIN(PD, 0), LOW);
		while(!timerExpired(TS_timer));
		timerStart(TS_timer, TIMER_US2TICKS(SAMPLE_PERIOD*10e6), TIM_MODE_SINGLESHOT, NULL);
	}
	ESCDisarm();
	while(1){                                // forever blink
		gpioToggle(LED_1);
		timerDelay(TIMER_MS2TICKS(250));
	}
}
double referenceIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR][1] = {{0}, {0}};
double referenceProportional[ROWS_PROPORTIONAL_ERROR_VECTOR][1] = {{0}, {0}, {0}, {0}, {0}, {0}};


double Kx[KX_ROWS][KX_COLUMNS] = {
		{0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000},
		{3.3181418, 0.5574807, -0.0000000, -0.0000000, 0.0000000, 0.0000000},
		{-0.0000000, -0.0000000, 3.3181418, 0.5574807, -0.0000000, -0.0000000},
		{0.0000000, 0.0000000, -0.0000000, -0.0000000, 0.9503350, 0.9558509}
};

double Ki[KI_ROWS][KI_COLUMNS] = {
		{-0.0000000, 0.0000000},
		{0.9950204, -0.0000000},
		{0.0000000, 0.9950204},
		{0.0000000, -0.0000000}
};


void runControlStep(EulerAngles *Angles, EulerAnglesRates *AnglesRates, double U_PWM[4]){
	double statesIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR][1] = {{Angles->roll}, {Angles->pitch}};
	double outputIntError[ROWS_INTEGRATOR_ERROR_VECTOR][1];
	integrateError(statesIntegrator, referenceIntegrator,
				   SAMPLE_PERIOD, outputIntError);

	double outputPropError[ROWS_PROPORTIONAL_ERROR_VECTOR][1];
	double statesProportional[ROWS_PROPORTIONAL_ERROR_VECTOR][1] = {{Angles->roll}, {AnglesRates->roll_dot}, {Angles->pitch}, {AnglesRates->pitch_dot}, {Angles->yaw}, {AnglesRates->yaw_dot}};
	proportionalError(statesProportional, referenceProportional,
					  outputPropError);

	double KxU[KX_ROWS][1];
	double KiU[KI_ROWS][1];
	denormalized_Kx_U_Values(Kx, outputPropError, KxU);
	denormalized_Ki_U_Values(Ki, outputIntError, KiU);
	double U[4][1];
	denormalized_U_total(KxU, KiU, U);
	U2PWM(U, U_PWM);
	U_PWM[0] = U[0][0];
	U_PWM[1] = U[1][0];
	U_PWM[2] = U[2][0];
	U_PWM[3] = U[3][0];
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



