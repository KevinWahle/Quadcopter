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
#include "UART/UART.h"
#include "SBUS/SBUS.h"
#include <arm_math.h>

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

#define DEG2RAD (PI/180)
#define RAD2DEG (180/PI)
#define SAMPLE_PERIOD (0.000570f) // replace this with actual sample period

#define LED_1 PORTNUM2PIN(PB, 2)
#define LED_2 PORTNUM2PIN(PB, 3)
#define LED_3 PORTNUM2PIN(PB, 10)
#define LED_4 PORTNUM2PIN(PB, 11)
#define LED_5 PORTNUM2PIN(PC, 11)
#define LED_6 PORTNUM2PIN(PC, 10)
#define LOOP_TIME_PIN PORTNUM2PIN(PB, 20)


static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, double Ts);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
static void getEulerAnglesRates(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
static void runControlStep(EulerAngles *Angles, EulerAnglesRates *AnglesRates, double U_PWM[4]);
static double getU1fromSBUS();
static void getEulerAnglesRatesFAST(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates);
Acc accelData;
Gyro gyroData;
EulerAngles AnglesIntegrated;
tim_id_t I2CtriggerAcc;
SBUSData_t sbus;

static char strChannels[100];
static void sendUartMessage3Channels(double* msg1)
{
	uint16_t charCount= sprintf(strChannels, "%.1f, %.1f, %.1f \r\n",
						msg1[0], msg1[1], msg1[2]);
	uartWriteMsg(UART_ID, strChannels, charCount);
}


void App_Init (void)
{

	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
	gpioMode (PORTNUM2PIN(PD, 0), OUTPUT);
	gpioWrite(PORTNUM2PIN(PD, 0), LOW);

	gpioMode(LED_1, OUTPUT);
	gpioWrite(LED_1, LOW);
	gpioMode(LED_2, OUTPUT);
	gpioWrite(LED_2, LOW);
	gpioMode(LED_3, OUTPUT);
	gpioWrite(LED_3, LOW);
	gpioMode(LED_4, OUTPUT);
	gpioWrite(LED_4, LOW);
	gpioMode(LED_5, OUTPUT);
	gpioWrite(LED_5, LOW);
	gpioMode(LED_6, OUTPUT);
	gpioWrite(LED_6, LOW);
	gpioMode(LOOP_TIME_PIN, OUTPUT);
	gpioWrite(LOOP_TIME_PIN, LOW);

	SBUSInit(&sbus);
	ESCInit();
	gpioMode(PIN_SW2, SW2_INPUT_TYPE);


}
static void startI2CreadingCallBack(){
	mpu6050_readGyroData_async();
	timerStart(I2CtriggerAcc, TIMER_MS2TICKS(0.020), TIM_MODE_PERIODIC, startI2C_ACCELEROMETER_CallBack);
}
static void startI2C_ACCELEROMETER_CallBack(){
	if(!isI2CBusy(I2C_ACC)){
		mpu6050_readAccelData_async();
		timerStop(I2CtriggerAcc);
	}
}
void App_Run (void)
{
	gpioWrite(LED_1, HIGH); // Calibrations Start
	ESCCalibrate();
	gpioWrite(LED_1, LOW); // Calibrations End
	timerInit();
	for(uint8_t i = 0; i < 6; i++){
		gpioToggle(LED_1);
		gpioToggle(LED_2);
		gpioToggle(LED_3);
		gpioToggle(LED_4);
		gpioToggle(LED_5);
		gpioToggle(LED_6);
		timerDelay(TIMER_MS2TICKS(750));
	}


//======================================================================
//========================== Madwick init ==============================
//======================================================================
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
//======================================================================
//======================================================================
//======================================================================


	tim_id_t TS_timer;
	TS_timer = timerGetId();

	I2CtriggerAcc = timerGetId();

	tim_id_t timerUart;
	timerUart = timerGetId();


	timerDelay(TIMER_MS2TICKS(100));
	initMPU6050();
	timerDelay(TIMER_MS2TICKS(100));
	calibrateMPU6050();
	//======================================================================
	// ========================= SBUS init =================================
	//======================================================================


	// =====================================================================
	// ========================= ESC =======================================
	// =====================================================================
	//======================================================================
	//======================================================================
	//======================================================================
	gpioWrite(LED_1, LOW);   // Quad Starts
	while(gpioRead(PIN_SW2));	// Espero SW
	while(!gpioRead(PIN_SW2));


	ESCArm();
	gpioWrite(LED_1, HIGH); // ARM Start
	timerDelay(TIMER_MS2TICKS(5000));
	gpioWrite(LED_1, LOW); // ARM stops


	mpu6050_readGyroData(&gyroData);
	mpu6050_readAccelData(&accelData);

	timerStart(TS_timer, TIMER_MS2TICKS(1), TIM_MODE_SINGLESHOT, startI2CreadingCallBack);
	timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);

	startI2CreadingCallBack();
	double speed[4] = {0.0, 0.0, 0.0, 0.0};
	while(gpioRead(PIN_SW2)){
		gpioWrite(LOOP_TIME_PIN, HIGH);
		mpu6050_getLastGyroRead(&gyroData);
		mpu6050_getLastAccelRead(&accelData);
		Gyro sampleGyro = gyroData;// Esto es peligroso porque asume que el ultimo timer que se lanzo aun el I2C no le transmitio
		Acc sampleAcc = accelData;// Esto es peligroso porque asume que el ultimo timer que se lanzo aun el I2C no le transmitio

		// ================== Observador de Estados ========================

        const FusionVector gyroscope = {.axis.x =  sampleGyro.X, .axis.y = sampleGyro.Y, .axis.z = sampleGyro.Z}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {.axis.x = sampleAcc.X, .axis.y = sampleAcc.Y, .axis.z = sampleAcc.Z}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1e-3);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        EulerAngles eulerAngles = {.pitch = euler.angle.pitch, .roll = euler.angle.roll, .yaw = euler.angle.yaw};
        EulerAnglesRates eulerRates;
        getEulerAnglesRatesFAST(&sampleGyro, &eulerAngles, &eulerRates);

        // ==================================================================
		//runControlStep(&eulerAngles, &eulerRates, speed);
		//ESCSetSpeed(speed);

		if(timerExpired(timerUart)){
			double tmp[3] = {eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw};
			sendUartMessage3Channels(tmp);
			timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);
		}

		gpioWrite(LOOP_TIME_PIN, LOW);
		while(!timerExpired(TS_timer));
		timerStart(TS_timer, TIMER_MS2TICKS(1), TIM_MODE_SINGLESHOT, startI2CreadingCallBack);
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
	double statesIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR][1] = {{Angles->roll * DEG2RAD}, {Angles->pitch * DEG2RAD}};
	double outputIntError[ROWS_INTEGRATOR_ERROR_VECTOR][1];
	integrateError(statesIntegrator, referenceIntegrator,
				   1e-3, outputIntError);

	double outputPropError[ROWS_PROPORTIONAL_ERROR_VECTOR][1];
	double statesProportional[ROWS_PROPORTIONAL_ERROR_VECTOR][1] = {{Angles->roll * DEG2RAD}, {AnglesRates->roll_dot * DEG2RAD}, {Angles->pitch * DEG2RAD}, {AnglesRates->pitch_dot * DEG2RAD}, {Angles->yaw * DEG2RAD}, {AnglesRates->yaw_dot * DEG2RAD}};
	proportionalError(statesProportional, referenceProportional,
					  outputPropError);

	double KxU[KX_ROWS][1];
	double KiU[KI_ROWS][1];
	denormalized_Kx_U_Values(Kx, outputPropError, KxU);
	denormalized_Ki_U_Values(Ki, outputIntError, KiU);
	double U[4][1];
	denormalized_U_total(KxU, KiU, U);
	U[0][0] = getU1fromSBUS();
	U2PWM(U, U_PWM);
}

static double getU1fromSBUS(){
	double linealFactor = 10;
	if(sbus.channels[2] < 240.0)
		return 0.0;
	if(sbus.channels[2] > 1800.0)
		return 1.0 * linealFactor;
	return (sbus.channels[2]-240.0)/(1800.0-240.0) * linealFactor;
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

/*	double phi_dot = GyroRates_rad->X +
					 GyroRates_rad->Y*arm_sin_f32((float32_t)lastAngle.roll*DEG2RAD)*tan(lastAngle.pitch*DEG2RAD) +
					 GyroRates_rad->Z*cos(lastAngle.roll*DEG2RAD)*tan(lastAngle.pitch*DEG2RAD);
*/

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

static void getEulerAnglesRatesFAST(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates){

	rates->roll_dot = GyroRates->X +
					  GyroRates->Y*arm_sin_f32(newAngles->roll*DEG2RAD)*(arm_sin_f32(newAngles->pitch*DEG2RAD)/arm_cos_f32(newAngles->pitch*DEG2RAD)) +
					  GyroRates->Z*arm_cos_f32(newAngles->roll*DEG2RAD)*(arm_sin_f32(newAngles->pitch*DEG2RAD)/arm_cos_f32(newAngles->pitch*DEG2RAD));

	rates->pitch_dot = GyroRates->Y*arm_cos_f32(newAngles->roll*DEG2RAD) -
					   GyroRates->Z*arm_sin_f32(newAngles->roll*DEG2RAD);

	//psidot=(qsin(0)-rcos(0))*sec(1)
	rates->yaw_dot = (GyroRates->Y*arm_sin_f32(newAngles->roll*DEG2RAD) +
					  GyroRates->Z*arm_cos_f32(newAngles->roll*DEG2RAD))/arm_cos_f32(newAngles->pitch*DEG2RAD);
}

