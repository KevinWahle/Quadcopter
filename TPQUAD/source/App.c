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
#include "ControlPC/ControlPC.h"
#include "Biquad/Biquad.h"
#include "NRF/RF.h"

typedef struct{
	float roll;
	float pitch;
	float yaw;
}EulerAngles;

typedef struct{
	float roll_dot;
	float pitch_dot;
	float yaw_dot;
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

#define LPF_ACC_ALPHA 0.7f

static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, float Ts);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
static void getEulerAnglesRates(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates);
static void startI2CreadingCallBack();
static void startI2C_ACCELEROMETER_CallBack();
static void runControlStep(EulerAngles *Angles, EulerAnglesRates *AnglesRates, float U_PWM[4], EulerAngles * setPointAnglesPtr);
static float getU1fromSBUS();
static void getEulerAnglesRatesFAST(Gyro *GyroRates, EulerAngles* newAngles, EulerAnglesRates* rates);
static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, float Ts);

void calcRollPitch(float accelX, float accelY, float accelZ, float *roll, float *pitch);
Acc accelData;
Gyro gyroData;
EulerAngles AnglesIntegrated;
tim_id_t I2CtriggerAcc;
SBUSData_t sbus;
FusionAhrsFlags fusionFlags;
bool initialising = true;
float speed[4] = {0.0, 0.0, 0.0, 0.0};
BiQuad filter_roll_dot_stgs[5];
BiQuad filter_pitch_dot_stgs[5];
BiQuad filter_yaw_dot_stgs[5];

bool flagBorrable = false;
 
EulerAnglesRates lastRates;

OrientationRF RFchannel;

static char strChannels[100];
static void sendUartMessage3Channels(float* msg1)
{
	uint16_t charCount= sprintf(strChannels, "%.1f, %.1f, %.1f \r\n",
						msg1[0], msg1[1], msg1[2]);
	uartWriteMsg(UART_ID, strChannels, charCount);
}
static void sendUartMessage6Channels(float* msg1)
{
	uint16_t charCount= sprintf(strChannels, "%.1f, %.1f, %.1f,%.1f, %.1f, %.1f \r\n",
						msg1[0], msg1[1], msg1[2], msg1[3], msg1[4], msg1[5]);
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

	gpioMode(PIN_SW2, SW2_INPUT_TYPE);
/*
float b[5][3] = {
	{0.0095958, 0.0191915, 0.0095958},
	{0.1084782, 0.1084782, 0.0000000},
	{0.0293045, 0.0586090, 0.0293045},
	{0.0179138, 0.0358276, 0.0179138},
	{0.0468088, 0.0936176, 0.0468088}
};*/
/*
float a[5][3] = {
	{1.0000000, -1.7792211, 0.8851647},
	{1.0000000, -0.9213973, 0.0000000},
	{1.0000000, -1.7306458, 0.9736852},
	{1.0000000, -1.8238846, 0.8584440},
	{1.0000000, -1.7378414, 0.9250766}
};
*/

float b[5][3] = {
		{0.0087849, 0.0175698, 0.0087849},
		{0.0071376, 0.0142751, 0.0071376},
		{0.0045431, 0.0090861, 0.0045431},
		{0.0019735, 0.0039470, 0.0019735},
		{0.0003915, 0.0007830, 0.0003915}
};
float a[5][3] = {
		{1.0000000, -1.9564952, 0.9916347},
		{1.0000000, -1.9473261, 0.9758764},
		{1.0000000, -1.9444096, 0.9625818},
		{1.0000000, -1.9450646, 0.9529586},
		{1.0000000, -1.9463388, 0.9479047}
};

	for (uint8_t i = 0; i < 5; i++)
	{
		BiQuad_init(&filter_pitch_dot_stgs[i], b[i], a[i]);
		BiQuad_init(&filter_roll_dot_stgs[i], b[i], a[i]);
		BiQuad_init(&filter_yaw_dot_stgs[i], b[i], a[i]);
	}
	

	//controlPCInit(UART_ID, UART_BAUDRATE,'W','S');


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
	timerInit();
	tim_id_t TS_timer;
	TS_timer = timerGetId();

	I2CtriggerAcc = timerGetId();

	tim_id_t timerUart;
	timerUart = timerGetId();

	tim_id_t timerStationary;
	timerStationary = timerGetId();

	for(uint8_t i = 0; i < 6; i++){
		gpioToggle(LED_1);
		gpioToggle(LED_2);
		gpioToggle(LED_3);
		gpioToggle(LED_4);
		gpioToggle(LED_5);
		gpioToggle(LED_6);
		timerDelay(TIMER_MS2TICKS(500));
	}

	gpioWrite(LED_1, HIGH); // Calibrations Start
	ESCInit();
	//ESCCalibrate();
//========================== RF init ===================================
	/*RFinit();
	RFbegin();
	gpioWrite(LED_3, HIGH); // Calibration RF starts
	timerDelay(TIMER_MS2TICKS(10));
	RFcalibrate();
	gpioWrite(LED_3, LOW); // Calibration RF ends*/
//======================================================================
//========================== Madwick init ==============================
//======================================================================
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
//======================================================================
//======================================================================
//======================================================================

	timerDelay(TIMER_MS2TICKS(100));
	initMPU6050();
	timerDelay(TIMER_MS2TICKS(100));
	calibrateMPU6050(); // takes a while 
	//======================================================================
	// ========================= SBUS init =================================
	//======================================================================


	// =====================================================================
	// ========================= ESC =======================================
	// =====================================================================

	gpioWrite(LED_1, LOW);   // Quad Starts
	while(gpioRead(PIN_SW2));	// Espero SW
	while(!gpioRead(PIN_SW2));
	gpioWrite(LED_1, HIGH); // ARM Start


	ESCArm();
	timerDelay(TIMER_MS2TICKS(5000)); // delay que hay que poner para que se arme cheto


	mpu6050_readGyroData(&gyroData);  // blocking
	mpu6050_readAccelData(&accelData);  // blocking 

	timerStart(TS_timer, TIMER_MS2TICKS(1), TIM_MODE_SINGLESHOT, startI2CreadingCallBack);
	timerStart(timerUart, TIMER_MS2TICKS(15), TIM_MODE_SINGLESHOT, NULL);
	timerStart(timerStationary, TIMER_MS2TICKS(10000), TIM_MODE_SINGLESHOT, NULL);


	startI2CreadingCallBack();  // lanzo la primer leida de Gyro y Accel
	
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
		/*float rollAcc, pitchAcc;
		float rollAccSINFIL;
		calcRollPitch(sampleAcc.X, sampleAcc.Y, sampleAcc.Z, &rollAccSINFIL, &pitchAcc);

		for (uint8_t i = 0; i < 5; i++)
		{
			rollAcc = BiQuad_filter(&filter_roll_dot_stgs[i], rollAcc);
		}
		
		calcRollPitch(sampleAcc.X, sampleAcc.Y, sampleAcc.Z, &rollAcc, &pitchAcc);
		*/
		for (uint8_t i = 0; i < 5; i++)
		{
			eulerRates.roll_dot = BiQuad_filter(&filter_roll_dot_stgs[i], eulerRates.roll_dot);
			eulerRates.pitch_dot = BiQuad_filter(&filter_pitch_dot_stgs[i], eulerRates.pitch_dot);
			eulerRates.yaw_dot = BiQuad_filter(&filter_yaw_dot_stgs[i], eulerRates.yaw_dot);
		}
		
        // ==================================================================
		if(initialising == false){     // si ya inicializo, corro el sistema de control
			//RFgetDeNormalizedData(&RFchannel);
			//RF2Newton(&RFchannel);
			//EulerAngles anglesSetPoint = {.pitch = RFchannel.pitch, .roll = RFchannel.roll, .yaw = RFchannel.yaw};
			EulerAngles anglesSetPoint = {0.0, 0.0, 0.0};
			runControlStep(&eulerAngles, &eulerRates, speed, &anglesSetPoint);
			speed[0] = 0.2;
			speed[1] = 0.2;
			speed[2] = 0.2;
			speed[3] = 0.2;
			ESCSetSpeed(speed);

			//getAnglesGyro(&sampleGyro, &AnglesIntegrated, 1e-3);

			if(timerExpired(timerUart)){
				//float pitch = atan2(-sampleAcc.X, sqrt(pow(sampleAcc.Y , 2)+ pow(sampleAcc.Z, 2)))*RAD2DEG;
				//float roll = atan2(sampleAcc.Y, sampleAcc.Z)*RAD2DEG;
				float tmp[3] = {eulerAngles.roll, eulerAngles.pitch, eulerAngles.yaw}; //, pitch, roll, 180};
				//float tmp[3] = {RFchannel.throttle, RFchannel.pitch, RFchannel.roll};
				//float tmp[6] = {eulerAngles.roll, rollAcc, rollAccSINFIL};
				sendUartMessage3Channels(tmp);
				timerStart(timerUart, TIMER_MS2TICKS(15), TIM_MODE_SINGLESHOT, NULL);
			}
		}
		else{
			fusionFlags = FusionAhrsGetFlags(&ahrs);
			if(fusionFlags.initialising == false){
				initialising = false;
				gpioWrite(LED_1, LOW); // Indica que ahora arrancan las propelas
			}
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
float referenceIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR][1] = {{0}, {0}};
float referenceProportional[ROWS_PROPORTIONAL_ERROR_VECTOR][1] = {{0}, {0}, {0}, {0}, {0}, {0}};


float Kx[KX_ROWS][KX_COLUMNS] = {
	{0.0000000, -0.0000000, -0.0000000, -0.0000000, 0.0000000, 0.0000000},
	{15.9074356, 1.0123648, 0.0000000, 0.0000000, 0.0000000, 0.0000000},
	{-0.0000000, 0.0000000, 15.9074356, 1.0123648, -0.0000000, -0.0000000},
	{0.0000000, 0.0000000, -0.0000000, -0.0000000, 9.1151589, 0.9652512}
};




float Ki[KI_ROWS][KI_COLUMNS] = {
	{0.0000000, 0.0000000},
	{5.3903525, -0.0000000},
	{-0.0000000, 5.3903525},
	{-0.0000000, 0.0000000}
};

void runControlStep(EulerAngles *Angles, EulerAnglesRates *AnglesRates, float U_PWM[4], EulerAngles * setPointAnglesPtr){
	float statesIntegrator[ROWS_INTEGRATOR_ERROR_VECTOR][1] = {{Angles->roll * DEG2RAD}, {Angles->pitch * DEG2RAD}};
	float outputIntError[ROWS_INTEGRATOR_ERROR_VECTOR][1];
	referenceIntegrator[0][0] = setPointAnglesPtr->roll * DEG2RAD;
	referenceIntegrator[1][0] = setPointAnglesPtr->pitch * DEG2RAD;
	bool saturation = integrateError(statesIntegrator, referenceIntegrator,
				   1e-3, outputIntError, Ki[1][0]);
	gpioWrite(LED_6, saturation);


	float outputPropError[ROWS_PROPORTIONAL_ERROR_VECTOR][1];
	float statesProportional[ROWS_PROPORTIONAL_ERROR_VECTOR][1] = {{Angles->roll * DEG2RAD}, {AnglesRates->roll_dot * DEG2RAD}, {Angles->pitch * DEG2RAD}, {AnglesRates->pitch_dot * DEG2RAD}, {Angles->yaw * DEG2RAD}, {AnglesRates->yaw_dot * DEG2RAD}};
	referenceProportional[0][0] = setPointAnglesPtr->roll * DEG2RAD;
	referenceProportional[2][0] = setPointAnglesPtr->pitch * DEG2RAD;
	proportionalError(statesProportional, referenceProportional,
					  outputPropError);

	float KxU[KX_ROWS][1];
	float KiU[KI_ROWS][1];
	denormalized_Kx_U_Values(Kx, outputPropError, KxU);
	denormalized_Ki_U_Values(Ki, outputIntError, KiU);
	float U[4][1];
	denormalized_U_total(KxU, KiU, U);
	//U[0][0] = (float)getDataFromPC();
	U[0][0] = RFchannel.throttle;
	if(U[0][0] > 8){
		gpioWrite(LED_2, HIGH);
	}
	else
		gpioWrite(LED_2, LOW);

	U2PWM(U, U_PWM);
}

static float getU1fromSBUS(){
	float linealFactor = 10;
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


static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, float Ts){

	static EulerAngles lastAngle = {.roll = 0, .pitch = 0, .yaw = 0};
	float phi_dot = GyroRates_rad->X +
					 GyroRates_rad->Y*arm_sin_f32(lastAngle.roll*DEG2RAD)*(arm_sin_f32(lastAngle.pitch*DEG2RAD)/arm_cos_f32(lastAngle.pitch*DEG2RAD)) +
					 GyroRates_rad->Z*arm_cos_f32(lastAngle.roll*DEG2RAD)*(arm_sin_f32(lastAngle.pitch*DEG2RAD)/arm_cos_f32(lastAngle.pitch*DEG2RAD));

	float theta_dot = GyroRates_rad->Y*arm_cos_f32(lastAngle.roll*DEG2RAD) -
					   GyroRates_rad->Z*arm_sin_f32(lastAngle.roll*DEG2RAD);

	//psidot=(qsin(0)-rcos(0))*sec(1)
	float psi_dot = (GyroRates_rad->Y*arm_sin_f32(lastAngle.roll*DEG2RAD) +
					  GyroRates_rad->Z*arm_cos_f32(lastAngle.roll*DEG2RAD))/arm_cos_f32(lastAngle.pitch*DEG2RAD);

	/*float phi_dot = GyroRates_rad->X;
	float theta_dot = GyroRates_rad->Y;
	float psi_dot = GyroRates_rad->Z;*/

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

// Function to calculate roll and pitch angles from accelerometer measurements
void calcRollPitch(float accelX, float accelY, float accelZ, float *roll, float *pitch) {
  // Calculate total acceleration vector magnitude
  float accelMag = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  // Calculate pitch angle
  *pitch = asin(-accelX / accelMag) * 180.0 / M_PI;

  // Calculate roll angle
  *roll = atan2(accelY, accelZ) * 180.0 / M_PI;
}
