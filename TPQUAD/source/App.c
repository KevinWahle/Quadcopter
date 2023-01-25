/***************************************************************************//**
  @file     App.c
  @brief    TPF: Reproductor MP3
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "I2Cm/I2Cm.h"
#include "UART/uart.h"
#include "MPU9250/MPU9250.h"
#include <stdio.h>
#include "timer/timer.h"
#include <math.h>
#include "MCAL/gpio.h"

#define UART_ID			0
#define UART_BAUDRATE	115200

#define PI			3.14159265F
#define RAD2DEG		(180/PI)
#define DEG2RAD		(PI/180)
#define GRAVITY		9.81F

/*******************************************************************************
 *******************************************************************************
                        	GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/
static void sendUartMessage3Channels(double* msg1);
static void sendUartMessage6Channels(double* msg1, double* msg2);
/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
static void getAnglesAcc(double* Acc, double* Angles);
static void getAnglesAccV2(double* Acc, double* Angles);
static void getAnglesGyro(double* GyroRates_rad, double* lastAngles_rad, double* newAngles_rad, double Ts);
static void getZposition(double * Angles, double* body_ACC_coordinates, double* lastZ, double* lastVZ, double Ts);
/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	gpioMode (PORTNUM2PIN(PB,2), OUTPUT);
	gpioWrite(PORTNUM2PIN(PB,2), LOW);
	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
}


int16_t accelData[3];
int16_t gyroData[3];

double newAngles_rad[3];
double lastAngles_rad[3];

double lastVZ[3];
double lastZ[3];

double gravityMean;
/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	double Acc[3] = {0, 0, 0};
	double Gyro[3] = {0, 0, 0}; 
	I2CmInit(I2C_ACC);
	uint8_t AVER;
	whoAmI(&AVER);

	resetMPU9250();
	initMPU9250();
	calibrateMPU9250();
    gravityMean = calibrateGravity();
	// initial conditions 

	lastAngles_rad[0] = 0;
	lastAngles_rad[1] = 0;
	lastAngles_rad[2] = 0;

	const double Ts = 5e-3;

	tim_id_t TS_timer;
	TS_timer = timerGetId();

	timerStart(TS_timer, TIMER_MS2TICKS(5), TIM_MODE_SINGLESHOT, NULL);
	while(1){
 	
		double Acc_G[3];
		readAccelData(accelData);
		int2doubleAcc(Acc, accelData);  // Acc -> [G]
		Acc_G[0] = Acc[0];
		Acc_G[1] = Acc[1];
		Acc_G[2] = Acc[2];
		Acc[0] = Acc[0]*gravityMean;
		Acc[1] = Acc[1]*gravityMean;
		Acc[2] = Acc[2]*gravityMean;  // Acc -> [m/s^2]
		
		readGyroData(gyroData); // raw data
		int2doubleGyro(Gyro, gyroData);  // Gyro [deg/s]
		
		Gyro[0] = Gyro[0] * DEG2RAD;
		Gyro[1] = Gyro[1] * DEG2RAD;
		Gyro[2] = Gyro[2] * DEG2RAD; //  [rad/s]
		
		getAnglesGyro(Gyro, lastAngles_rad, newAngles_rad, Ts);

		lastAngles_rad[0] = newAngles_rad[0];
		lastAngles_rad[1] = newAngles_rad[1];
		lastAngles_rad[2] = newAngles_rad[2]; 

		//getZposition(lastAngles_rad, Acc, lastZ, lastVZ, Ts);

		newAngles_rad[0] = newAngles_rad[0] * RAD2DEG;
		newAngles_rad[1] = newAngles_rad[1] * RAD2DEG;
		newAngles_rad[2] = newAngles_rad[2] * RAD2DEG;
		
		gpioWrite(PORTNUM2PIN(PB,2), LOW);
		while(!timerExpired(TS_timer));
		gpioWrite(PORTNUM2PIN(PB,2), HIGH);
		timerStart(TS_timer, TIMER_MS2TICKS(5), TIM_MODE_SINGLESHOT, NULL);
		sendUartMessage3Channels(Acc);  // esta en m/s^2
		//timerDelay(TIMER_US2TICKS(5000));
	}
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/



static void getAnglesAcc(double* Acc, double* Angles)
{
	// ANGLES: roll = phi , pitch = theta, yaw = psi
	Angles[0] = atan2(Acc[1], Acc[2]) * RAD2DEG;
	Angles[1] = atan2(Acc[0], sqrt(Acc[1]*Acc[1] + Acc[2]*Acc[2])) * RAD2DEG;
	Angles[2] = 0;		// no se puede medir con acelerometro
}

static void getAnglesAccV2(double* Acc, double* Angles){
	Angles[0] = atan2(Acc[1], Acc[2]) * RAD2DEG;
	Angles[1] = asin(Acc[0]/ sqrt(pow(Acc[0], 2)+pow(Acc[1], 2)+pow(Acc[2], 2)) ) * RAD2DEG;
	Angles[2] = 0;
}

static void getZposition(double * Angles, double* body_ACC_coordinates, double* lastZ, double* lastVZ, double Ts)
{
	double x_earth_acc = body_ACC_coordinates[0]*cos(Angles[2])*cos(Angles[1]) + 
						 body_ACC_coordinates[1]*(-sin(Angles[2])*cos(Angles[0]) - cos(Angles[2])*sin(Angles[1])*sin(Angles[0])) + 
						 body_ACC_coordinates[2]*(sin(Angles[2])*sin(Angles[0]) - cos(Angles[2])*sin(Angles[1])*cos(Angles[0]));

	double y_earth_acc = body_ACC_coordinates[0]*sin(Angles[2])*cos(Angles[1]) + 
						 body_ACC_coordinates[1]*(cos(Angles[2])*cos(Angles[0]) - sin(Angles[2])*sin(Angles[1])*sin(Angles[0])) + 
						 body_ACC_coordinates[2]*(-cos(Angles[2])*sin(Angles[0]) - sin(Angles[2])*sin(Angles[1])*cos(Angles[0]));

	double z_earth_acc = body_ACC_coordinates[0]*sin(Angles[1]) + 
						 body_ACC_coordinates[1]*cos(Angles[1])*sin(Angles[0]) + 
						 body_ACC_coordinates[2]*cos(Angles[1])*cos(Angles[0]);

	double aux  = (z_earth_acc - gravityMean);
	if (aux < 0.008)
	{
		aux = 0;
	}
	(*lastVZ) = (*lastVZ) + Ts*aux; // 9,815   0,005  Acc   
	(*lastZ) = (*lastZ) + Ts*(*lastVZ);
}

/* 
rotacion alrededor del x del sensor (de y a z) -> GyroRates_deg[0] == p
rotacion alrededor del y del sensor (de x a z) -> GyroRates_deg[1] == q
rotacion alrededor del z del sensor (de y a x) -> GyroRates_deg[2] == r

newAngles[0] = roll ~ phi (de y a z)
newAngles[1] = pitch ~ theta (de x a z)
newAngles[2] = yaw ~ psi (de x a y)
*/

// TODO: pasar argumentos en radianes (estaban en deg cuando se calcularon)
static void getAnglesGyro(double* GyroRates_rad, double* lastAngles_rad, double* newAngles_rad, double Ts){
	 	 
	double phi_dot = GyroRates_rad[0] - 
					 GyroRates_rad[1]*sin(lastAngles_rad[0])*tan(lastAngles_rad[1]) - 
					 GyroRates_rad[2]*cos(lastAngles_rad[0])*tan(lastAngles_rad[1]);

	double theta_dot = GyroRates_rad[1]*cos(lastAngles_rad[0]) - 
					   GyroRates_rad[2]*sin(lastAngles_rad[0]);

	//psidot=(qsin(0)-rcos(0))*sec(1)
	double psi_dot = (GyroRates_rad[1]*sin(lastAngles_rad[0]) + 
					  GyroRates_rad[2]*cos(lastAngles_rad[0]))/cos(lastAngles_rad[1]);

	newAngles_rad[0] = lastAngles_rad[0] + Ts * phi_dot;
	newAngles_rad[1] = lastAngles_rad[1] + Ts * theta_dot;
	newAngles_rad[2] = lastAngles_rad[2] + Ts * psi_dot;
}




static void sendUartMessage3Channels(double* msg1)
{
	char str[100];
	uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}

static void sendUartMessage6Channels(double* msg1, double* msg2)
{
	char str[100];
	uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2],
						msg2[0], msg2[1], msg2[2]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}

/*******************************************************************************
 ******************************************************************************/
