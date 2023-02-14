#include "I2Cm/I2Cm.h"
#include "timer/timer.h"
#include "MPU6050.h"
#include "UART/uart.h"
#include "MCAL/gpio.h"
#include <stdio.h>
#include "Fusion/Fusion.h"
#include <math.h>
#define UART_ID			0
#define UART_BAUDRATE	115200

#define PI (3.141592)
#define DEG2RAD (PI/180)
#define RAD2DEG (180/PI)
#define LPF_ACC_ALPHA 0.9f

Gyro shairo;
void App_Init (void)
{
	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
	gpioMode (PORTNUM2PIN(PD, 0), OUTPUT);
	gpioWrite(PORTNUM2PIN(PD, 0), LOW);
}
void SENSCB(){
	mpu6050_readGyroData(&shairo);
}

typedef struct{
	double roll;
	double pitch;
	double yaw;
}EulerAngles;




static void sendUartMessage6Channels(double* msg1, double* msg2)
{
	char str[100];
	uint16_t charCount= sprintf(str, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f \r\n",
						msg1[0], msg1[1], msg1[2],
						msg2[0], msg2[1], msg2[2]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}
static void getAnglesGyro(Gyro* GyroRates_rad, EulerAngles* newAngles, double Ts);
static double simpson(double f[3], double h) {
    return ((f[0] + 4*f[1] + f[2])*h/3.0);
}
double Xgyros[3];
uint8_t counter;
double AngleYSimpson;
double AngleYEuler;
char buff[10];
double fff[3];

double tmp1[3];
double tmp2[3];

Acc accelData;
Gyro gyroData;

Acc prev;

static void sendUartMessage3Channels(double* msg1)
{
	char str[100];
	uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

EulerAngles AnglesIntegrated;

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

	tim_id_t timerUart;
	timerUart = timerGetId();

	timerDelay(TIMER_MS2TICKS(100));
	initMPU6050();
	timerDelay(TIMER_MS2TICKS(100));
	calibrateMPU6050();

	//======================================================================
	//======================================================================
	//======================================================================

	timerStart(TS_timer, TIMER_MS2TICKS(10), TIM_MODE_SINGLESHOT, NULL);
	timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);

	while(1){
		/*
		mpu6050_readGyroData(&shairo);
		fff[counter] = shairo.Y;
		counter++;
		if(counter == 3){
			counter = 1;
			AngleYSimpson += simpson(fff, 700e-6);
			fff[0] = fff[2];
		}
		AngleYEuler += (700e-6)*shairo.Y;
		if (timerExpired(timerUart)) {
			tmp1[0] = AngleYEuler;
			tmp1[1] = AngleYSimpson;
			tmp2[0] = shairo.Y;
			sendUartMessage6Channels(tmp1, tmp2);
			timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);
		}
		*/
		mpu6050_readGyroData(&gyroData);
		mpu6050_readAccelData(&accelData);

		getAnglesGyro(&gyroData, &AnglesIntegrated, SAMPLE_PERIOD);

/*
		accelData.X = LPF_ACC_ALPHA * prev.X + (1.0f - LPF_ACC_ALPHA) * accelData.X;
		accelData.Y = LPF_ACC_ALPHA * prev.Y + (1.0f - LPF_ACC_ALPHA) * accelData.Y;
		accelData.Z = LPF_ACC_ALPHA * prev.Z + (1.0f - LPF_ACC_ALPHA) * accelData.Z;
*/

		gpioWrite(PORTNUM2PIN(PD, 0), HIGH);
        const FusionVector gyroscope = {.axis.x =  gyroData.X, .axis.y = gyroData.Y, .axis.z = gyroData.Z}; // replace this with actual gyroscope data in degrees/s
        const FusionVector accelerometer = {.axis.x = accelData.X, .axis.y = accelData.Y, .axis.z = accelData.Z}; // replace this with actual accelerometer data in g

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		gpioWrite(PORTNUM2PIN(PD, 0), LOW);
		if (timerExpired(timerUart)) {

			tmp1[0] = euler.angle.roll;
		 	tmp1[1] = euler.angle.pitch;
			tmp1[2] = euler.angle.yaw;
			/*tmp1[0] = gyroData.X;
			tmp1[1] = gyroData.Y;
			tmp1[2] = gyroData.Z;*/
			tmp2[0] = AnglesIntegrated.pitch;
			tmp2[1] = euler.angle.pitch;
			tmp2[2] = 0;
			sendUartMessage6Channels(tmp1, tmp2);
			timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);
		}
		while(!timerExpired(TS_timer));
		timerStart(TS_timer, TIMER_MS2TICKS(10), TIM_MODE_SINGLESHOT, NULL);
	}
}


/*
rotacion alrededor del x del sensor (de y a z) -> GyroRates_deg[0] == p
rotacion alrededor del y del sensor (de x a z) -> GyroRates_deg[1] == q
rotacion alrededor del z del sensor (de y a x) -> GyroRates_deg[2] == r

newAngles[0] = roll ~ phi (de y a z) -> p
newAngles[1] = pitch ~ theta (de x a z) -> q
newAngles[2] = yaw ~ psi (de x a y) -> r
*/

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







