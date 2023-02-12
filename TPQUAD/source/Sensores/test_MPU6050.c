#include "I2Cm/I2Cm.h"
#include "timer/timer.h"
#include "MPU6050.h"
#include "UART/uart.h"
#include "MCAL/gpio.h"
#include <stdio.h>

#define UART_ID			0
#define UART_BAUDRATE	115200

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
char str[100];
static void sendUartMessage6Channels(double* msg1, double* msg2)
{
	uint16_t charCount= sprintf(str, "%.1f, %.1f, %.1f\r\n",
						msg1[0], msg1[1],
						msg2[0]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}

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

double SAVER[256];
uint8_t CC = 0;
void App_Run (void)
{
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

	timerStart(TS_timer, TIMER_MS2TICKS(0.700), TIM_MODE_SINGLESHOT, NULL);
	timerStart(timerUart, TIMER_MS2TICKS(20), TIM_MODE_SINGLESHOT, NULL);
	while(1){
		gpioWrite(PORTNUM2PIN(PD, 0), HIGH);
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
		gpioWrite(PORTNUM2PIN(PD, 0), LOW);
		while(!timerExpired(TS_timer));
		timerStart(TS_timer, TIMER_MS2TICKS(0.700), TIM_MODE_SINGLESHOT, NULL);
	}
}

