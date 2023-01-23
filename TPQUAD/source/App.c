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

#define UART_ID			0
#define UART_BAUDRATE	115200
/*******************************************************************************
 *******************************************************************************
                        	GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/

static void sendUartMessage(double* msg);
/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
}


int16_t accelData[3];
int16_t gyroData[3];
/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	double Acc[3] = {0,0,0};
	double Gyro[3] = {0,0,0};
	I2CmInit(I2C_ACC);
	uint8_t AVER;
	whoAmI(&AVER);
	// should be 0x71
	resetMPU9250();
	initMPU9250();
	calibrateMPU9250();
	while(1){
		readAccelData(accelData);
		// 2G =~ 32,767
		// double xAcc = (double)accelData[0]/32767.0 * 2; // 2 por 2G
		// double yAcc = (double)accelData[1]/32767.0 * 2;
		// double zAcc = (double)accelData[2]/32767.0 * 2;
		int2doubleAcc(Acc,accelData);

		readGyroData(gyroData);

		// double xGyro = (double)gyroData[0]/32767.0 * 250;
		// double yGyro = (double)gyroData[1]/32767.0 * 250;
		// double zGyro = (double)gyroData[2]/32767.0 * 250;
		int2doubleGyro(Gyro, gyroData);
		timerDelay(TIMER_US2TICKS(2000));
		sendUartMessage(Gyro);
	}

}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
static void sendUartMessage(double* msg)
{
	char str[100];
	if(uartIsTxMsgComplete(UART_ID)) {
		sprintf(str, "%f", msg[0]);
		str[5] = ',';
		str[7] = '\r';
		uartWriteMsg(UART_ID, str, 6);
	}
	while(!uartIsTxMsgComplete(UART_ID));
	sprintf(str, "%f", msg[1]);
	str[5] = ',';
	str[7] = '\n';
	uartWriteMsg(UART_ID, str, 6);

	while(!uartIsTxMsgComplete(UART_ID));
	sprintf(str, "%f", msg[2]);
	str[6] = ',';
	str[6] = '\n';
	uartWriteMsg(UART_ID, str, 7);
}

/*******************************************************************************
 ******************************************************************************/
