/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "RF.h"
#include <stdbool.h>
#include <stdio.h>
#include "timer/timer.h"
#include "UART/uart.h"
#include <stdint.h>
#define UART_ID			0
#define UART_BAUDRATE	115200
static char strChannels[100];
static void sendUartMessage3Channels(double* msg1)
{
	uint16_t charCount= sprintf(strChannels, "%.1f, %.1f, %.1f \r\n",
						msg1[0], msg1[1], msg1[2]);
	uartWriteMsg(UART_ID, strChannels, charCount);
}
/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{ 
    timerInit();
	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
    OrientationRF orRF;
    RFinit();
    RFbegin();
    timerDelay(TIMER_MS2TICKS(10));
    RFcalibrate();
    while(1){
        timerDelay(TIMER_MS2TICKS(100));
        RFgetDeNormalizedData(&orRF);
        //RF2Newton(&orRF);
        double tmp[3];
        tmp[0] = orRF.throttle;
        tmp[1] = orRF.pitch;
        tmp[2] = orRF.roll;
        sendUartMessage3Channels(tmp);
    }
}
