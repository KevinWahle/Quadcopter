/***************************************************************************//**
  @file     App.c
  @brief    TP2: Comunicacion Serie
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "SBUS.h"
#include "MCAL/gpio.h"
#include "timer/timer.h"
#include "MCAL/board.h"
#include "UART/uart.h"

#include <stdio.h>
#include <string.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define UART_ID			0
#define UART_BAUDRATE	115200
#define UART_ID_SBUS	3
/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 *******************************************************************************
                        GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/

uint8_t cont[] = {1, 1, 1};

tim_id_t timer;

SBUSData_t sbus;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{

	timerInit();

	gpioMode(PIN_LED_RED, OUTPUT);
	gpioMode(PIN_LED_GREEN, OUTPUT);

	gpioWrite(PIN_LED_RED, HIGH);
	gpioWrite(PIN_LED_GREEN, HIGH);

	SBUSInit(&sbus);
	uart_cfg_t cfg = {.MSBF = false, .baudrate = UART_BAUDRATE, .parity = NO_PARITY};
	uartInit(UART_ID, cfg);

}

static char strChannels[100];

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{

	timerDelay(TIMER_MS2TICKS(100));

/*
	char str[256];
	uint8_t num = 0;
	for (uint8_t i = 0; i < 16; i++) {
		num += sprintf(str+num, "%u,", sbus.channels[i]);
	}
	str[num-1] = '\n';*/

	uint16_t charCount= sprintf(strChannels, "%.1f, %.1f, %.1f \r\n",
						3.14, 2.47, 3.0);
	//uartWriteMsg(UART_ID, strChannels, charCount);

	SBUSWriteMsg(UART_ID_SBUS, strChannels, charCount);
	//uartWriteMsg(UART_ID_SBUS, str, num);

}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
