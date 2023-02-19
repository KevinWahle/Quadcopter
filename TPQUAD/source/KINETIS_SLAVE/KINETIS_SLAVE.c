#include "SBUS/SBUS.h"

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

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	while (uartIsRxMsg(UART_ID_SBUS)) {
		char c;
		uartReadMsg(UART_ID_SBUS, &c, 1);
		uartWriteMsg(UART_ID, &c, 1);
	}
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
