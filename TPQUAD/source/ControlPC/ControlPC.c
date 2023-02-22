/***************************************************************************//**
  @file		ControlPC
  @brief	+Descripcion del archivo+
  @author	++
  @date		++
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <ControlPC/ControlPC.h>
#include "../UART/uart.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/
#define NO_MESSAGE 0
#define MAX_CANT_BYTES 10
/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/
static bool init = false;
static uint8_t uartId;
static uint16_t uartBaudrate;
static char sUp;
static char sDown;
static float steps = 0;
/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void controlPCInit(uint8_t id, uint16_t baudrate, char speedUp, char speedDown){ //Number of consecutive samples that must agree

	uart_cfg_t config = {.MSBF = false, .baudrate = uartBaudrate, .parity = NO_PARITY};
	uartInit(uartId,config);
	uartBaudrate = baudrate;
	uartId = id;
	sUp = speedUp;
	sDown = speedDown;
	init = true;

}
float getDataFromPC(void){
	if(uartIsRxMsg(uartId))
	{
		char msg[MAX_CANT_BYTES];
		uint8_t cant = uartGetRxMsgLength(uartId);
		uartReadMsg(uartId,msg,cant);
		for(uint8_t i = 0; i<cant ; i++)
		{
			if(msg[i] == sUp)
			{
				steps+=0.25;
			}
			else if(msg[i] == sDown)
			{
				steps-= 0.25;
			}
		}
	}
    if(steps > 10)
    {
        steps = 10;
    }
    else if(steps < 0)
    {
        steps = 0;
    }
    return steps;
}
/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/
