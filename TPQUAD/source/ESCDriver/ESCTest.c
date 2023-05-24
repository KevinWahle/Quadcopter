/***************************************************************************//**
  @file     FTMTest.c
  @brief    Archivo de prueba para modulo FTM
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <ESCDriver/ESCDriver.h>
#include "MCAL/gpio.h"
#include "MCAL/board.h"
#include <stdio.h>
#include "timer/timer.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 *******************************************************************************
                        GLOBAL VARIABLES
 *******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	timerInit();
	ESCInit();
	gpioMode(PIN_SW2, SW2_INPUT_TYPE);

}

bool flag = 0;
float speed[MOTOR_COUNT];

void DIMMER_PWM(){
    speed[0] = speed[0] + 0.0001 > 1.0 ? 1.0 : speed[0] + 0.0001;
    speed[1] = speed[1] + 0.0001 > 1.0 ? 1.0 : speed[1] + 0.0001;
    speed[2] = speed[2] + 0.0001 > 1.0 ? 1.0 : speed[2] + 0.0001;
    speed[3] = speed[3] + 0.0001 > 1.0 ? 1.0 : speed[3] + 0.0001;
    ESCSetSpeed(speed);
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	tim_id_t TS_timer;
	TS_timer = timerGetId();
//	while(gpioRead(PIN_SW2));	// Espero SW
	ESCCalibrate();
	while(gpioRead(PIN_SW2));	// Espero SW
	while(!gpioRead(PIN_SW2));
	ESCArm();
	timerDelay(TIMER_MS2TICKS(5000));
	//timerStart(TS_timer, TIMER_MS2TICKS(100), TIM_MODE_PERIODIC, DIMMER_PWM);
	speed[0] = 0.6;
	speed[1] = 0.6;
	speed[2] = 0.6;
	speed[3] = 0.6;
    ESCSetSpeed(speed);
	while(1);
/*
	for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
		speed[i] = 0.0;
	}

	while(speed[0] < 1.07) {
		while(gpioRead(PIN_SW2));	// Espero SW
		while(!gpioRead(PIN_SW2));
		ESCSetSpeed(speed);
		for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
			speed[i] += 0.05;
		}
	}

	while(speed[0] > 0) {
		while(gpioRead(PIN_SW2));	// Espero SW
		while(!gpioRead(PIN_SW2));
		for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
			speed[i] -= 0.05;
		}
		ESCSetSpeed(speed);
	}
	*/
	ESCDisarm();

	while(1);

}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 ******************************************************************************/
