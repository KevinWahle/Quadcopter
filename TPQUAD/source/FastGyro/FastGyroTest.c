/***************************************************************************//**
  @file     App.c
  @brief    TPF: Reproductor MP3
  @author   Grupo 5
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "UART/uart.h"
#include <stdio.h>
#include "timer/timer.h"
#include <math.h>
#include <Sensores/MPU9250.h>
#include "MCAL/gpio.h"
#include "I2Cm/I2Cm.h"

#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
}

void App_Run (void)
{
	I2CmInit(I2C_ACC);
	initMPU9250();
	uint8_t who;
	whoAmI(&who);
	while(1);
	/*
	tim_id_t TS_timer;
	TS_timer = timerGetId();


	timerStart(TS_timer, TIMER_MS2TICKS(10), TIM_MODE_SINGLESHOT, NULL);
	while(1){

		while(!timerExpired(TS_timer));
		timerStart(TS_timer, TIMER_MS2TICKS(10), TIM_MODE_SINGLESHOT, NULL);

	}
	*/
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

