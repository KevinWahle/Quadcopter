/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "NRF.h"
#include <stdbool.h>
#include <stdio.h>

uint8_t text[33];

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{ 
	text[0] = 1;
	text[12] = 1;
	text[13] = 1;
	text[18] = 1;
	text[13] = 1;
	text[14] = 1;
	text[15] = 1;
	text[16] = 1;
	text[17] = 1;text[32] = 33;text[31] = 33;
}

/* Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	RF24begin();
    openReadingPipe(0, address);
    setPALevel(RF24_PA_MIN);
    startListening();
    while(1){
        if(available()){
            read(text, 32);
            printf("%c\n", 'c');
        }
    }
}
