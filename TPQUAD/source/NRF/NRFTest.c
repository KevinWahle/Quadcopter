/***************************************************************************//**
  @file     App.c
  @brief    Application functions
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include "NRF.h"

uint8_t address[6] = "00001";
uint8_t text[32] = "";

/* Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
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
            read(text, sizeof(text));
        }
    }
}
