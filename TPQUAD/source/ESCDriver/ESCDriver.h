/***************************************************************************//**
  @file     ESCDriver.h
  @brief    Funciones para utilizacion del ESC
  @author   Grupo 5
  @date		9 feb. 2023
 ******************************************************************************/

#ifndef _ESC_H_
#define _ESC_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// ESC Configuration:

// Cantidad de motores
#define MOTOR_COUNT		4

// Modulo FTM a utilizar
#define ESC_FTM_MOD		FTM_0			// Solo se puede usar la 0 o la 3, que tienen al menos 4 canales

// Canales del FTM a utilizar
#define ESC_FTM_CHS		{0, 1, 2, 3}	// Corresponde a PC1, PC2 PC3 y PC4 respectivamente

#define INIT_MOTOR_VALUES	{0.0, 0.0, 0.0, 0.0}
/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum {FTM_0, FTM_1, FTM_2, FTM_3, FTM_COUNT} FTM_MODULE;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

// TODO: Chequear bien que señal hay que mandar antes de armar

/**
 * @brief Inicializa el driver del ESC. Mantiene la salida en valor menor a STOP, para no armar.
*/
void ESCInit();

/**
 * @brief Arma los motores. Envia señal de STOP.
 * @obs: Luego de llamar esta funcion, los motores responderan directamente a la velocidad seteada.
*/
void ESCArm();

/**
 * @brief Desrma los motores.
*/
void ESCDisarm();

/**
 * @brief Realiza la calibraión de los ESC.
 * @obs: Es bloqueante y tarda unos 15seg. NO LLAMAR SI LOS MOTORES ESTAN ARMADOS!!!!!
*/
void ESCCalibrate();

/**
 * @brief Setea la velocidad de los motores
 * @param spped: Arreglo con las velocidades de cada motor. float de 0.0 a 1.0.
 * 				Otros valores tienen comportamiento indeterminado
*/
void ESCSetSpeed(float speed[MOTOR_COUNT]);

/*******************************************************************************
 ******************************************************************************/

#endif // _ESC_H_
