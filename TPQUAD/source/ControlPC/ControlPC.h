/*
 * ControlPC.h
 *
 *  Created on: Feb 20, 2023
 *      Author: sch_b
 */

#ifndef CONTROLPC_CONTROLPC_H_
#define CONTROLPC_CONTROLPC_H_


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/



/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: extern unsigned int anio_actual;+


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/**
 * @brief initialize comparator
 * @param cmp_n number of cmp to use
 * @param hyst_level hysteresis level (5mV,10mV,20mV,30mV)
 * @param polarity decide if inverted output or not
*/
void controlPCInit(uint8_t id, uint16_t baudrate, char speedUp, char speedDown);
float getDataFromPC(void);

/*******************************************************************************
 ******************************************************************************/


#endif /* CONTROLPC_CONTROLPC_H_ */
