/***************************************************************************//**
  @file     UART.c
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature
  @author   Nicol�s Magliola
 ******************************************************************************/

#ifndef _SBUS_H_
#define _SBUS_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	uint16_t channels[16];	// Los 16 primeros canales son de 11 bits
	union {
		struct {
			bool ch17		: 1;	// Los ultimos dos canales son de 1 bit
			bool ch18		: 1;
			bool frameLost	: 1;
			bool failsafe	: 1;
		};
		uint8_t flags;		// Para poder guardar y acceder mas facil
	};
} SBUSData_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize SBUS driver
 * @param SBUSData Pointer to structure to save last channels data
 * @note: La entrada es por el pin PC3 y usa el UART1
*/
void SBUSInit (SBUSData_t* SBUSData);


/*******************************************************************************
 ******************************************************************************/

#endif // _SBUS_H_
