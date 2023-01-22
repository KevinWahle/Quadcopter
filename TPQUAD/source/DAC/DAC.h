/***************************************************************************//**
  @file     DAC.h
  @brief    +Descripcion del archivo+
  @author   KevinWahle
  @date		15 oct. 2022
 ******************************************************************************/

#ifndef DAC_DAC_H_
#define DAC_DAC_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/



/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/
typedef enum {DAC_0, DAC_1} DAC_n;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

// +ej: extern unsigned int anio_actual;+
typedef uint16_t DACData_t;


/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief TODO: completar descripcion
 * @param param1 Descripcion parametro 1
 * @param param2 Descripcion parametro 2
 * @return Descripcion valor que devuelve
*/
// +ej: char lcd_goto (int fil, int col);+
void DAC_Init (DAC_n dac_n);
void DAC_SetData (DAC_n dac_n, uint16_t **fskptr);		// 12bit length


/*******************************************************************************
 ******************************************************************************/

#endif // _DAC_DAC_H_
