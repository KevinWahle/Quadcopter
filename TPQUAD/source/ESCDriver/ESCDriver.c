/***************************************************************************//**
  @file     ESCDriver.c
  @brief    Funciones para utilizacion del ESC
  @author   Grupo 5
  @date		9 feb. 2023
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <ESCDriver/ESCDriver.h>
#include "MK64F12.h"
#include "hardware.h"
#include "MCAL/gpio.h"
#include "timer/timer.h"
#include <stdlib.h>

#include "MCAL/gpio.h"
#include "hardware.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// FTM:

#define FTM_CH_COUNT	8

#define FTM_SRC_CLK	(50000000UL)	// 50MHz Bus Clock

#define FTM_PRES_VAL	(0U)	// prescaler al minimo, maxima precision

#define FTM_CLK	(FTM_SRC_CLK >> FTM_PRES_VAL)	// FTM Clock freq

#define FTM_CLK_PTRS {&(SIM->SCGC6), &(SIM->SCGC6), &(SIM->SCGC6), &(SIM->SCGC3)}
#define FTM_CLK_MASKS {SIM_SCGC6_FTM0_MASK, SIM_SCGC6_FTM1_MASK, SIM_SCGC6_FTM2_MASK, SIM_SCGC3_FTM3_MASK}

// ESC:

#define ESC_PWM_FREQ		4000	//Hz
//TODO: En teroria se puede subir hasta 40kHz
// TODO: Maybe no este tan bueno usar la maxima frecuencua en el PWM. (speed = 1.0 => Señal DC)

#define ESC_DISARM_SPEED	(-0.5F)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint8_t FTM_CHANNEL;

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static FTM_Type* const FTMPtrs[] = FTM_BASE_PTRS;

static __IO uint32_t* const FTMClkSimPtr[] = FTM_CLK_PTRS;
static const uint32_t FTMClkSimMask[] = FTM_CLK_MASKS;

static PORT_Type* const portPtr[] = PORT_BASE_PTRS;

// Pin of each channel of each FTM
// Todos los pines disponibles en package 100LQFP
//															CH0	CH1	CH2	CH3	CH4	CH5	CH6	CH7
static const uint8_t FTMPinPort[][FTM_CH_COUNT] =	{	{	PC,	PC,	PC,	PC,	PD,	PD,	PD,	PD,	},	// FTM0
														{	PB,	PB	},							// FTM1
														{	PB,	PB	},							// FTM2
														{	PD,	PD,	PD,	PD,	PC,	PC,	PC,	PC,	}	// FTM3
													};

static const uint8_t FTMPinNum[][FTM_CH_COUNT] =	{	{	1,	2,	3,	4,	4,	5,	6,	7,	},	// FTM0
														{	0,	1	},							// FTM1
														{	18,	19	},							// FTM2
														{	0,	1,	2,	3,	8,	9,	10,	11,	}	// FTM3
													};

static const uint8_t FTMPinMuxAlt[][FTM_CH_COUNT] =	{	{	4,	4,	4,	4,	4,	4,	4,	4,	},	// FTM0
														{	3,	3,	},							// FTM1
														{	3,	3	},							// FTM2
														{	4,	4,	4,	4,	3,	3,	3,	3,	}	// FTM3
													};

static const FTM_CHANNEL motorChannel[MOTOR_COUNT] = ESC_FTM_CHS;

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/**
 * @brief Inicializa el driver del ESC. Mantiene la salida en valor menor a STOP, para no armar.
*/
void ESCInit(){

	timerInit();

	// Enable FTMx clock gating
	*(FTMClkSimPtr[ESC_FTM_MOD]) |= FTMClkSimMask[ESC_FTM_MOD];

	FTM_Type* const pFTM = FTMPtrs[ESC_FTM_MOD];

	// Disable write protection (if enabled)
	if (pFTM->FMS & FTM_FMS_WPEN_MASK)	pFTM->MODE = FTM_MODE_WPDIS_MASK;

	// Disblae FTM for PWM mode
	pFTM->MODE &= ~FTM_MODE_FTMEN_MASK;		// FTMEN = 0

	// frequency configuration
	uint16_t mod = FTM_CLK/ESC_PWM_FREQ - 1;
	pFTM->MOD =	mod;

	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {

		uint8_t channel = motorChannel[i];

		// Set PWM Mode to edge aligned with high-true polarity
		pFTM->CONTROLS[channel].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;		// MSB = ELSB = 1
		// Start with duty = 0
		pFTM->CONTROLS[channel].CnV = (ESC_DISARM_SPEED/2+0.5)*(pFTM->MOD & FTM_MOD_MOD_MASK);

		// Pin configuration
		portPtr[FTMPinPort[ESC_FTM_MOD][channel]]->PCR[FTMPinNum[ESC_FTM_MOD][channel]] = PORT_PCR_MUX(FTMPinMuxAlt[ESC_FTM_MOD][channel]);
	}



	// Set clock source and disable interrupts.
	FTMPtrs[ESC_FTM_MOD]->SC = FTM_SC_CLKS(0x01) | FTM_SC_PS(FTM_PRES_VAL);		// Start clock

}

/**
 * @brief Setea la velocidad de los motores
 * @param spped: Arreglo con las velocidades de cada motor. double de 0.0 a 1.0.
 * 				Valor negativo desarma el motor. Valor mayor tiene comportamiento indeterminado
*/

//TODO: No es necesario mandar pulsos todo el tiempo, se puede hacer single pulse en vez de PWM
void ESCSetSpeed(double speed[MOTOR_COUNT]) {
	FTM_Type* const pFTM = FTMPtrs[ESC_FTM_MOD];


	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		// Update channel value
		uint8_t channel = motorChannel[i];
		// Mapea speed a [MOD/2, MOD]
		pFTM->CONTROLS[channel].CnV = (speed[i]/2+0.5)*(pFTM->MOD & FTM_MOD_MOD_MASK);
	}

}

/**
 * @brief Arma los motores. Envia señal de STOP.
 * @obs: Luego de llamar esta funcion, los motores responderan directamente a la velocidad seteada.
*/
void ESCArm() {

	double speed[MOTOR_COUNT];

	for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
		speed[i] = 0.0;
	}

	ESCSetSpeed(speed);	// Señal de STOP
}

/**
 * @brief Desrma los motores.
*/
void ESCDisarm() {
	double speed[MOTOR_COUNT];

	for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
		speed[i] = ESC_DISARM_SPEED;
	}
	ESCSetSpeed(speed);
}

/**
 * @brief Realiza la calibraión de los ESC.
 * @obs: Es bloqueante y tarda unos 5seg. NO LLAMAR SI LOS MOTORES ESTAN ARMADOS!!!!!
*/
void ESCCalibrate() {

	double speed[MOTOR_COUNT];

	for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
		speed[i] = 1.0;
	}
	ESCSetSpeed(speed);
	timerDelay(TIMER_MS2TICKS(2500));
	for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
		speed[i] = 0.0;
	}
	ESCSetSpeed(speed);
	timerDelay(TIMER_MS2TICKS(2500));
	for (uint8_t i = 0; i < MOTOR_COUNT; i++)  {
		speed[i] = ESC_DISARM_SPEED;
	}
	ESCSetSpeed(speed);
}
