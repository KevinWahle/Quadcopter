/***************************************************************************//**
  @file		uart.c
  @brief	Funciones para comunicación por UART
  @author	Sergio Peralta
  @date		9 sep. 2022
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "SBUS.h"
#include "MK64F12.h"
#include "hardware.h"
#include "MCAL/gpio.h"
#include "buffer/circular_buffer.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define CORE_CLOCK	__CORE_CLOCK__

#define BUS_CLOCK	(CORE_CLOCK >> 1)

#define SBUS_BAUDRATE	100000	// bauds/sec

#define SBUS_CANT_IDS	5

#define SBUS_UART_ID	1

#define SBUS_START_BYTE		(0x0FU)
#define SBUS_STOP_BYTE		(0x0U)

// Useful Macros

#define MIN(x, y)	((x) < (y) ? (x) : (y) )
#define MAX(x, y)	((x) > (y) ? (x) : (y) )


/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void UART_SBUS_IRQ();

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static UART_Type* const UARTPorts[] = UART_BASE_PTRS;
static PORT_Type* const portPtr[] = PORT_BASE_PTRS;

static const uint32_t UARTClockSource[] = {CORE_CLOCK, CORE_CLOCK, BUS_CLOCK, BUS_CLOCK, BUS_CLOCK, BUS_CLOCK};

static __IO uint32_t* const UARTSIMR[] = {&(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC4), &(SIM->SCGC1)};
static const uint32_t UARTSIMMask[] = {SIM_SCGC4_UART0_MASK, SIM_SCGC4_UART1_MASK, SIM_SCGC4_UART2_MASK, SIM_SCGC4_UART3_MASK, SIM_SCGC1_UART4_MASK};

static const IRQn_Type UART_RX_TX_Vectors[] = UART_RX_TX_IRQS;

static const uint8_t UARTPinPort[] = {PB, PC, PD, PB, PE};

static const uint8_t UARTPinNumRX[] = {16, 3, 2, 10, 24};

static const uint8_t UARTPinMuxAlt[] = {3, 3, 3, 3, 3};

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static SBUSData_t* pSBUSData;

//static circularBuffer TxBuffer[UART_CANT_IDS];
static circularBuffer RxBuffer[SBUS_CANT_IDS];

//typedef enum {
//	START,
//	SAVE,
//	STOP,
//	PROCESS,
//} SBUS_STATES;
//
//static SBUS_STATES actualState;

static uint8_t contBytes;

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/**
 * @brief Initialize SBUS driver
 * @param SBUSData Pointer to structure to save last channels data
*/
void SBUSInit (SBUSData_t* SBUSData) {

	pSBUSData = SBUSData;

	UART_Type* uart = UARTPorts[SBUS_UART_ID];

// Clock Gating

	uint32_t clkFreq = UARTClockSource[SBUS_UART_ID];
	*(UARTSIMR[SBUS_UART_ID]) |= UARTSIMMask[SBUS_UART_ID];


// Pinout setup

	portPtr[UARTPinPort[SBUS_UART_ID]]->PCR[UARTPinNumRX[SBUS_UART_ID]] = PORT_PCR_MUX(UARTPinMuxAlt[SBUS_UART_ID]);		// Borra otros flags

// Baud rate config

	uint32_t baudrate;
	uint16_t sbr, brfa;

	baudrate = SBUS_BAUDRATE;

	sbr = clkFreq / (baudrate << 4);
	brfa = (clkFreq << 1) / baudrate - (sbr << 5);

	// IMPORTANTE: Primero el HIGH
	uart->BDH = UART_BDH_SBNS_MASK | UART_BDH_SBR(sbr >> 8);	// Seteo de 2 stops (SBUS)
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = UART_C4_BRFA(brfa);

// Enable NVIC Interrupts
	NVIC_EnableIRQ(UART_RX_TX_Vectors[SBUS_UART_ID]);

// Control

	uart->S2 = UART_S2_RXINV_MASK;		// Inversion de la polaridad (SBUS)
	uart->C5 = 0x0;		// Disable DMA Requests
	uart->IR = 0x0;		// Disable IR
	uart->MODEM = 0x0;		// Disable CTS, RTS

	// Parity (Si hay paridad, habilitar 9 bits de data)
	uart->C1 = UART_C1_PE_MASK | UART_C1_M_MASK;	// Paridad par (SBUS)

	// Habilita interrupcion por Parity Error y Overrun
	uart->C3 = 0x0;

	// Habilitar Rx e interrupciones
	uart->C2 = UART_C2_RIE_MASK | UART_C2_RE_MASK;	// Rx activado, Tx dessactivado


// Initialize Buffer
	CBinit(RxBuffer+SBUS_UART_ID, BUFFER_SIZE);

}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void UART_SBUS_IRQ() {

	if (UARTPorts[SBUS_UART_ID]->S1 & UART_S1_RDRF_MASK) {	// Something received

		contBytes++;

		if (contBytes == 1) {	// START
			if (UARTPorts[SBUS_UART_ID]->D == SBUS_START_BYTE) CBreset(RxBuffer+SBUS_UART_ID);	// Buffer reset
			else contBytes = 0;	// Me quedo esperando START
		}
		else if (contBytes <= 24) {	// DATA
			CBputByte(RxBuffer + SBUS_UART_ID, UARTPorts[SBUS_UART_ID]->D);		// Guardo la data
		}
		else {	// STOP

			if (UARTPorts[SBUS_UART_ID]->D == SBUS_STOP_BYTE) {	// Chequeo data valida

				// Proceso

				uint8_t prevByte = CBgetByte(RxBuffer+SBUS_UART_ID);
				uint8_t bitsLeft = 8;	// bits que me quedan del byte anterior

				for (uint8_t i = 0; i < 16; i++) {
					pSBUSData->channels[i] = prevByte & (0xFFU >> (8-bitsLeft));

					prevByte = CBgetByte(RxBuffer+SBUS_UART_ID);		// Guardamos el byte antes de usarlo
					pSBUSData->channels[i] |= prevByte << bitsLeft;

					if (bitsLeft < 3) {			// Necesito un byte mas
						prevByte = CBgetByte(RxBuffer+SBUS_UART_ID);		// Guardamos el byte antes de usarlo
						pSBUSData->channels[i] |= prevByte << (bitsLeft+8);
						bitsLeft += 5;	// 8-(11-(bitLeft+8))
					}
					else {
						bitsLeft -= 3;	// 8-(11-bitsLeft)
					}

					pSBUSData->channels[i] &= 0x7FFU;	// Trim to 11 bits
				}

				//  TODO; Esto hay que chequearlo bien, igual no se usa
				pSBUSData->flags = CBgetByte(RxBuffer+SBUS_UART_ID) >> 4U;

			}
//			else {
//			// TODO: ERROR
//				printf("ERROR!\n");
//			}
			contBytes = 0;
		}

	}

}


__ISR__ UART1_RX_TX_IRQHandler(void){

	UART_SBUS_IRQ();

}
