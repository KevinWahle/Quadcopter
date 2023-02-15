/***************************************************************************//**
  @file     SerialOscilloscope.c
  @brief    SerialOscilloscope
  @author   Chad
 ******************************************************************************/

#ifndef _SERIAL_OSCILLOSCOPE_H_
#define _SERIAL_OSCILLOSCOPE_H_

#include <stdio.h>
#include <stdint.h>

static void sendUartMessage6Channels(double* msg1, double* msg2)
{
	char str[100];
	uint16_t charCount= sprintf(str, "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f \r\n",
						msg1[0], msg1[1], msg1[2],
						msg2[0], msg2[1], msg2[2]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}



static void sendUartMessage3Channels(double* msg1)
{
	char str[100];
	uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);
	/*uint16_t charCount= sprintf(str, "%.4f, %.4f, %.4f \r\n",
						msg1[0], msg1[1], msg1[2]);*/
	uartWriteMsg(UART_ID, str, charCount);
}
#endif // _UART_H_
