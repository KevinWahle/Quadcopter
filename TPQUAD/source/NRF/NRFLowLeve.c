#include "NRF.h"
#include "../timer/timer.h"
#include "../SPI/SPI.h"
/*start of low level functions, specific to the mcu and compiler*/
static SPI_config_t config;
static SPI_config_t * myconfig;
/*delay in miliseconds*/
void delay_function(uint32_t duration_ms)
{
    timerDelay(TIMER_MS2TICKS(duration_ms));
}

/*contains all SPI configuations, such as pins and control registers*/
/*SPI control: master, interrupts disabled, clock polarity low when idle, clock phase falling edge, clock up tp 1 MHz*/
void SPI_Initializer()
{
    myconfig=&config;
    myconfig->type=MASTER;
    myconfig->PCS_inactive_state=1;
    myconfig->LSB_fist=0;
    myconfig->frame_size=8;
    myconfig->clk_pol=0;
    myconfig->clk_phase=0;
    myconfig->Baud_rate_scaler=0b0011;

    SPI_config(SPI_0,myconfig);
}

/*contains all CSN and CE pins gpio configurations, including setting them as gpio outputs and turning SPI off and CE '1'*/
void pinout_Initializer()
{
}

/*CSN pin manipulation to high or low (SPI on or off)*/
void nrf24_SPI(uint8_t input)
{
}

/*1 byte SPI shift register send and receive routine*/
uint8_t SPI_send_command(uint8_t command)
{
}

/*CE pin maniplation to high or low*/
void nrf24_CE(uint8_t input)
{
}