#include "NRF.h"
#include "timer/timer.h"
#include <stdint.h>
#include "SPI/SPI.h"
#include <stdbool.h>
#include "MCAL/gpio.h"
#include <string.h>

#define CE          PORTNUM2PIN(PB, 2)

/* Memory Map */
#define NRF_CONFIG  0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define CONT_WAVE   7
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5      5
#define DPL_P4      4
#define DPL_P3      3
#define DPL_P2      2
#define DPL_P1      1
#define DPL_P0      0
#define EN_DPL      2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define RF24_NOP      0xFF

/* Non-P omissions */
#define LNA_HCURR 0

/* P model memory Map */
#define RPD                 0x09
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define rf24_max(a, b) (a > b ? a : b)
#define rf24_min(a, b) (a < b ? a : b)

#define _BV(x) (1 << (x))

#define RF24_POWERUP_DELAY 5000

static SPI_config_t SPIconfig;
volatile static uint8_t finishSPI = 0;
static bool _is_p_variant;
static bool ack_payloads_enabled;
static bool dynamic_payloads_enabled;
static uint8_t payload_size;
static uint8_t addr_width;
static uint8_t config_reg;
static uint8_t pipe0_reading_address[5]; /* Last address set on pipe 0 for reading. */
static bool _is_p0_rx;                   /* For keeping track of pipe 0's usage in user-triggered RX mode. */
static uint8_t status = 1;

static uint8_t child_pipe[] = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2,
                                             RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};

static uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2,
                                                    ERX_P3, ERX_P4, ERX_P5};


/*Prototipos locales*/
static void setRetries(uint8_t delay, uint8_t count);
static void writeRegister(uint8_t reg, uint8_t value);
static uint8_t readRegister(uint8_t reg);
static void setDataRate(rf24_datarate_e speed);
static void callBackSPI();
static void toggle_features(void);
static void setPayloadSize(uint8_t size);
static void setAddressWidth(uint8_t a_width);
static void setChannel(uint8_t channel);
static void flush_rx(void);
static void flush_tx();
static void powerUp(void);
void setPALevel(uint8_t level);
static uint8_t _pa_level_reg_value(uint8_t level, bool lnaEnable);
static void writeSeveralRegisters(uint8_t reg, const uint8_t* buf, uint8_t len);
static uint8_t get_status();
static void read_payLoad(uint8_t * buf, uint8_t data_len);

bool RF24begin(){

    gpioMode(CE, OUTPUT);
    gpioWrite(CE, LOW);

    SPIconfig.type=MASTER;
    SPIconfig.PCS_inactive_state=1;
    SPIconfig.LSB_fist=0;
    SPIconfig.frame_size=8;
    SPIconfig.clk_pol=0;
    SPIconfig.clk_phase=0;
    SPIconfig.Baud_rate_scaler=0b0011;

    SPI_config(SPI_0, &SPIconfig);

    timerInit();
    timerDelay(TIMER_MS2TICKS(5));
    
    // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
    // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
    // sizes must never be used. See datasheet for a more complete explanation.
    setRetries(5, 15);
    setDataRate(RF24_1MBPS);

    // detect if is a plus variant & use old toggle features command accordingly
    uint8_t before_toggle = readRegister(FEATURE);
    toggle_features();
    uint8_t after_toggle = readRegister(FEATURE);
    _is_p_variant = before_toggle == after_toggle;
    if (after_toggle) {
        if (_is_p_variant) {
            // module did not experience power-on-reset (#401)
            toggle_features();
        }
        // allow use of multicast parameter and dynamic payloads by default
        writeRegister(FEATURE, 0);
    }
    ack_payloads_enabled = false; // ack payloads disabled by default
    writeRegister(DYNPD, 0);     // disable dynamic payloads by default (for all pipes)
    dynamic_payloads_enabled = false;
    writeRegister(EN_AA, 0x3F);  // enable auto-ack on all pipes
    writeRegister(EN_RXADDR, 3); // only open RX pipes 0 & 1
    setPayloadSize(32);           // set static payload size to 32 (max) bytes by default
    setAddressWidth(5);           // set default address length to (max) 5 bytes

    // Set up default configuration.  Callers can always change it later.
    // This channel should be universally safe and not bleed over into adjacent
    // spectrum.
    setChannel(76);

    // Reset current status
    // Notice reset and flush is the last thing we do
    writeRegister(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Flush buffers
    flush_rx();
    flush_tx();

    // Clear CONFIG register:
    //      Reflect all IRQ events on IRQ pin
    //      Enable PTX
    //      Power Up
    //      16-bit CRC (CRC required by auto-ack)
    // Do not write CE high so radio will remain in standby I mode
    // PTX should use only 22uA of power
    writeRegister(NRF_CONFIG, (_BV(EN_CRC) | _BV(CRCO)));
    config_reg = readRegister(NRF_CONFIG);
    powerUp();
    return config_reg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)) ? true : false;
}

static void setRetries(uint8_t delay, uint8_t count){
    writeRegister(SETUP_RETR, rf24_min(15, delay) << ARD | rf24_min(15, count));
}

static void writeRegister(uint8_t reg, uint8_t value){
    package pkg[2];
	pkg[0].msg = W_REGISTER | reg;
	pkg[0].pSave = &status;
	pkg[0].cb = NULL;
	pkg[0].read = 1;
	pkg[0].cs_end = 0;

	pkg[1].msg = value;
	pkg[1].pSave = NULL;
	pkg[1].cb = callBackSPI;
	pkg[1].read = 0;
	pkg[1].cs_end = 1;

    finishSPI = 0;
    SPISend(SPI_0, pkg, 2, 0);
    while(!finishSPI);
}

static uint8_t readRegister(uint8_t reg){
    package pkg[2];
    uint8_t readValue;
	pkg[0].msg = R_REGISTER | reg;
	pkg[0].pSave = &status;
	pkg[0].cb = NULL;
	pkg[0].read = 1;
	pkg[0].cs_end = 0;

	pkg[1].msg = 0xff;
	pkg[1].pSave = &readValue;
	pkg[1].cb = callBackSPI;
	pkg[1].read = 1;
	pkg[1].cs_end = 1;

    finishSPI = 0;
    SPISend(SPI_0, pkg, 2, 0);
    while(!finishSPI);
    return readValue;
}

static void setDataRate(rf24_datarate_e speed){
    uint8_t setup = readRegister(RF_SETUP);

    // HIGH and LOW '00' is 1Mbs - our default      
    setup = setup & ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));  // HARDCODEADO FUERZO EL 1MBps

    writeRegister(RF_SETUP, setup);
}

static void callBackSPI(){
    finishSPI = 1;
}

static void toggle_features(void)
{
    package pkg[2];
	pkg[0].msg = ACTIVATE;
	pkg[0].pSave = NULL;
	pkg[0].cb = NULL;
	pkg[0].read = 0;
	pkg[0].cs_end = 0;

	pkg[1].msg = 0x73;
	pkg[1].pSave = NULL;
	pkg[1].cb = callBackSPI;
	pkg[1].read = 0;
	pkg[1].cs_end = 1;

    finishSPI = 0;
    SPISend(SPI_0, pkg, 2, 0);
    while(!finishSPI);
}
static void setPayloadSize(uint8_t size)
{
    // payload size must be in range [1, 32]
    payload_size = rf24_max(1, rf24_min(32, size));

    // write static payload size setting for all pipes
    for (uint8_t i = 0; i < 6; ++i) {
        writeRegister(RX_PW_P0 + i, payload_size);
    }
}

static void setAddressWidth(uint8_t a_width)
{
    a_width = a_width - 2;
    if (a_width) {
        writeRegister(SETUP_AW, a_width % 4);
        addr_width = (a_width % 4) + 2;
    }
    else {
        writeRegister(SETUP_AW, 0);
        addr_width = 2;
    }
}

static void setChannel(uint8_t channel)
{
    const uint8_t max_channel = 125;
    writeRegister(RF_CH, rf24_min(channel, max_channel));
}

static void flush_rx(void)
{
    writeRegister(FLUSH_RX, RF24_NOP);
}

static void flush_tx(){
    writeRegister(FLUSH_TX, RF24_NOP);
}

static void powerUp(void)
{
    // if not powered up then power up and wait for the radio to initialize
    if (!(config_reg & _BV(PWR_UP))) {
        config_reg |= _BV(PWR_UP);
        writeRegister(NRF_CONFIG, config_reg);

        // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
        // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
        // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
        timerDelay(TIMER_US2TICKS(RF24_POWERUP_DELAY));
    }
}

void openReadingPipe(uint8_t child, uint8_t* address)
{
    // If this is pipe 0, cache the address.  This is needed because
    // openWritingPipe() will overwrite the pipe 0 address, so
    // startListening() will have to restore it.
    if (child == 0) {
        memcpy(pipe0_reading_address, address, addr_width);
        _is_p0_rx = true;
    }

    if (child <= 5) {
        // For pipes 2-5, only write the LSB
        if (child < 2) {
            writeSeveralRegisters(child_pipe[child], address, addr_width);
        }
        else {
           writeSeveralRegisters(child_pipe[child], address, 1);
        }

        // Note it would be more efficient to set all of the bits for all open
        // pipes at once.  However, I thought it would make the calling code
        // more simple to do it this way.
        writeRegister(EN_RXADDR, readRegister(EN_RXADDR) | _BV(child_pipe_enable[child]) );
    }

}
void setPALevel(uint8_t level)
{
    bool lnaEnable = 1;
    uint8_t setup = readRegister(RF_SETUP) & (0xF8);
    setup |= _pa_level_reg_value(level, lnaEnable);
    writeRegister(RF_SETUP, setup);
}

static uint8_t _pa_level_reg_value(uint8_t level, bool lnaEnable)
{
    // If invalid level, go to max PA
    // Else set level as requested
    // + lnaEnable (1 or 0) to support the SI24R1 chip extra bit
    return (((level > RF24_PA_MAX ? (uint8_t)(RF24_PA_MAX) : level) << 1) + lnaEnable);
}

void startListening(void)
{
    config_reg |= _BV(PRIM_RX);
    writeRegister(NRF_CONFIG, config_reg);
    writeRegister(NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    //ce(HIGH);
    gpioWrite(CE, HIGH);
    // Restore the pipe0 address, if exists
    if (_is_p0_rx) {
        writeSeveralRegisters(RX_ADDR_P0, pipe0_reading_address, addr_width);
    }
    else {
       // closeReadingPipe(0);
    }
}

static void writeSeveralRegisters(uint8_t reg, const uint8_t* buf, uint8_t len)
{
    package pkg [13];  // VER BIEN LA CANTIDAD MAXIMA DE SIZE DE ADDRESSES
	pkg[0].msg = W_REGISTER | reg;
	pkg[0].pSave = NULL;
	pkg[0].cb = NULL;
	pkg[0].read = 0;
	pkg[0].cs_end = 0;

    for(uint8_t i = 1; i <= len; i++){
        pkg[i].msg = *buf++;
        pkg[i].pSave = NULL;
        pkg[i].cb = i == len ? callBackSPI : NULL;
        pkg[i].read = 0;
        pkg[i].cs_end = i == len ? 1 : 0;
    }

    finishSPI = 0;
    SPISend(SPI_0, pkg, len + 1, 0);
    while(!finishSPI);
}

bool available()
{
    // get implied RX FIFO empty flag from status byte
    uint8_t pipe = (get_status() >> RX_P_NO) & 0x07;
    if (pipe > 5)
        return 0;

    return 1;
}
static uint8_t get_status(){
    return readRegister(R_REGISTER | NRF_STATUS);
}

void read(uint8_t* buf, uint8_t len)
{
    // Fetch the payload
	read_payLoad(buf, len);

    //Clear the only applicable interrupt flags
    writeRegister(NRF_STATUS, _BV(RX_DR));
}

static void read_payLoad(uint8_t * buf, uint8_t data_len){
    package pkg [35];  // VER BIEN LA CANTIDAD MAXIMA DE SIZE DE ADDRESSES
	pkg[0].msg = R_RX_PAYLOAD;
	pkg[0].pSave = NULL;
	pkg[0].cb = NULL;
	pkg[0].read = 0;
	pkg[0].cs_end = 0;

    for(uint8_t i = 1; i <= data_len; i++){
        pkg[i].msg = 0xff;
        pkg[i].pSave = &buf[i];
        pkg[i].cb = i == data_len ? callBackSPI : NULL;
        pkg[i].read = 1;
        pkg[i].cs_end = i == data_len ? 1 : 0;
    }

    finishSPI = 0;
    SPISend(SPI_0, pkg, data_len + 1, 0);
    while(!finishSPI);
}
