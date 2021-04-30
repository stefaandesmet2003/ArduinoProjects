/*
  twi.c - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

/*
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/twi.h>
*/
#include "Arduino.h" // for digitalWrite

//#include "pins_arduino.h"
#include "twi_tx.h"

static volatile uint8_t twi_state;
static volatile uint8_t twi_sendStop;			// should the transaction end with a stop

// twi_timeout_us > 0 prevents the code from getting stuck in various while loops here
// if twi_timeout_us == 0 then timeout checking is disabled (the previous Wire lib behavior)
// at some point in the future, the default twi_timeout_us value could become 25000
// and twi_do_reset_on_timeout could become true
// to conform to the SMBus standard
// http://smbus.org/specs/SMBus_3_1_20180319.pdf
static volatile uint32_t twi_timeout_us = 0ul;
static volatile bool twi_timed_out_flag = false;  // a timeout has been seen
static volatile bool twi_do_reset_on_timeout = false;  // reset the TWI registers on timeout

static void (*twi_onSlaveTransmit)(void);
static void (*twi_onSlaveReceive)(uint8_t*, int);

static uint8_t twi_masterBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_masterBufferIndex;
static volatile uint8_t twi_masterBufferLength;

static uint8_t twi_txBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_txBufferIndex;
static volatile uint8_t twi_txBufferLength;

static uint8_t twi_rxBuffer[TWI_BUFFER_LENGTH];
static volatile uint8_t twi_rxBufferIndex;

static volatile uint8_t twi_error;

// for now - need this for easy reinit & modify frequency
static uint8_t twi_ownAddress = 8;
static uint32_t twi_frequency = I2C_MAX_STANDARD_FREQ;

// return codes in twi_writeto 
#define TWI_ERROR_NONE        0
#define TWI_ERROR_ADDR_NACK   2
#define TWI_ERROR_DATA_NACK   3
#define TWI_ERROR_OTHER       4

// TODO : cleanup this copy of ST-code for masterISR
volatile uint8_t STATE; // curent I2C states machine state
// Define I2C STATE MACHINE :

#define INI_00 00

// Write states 0x
#define SB_01 01
#define ADD10_02 02
#define ADDR_03 03
#define BTF_04 04

// Read states 1x
#define SB_11 11
#define ADD10_12 12
#define ADDR_13 13
#define BTF_14 14
#define BTF_15 15
#define RXNE_16 16
#define BTF_17 17
#define RXNE_18 18

/* --- SDuino additions -------------------------------------------------- */
/* This part is common with the I2C library
 */
static uint16_t twi_timeOutDelay;
static uint8_t returnStatus;

static uint8_t twi_sendAddress(uint8_t, uint8_t);
static uint8_t twi_sendByte(uint8_t);

static void twi_lockupTOREMOVE(void);

static uint16_t startingTime;
static bool timeout_expired;
static void tout_start(void);
static bool tout(void);

#define SLA_W(address)	(address << 1)
#define SLA_R(address)	((address << 1) + 0x01)

#define MODE_WRITE 4

#define SEND_ADDRESS_W(ADDR) \
	returnStatus = twi_sendAddress(SLA_W(ADDR),MODE_WRITE); \
	if (returnStatus) return(returnStatus);

// wait while the condition is still true (wait for a bit to become zero)
#define TIMEOUT_WAIT_FOR_ZERO(CONDITION,ERROR) \
	while (CONDITION) 	/* wait while the condition is still true */ \
	{ \
		if (tout()) \
		{ \
			twi_lockupTOREMOVE(); \
			return(ERROR);/* return the appropriate error code */ \
		} \
	}

// wait while the condition is not true (wait for a bit to become one)
#define TIMEOUT_WAIT_FOR_ONE(CONDITION,ERROR) TIMEOUT_WAIT_FOR_ZERO(!(CONDITION), ERROR)


/* --- public methods ---------------------------------------------------- */

/*
 * Function twi_init
 * Desc     readys twi pins and sets twi bitrate
 * Input    none
 * Output   none
 */
void twi_init(void)
{
  // initialize state
  twi_state = TWI_READY;
  twi_sendStop = true;		// default value

	// set I2C frequency to 100kHz and do a full init
	twi_setFrequency(I2C_MAX_STANDARD_FREQ);

	// set default timeout to 20ms
	twi_timeOutDelay = 20;

  // TODO : enable I2C interrupts for master & slave
  // for now : only SRX/STX using interrupts
  //I2C->ITR = 0x07;
  I2C->ITR = 0x0;
}

/*
 * Function twi_disable
 * Desc     disables twi pins
 * Input    none
 * Output   none
 */
void twi_disable(void)
{
  // disable twi module, acks, and twi interrupt
	I2C->CR1 = 0; // this clears CR2 too, TODO : check SWRST bit!!
  I2C->ITR = 0;
}

/*
 * Function twi_setClock
 * Desc     sets twi bit rate
 * Input    Clock Frequency
 * Output   none
 */
void twi_setFrequency(uint32_t frequency)
{
  twi_frequency = frequency; // store for easy reinit after timeout
	// the easiest way to change the frequency is a full re-init
	I2C_Init(
		twi_frequency,		        // I2C_SPEED,
		(uint16_t)twi_ownAddress << 1,  // OwnAddress
		I2C_DUTYCYCLE_2,      // 0x00
		I2C_ACK_CURR,         // 0x01
		I2C_ADDMODE_7BIT,     // 0x00
		F_CPU/1000000u        // InputClockFrequencyMhz
	);
}



/* 
 * Function twi_writeTo
 * Desc     attempts to become twi bus master and write a
 *          series of bytes to a device on the bus
 * Input    address: 7bit i2c device address
 *          data: pointer to byte array
 *          length: number of bytes in array
 *          wait: boolean indicating to wait for write or not
 *          sendStop: boolean indicating whether or not to send a stop at the end
 * Output   0 .. success
 *          1 .. length to long for buffer
 *          2 .. address send, NACK received
 *          3 .. data send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 *          5 .. timeout
 */


// sduino 0.5
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
	(void) wait;	//FIXME: no interrupt support, ignore the wait parameter for now

	SEND_ADDRESS_W(address);
	while (length--) {
		if (twi_sendByte(*data++)) return (3);
	}

	if (sendStop) {
		twi_stop();
	}

	return 0;
}



// TODO: twi_stop can't be used in SRX/STX, because of the first while clause
/*
 * Function twi_stop
 * Desc     relinquishes bus master status
 * Input    none
 * Output   none
 */
void twi_stop(void)
{
	tout_start();

	/* Test on EV8_2: TXE and BTF flags */
//	TIMEOUT_WAIT_FOR_ONE((I2C->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)) ==
//			     (I2C_SR1_TXE | I2C_SR1_BTF));
	while ((I2C->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)) != (I2C_SR1_TXE | I2C_SR1_BTF))
	{
		if (tout())
    {
      twi_lockupTOREMOVE();
      return; 	// don't update twi_state
		}
	}

	/* Generate a STOP condition */
	I2C->CR2 |= I2C_CR2_STOP;

	// wait for the end of the STOP condition
	//
	// The reference manual rm0016 is not clear on how to check for this
	// condition. Maybe BUSY, BTF, TRA or even MSL.
	// Waiting for BTF works.
	// AN3281, Fig. 4 specifies to wait for STOPF, but that does not work.
	// The source code attached to AN3281 waits for the STOP bit in CR2
	// to flip back to zero. This works, so this method is used.
//	TIMEOUT_WAIT_FOR_ONE((I2C->SR1 & I2C_SR1_BTF), 7);	// works
//	TIMEOUT_WAIT_FOR_ONE((I2C->SR1 & I2C_SR1_STOPF), 7);	// doesn't work
//	TIMEOUT_WAIT_FOR_ZERO(I2C->CR2 & I2C_CR2_STOP);	// works
	while (I2C->CR2 & I2C_CR2_STOP)		// works
	{
		if (tout())
        	{
	        	twi_lockupTOREMOVE();
	        	return; 	// don't update twi_state
		}
	}

/*
  // send stop condition
  TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

  // wait for stop condition to be exectued on bus
  // TWINT is not set after a stop condition!
  while(TWCR & _BV(TWSTO)){
    continue;
  }
*/

  // update twi state
  twi_state = TWI_READY;
}


/* --- SDuino additions -------------------------------------------------- */

/**
 * send start condition and the target address and wait for the ADDR event
 *
 * The flag handling for POS and ACK is determined by the mode byte.
 * At the end, ADDR is cleared by reading SR3.
 *
 * @parms mode: set the flag handling for POS and ACK
 *		1: clear ACK in ADDR event, before clearing ADDR (receive 1)
 *		2: set ACK, POS before ADDR event (receive 2)
 *		3: set ACK before ADDR event (receive > 2, write)
 * @returns: 0 .. success
 *          2 .. address send, NACK received
 *          4 .. other twi error (lost bus arbitration, bus error, ..)
 */
static uint8_t twi_sendAddress(uint8_t i2cAddress, uint8_t mode)
{
	tout_start();

	/* do not wait for BUSY==0 as this would block for repeated start */
//	TIMEOUT_WAIT_FOR_ZERO((I2C->SR3 & I2C_SR3_BUSY), 4);

	I2C->CR2 |= I2C_CR2_ACK;	// set ACK
	/* send start sequence */
	I2C->CR2 |= I2C_CR2_START;	// send start sequence

	/* Test on EV5 and clear it (SB flag) */
	TIMEOUT_WAIT_FOR_ONE(I2C->SR1 & I2C_SR1_SB, 4);

	/* Send the Address + Direction */
	I2C->DR = i2cAddress;	// I2C_Send7bitAddress()

	/* Test on EV6, but don't clear it yet (ADDR flag) */
	// error code 2: no ACK received on address transmission
	TIMEOUT_WAIT_FOR_ONE(I2C->SR1 & I2C_SR1_ADDR, 2);

	if (mode == 1) {
		I2C->CR2 &= ~I2C_CR2_ACK;	// clear ACK
		BEGIN_CRITICAL			// disable interrupts
		(void) I2C->SR3;		// read SR3 to clear ADDR event bit
		I2C->CR2 |= I2C_CR2_STOP;	// send STOP soon
		END_CRITICAL			// enable interrupts
	} else if (mode == 2) {
		I2C->CR2 |= I2C_CR2_POS;	// set POS
		BEGIN_CRITICAL			// disable interrupts
		(void) I2C->SR3;		// read SR3 to clear ADDR event bit
		I2C->CR2 &= ~I2C_CR2_ACK;	// clear ACK
		END_CRITICAL			// enable interrupts
	} else {
		(void)I2C->SR3;		// read SR3 to clear ADDR event bit
	}

	return 0;
}

/**
 * send one data byte via I2C (blocking)
 *
 * @returns:0 .. success
 *          3 .. data send, NACK received
 */
uint8_t twi_sendByte(uint8_t i2cData)
{
	tout_start();

	/* Test on EV8 (wait for TXE flag) */
	/* On fail: 3: no ACK received on data transmission */
	TIMEOUT_WAIT_FOR_ONE(I2C->SR1 & I2C_SR1_TXE, 3);

	I2C->DR = i2cData;
	return 0;
}

static void twi_lockupTOREMOVE(void)
{
#if 0
	TWCR = 0;		//releases SDA and SCL lines to high impedance
	TWCR = _BV(TWEN) | _BV(TWEA);	//reinitialize TWI 
#endif
	//FIXME: this needs to be checked in detail. CR1 might be involved
	// don't do a full software reset here. That would require a full
	// re-initialization before the next transfer could happen.
	I2C->CR2 = 0;

  // TODO : in current sduino code, we come here after e.g. ADDR NACK 
  // that's not a lockup
  // wat is full software reset? STM8 reset? en waarom geen SWRST van TWI?
  // we zouden hier om te beginnen een while(1) kunnen zetten met een blinkie
}


/**
 * start the timeout timer
 *
 * The current time is set as the starting time.
 */
static void tout_start(void)
{
	startingTime = millis();
	timeout_expired = false;
}

/**
 * check if the timeout period expired
 *
 * @returns:
 *   false: Still within the waiting period
 *   true: timeout expired
 */
static bool tout(void)
{
	if (!twi_timeOutDelay)
	{
		return false;	// no timeout set
	}
	if (!timeout_expired)
	{
		timeout_expired =
		((((uint16_t)millis()) - startingTime) >= twi_timeOutDelay);
	}
	return timeout_expired;
}
