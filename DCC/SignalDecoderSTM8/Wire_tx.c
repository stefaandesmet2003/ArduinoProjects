/*
  TwoWire.cpp - TWI/I2C library for Wiring & Arduino
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

  Modified 2017 by Michael Mayer to plain C for use with Sduino
  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
*/

//extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include "twi_tx.h"
//}

#include "Wire_tx.h"

static uint8_t txAddress = 0;
static uint8_t txBuffer[BUFFER_LENGTH];
static uint8_t txBufferIndex = 0;
static uint8_t txBufferLength = 0;

// Public Methods //////////////////////////////////////////////////////////////

void Wire_begin(void) {
  txBufferIndex = 0;
  txBufferLength = 0;

  twi_init();
}

void Wire_end(void) {
	twi_disable();
}

void Wire_beginTransmission(uint8_t address) {
  // set address of targeted slave
  txAddress = address;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
}

uint8_t Wire_endTransmission(void) {
  // transmit buffer (blocking)
  uint8_t ret = twi_writeTo(txAddress, txBuffer, txBufferLength, 1, true);
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  return ret;
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
size_t Wire_write(uint8_t data) {
  // in master transmitter mode
  // don't bother if buffer is full
  if(txBufferLength >= BUFFER_LENGTH){
    return 0;
  }
  // put byte in tx buffer
  txBuffer[txBufferIndex] = data;
  ++txBufferIndex;
  // update amount in buffer
  txBufferLength = txBufferIndex;
  return 1;
}
