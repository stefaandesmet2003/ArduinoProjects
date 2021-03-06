/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
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
  Modified 2017 by Michael Mayer to plain C for use with Sduino
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <stdint.h>
#include <stdbool.h>
//#include "Stream.h"

#define BUFFER_LENGTH 32

// WIRE_HAS_END means Wire has end()
#define WIRE_HAS_END 1


/* only this minimal interface is currently implemented: */
void Wire_begin(void);
void Wire_end(void);

void Wire_beginTransmission(uint8_t);
uint8_t Wire_endTransmission();

size_t Wire_write(uint8_t);

#endif
