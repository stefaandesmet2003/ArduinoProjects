/*!
 * @file Adafruit_NeoPixel.cpp
 *
 * @mainpage Arduino Library for driving Adafruit NeoPixel addressable LEDs,
 * FLORA RGB Smart Pixels and compatible devicess -- WS2811, WS2812, WS2812B,
 * SK6812, etc.
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's NeoPixel library for the
 * Arduino platform, allowing a broad range of microcontroller boards
 * (most AVR boards, many ARM devices, ESP8266 and ESP32, among others)
 * to control Adafruit NeoPixels, FLORA RGB Smart Pixels and compatible
 * devices -- WS2811, WS2812, WS2812B, SK6812, etc.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Phil "Paint Your Dragon" Burgess for Adafruit Industries,
 * with contributions by PJRC, Michael Miller and other members of the
 * open source community.
 *
 * @section license License
 *
 * This file is part of the Adafruit_NeoPixel library.
 *
 * Adafruit_NeoPixel is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * Adafruit_NeoPixel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with NeoPixel. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * SDS 2021 : modified for STM8 + SDCC (sduino)
 * support 800kHz / 16MHz only for now
 */

#include "NeoPixel.h"

#ifdef NEO_KHZ400  // If 400 KHz NeoPixel support enabled...
  static bool              is800KHz;   ///< true if 800 KHz pixels
#endif
  static bool              begun;      ///< true if begin() previously called
  static uint16_t          numLEDs;    ///< Number of RGB LEDs in strip
  static uint16_t          numBytes;   ///< Size of 'pixels' buffer below
  static int16_t           pin;        ///< Output pin number (-1 if not yet set)
  static uint8_t           brightness; ///< Strip brightness 0-255 (stored as +1)
  static uint8_t          *pixels;     ///< Holds LED color values (3 or 4 bytes each)
  static uint8_t           rOffset;    ///< Red index within each 3- or 4-byte pixel
  static uint8_t           gOffset;    ///< Index of green byte
  static uint8_t           bOffset;    ///< Index of blue byte
  static uint8_t           wOffset;    ///< Index of white (==rOffset if no white)
  static uint32_t          endTime;    ///< Latch timing reference
  static volatile uint8_t *port;       ///< Output PORT register
  static uint8_t           pinMask;    ///< Output PORT bitmask

// STM8 assembler can only interface with globals
volatile uint8_t
  hi,             // PORT w/output bit set high
  lo;             // PORT w/output bit set low

// SDS TEMP (see note in NeoPixel.h)
uint8_t thePixels[MAX_PIXEL_BYTES];

/*!
  @brief   NeoPixel constructor when length, pin and pixel type are known
           at compile-time.
  @param   n  Number of NeoPixels in strand.
  @param   p  Arduino pin number which will drive the NeoPixel data in.
  @param   t  Pixel type -- add together NEO_* constants defined in
              Adafruit_NeoPixel.h, for example NEO_GRB+NEO_KHZ800 for
              NeoPixels expecting an 800 KHz (vs 400 KHz) data stream
              with color bytes expressed in green, red, blue order per
              pixel.
  @return  Adafruit_NeoPixel object. Call the begin() function before use.
*/
void NeoPixel_init(uint16_t n, uint16_t p, neoPixelType t) {
  begun = false;
  brightness = 0;
  //SDS pixels = NULL;
  pixels = thePixels;
  endTime = 0;
  numLEDs = 0;
  numBytes = 0;
  NeoPixel_updateType(t);
  NeoPixel_updateLength(n);
  NeoPixel_setPin(p);
}

/*!
  @brief   Deallocate Adafruit_NeoPixel object, set data pin back to INPUT.
*/
void NeoPixel_exit() {
  //SDS free(pixels);
  if(pin >= 0) pinMode(pin, INPUT);
}

/*!
  @brief   Configure NeoPixel pin for output.
*/
void NeoPixel_begin(void) {
  if(pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  begun = true;
}

  /*!
    @brief   Check whether a call to show() will start sending data
             immediately or will 'block' for a required interval. NeoPixels
             require a short quiet time (about 300 microseconds) after the
             last bit is received before the data 'latches' and new data can
             start being received. Usually one's sketch is implicitly using
             this time to generate a new frame of animation...but if it
             finishes very quickly, this function could be used to see if
             there's some idle time available for some low-priority
             concurrent task.
    @return  1 or true if show() will start sending immediately, 0 or false
             if show() would block (meaning some idle time is available).
  */
bool NeoPixel_canShow(void) {
    if (endTime > micros()) {
      endTime = micros();
    }
    return (micros() - endTime) >= 300L;
}


/*!
  @brief   Change the length of a previously-declared Adafruit_NeoPixel
           strip object. Old data is deallocated and new data is cleared.
           Pin number and pixel format are unchanged.
  @param   n  New length of strip, in pixels.
  @note    This function is deprecated, here only for old projects that
           may still be calling it. New projects should instead use the
           'new' keyword with the first constructor syntax (length, pin,
           type).
*/
void NeoPixel_updateLength(uint16_t n) {
  //SDS free(pixels); // Free existing data (if any)

  // Allocate new data -- note: ALL PIXELS ARE CLEARED
  numBytes = n * ((wOffset == rOffset) ? 3 : 4);
  //SDS if((pixels = (uint8_t *)malloc(numBytes))) {
  //SDS the fixed pixel array is a temporary solution, so I don't care about range checks! (memset)
  if(pixels) {
    memset(pixels, 0, numBytes);
    numLEDs = n;
  } else {
    numLEDs = numBytes = 0;
  }
}

/*!
  @brief   Change the pixel format of a previously-declared
           Adafruit_NeoPixel strip object. If format changes from one of
           the RGB variants to an RGBW variant (or RGBW to RGB), the old
           data will be deallocated and new data is cleared. Otherwise,
           the old data will remain in RAM and is not reordered to the
           new format, so it's advisable to follow up with clear().
  @param   t  Pixel type -- add together NEO_* constants defined in
              Adafruit_NeoPixel.h, for example NEO_GRB+NEO_KHZ800 for
              NeoPixels expecting an 800 KHz (vs 400 KHz) data stream
              with color bytes expressed in green, red, blue order per
              pixel.
  @note    This function is deprecated, here only for old projects that
           may still be calling it. New projects should instead use the
           'new' keyword with the first constructor syntax
           (length, pin, type).
*/
void NeoPixel_updateType(neoPixelType t) {
  bool oldThreeBytesPerPixel = (wOffset == rOffset); // false if RGBW

  wOffset = (t >> 6) & 0b11; // See notes in header file
  rOffset = (t >> 4) & 0b11; // regarding R/G/B/W offsets
  gOffset = (t >> 2) & 0b11;
  bOffset =  t       & 0b11;
#if defined(NEO_KHZ400)
  is800KHz = (t < 256);      // 400 KHz flag is 1<<8
#endif

  // If bytes-per-pixel has changed (and pixel data was previously
  // allocated), re-allocate to new size. Will clear any data.
  if(pixels) {
    bool newThreeBytesPerPixel = (wOffset == rOffset);
    if(newThreeBytesPerPixel != oldThreeBytesPerPixel) NeoPixel_updateLength(numLEDs);
  }
}

/*!
  @brief   Transmit pixel data in RAM to NeoPixels.
  @note    On most architectures, interrupts are temporarily disabled in
           order to achieve the correct NeoPixel signal timing. This means
           that the Arduino millis() and micros() functions, which require
           interrupts, will lose small intervals of time whenever this
           function is called (about 30 microseconds per RGB pixel, 40 for
           RGBW pixels). There's no easy fix for this, but a few
           specialized alternative or companion libraries exist that use
           very device-specific peripherals to work around it.
*/
void NeoPixel_show(void) {

  if(!pixels) return;

  // Data latch = 300+ microsecond pause in the output stream. Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed. This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while(!NeoPixel_canShow());
  // endTime is a private member (rather than global var) so that multiple
  // instances on different pins can be quickly issued in succession (each
  // instance doesn't delay the next).

  // In order to make this code runtime-configurable to work with any pin,
  // SBI/CBI instructions are eschewed in favor of full PORT writes via the
  // OUT or ST instructions. It relies on two facts: that peripheral
  // functions (such as PWM) take precedence on output pins, so our PORT-
  // wide writes won't interfere, and that interrupts are globally disabled
  // while data is being issued to the LEDs, so no other code will be
  // accessing the PORT. The code takes an initial 'snapshot' of the PORT
  // state, computes 'pin high' and 'pin low' values, and writes these back
  // to the PORT register as needed.

  disableInterrupts(); // Need 100% focus on instruction timing

  hi   = *port |  pinMask;
  lo   = *port & ~pinMask;

  __asm
      ldw y, _pixels      ; // ptr naar pixels in Y index register
      ld a, (y)
      push a              ; // current pixel byte -> (5,sp), will be left shifting bits out to carry bit for test
      ldw x, _numBytes    ; 
      pushw x             ; // -> (3,sp) - 2 bytes!
      push #8             ; // bit counter -> (2,sp)
      push _lo            ; // _next op stack zetten -> (1,sp)

    0020$:                ; // head20 (avr ref)
      ldw x, _port        ; // ptr naar IOmem port
      ld a, _hi
      ld (x), a           ; // PORT=hi
      nop                 ; // increase T0H to 6cy = 375ns
      sla (5,sp)          ; // carry = bit to test; 0 -> TO signal, 1 -> T1 signal
      jrnc 0021$          ; // if (bit!=1)
      ld (1,sp), a        ; // next=hi

    0021$:                ; 
      ld a, (1,sp)
      ld (x), a           ; // PORT=next
      nop                 ; // increase T1H to 13cy
      nop
      nop
      ld a, _lo           ; 
      ld (1,sp), a        ; // next=lo
      ld (x), a           ; // PORT = lo
      dec (2,sp)       ; // bit--
      jrne 0020$          ; // if (bit!=0) from dec above, 0020$ = next bit

    0022$:                ; //nextbyte20
      ld a, #8
      ld (2,sp), a        ; // bit = 8
      incw y              ; // ptr++
      ld a, (y)
      ld (5,sp), a        ; // current pixel byte in SP+4
      ldw x, (3,sp)
      decw x              ; // numBytes-- ;16-bit dec only on X,Y
      ldw (3,sp), x
      jrne 0020$          ; // if (i!=0) -> next byte
      addw sp, #5         ; // 4 bytes terug van de stack halen, hoeft niet met afzonderlijke pop instructions

  __endasm;

  enableInterrupts();
  endTime = micros(); // Save EOD time for latch on next call
}

/*!
  @brief   Set/change the NeoPixel output pin number. Previous pin,
           if any, is set to INPUT and the new pin is set to OUTPUT.
  @param   p  Arduino pin number (-1 = no pin).
*/
void NeoPixel_setPin(uint16_t p) {
  if(begun && (pin >= 0)) pinMode(pin, INPUT);
  pin = p;
  if(begun) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  port    = portOutputRegister(digitalPinToPort(p));
  pinMask = digitalPinToBitMask(p);
}

/*!
  @brief   Set a pixel's color using separate red, green and blue
           components. If using RGBW pixels, white will be set to 0.
  @param   n  Pixel index, starting from 0.
  @param   r  Red brightness, 0 = minimum (off), 255 = maximum.
  @param   g  Green brightness, 0 = minimum (off), 255 = maximum.
  @param   b  Blue brightness, 0 = minimum (off), 255 = maximum.
*/
void NeoPixel_setPixelColor(
 uint16_t n, uint8_t r, uint8_t g, uint8_t b) {

  if(n < numLEDs) {
    if(brightness) { // See notes in setBrightness()
      r = (r * brightness) >> 8;
      g = (g * brightness) >> 8;
      b = (b * brightness) >> 8;
    }
    uint8_t *p;
    if(wOffset == rOffset) { // Is an RGB-type strip
      p = &pixels[n * 3];    // 3 bytes per pixel
    } else {                 // Is a WRGB-type strip
      p = &pixels[n * 4];    // 4 bytes per pixel
      p[wOffset] = 0;        // But only R,G,B passed -- set W to 0
    }
    p[rOffset] = r;          // R,G,B always stored
    p[gOffset] = g;
    p[bOffset] = b;
  }
}

/*!
  @brief   Fill the whole NeoPixel strip with 0 / black / off.
*/
void NeoPixel_clear(void) {
  memset(pixels, 0, numBytes);
}

