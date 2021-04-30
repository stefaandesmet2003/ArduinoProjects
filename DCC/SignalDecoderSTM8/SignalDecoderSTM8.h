
#ifndef _SIGNALDECODER_H_
#define _SIGNALDECODER_H_

//#define DEBUG // adds debug prints on serial

// common definitions for alle accessory decoder types

//SDS manufacturer CV's
#define CV_SoftwareMode   33  // not used yet for signal decoder
               
// values for CV33

// hardware definitions (of is dit afhankelijk van softwaremode?)
//progled en progkey
#define PIN_PROGLED PB5 // reuse builtin led
#define PIN_PROGKEY PA1
#define PIN_ACKOUT  PA3
// dcc input
#define PIN_DCCIN   PA2

// signal decoder specific
#define EXPANDER_I2C_ADDRESS  0x20 // voorlopig 1 expander
// todo output definitions here??

//keycodes
#define KEYEVENT_UP         0x0
#define KEYEVENT_DOWN       0x1
#define KEYEVENT_LONGDOWN   0x2
#define KEY_PROG            (0x0 << 2)
#define KEY_KEY2            (0x1 << 2)
#define KEY_KEY3            (0x2 << 2)
#define KEY_KEY4            (0x3 << 2)
// TODO : beetje vies want deze KEY_xxx worden rechtstreeks als outputId gebruikt ook ...
//other keys here 

// led flash modes
#define LED_SLOW_FLASH  1000
#define LED_FAST_FLASH  200

typedef struct {
  uint16_t  CV;
  uint8_t   Value;
} CVPair;

#endif // _SIGNALDECODER_H_
