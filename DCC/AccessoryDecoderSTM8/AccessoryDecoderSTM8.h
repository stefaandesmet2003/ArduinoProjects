
#ifndef AccessoryDecoder_h
#define AccessoryDecoder_h

#define DEBUG // adds debug prints on serial

//a manufacturer CV for this decoder
#define CV_SOFTWARE_MODE   33
               
// values for CV33
#define SOFTWAREMODE_TURNOUT_DECODER    0
#define SOFTWAREMODE_OUTPUT_DECODER     1

// hardware definitions
#ifdef DEBUG
  #define KEYPINS     {PD4,PA1,PA1,PA1} // not using KEY2/KEY3 in debug, need serial pins
#else
  #define KEYPINS     {PD4,PD5,PD6,PA1}
#endif
#define OUTPUTPINS  {PD3,PD2,PD1,PC7,PC6,PC6,PC4,PC3}

#define PIN_PROGLED PB5
#define PIN_ACKOUT  PA3

// led flash modes
#define LED_SLOW_FLASH  1000
#define LED_FAST_FLASH  200

uint16_t getDecoderAddress();

#endif //AccessoryDecoder_h
