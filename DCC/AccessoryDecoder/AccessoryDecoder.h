
#ifndef AccessoryDecoder_h
#define AccessoryDecoder_h

#define DEBUG // adds debug prints on serial

// Uncomment to print all DCC Packets
//#define NOTIFY_DCC_MSG

//a manufacturer CV for this decoder
#define CV_SOFTWARE_MODE   33
               
// values for CV33
#define SOFTWAREMODE_TURNOUT_DECODER    0 // standard turnout decoder for 4 turnouts
#define SOFTWAREMODE_OUTPUT_DECODER     1 // decoder for 8 individual outputs, eg. light outputs, decoupler rails etc.

// hardware definitions
#define KEYPINS     {14,17,12,11}
#define OUTPUTPINS  {10,9,8,7,6,5,4,3}

#define PIN_PROGLED 15 // A1
#define PIN_ACKOUT  16 // A2
#define PIN_DCCIN   2 // D2

// led flash modes
#define LED_SLOW_FLASH  1000
#define LED_FAST_FLASH  200

uint16_t getDecoderAddress();

#endif //AccessoryDecoder_h
