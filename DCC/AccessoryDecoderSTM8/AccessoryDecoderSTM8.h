
#ifndef AccessoryDecoder_h
#define AccessoryDecoder_h

#define DEBUG // adds debug prints on serial

//SDS manufacturer CV's
#define CV_SoftwareMode   33
               
// values for CV33
#define SOFTWAREMODE_TURNOUT_DECODER    0
#define SOFTWAREMODE_OUTPUT_DECODER     1

// hardware definitions? (of is dit afhankelijk van softwaremode?)
// hardware-def van de turnout-decoder
#ifdef DEBUG
  #define PIN_KEY2    PA1 // not using KEY2/KEY3 in debug, need serial pins
  #define PIN_KEY3    PA1 // not using KEY2/KEY3 in debug, need serial pins
#else
  #define PIN_KEY2    PA1 // PD5
  #define PIN_KEY3    PD6 // PD6 is correct
#endif
#define PIN_KEY4    PA1
#define PIN_OUTPUT0 PD3
#define PIN_OUTPUT1 PD2
#define PIN_OUTPUT2 PD1
#define PIN_OUTPUT3 PC7
#define PIN_OUTPUT4 PC6
#define PIN_OUTPUT5 PC5
#define PIN_OUTPUT6 PC4
#define PIN_OUTPUT7 PC3
//progled en progkey
#define PIN_PROGKEY PD4
#define PIN_PROGLED PB5
#define PIN_ACKOUT  PA3

//keycodes
#define KEYEVENT_UP         0x0
#define KEYEVENT_DOWN       0x1
#define KEYEVENT_LONGDOWN   0x2
#define KEY_PROG            (0x0 << 2)
#define KEY_KEY2            (0x1 << 2)
#define KEY_KEY3            (0x2 << 2)
#define KEY_KEY4            (0x3 << 2)
//other keys here 

// led flash modes
#define LED_SLOW_FLASH  1000
#define LED_FAST_FLASH  200

typedef struct {
  uint16_t  CV;
  uint8_t   Value;
} CVPair;

uint16_t getDecoderAddress();
#endif //AccessoryDecoder_h
