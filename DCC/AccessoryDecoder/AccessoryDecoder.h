
#ifndef AccessoryDecoder_h
#define AccessoryDecoder_h

#define DEBUG // adds debug prints on serial

// common definitions for alle accessory decoder types

// Uncomment to print all DCC Packets
//#define NOTIFY_DCC_MSG

//SDS manufacturer CV's
#define CV_SoftwareMode   33
               
// values for CV33
#define SOFTWAREMODE_TURNOUT_DECODER    0
#define SOFTWAREMODE_LIGHT_DECODER      1

// hardware definitions? (of is dit afhankelijk van softwaremode?)
// hardware-def van de turnout-decoder
#define PIN_KEY2    17 // A3 , omdat D13 niet werkte (moet blijkbaar met externe pulldown omwille van de LED)
#define PIN_KEY3    12 // D12
#define PIN_KEY4    11 // D11
#define PIN_OUTPUT0 10 // D10
#define PIN_OUTPUT1 9 // D9
#define PIN_OUTPUT2 8 // D8
#define PIN_OUTPUT3 7 // D7
#define PIN_OUTPUT4 6 // D6
#define PIN_OUTPUT5 5 // D5
#define PIN_OUTPUT6 4 // D4
#define PIN_OUTPUT7 3 // D3
//progled en progkey
#define PIN_PROGKEY 14 // A0
#define PIN_PROGLED 15 // A1
#define PIN_ACKOUT  16 // A2

#define PIN_DCCIN   2 // D2

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

#endif //AccessoryDecoder_h
