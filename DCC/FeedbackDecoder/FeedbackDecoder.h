
#ifndef _FEEDBACKDECODER_H_
#define _FEEDBACKDECODER_H_

//SDS manufacturer CV's
#define CV_NUMBER_OF_EXPANDERS  33
#define CV_FEEDBACK_DELAY_MS    34
#define CV_XPNET_ADDRESS        35

// hardware definitions
#define NUMBER_OF_KEYS              1   // only the programming key
#define PIN_PROGLED                 13  // reuse builtin led
#define PIN_PROGKEY                 A1
#define PIN_ACKOUT                  A3
#define PIN_DCCIN                   2   // dcc input
#define RS485_DIRECTION_PIN         12  // xpnet rs485
#define EXPANDER_BASE_I2C_ADDRESS   0x20

//#define LOCAL_INPUTS {PD3,PD2,PD1,PC7,PC6,PC5,PC4,PC3} // STM8 config
#define LOCAL_INPUTS {11,10,7,6,5,4,3,A2} // nano test config

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

#endif // _FEEDBACKDECODER_H_
