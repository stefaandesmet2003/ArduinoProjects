
#ifndef _DECODER_H_
#define _DECODER_H_

#ifdef ARDUINO_AVR_ATTINYX5
  #undef DEBUG
#else
  #define DEBUG // adds debug prints on serial
#endif


#define PIN_DCCIN   2 // D2

// Uncomment to print all DCC Packets
//#define NOTIFY_DCC_MSG

//SDS manufacturer CV's


typedef struct {
  uint16_t  CV;
  uint8_t   Value;
} CVPair;

uint16_t getDecoderAddress();

#endif //_DECODER_H_
