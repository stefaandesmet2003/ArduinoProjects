
#ifndef _DECODER_H_
#define _DECODER_H_

#ifdef ARDUINO_AVR_ATTINYX5
  #undef DEBUG
#else
  #define DEBUG // adds debug prints on serial
#endif


typedef struct {
  uint16_t  CV;
  uint8_t   Value;
} CVPair;


#endif //_DECODER_H_
