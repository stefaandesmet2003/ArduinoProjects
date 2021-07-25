//------------------------------------------------------------------------
//
// OpenDCC - NmraDcc.h 
//
// Copyright (c) 2008 Alex Shepherd
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      NmraDcc.h
// author:    Alex Shepherd
// webpage:   http://opendcc.org/
// history:   2008-03-20 Initial Version
//						2021 - adapted for STM8
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			  and build DCC MultiFunction and Stationary Decoders
//
//------------------------------------------------------------------------
#ifndef __NMRADCC_H__
#define __NMRADCC_H__

// these defines statically config the library

//#define NMRA_DCC_MOBILE_DECODER // handle packets addressed to mobile decoder addresses
#define NMRA_DCC_ACCESSORY_DECODER // handle packets addressed to accessory decoder addresses
// both defines can coexist, but some address CV settings are incompatible between MOB & ACC
#define NMRA_DCC_PROCESS_SERVICEMODE // handle service mode packets
#define NMRA_DCC_PROCESS_POM // handle programming on main packets
//#define NMRA_DCC_PROCESS_FASTCLOCK // handle fast clock dcc packets

#include "Arduino.h"

#define PIN_DCCIN   				PA2  // fixed DCC-IN pin for STM8 to make our life easier
#define MAX_DCC_MESSAGE_LEN 7    // including XOR-Byte, (note SDS fast clock message has 7 bytes)

typedef struct {
  uint8_t	Size;
  uint8_t	PreambleBits;
  uint8_t Data[MAX_DCC_MESSAGE_LEN];
} DCC_MSG;

// Standard CV Addresses
#define CV_ACCESSORY_DECODER_ADDRESS_LSB       	1
#define CV_ACCESSORY_DECODER_ADDRESS_MSB       	9

#define CV_MULTIFUNCTION_PRIMARY_ADDRESS       	1   // 7-bit address, msb=0
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB 	17
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB 	18

#define CV_VERSION_ID                          	7
#define CV_MANUFACTURER_ID                     	8
#define CV_DECODER_CONFIGURATION               	29
#define CV_RAILCOM_CONFIGURATION								28

#define MAXCV                                 	E2END     // the upper limit of the CV value currently defined to max memory.

// CV8 : NMRA Assigned DCC Manufacturer Id Codes
#define MAN_ID_JMRI             0x12
#define MAN_ID_DIY              0x0D
#define MAN_ID_SILICON_RAILWAY  0x21

// types of dcc speed packets supported
#define DCC_SPEED_28STEPS   1
#define DCC_SPEED_128STEPS  2

typedef enum {
  FN_0_4 = 1,
  FN_5_8,
  FN_9_12,
  FN_13_20,
  FN_21_28
} FN_GROUP;

typedef struct {
  uint8_t minute; 
  uint8_t hour;
  uint8_t day_of_week;
  uint8_t ratio;
} dccFastClock_t;

// pin + init, remove constructor was empty anyway
void 		DCC_init();
uint8_t DCC_process();
uint8_t DCC_getCV(uint16_t CV);
uint8_t DCC_setCV(uint16_t CV, uint8_t Value);
bool    DCC_isSetCVReady(void);
void    DCC_setFilterAddress(uint16_t dccAddress); // let the lib filter this particular dcc address, only needed if FLAGS_MY_ADDRESS_ONLY flag is used
uint16_t DCC_readDccAddress(); // read from CV's
void    DCC_writeDccAddress(uint16_t dccAddress); // write to CV's

void notifyDccReset(uint8_t hardReset);
void notifyDccIdle();
void notifyDccMsg(DCC_MSG * Msg);
// not used for now
//void notifyDccNop(uint16_t decoderAddress)); // RCN-213 NOP packet for accessories

#ifdef NMRA_DCC_MOBILE_DECODER
  void notifyDccSpeed(uint16_t decoderAddress, uint8_t speed, bool directionIsForward, uint8_t dccSpeedType);
  void notifyDccFunc( uint16_t decoderAddress, FN_GROUP FuncGrp, uint8_t FuncState);
#endif // NMRA_DCC_MOBILE_DECODER

#ifdef NMRA_DCC_ACCESSORY_DECODER
  void notifyDccAccState(uint16_t decoderAddress, uint8_t outputId, bool activate);
  void notifyDccSigState(uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect);
#endif // NMRA_DCC_ACCESSORY_DECODER


#ifdef NMRA_DCC_PROCESS_SERVICEMODE
  bool notifyCVValid(uint16_t cv, uint8_t writable);
  uint8_t notifyCVWrite(uint16_t cv, uint8_t cvValue);
  void    notifyCVAck();
  // uint8_t notifyCVRead( uint16_t cv);
  // bool    notifyIsSetCVReady();
  // void    notifyCVChange(uint16_t cv, uint8_t cvValue);
#endif // NMRA_DCC_PROCESS_SERVICEMODE

#ifdef NMRA_DCC_PROCESS_POM
  bool notifyPoMValid(uint16_t decoderAddress, uint16_t cv, bool writable);
#endif // NMRA_DCC_PROCESS_POM

#ifdef NMRA_DCC_PROCESS_FASTCLOCK
  void notifyDccFastClock (dccFastClock_t *clock);
#endif

#endif // __NMRADCC_H__
