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
//
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			  and build DCC MultiFunction and Stationary Decoders
//
//------------------------------------------------------------------------

#ifndef __NMRADCC_H__
#define __NMRADCC_H__

// these defines statically config the library

#define NMRA_DCC_MOBILE_DECODER // handle packets addressed to mobile decoder addresses
//#define NMRA_DCC_ACCESSORY_DECODER // handle packets addressed to accessory decoder addresses
// both defines can coexist, but some address CV settings are incompatible between MOB & ACC
#define NMRA_DCC_PROCESS_SERVICEMODE // handle service mode packets
#define NMRA_DCC_PROCESS_POM // handle programming on main packets

#include "Arduino.h"

#define MAX_DCC_MESSAGE_LEN 6    // including XOR-Byte

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

class NmraDcc {
  private:
    DCC_MSG Msg;
    
  public:
    NmraDcc();

  void init (uint8_t ExtIntNum = 0, uint8_t ExtIntPinNum = 2, uint8_t EnablePullup = true);
  uint8_t process();
  uint8_t getCV (uint16_t CV);
  uint8_t setCV (uint16_t CV, uint8_t Value);
  bool isSetCVReady ();
  void setFilterAddress(uint16_t dccAddress); // let the lib filter this particular dcc address, only needed if FLAGS_MY_ADDRESS_ONLY flag is used
  uint16_t readDccAddress(); // read from CV's
  void writeDccAddress(uint16_t dccAddress); // write to CV's
};

// callbacks
extern void notifyDccReset(uint8_t hardReset) __attribute__ ((weak));
extern void notifyDccIdle() __attribute__ ((weak));
extern void notifyDccSpeed(uint16_t decoderAddress, uint8_t speed, bool directionIsForward, uint8_t dccSpeedType) __attribute__ ((weak));
extern void notifyDccFunc( uint16_t decoderAddress, FN_GROUP FuncGrp, uint8_t FuncState) __attribute__ ((weak));

extern void notifyDccAccState(uint16_t decoderAddress, uint8_t outputId, bool activate) __attribute__ ((weak));
extern void notifyDccSigState(uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) __attribute__ ((weak));

extern void notifyDccMsg(DCC_MSG * msg) __attribute__ ((weak));
extern void	notifyDccNop(uint16_t decoderAddress) __attribute__ ((weak)); // RCN-213 NOP packet for accessories

extern bool     notifyCVValid(uint16_t cv, bool writable) __attribute__ ((weak));
extern uint8_t  notifyCVWrite( uint16_t cv, uint8_t cvValue) __attribute__ ((weak));
extern void    	notifyCVAck() __attribute__ ((weak));
extern uint8_t 	notifyCVRead(uint16_t cv) __attribute__ ((weak));
extern bool 		notifyIsSetCVReady() __attribute__ ((weak));
extern void    	notifyCVChange(uint16_t cv, uint8_t cvValue) __attribute__ ((weak));
#endif // __NMRADCC_H__
