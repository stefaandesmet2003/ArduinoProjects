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

// Uncomment the following Line to Enable Service Mode CV Programming
#define NMRA_DCC_PROCESS_SERVICEMODE

// Uncomment the following line to Enable MultiFunction Decoder Operations
//#define NMRA_DCC_PROCESS_MULTIFUNCTION

#include "Arduino.h"

#define PIN_DCCIN   				PA2  // fixed DCC-IN pin for STM8 to make our life easier
#define MAX_DCC_MESSAGE_LEN 6    // including XOR-Byte

typedef struct {
  uint8_t	Size;
  uint8_t	PreambleBits;
  uint8_t Data[MAX_DCC_MESSAGE_LEN];
} DCC_MSG;

//--------------------------------------------------------------------------
//  This section contains the NMRA Assigned DCC Manufacturer Id Codes that
//  are used in projects
//
//  This value is to be used for CV8 
//--------------------------------------------------------------------------
#define MAN_ID_JMRI             0x12
#define MAN_ID_DIY              0x0D
#define MAN_ID_SILICON_RAILWAY  0x21
//--------------------------------------------------------------------------
//  This section contains the Product/Version Id Codes for projects
//
//  This value is to be used for CV7 
//
//  NOTE: Each Product/Version Id Code needs to be UNIQUE for that particular
//  the DCC Manufacturer Id Code
//--------------------------------------------------------------------------
// VERSION_ID -> define in your sketch, and write to CV_VERSION_ID

// Standard CV Addresses
#define CV_ACCESSORY_DECODER_ADDRESS_LSB       	1
#define CV_ACCESSORY_DECODER_ADDRESS_MSB       	9

#define CV_MULTIFUNCTION_PRIMARY_ADDRESS       	1
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB 	17
#define CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB 	18

#define CV_VERSION_ID                          	7
#define CV_MANUFACTURER_ID                     	8
#define CV_DECODER_CONFIGURATION               	29
#define CV_RAILCOM_CONFIGURATION								28

#define MAXCV                                 	E2END     // the upper limit of the CV value currently defined to max memory.

typedef enum {
  FN_0_4 = 1,
  FN_5_8,
  FN_9_12,
  FN_13_20,
  FN_21_28
} FN_GROUP;

#define FN_BIT_00	0x10
#define FN_BIT_01	0x01
#define FN_BIT_02	0x02
#define FN_BIT_03	0x04
#define FN_BIT_04	0x08

#define FN_BIT_05	0x01
#define FN_BIT_06	0x02
#define FN_BIT_07	0x04
#define FN_BIT_08	0x08

#define FN_BIT_09	0x01
#define FN_BIT_10	0x02
#define FN_BIT_11	0x04
#define FN_BIT_12	0x08

#define FN_BIT_13	0x01
#define FN_BIT_14	0x02
#define FN_BIT_15	0x04
#define FN_BIT_16	0x08
#define FN_BIT_17	0x10
#define FN_BIT_18	0x20
#define FN_BIT_19	0x40
#define FN_BIT_20	0x80

#define FN_BIT_21	0x01
#define FN_BIT_22	0x02
#define FN_BIT_23	0x04
#define FN_BIT_24	0x08
#define FN_BIT_25	0x10
#define FN_BIT_26	0x20
#define FN_BIT_27	0x40
#define FN_BIT_28	0x80

// Flag values to be logically ORed together and passed into the init() method
#define FLAGS_MY_ADDRESS_ONLY				0x01	// Only process DCC Packets with My Address
#define FLAGS_OUTPUT_ADDRESS_MODE		0x40  // CV 29/541 bit 6
#define FLAGS_DCC_ACCESSORY_DECODER	0x80  // CV 29/541 bit 7

// pin + init, remove constructor was empty anyway
void 		DCC_init(uint8_t Flags, uint8_t OpsModeAddressBaseCV, bool ExtIntPinPullupEnable);
uint8_t DCC_process();
uint8_t DCC_getCV(uint16_t CV);
uint8_t DCC_setCV(uint16_t CV, uint8_t Value);
bool    DCC_isSetCVReady(void);
void 		DCC_setAddress(uint16_t decoderAddress); // only needed if FLAGS_MY_ADDRESS_ONLY flag is used
void    DCC_setFilterAddress(uint16_t dccAddress); // let the lib filter this particular dcc address, only needed if FLAGS_MY_ADDRESS_ONLY flag is used
uint16_t DCC_readDccAddress(); // read from CV's
void    DCC_writeDccAddress(uint16_t dccAddress); // write to CV's

// TODO : rename DCC_resetNotify, etc
void notifyDccReset(uint8_t hardReset);
void notifyDccIdle();
void notifyDccSpeed(uint16_t decoderAddress, uint8_t Speed, uint8_t ForwardDir, uint8_t MaxSpeed);
void notifyDccFunc(uint16_t Addr, FN_GROUP FuncGrp, uint8_t FuncState);
void notifyDccAccState(uint16_t decoderAddress, uint8_t outputId, bool activate);
void notifyDccSigState(uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect);

void notifyDccMsg(DCC_MSG * Msg);
// not used for now
//void notifyDccNop(uint16_t decoderAddress)); // RCN-213 NOP packet for accessories

bool    notifyCVValid(uint16_t cv, uint8_t writable);
uint8_t notifyCVWrite(uint16_t cv, uint8_t cvValue);
void    notifyCVAck();
// uint8_t notifyCVRead( uint16_t cv);
// bool    notifyIsSetCVReady();
// void    notifyCVChange(uint16_t cv, uint8_t cvValue);

#endif // __NMRADCC_H__
