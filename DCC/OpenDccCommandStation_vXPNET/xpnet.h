//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2008 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      xpnet.h
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   xpressnet Interface (protocol layer)

#define ACK_ID      0x00
#define FUTURE_ID   0x20        // A message with future ID to slot is Feedback broadcast
#define CALL_ID     0x40
#define MESSAGE_ID  0x60

typedef struct {
  unsigned char statusChanged: 1; // if != 0: there was a state change
                                  // set by status_SetState - cleared by xpnet parser
  unsigned char clockChanged: 1;  // there was a minute tick for DCC Layout time
                                  // set by fast_clock - cleared by xpressnet master
} xpnet_Event_t;

extern xpnet_Event_t xpnet_Event;


void xpnet_Init();
void xpnet_Run();                       // multitask replacement
void xpnet_SendMessage(unsigned char callByte, unsigned char *str);   // send a message with this callbyte (ID+slot)

// TODO SDS2021 : is dit de beste oplossing als een lok wordt gestolen door de UI?
// nu wordt locobuffer.owner_changed flag niet meer gebruikt, maar wel die mottige orgz_old_owner
void xpnet_SendLocStolen(unsigned char slot, unsigned int locAddress);
