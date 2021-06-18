//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      status.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2007-10-15 V0.02 key_go_at_start_pressed neu dazu.
//            2009-06-23 V0.03 added clock to status event
//            2009-06-23 V0.04 added read_occ to status event -> sds removed
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   control of LED
//            timertick
//            check for key and output short
//
// 2do:       
//-----------------------------------------------------------------
#ifndef __STATUS_H__
#define __STATUS_H__

/* TODO SDS2021
 * - onderscheid ts RUN_STOP & RUN_PAUSE nodig??
 * - onderscheid ts RUN_SHORT & RUN_OFF nodig??
 * RUN_STOP is compatibel met xpnet spec (emergency off), 
 * RUN_PAUSE is eventueel een smoothere stop van de baan (locs bollen uit ipv direct te stoppen)?
 * RUN_PAUSE wordt nog niet gebruikt
 * onderscheid RUN_OFF & RUN_SHORT is niet zo duidelijk, OFF = gevraagd door UI bv, en SHORT is externe oorzaak?
 * maar is dat onderscheid nodig in status??
*/
typedef enum {
  INIT,
  RUN_OKAY,     // DCC Running
  RUN_STOP,     // DCC Running, all Engines Emergency Stop
  RUN_OFF,      // Output disabled (2*Taste, PC)
  RUN_SHORT,    // Kurzschluss;
  RUN_PAUSE,    // DCC Running, all Engines Speed 0
  PROG_OKAY,    // Prog-Mode running 
  PROG_SHORT,   // Prog-Mode + Kurzschluss
  PROG_OFF,     // Prog-Mode, abgeschaltet
  PROG_ERROR    // Prog-Mode, Fehler beim Programmieren
} t_opendcc_state;

// TODO SDS2021 : nog enkel nodig voor lenz_parser
typedef struct {      // each member will be decremented on every tick until 0
  unsigned int parser;       // high level parser timeout
} t_no_timeout;

// Interface for OpenDCC-Parser
typedef struct 
{
  unsigned char changed: 1;       // if != 0: there was (could be) a state change
                                  // set by status_SetState - cleared by host parser
  unsigned char changed_xp: 1;    // mirror for Xpressnet
                                  // set by status_SetState - cleared by xpressnet master
  unsigned char clock: 1;         // there was a minute tick for DCC Layout time
                                  // set by fast_clock - cleared by xpressnet master
} t_status_event;

// SDS : mijn intf naar de display
typedef enum {
  HWEVENT_MAINSHORT, HWEVENT_PROGSHORT, HWEVENT_EXTSTOP, HWEVENT_NODCC
} hwEvent_t;

extern t_opendcc_state opendcc_state; // this is the current state of the box
extern t_status_event status_event;   // Message flag: there was a change in status
extern unsigned char ext_stop_enabled;
// TODO SDS2021 : nog enkel nodig voor lenz_parser
extern t_no_timeout no_timeout;

#if (DCC_FAST_CLOCK==1)
// fast clock record
extern t_fast_clock fast_clock;
#endif

//===================================================================================
// Interface-Routines to other programs
void status_Init();
void status_SetState(t_opendcc_state next);
void status_Run();
bool status_IsProgState(); // TODO SDS2021 : enkel gebruikt in lenz parser config, mag dat niet weg ??
extern void hwEvent_Handler( hwEvent_t event ) __attribute__ ((weak));

#endif // __STATUS_H__

