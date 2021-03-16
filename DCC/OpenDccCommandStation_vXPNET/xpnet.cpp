//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2008,2009 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      xpnet.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-07-08 V0.01 started
//            2008-07-31 V0.02 added status_event
//            2008-08-20 V0.03 send lok stolen changed owner-addr
//            2008-08-22       added F13 - F28 support
//            2008-09-01 V0.04 watch handling by slot_use_counter
//            2008-11-05 V0.05 tunnel_fifo neu dazu
//            2008-12-03 V0.06 Adresssplitting bei Rückmeldern
//                             Feedback mit Call-ID 0x20
//            2009-02-22 V0.07 Bugfix for Emergency Stop
//            2009-03-09 V0.08 added special command roco multimaus f13-f20
//            2009-03-11       data base is sent as message or as call
//            2009-05-25 V0.09 jens.kohn: Accessory Decoder also as broad cast
//            2009-06-23 V0.10 kw: added status_event.clock to send fast clock 
//                             added handling of fast clock messages in parser                   
//            2009-11-09 V0.11 kw: extended accessory 
//            2010-02-17 V0.12 kw: Pom cv rd command added 
//            2010-03-16 V0.13 kw: extended accessory on 0x1* 0x13  
//            2010-04-02 V0.14 kw: PomRead with 0xE4 
//            2010-04-04 V0.15 kim: Tunnel bugfix             
//            2010-06-07 V0.16 kw: command 0x80 repaired (bug in V0.23.4 and V0.23.5)
//            2010-06-17 V0.17 kw added PoM for accessory and extended accessory
//            2010-11-07 V0.18 kw added feedback and PomCV reading
//            2011-03-08 V0.19 kw bug fix for roco functions f13-f20
//
// differences to Xpressnet - docu:
// a) There is no request for acknowledge - we act different:
//    if an client creates an error:
//    - it will get an data error message
//    - it will be put back to nearly faded out.
// b) if a client does not answer during 255 calls,
//    it will dynamically fade out to unused, 
//    so there is no need to put it on a watch list  
//      
//-----------------------------------------------------------------
//
// purpose:   central station for dcc
// content:   XPressnet Interface (Master)
//            Only Xpressnet V3 is supported.
//            no support for Multiheader
// used sw:   status.c:     
//                          reading status events
//            rs485.c:      low level rs485 io
//            organizer.c:  loco stack management   
// used hw:   Timer2 (read only, for short timeouts)
//            
//-----------------------------------------------------------------

#include "config.h"                // general structures and definitions
#include "status.h"
#include "database.h"              // for database broadcast
#include "rs485.h"                 // rx and tx serial if, made for xpressnet

#include "programmer.h"

//SDS #include "s88.h"
#include "organizer.h"
#include "xpnet.h"        

#if (XPRESSNET_ENABLED == 1)

// generell fixed messages
unsigned char xp_datenfehler[] = {0x61, 0x80};             // xor wrong
unsigned char xp_busy[] = {0x61, 0x81};                    // busy
unsigned char xp_unknown[] = {0x61, 0x82};                 // unknown command
unsigned char xp_BC_alles_aus[] = {0x61, 0x00};            // Kurzschlussabschaltung
unsigned char xp_BC_alles_an[] = {0x61, 0x01};             // DCC wieder einschalten
unsigned char xp_BC_progmode[] = {0x61, 0x02};             // Progmode (das auslösende Gerät muss alles an senden)
unsigned char xp_BC_progshort[] = {0x61, 0x12};            // Progmode and short
unsigned char xp_BC_locos_aus[] = {0x81, 0x00};            // Alle Loks gestoppt

//===============================================================================
//
// 1. Slot Scheduler
//
//===============================================================================
// rules:   a) if there is an answer in one slot, this slot is marked as used;
//          b) every time a slot is called, the use_counter is decremented.
//          c) slot with use_counter == 0 are unused.
//          d) every time when a round with all used slots is done, one unused
//             slot is called.

unsigned char slot_use_counter[32];
unsigned char used_slot;        // 1 .. 31 (actual position for used ones)
unsigned char unused_slot;      // 1 .. 31 (actual position for unused ones)

static unsigned char get_next_slot(void)
{
  used_slot++;             // advance
  while (used_slot < 32) {        
    if (slot_use_counter[used_slot] > 0)
    {
      // this is a used slot, try it
      slot_use_counter[used_slot]--;
      return(used_slot);
    }
    used_slot++;
  }
  used_slot=0;
  
  // no more used slot found - return a unsued one
  unused_slot++;
  if (unused_slot == 32) unused_slot = 1;  // wrap
  return(unused_slot);
} // get_next_slot


static void set_slot_used(unsigned char slot)
{
  slot_use_counter[slot] = 255;           // alive
}

static void set_slot_to_watch(unsigned char slot)
{
  slot_use_counter[slot] = 10;           // nearly dead
}

//===============================================================================
//
// 3. Feedback Event Queue 
//
//===============================================================================
//
// Whenever there is a feedback event, we must tell these events to
// our clients.
//

//===============================================================================
//
// 4. Xpressnet Parser
//
//===============================================================================
//
// 4.a) Xpressnet variables
//
//-------------------------------------------------------------------------------
unsigned char current_slot;     // 1 .. 31
#define ACK_ID      0x00
#define FUTURE_ID   0x20        // A message with future ID to slot is Feedback broadcast (see also s88.c)
#define CALL_ID     0x40
#define MESSAGE_ID  0x60

unsigned char rx_message[17];             // current message from client
unsigned char rx_index;
unsigned char rx_size;

unsigned char tx_message[17];             // current message from master
unsigned char *tx_ptr;

// predefined messages
unsigned char xpnet_version[] = {0x63, 0x21, 
                                 0x36,      // Version 3.6
                                 0x00};     // 0: = LZ100 Zentrale 1 = LH 200, 2= DPC, 3= Control Plus
                                            // 0x10: Roco Zentrale

// 2.b) Xpressnet Send routines

void xp_send_message(unsigned char slot_id, unsigned char *str)
{
  unsigned char n, total, my_xor;

  n = 0;
  my_xor = str[0];
  total = str[0] & 0x0F;
  
  while (!XP_tx_ready()) ;                 // busy waiting! (but shouldn't happen)

  XP_send_call_byte(slot_id);              // send slot (9th bit is 1)
  XP_send_byte(str[0]);                    // send header

  while (n != total) {
    n++;
    my_xor ^= str[n];
    XP_send_byte(str[n]);              // send data
  }    
  XP_send_byte(my_xor);                   // send xor
} // xp_send_message

void xp_send_message_to_current_slot(unsigned char *str)
{
  xp_send_message(MESSAGE_ID | current_slot, str);
}

void xp_send_bc_message(void)
{
  switch(opendcc_state) {
    case RUN_OKAY:             // DCC running
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_alles_an);
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_alles_an);
      break;
    case RUN_STOP:             // DCC Running, all Engines Emergency Stop
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_locos_aus);  
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_locos_aus);  
      break;
    case RUN_OFF:              // Output disabled (2*Taste, PC)
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_alles_aus);  
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_alles_aus);  
      break;
    case RUN_SHORT:            // Kurzschluss
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_alles_aus);  
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_alles_aus);  
      break;
    case RUN_PAUSE:            // DCC Running, all Engines Speed 0
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_locos_aus);  
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_locos_aus);  
      break;

    case PROG_OKAY:
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_progmode);  
      break;
    case PROG_SHORT:           //
      xp_send_message(MESSAGE_ID |0, tx_ptr = xp_BC_progshort);  
      break;
    case PROG_OFF:
      break;
    case PROG_ERROR:
      break;
    }
  status_event.changed_xp = 0;            // broad cast done
} // xp_send_bc_message

#if (DCC_FAST_CLOCK == 1)
void xp_send_fast_clock(unsigned char slot_id)
{
  // 0x05 0x01 TCODE0 {TCODE1 TCODE2 TCODE3} 
  tx_message[0] = 0x05;
  tx_message[1] = 0xF1;
  tx_message[2] = 0x00 | fast_clock.minute;
  tx_message[3] = 0x80 | fast_clock.hour;
  tx_message[4] = 0x40 | fast_clock.day_of_week;
  tx_message[5] = 0xC0 | fast_clock.ratio;

  xp_send_message(MESSAGE_ID | slot_id, tx_ptr = tx_message);
  status_event.clock = 0;            // fast clock broad cast done
} // xp_send_fast_clock
#endif

void xpnet_send_prog_result(void)
{
  // Messages:
  // 61 11: ready
  // 61 12: short - Kurzschluss
  // 61 13: cant read - Daten nicht gefunden
  // 61 1f: busy
  // 63 10 EE D: EE=adr, D=Daten; nur für Register oder Pagemode, wenn bei cv diese Antwort, dann kein cv!
  // 63 14 CV D: CV=cv, D=Daten: nur wenn cv gelesen wurde; 14,15,16,17

  if (prog_event.busy) {
    tx_message[0] = 0x61;
    tx_message[1] = 0x1f;
    xp_send_message_to_current_slot(tx_ptr = tx_message); 
  }
  else {
    switch (prog_result) {
      case PT_OKAY:
        switch (prog_qualifier) {
          case PQ_REGMODE:
            tx_message[0] = 0x63;
            tx_message[1] = 0x10;
            tx_message[2] = prog_cv;
            tx_message[3] = prog_data;
            xp_send_message_to_current_slot(tx_ptr = tx_message); 
            break;
          case PQ_CVMODE_B0:
            tx_message[0] = 0x63;
            tx_message[1] = prog_cv / 256;              // use higher address bit to modify header code
            tx_message[1] &= 0x3; 
            tx_message[1] += 0x14;                      // header codes 0x14, 0x15, 0x16, 0x17
            tx_message[2] = (unsigned char) prog_cv;    // in any case: use fraktional part.
            tx_message[3] = prog_data;
            xp_send_message_to_current_slot(tx_ptr = tx_message); 
            break;
          default:
            tx_message[0] = 0x61;
            tx_message[1] = 0x11;     // ready
            xp_send_message_to_current_slot(tx_ptr = tx_message); 
            break; 
        }
        break;
      case PT_TIMEOUT:            // here all other stuff as well
      case PT_NOACK:
      case PT_NODEC:               // No decoder detected
      case PT_ERR:      
      case PT_BITERR:  
      case PT_PAGERR: 
      case PT_SELX:    
      case PT_DCCQD_Y:
      case PT_DCCQD_N: 
      case PT_TERM:
      case PT_NOTASK:  
      case PT_NOTERM:
        tx_message[0] = 0x61;
        tx_message[1] = 0x13;               // not found
        xp_send_message_to_current_slot(tx_ptr = tx_message); 
        break; 

      case PT_SHORT:
        tx_message[0] = 0x61;
        tx_message[1] = 0x12;
        xp_send_message_to_current_slot(tx_ptr = tx_message); 
        break;
    }
  }
} // xpnet_send_prog_result

void xp_send_busy(void)
{
  xp_send_message_to_current_slot(tx_ptr = xp_busy); 
}

// bit0-1 : zijn blijkbaar omgewisseld tussen xpnet v3 en v3.6!
// we houden hier v3.6
void xp_send_status(void)
{
  // Format: Headerbyte Daten 1 Daten 2 X-Or-Byte
  // Hex : 0x62 0x22 S X-Or-Byte
  // S:
  // Bit 0: wenn 1, Anlage in Notaus
  // Bit 1: wenn 1, Anlage in Nothalt
  // Bit 2: Zentralen-Startmode (0 = manueller Start, 1 = automatischer Start)
  // Bit 3: wenn 1, dann Programmiermode aktiv
  // Bit 4: reserviert
  // Bit 5: reserviert
  // Bit 6: wenn 1, dann Kaltstart in der Zentrale
  // Bit 7: wenn 1, dann RAM-Check-Fehler in der Zentrale
  // Besonderheiten: siehe bei Lenz
  unsigned char my_status = 0;
  tx_message[0] = 0x62;
  tx_message[1] = 0x22;
  if (opendcc_state == RUN_OFF) my_status |= 0x01; //SDS bits omgewisseld
  if (opendcc_state == RUN_STOP) my_status |= 0x02; //SDS bits omgewisseld
  // my_status &= ~0x04;  // manueller Start
  if ( (opendcc_state == PROG_OKAY)
      | (opendcc_state == PROG_SHORT)
      | (opendcc_state == PROG_OFF)
      | (opendcc_state == PROG_ERROR) ) my_status |= 0x08;          // Programmiermode
  tx_message[2] = my_status;
  xp_send_message_to_current_slot(tx_ptr = tx_message); 
} // xp_send_status


void xp_send_loco_addr(unsigned int addr)     
{
  tx_message[0] = 0xE3;
  tx_message[1] = 0x30;                   // 0x30 + KKKK; here KKKK=0, normal loco addr
  if (addr == 0) tx_message[1] |= 0x04;   // KKKK=4 -> no result fould
  if (addr > XP_SHORT_ADDR_LIMIT) {
    tx_message[2] = addr / 256;
    tx_message[2] |= 0xC0;
  }
  else tx_message[2] = 0;
  tx_message[3] = (unsigned char)addr;
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_loco_addr

unsigned char convert_format[4] = {
  0b000,      // DCC14
  0b001,      // DCC27
  0b010,      // DCC28
  0b100,      // DCC128
};

void xp_send_lokdaten(unsigned int addr)
{
  register unsigned char i, data;
  register unsigned char speed;
  
  i = scan_locobuffer(addr);

  tx_message[0] = 0xE4; // Headerbyte = 0xE4
  tx_message[1] = 0x00; // Byte1 = Kennung = 0000BFFF:  B=0: nicht besetzt
                        // FFF=Fahrstufen: 000=14, 001=27, 010=28, 100=128
  if (i==SIZE_LOCOBUFFER) { // not found - was not used yet
    tx_message[1] |= convert_format[get_loco_format(addr)];  // ask eeprom about speed steps
    tx_message[2] = 0;                          // no Speed
    tx_message[3] = 0;                          // no functions
    tx_message[4] = 0; 
  }
  else {
    if ((locobuffer[i].owned_by_pc) ||
        (locobuffer[i].slot != current_slot))  tx_message[1] |= 0b0001000;

    speed = convert_speed_to_rail(locobuffer[i].speed, locobuffer[i].format);
    switch(locobuffer[i].format) {
      case DCC14:
        tx_message[2] = speed;    //Byte2 = Speed = R000 VVVV;
        break;
      case DCC27:
        tx_message[1] |= 0b001;
        if (speed < 1) {
          tx_message[2] = speed; 
        }
        else {          
          data = (speed & 0x1F) + 2;    // map internal speed 2..29 to external 4..31
          data = (data>>1) | ((data & 0x01) <<4);
          tx_message[2] = data | (speed & 0x80); 
        }
        break;
      case DCC28:
        tx_message[1] |= 0b010;           
        if (speed < 1) {
            tx_message[2] = speed; 
        }
        else {          
          data = (speed & 0x1F) + 2;    // map internal speed 2..29 to external 4..31
          data = (data>>1) | ((data & 0x01) <<4);
          tx_message[2] = data | (speed & 0x80); 
        }
        break;
      case DCC128:
        tx_message[1] |= 0b100;          
        tx_message[2] = speed;    //Byte2 = Speed = RVVV VVVV;
        break;
    }
    tx_message[3] = (locobuffer[i].fl << 4) | locobuffer[i].f4_f1;
    tx_message[4] = (locobuffer[i].f12_f9 << 4) | locobuffer[i].f8_f5;
  }
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_lokdaten

#if (DCC_F13_F28 == 1)
void xp_send_funct_level_f13_f28(unsigned int addr)
{
  uint8_t i;
  
  i = scan_locobuffer(addr);

  tx_message[0] = 0xE3;
  tx_message[1] = 0x52;
  if (i==SIZE_LOCOBUFFER) { // not found - was not used yet
    tx_message[2] = 0; // no functions
    tx_message[3] = 0; // no functions
  }
  else {
    tx_message[2] = locobuffer[i].f20_f13;
    tx_message[3] = locobuffer[i].f28_f21;
  }
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_funct_level_f13_f28

//SDS added
// zelfde soort antwoord als func_status voor de F1-F12 (dummy data)
// TODO : waarom staat hier 0xE4 in de spec, er zijn toch maar 3 data bytes??
void pc_send_funct_status_f13_f28(unsigned int addr)
{
  // TODO : heeft nu geen zin om de locobuffer te doorzoeken, data staan er toch niet in
  //uint8_t i;
  //i = scan_locobuffer(addr);

  tx_message[0] = 0xE3;
  tx_message[1] = 0x51;
  tx_message[2] = 0; // no data
  tx_message[3] = 0; // no data
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // pc_send_funct_status_f13_f28

#endif

// we answer, but this is not really supported (not stored in locobuffer).
// SDs : DUMMY!
void xp_send_loco_func_status(unsigned int addr)
{
  tx_message[0] = 0xE3; // Headerbyte = 0xE3
  tx_message[1] = 0x50; // Byte1 = Kennung = 10000000 // SDS : waarom stond hier 0x80?? dat lijkt fout voor JMRI
  tx_message[2] = 0x00; // Byte2 = 000sSSSS; s=F0, SSSS=F4...F1
  tx_message[3] = 0;    // Byte3 = SSSSSSSS; SSSSSSSS=F12...F5
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_loco_func_status

void xp_send_lok_stolen(unsigned char slot, unsigned int lokaddr)
{
  tx_message[0] = 0xE3;
  tx_message[1] = 0x40;
  tx_message[2] = (unsigned char) (lokaddr / 256);                           
  tx_message[3] = (unsigned char) (lokaddr);   
  if (lokaddr > XP_SHORT_ADDR_LIMIT) {
    tx_message[2] |= 0xc0;                                              
  }
  xp_send_message(MESSAGE_ID | slot, tx_message);
} // xp_send_lok_stolen

void xp_send_stolen_loks(void)
{
  unsigned char i;

  for (i=0; i<SIZE_LOCOBUFFER; i++) {
    if (locobuffer[i].owner_changed  && locobuffer[i].owned_by_pc) {
      locobuffer[i].owner_changed = 0;
      xp_send_lok_stolen( locobuffer[i].slot, locobuffer[i].address);
      return;  // exit here, send only one locomotive
    }
  }    
  organizer_state.lok_stolen_by_pc = 0;      // thats all
} // xp_send_stolen_loks

//-----------------------------------------------------------------------------------------------------
// List of all messages from the client
// i: implemented, n: not implemented, -: currently not implemented,  s: simulated, t: tested
//-----------------------------------------------------------------------------------------------------
// i | s | t |Ver.|OP-Code
// - | - | - | new|0x05 0xF1 TCODE1 TCODE2 TCODE3 TCODE4 [XOR] "DCC FAST CLOCK set"
// - | - | - | new|0x01 0xF2 "DCC FAST CLOCK query"
// - | - | - | new|0x13 0x01 B+AddrH AddrL [XOR] "DCC extended accessory (B=aspect)"
// i | - | - |    |0x21 0x10 0x31 "Request for Service Mode results"
// - | - | - |    |0x22 0x11 REG [XOR] "Register Mode read request (Register Mode (REG=1..8))"
// - | - | - |    |0x23 0x12 REG DAT [XOR] "Register Mode write request (Register Mode)"
// - | - | - |    |0x22 0x14 CV [XOR] "Paged Mode read request (Paged Mode)"
// i | - | - |    |0x22 0x15 CV [XOR] "Direct Mode CV read request (CV mode)"
// - | - | - |    |0x23 0x16 CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=1..256))"
// - | - | - |    |0x23 0x17 CV DAT [XOR] "Paged Mode write request (Paged mode)"
// - | - | - | 3.6|0x23 0x1C CV DAT [XOR] "Direct Mode CV write request (CV mode (CV1024, CV=1..255))"
// - | - | - | 3.6|0x23 0x1D CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=256 ... 511))"
// - | - | - | 3.6|0x23 0x1E CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=512 ... 767))"
// - | - | - | 3.6|0x23 0x1F CV DAT [XOR] "Direct Mode CV write request (CV mode (CV=768 ... 1023))"
// i | - | - |    |0x21 0x21 0x00 "Command station software-version request"
// n | n | n |    |0x22 0x22 00000M00 [XOR] "Set command station power-up mode"
// i | - | - |    |0x21 0x24 0x05 "Command station status request"
// i | - | - | new|0x23 0x28 AddrH AddrL [XOR] "Command station special option read"
// i | - | - | new|0x21 0x29 AddrH AddrL DAT [XOR] "Command station special option read"
// i | - | - |    |0x21 0x80 0xA1 "Stop operations request (emergency off)"
// i | - | - |    |0x21 0x81 0xA0 "Resume operations request"
// i | - | - |    |0x42 Addr Nibble [XOR] "Accessory Decoder information request"
// i | - | - |    |0x52 Addr DAT [XOR] "Accessory Decoder operation request"
// i | - | - | new|0x74 0xE0 AddrH AddrL SPEED [XOR] Speed annoucment
// i | - | - | new|0x78 0xE1 SID_H SID_L AddrH AddrL CV_H CV_L DAT [XOR] CV Read
// i | - | - | new|0x73 0xF0 SID_H SID_L Detector is IDLE
// i | - | - | new|0x73 0xF1 SID_H SID_L Detector is OCCUPIED
// - | - | - | new|0x75 0xF2 SID_H SID_L D+AddrH AddrL Detector has Loco Detected
// i | - | - | new|0x7x 0xFE SID_H SID_L DAT0 [DAT1] [DAT2] ... [DAT7] [XOR] 	"occupancy vector"
// - | - | - | new|0x75 0xFF SID1_H SID1_L SID2_H SID2_request rescan
// i | - | - |    |0x80 0x80 "Stop all locomotives request (emergency stop)"
// i | - | - |    |0x91 loco_addr [XOR] "Emergency stop a locomotive"
// i | - | - |    |0x92 AddrH AddrL [XOR] "Emergency stop a locomotive"
// n | - | - | 2  |0x9N loco_addr_1 loco_addr_2 etc. loco_addr N [XOR] "Emergency stop selected locomotives"
// n | - | - | 1  |0xA1 loco_addr [XOR] "Locomotive information request" (X-Bus V1)
// n | - | - | 2  |0xA2 loco_addr ModSel [XOR] "Locomotive information request" (X-Bus V2)
// n | - | - |    |0xC3 0x05 loco_addr_1 loco_addr_2 [XOR] "Establish Double Header"
// n | - | - |    |0xC3 0x04 loco_addr_1 loco_addr_2 [XOR] "Dissolve Double Header"
// n | - | - | 1  |0xB3 loco_addr loco_data_1 loco_data_2 [XOR] "Locomotive operation" (X-Bus V1)
// n | - | - | 2  |0xB4 loco_addr loco_data_1 loco_data_2 ModSel [XOR] "Locomotive operation" (X-Bus V2)
// i | - | - |    |0xE3 0x00 AddrH AddrL [XOR] "Locomotive information request"
// n | - | - |    |0xE4 0x01+R MTR AddrH AddrL [XOR] "Address inquiry member of a Multi-unit request"
// n | - | - |    |0xE2 0x03+R MTR [XOR] "Address inquiry Multi-unit request"
// i | - | - |    |0xE3 0x05+R AddrH AddrL [XOR] "Address inquiry locomotive at command station stack request"
// i | - | - |    |0xE3 0x07 AddrH AddrL [XOR] "Function status request"
// - | - | - | 3.6|0xE3 0x08 AddrH AddrL [XOR] "Function status request F13-F28"
// - | - | - | 3.6|0xE3 0x09 AddrH AddrL [XOR] "Function level request F13-F28"
// i | - | - |    |0xE4 0x10 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 14 speed step"
// i | - | - |    |0xE4 0x11 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 27 speed step"
// i | - | - |    |0xE4 0x12 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 28 speed step"
// i | - | - |    |0xE4 0x13 AddrH AddrL Speed [XOR] "Locomotive speed and direction operation - 128 speed step"
// i | - | - |    |0xE4 0x20 AddrH AddrL Group [XOR] "Function operation instruction - Group 1 f0f4f3f2f1"
// i | - | - |    |0xE4 0x21 AddrH AddrL Group [XOR] "Function operation instruction - Group 2 f8f7f6f5"
// i | - | - |    |0xE4 0x22 AddrH AddrL Group [XOR] "Function operation instruction - Group 3 f12..f9"
// i | - | - | 3.6|0xE4 0x23 AddrH AddrL Group [XOR] "Function operation instruction - Group 4 f20..f13"
// n | - | - |    |0xE4 0x24 AddrH AddrL Group [XOR] "Set function state - Group 1"
// n | - | - |    |0xE4 0x25 AddrH AddrL Group [XOR] "Set function state - Group 2"
// n | - | - |    |0xE4 0x26 AddrH AddrL Group [XOR] "Set function state - Group 3"
// - | - | - | 3.6|0xE4 0x27 AddrH AddrL Group [XOR] "Set function state - Group 4"
// - | - | - | 3.6|0xE4 0x28 AddrH AddrL Group [XOR] "Function operation instruction - Group 5 f28..f21"
// i | - | - | 3.6|0xE4 0x2C AddrH AddrL Group [XOR] "Set function state - Group 5"
// - | - | - | 3.6|0xE5 0x2F AddrH AddrL RefMode [XOR] "Set function refresh mode"
// i | - | - |    |0xE6 0x30 AddrH AddrL 0xEA+C CV DAT [XOR] "Operations Mode Programming byte mode read request" -> 0xE4
// i | - | - |    |0xE6 0x30 AddrH AddrL 0xEC+C CV DAT [XOR] "Operations Mode Programming byte mode write request"
// n | - | - |    |0xE6 0x30 AddrH AddrL 0xE8+C CV DAT [XOR] "Operations Mode programming bit mode write request"
// i | - | - | new|0xE6 0x30 AddrH AddrL 0xF0+C CV DAT [XOR] "Operations Mode Programming Accessory write request"
// i | - | - | new|0xE5 0x30 AddrH AddrL 0xF4+C CV [XOR] "Operations Mode Programming Accessory read request"
// i | - | - | new|0xE6 0x30 AddrH AddrL 0xF8+C CV DAT [XOR] "Operations Mode Programming ExtAccessory write request"
// i | - | - | new|0xE5 0x30 AddrH AddrL 0xFC+C CV [XOR] "Operations Mode Programming ExtAccessory read request"
// n | - | - |    |0xE5 0x43 ADR1H ADR1L ADR2H ADR2L [XOR] "Establish Double Header"
// n | - | - |    |0xE5 0x43 ADR1H ADR1L 0x00 0x00 [XOR] "Dissolve Double Header"
// n | - | - |    |0xE4 0x40+R AddrH AddrL MTR [XOR] "Add a locomotive to a multi-unit request"
// n | - | - |    |0xE4 0x42 AddrH AddrL MTR [XOR] "Remove a locomotive from a Multi-unit request"
// i | - | - |    |0xE3 0x44 AddrH AddrL [XOR] "Delete locomotive from command station stack request"
// n | - | - | roc|0xE3 0xF0 AddrH AddrL [XOR] "Read Lok" (Multimaus only; Antwort dann: 
// n | - | - | 3.6|0xF0 [XOR] "Read Version of Interface"
// other commands (by Wim Ros on his S88-N-LI-Xpressnet)
// 0xF2 01 Addr [XOR] set/query Xpressnet-Address
// 0xF2 02 Baud [XOR] set/query Baud
// 0xF2 F1 Base [XOR] set base
// 0xF2 F2 CNT [XOR] set count
// 0xF2 FE TM [XOR] set test mode
// 0xF2 FF PRG [XOR] set prog mode
//-----------------------------------------------------------------------------------------------------
// Additional Tests
//-----------------------------------------------------------------------------------------------------
//
// i | s | t |    |Handover of locomotives between handhelds
// i | - | t |    |Handover of lokomotives between handheld and pc
// i | - | - |    |Feedback events (generated from s88)
// - | - | - |    |Feedback events (generated from accessory operations)



void xp_parser(void)
{
  unsigned int addr;
  t_data16 xaddr;
  unsigned char speed = 0;
  unsigned char activate, coil;
  t_format format;
  unsigned char processed = 0;
  unsigned char retval;
      
  switch(rx_message[0] >> 4) {   // this is the opcode
    case 0x0:
      #if (DCC_FAST_CLOCK == 1)
      switch(rx_message[1]) {
        case 0xF1:
          // set clock
          for(coil = 2; coil <= (rx_message[0] & 0x0F); coil++ ) {   // use coil as temp
            speed = 99;     // use speed as temp
            switch(rx_message[coil] & 0xC0) {
              case 0x00:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 60) fast_clock.minute = speed;
                break;
              case 0x80:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 24) fast_clock.hour = speed;
                break;
              case 0x40:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 7) fast_clock.day_of_week = speed;
                break;
              case 0xC0:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 32) fast_clock.ratio = speed;
                break;
            }
          }                            
          do_fast_clock(&fast_clock); // dcc msg
          xp_send_fast_clock(current_slot); // xpnet msg
          processed = 1;
          break;
        case 0xF2:
          // query clock
          xp_send_fast_clock(current_slot);
          processed = 1;
          break;
      }
      #endif
      break;
    case 0x1:
      switch(rx_message[1]) {
        case 0x01:
          // dcc extended accessory operations request
          addr = (unsigned int) (((unsigned char)(rx_message[2] & 0x07)) << 8) + rx_message[3];
          do_extended_accessory(addr, (unsigned char)(rx_message[2] >> 3) & 0x1F);
          // no answer
          processed = 1;
          break;
      }
      break;
    case 0x2:
      switch(rx_message[1]) {
        case 0x10: 
          // Prog.-Ergebnis anfordern 0x21 0x10 0x31
          if (opendcc_state >= PROG_OKAY) {
            xpnet_send_prog_result();
            processed = 1;
          }
          // else: Command void -> end of case
          break;
        case 0x11: 
          // Prog.-Lesen Registermode 0x22 0x11 REG X-Or
          // REG contains the Resister (1...8), this Command has no answer
          my_XPT_DCCRR (rx_message[2]);   // return code = 2 - if bad parameter
          processed = 1;
          break;
        case 0x12: 
          // Prog.-Schreiben Register 0x23 0x12 REG DAT X-Or
          my_XPT_DCCWR (rx_message[2], rx_message[3]);   
          processed = 1;
          break;
        case 0x14:
          // Prog-lesen Pagemode Hex : 0x22 0x14 CV X-Or-Byte
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          my_XPT_DCCRP (addr);
          processed = 1;
          break;
        case 0x15: 
          // Prog.-Lesen CV 0x22 0x15 CV X-Or    // old; according to Lenz we should return CV1024?
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          my_XPT_DCCRD (addr);
          processed = 1;
          break;
        case 0x16: 
          // Prog.-Schreiben CV 0x23 0x16 CV DAT X-Or
          // CV: 1..256
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          my_XPT_DCCWD (addr, rx_message[3]);    // direct mode
          processed = 1;
          break;
        case 0x17: 
          // Prog.-Schreiben Paging 0x23 0x17 CV DAT X-Or
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          my_XPT_DCCWP (addr, rx_message[3]);
          processed = 1;
          break;
        case 0x18:      // Prog.-Lesen CV 0x22 0x18 CV X-Or    // CV 1..255, 1024
        case 0x19:      // Prog.-Lesen CV 0x22 0x19 CV X-Or    // CV 256 .. 511
        case 0x1A:      // Prog.-Lesen CV 0x22 0x1A CV X-Or    // CV 512 .. 767
        case 0x1B:      // Prog.-Lesen CV 0x22 0x1B CV X-Or    // CV 768 .. 1023
          addr = ((rx_message[1] & 0x03) * 256) + rx_message[2];
          if (addr == 0) addr = 1024;
          my_XPT_DCCRD (addr);
          processed = 1;
          break;
        case 0x1C:      // Prog.-Schreiben CV 0x23 0x1C CV DAT X-Or; CV: 1..255, 1024
        case 0x1D:      // Prog.-Schreiben CV 0x23 0x1D CV DAT X-Or; CV: 256 .. 511
        case 0x1E:      // Prog.-Schreiben CV 0x23 0x1E CV DAT X-Or; CV: 512 ... 767
        case 0x1F:      // Prog.-Schreiben CV 0x23 0x1F CV DAT X-Or; CV: 768 ... 1024
          addr = ((rx_message[1] & 0x03) * 256) + rx_message[2];
          if (addr == 0) addr = 1024;
          my_XPT_DCCWD (addr, rx_message[3]);  // direct mode
          processed = 1;
          break;
        case 0x21:
          // Softwareversion anfordern 0x21 0x21 0x00
          xp_send_message_to_current_slot(tx_ptr = xpnet_version);
          processed = 1;
          break;
        case 0x22:
          // Power Up Mode einstellen 0x22 0x22 00000M00
          // we don't do this (always manual mode, no automatic power to tracks)
          // no answer
          break;
        case 0x24:
          // Statusabfrage 0x21 0x24 0x05 "Command station status request"
          xp_send_status();
          processed = 1; 
          break;
        case 0x80:    
          // 0x21 0x80 0xA1 "Stop operations request (emergency off)"
          set_opendcc_state(RUN_OFF);
          processed = 1;                              // no answer here, but a broadcast will occur
          break;
        case 0x81:
          // 0x21 0x81 0xA0 "Resume operations request"
          set_opendcc_state(RUN_OKAY);    
          processed = 1;                              // no answer here, but a broadcast will occur
          break;
      }
      break;

    case 0x4:
      // not yet tested: Schaltinformation anfordern 0x42 ADR Nibble X-Or
      // Hex : 0x42 Adresse 0x80 + N X-Or-Byte
      // für Weichen: Adresse = Adr / 4; N=Nibble
      // nicht für Rückmelder!
      // Antwort:
      // Hex : 0x42 ADR ITTNZZZZ X-Or-Byte
      // ADR = Adresse mod 4
      // I: 1=in work; 0=done -> bei uns immer 0
      // TT = Type: 00=Schaltempf. 01=Schaltempf. mit RM, 10: Rückmelder, 11 reserved
      // N: 0=lower Nibble, 1=upper
      // ZZZZ: Zustand; bei Weichen je 2 Bits: 00=not yet; 01=links, 10=rechts, 11:void
      // Bei Rückmeldern: Direkt die 4 Bits des Nibbles
      // TC interpretiert das alles als direkt übereinanderliegend;
      // also hier und in s88.c folgende Notlösung:
      //  Adressen 0..63  werden als DCC-Accessory interpretiert, aus dem Turnout-Buffer geladen
      //                  und mit TT 01 oder 00 quittiert. Das bedeutet 256 mögliche Weichen
      //  Adressen 64-127 werden als Feedback interpretiert, das bedeutet 512 mögliche Melder

    switch(xpressnet_feedback_mode) {
      default:
      case 0:                             // mixed mode    
        if (rx_message[1] < 64)  {
          // Nur für Schaltinfo:
          addr = (rx_message[1] << 2) + (unsigned char)((rx_message[2] & 0x01) << 1);
          tx_message[0] = 0x42;
          // TODO!! we moeten deze functie herimplementeren zonder S88
          //SDS create_xpressnet_schaltinfo(addr, &tx_message[1]);
          xp_send_message_to_current_slot(tx_ptr = tx_message);
          processed = 1;
        }
        else { // request feedback info, shift addr locally down 
          addr = ((rx_message[1] - 64) << 3) + (unsigned char)((rx_message[2] & 0x01) << 2);
          tx_message[0] = 0x42;
          // TODO!! we moeten deze functie herimplementeren zonder S88
          //SDS create_xpressnet_feedback(addr, &tx_message[1]);
          xp_send_message_to_current_slot(tx_ptr = tx_message);
          processed = 1;
        }
        break;
      case 1:                           	   // only feedback
        addr = ((rx_message[1]) << 3) + (unsigned char)((rx_message[2] & 0x01) << 2);
        tx_message[0] = 0x42;
        // TODO!! we moeten deze functie herimplementeren zonder S88
        //SDS create_xpressnet_feedback(addr, &tx_message[1]);
        xp_send_message_to_current_slot(tx_ptr = tx_message);
        processed = 1;
        break;
      case 2:                           	   // only schaltinfo
        addr = (rx_message[1] << 2) + (unsigned char)((rx_message[2] & 0x01) << 1);
        tx_message[0] = 0x42;
        // TODO!! we moeten deze functie herimplementeren zonder S88
        //SDS create_xpressnet_schaltinfo(addr, &tx_message[1]);
        xp_send_message_to_current_slot(tx_ptr = tx_message);
        processed = 1;
        break;
    }           
    break;

    case 0x5:
      // 0x52 Addr DAT [XOR] "Accessory Decoder operation request"
      // Schaltbefehl 0x52 ADR DAT X-Or
      // Hex: 0x52 Adresse 0x80 + SBBO; 
      // Adresse: = Decoder;   S= 1=activate, 0=deactivate,
      //                       BB=local adr,
      //                       O=Ausgang 0 (red) / Ausgang 1 (grün)
      // (das würde eigentlich schon passend für DCC vorliegen, aber lieber sauber übergeben)
      addr = (unsigned int) (rx_message[1] << 2) + ((rx_message[2] >> 1) & 0b011);
      activate = (rx_message[2] & 0b01000) >> 3;
      coil = rx_message[2] & 0b01;
      if (invert_accessory & 0b01) coil = coil ^ 1;

      do_accessory(current_slot, addr, coil, activate);
      tx_message[0] = 0x42;
        // TODO!! we moeten deze functie herimplementeren zonder S88
      //SDS create_xpressnet_schaltinfo(addr, &tx_message[1]);

      // at this point I was not sure how to react:
      // either answer the request or/and send out a broadcast
      // we do both
      xp_send_message_to_current_slot(tx_ptr = tx_message);
      xp_send_message(FUTURE_ID | 0, tx_message);
      processed = 1;
      break;

    case 0x7: // SDS xpnet extension for feedback decoders
      // 0x72 Addr DAT [XOR] "Accessory Decoder notify"
      // DAT = 8-bits = 8 I/O in 1 byte -> translate to xpnet nibbles
      // willen we dit eventueel ook voor wissels/seinen gebruiken?
      // voorlopig hangen de accdecoders niet aan xpnet, dus geen feedback
      // het volstaat dat de info op xpnet wordt doorgegeven door de centrale (accdecoder zonder feedback)
      // TODO : feedback bijhouden in iets à la s88!
      // TODO : for now just broadcast back to all xpnet clients ()
      // format according to §2.1.11 (nibbles), TT=10 (feedback decoder), I=0
      tx_message[0] = 0x44;
      tx_message[1] = rx_message[1];
      tx_message[2] = 0b01000000 | (rx_message[2] & 0x0F); // lower nibble
      tx_message[3] = rx_message[1];
      tx_message[4] = 0b00110000 | ((rx_message[2] & 0xF0)>>4); // higher nibble
      xp_send_message(FUTURE_ID | 0, tx_message);
      processed = 1;
      break;

    case 0x8:
      // Alle Loks anhalten 0x80 0x80
      if (rx_message[1] == 0x80) {
        set_opendcc_state(RUN_STOP);                     // from organizer.c 
        processed = 1;                                   // no answer here, but a broadcast will occur
      }
      break;
    case 0x9:
      // 0x91 loco_addr [XOR] "Emergency stop a locomotive"
      // 0x92 AddrH AddrL [XOR] "Emergency stop a locomotive"
      // 0x9N loco_addr_1 loco_addr_2 etc. loco_addr N [XOR] "Emergency stop selected locomotives"
      if (rx_message[0] == 0x91) {
        addr = rx_message[1];           // only short addr
        do_loco_speed(current_slot, addr, 1);         // 1 = emergency stop
        processed = 1;
      }
      else if (rx_message[0] == 0x92) {
        addr = ((rx_message[1] & 0x3F) * 256) + rx_message[2];
        do_loco_speed(current_slot, addr, 1);         // 1 = emergency stop
        processed = 1;
      }
      // no response is to be sent
      break;
    case 0xE:
      switch(rx_message[1] & 0xf0) { // high nibble von rx_message[1]:
        case 0x00:
          // 0xE3 0x00 AddrH AddrL [XOR] "Locomotive information request"
          // 0xE4 0x01+R MTR AddrH AddrL [XOR] "Address inquiry member of a Multi-unit request"
          // 0xE2 0x03+R MTR [XOR] "Address inquiry Multi-unit request"
          // 0xE3 0x05+R AddrH AddrL [XOR] "Address inquiry locomotive at command station stack request"
          // 0xE3 0x07 AddrH AddrL [XOR] "Function status request"
          // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          switch(rx_message[1] & 0x0f) {
            unsigned int result;
            case 0x00:
              xp_send_lokdaten(addr);
              processed = 1;
              break;
            case 0x05:
              result = addr_inquiry_locobuffer(addr, 1); // forward
              xp_send_loco_addr(result);
              processed = 1;
              break;
            case 0x06:
              result = addr_inquiry_locobuffer(addr, 0); // revers
              xp_send_loco_addr(result);
              processed = 1;
              break;
            case 0x07:
              xp_send_loco_func_status(addr);
              processed = 1;
              break;
            #if (DCC_F13_F28 == 1)
              // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
            case 0x08:
              pc_send_funct_status_f13_f28(addr); // SDS : sends dummy data
              processed = 1;
              break;
              // 0xE3 0x09 AddrH AddrL [XOR] "Function level request F13-F28"
            case 0x09:
              xp_send_funct_level_f13_f28(addr); // SDS : sends dummy data
              processed = 1;
              break;
            #endif
          }
          break;
        case 0x10:           
          // Lok Fahrbefehl ab V3 0xE4 Kennung ADR High ADR Low Speed X-Or
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          format = rx_message[1] & 0x03;   // 0=14, 1=27, 2=28, 3=128 see t_format Definition
          switch(format) {
            case DCC14:
              speed = (rx_message[4] & 0x80) | (rx_message[4] & 0x0F);
              processed = 1;   
              break;
            case DCC27:
            case DCC28:
              if ((rx_message[4] & 0x0F) <= 1)               // map 0x?0 to 0 and 0x?1 to 1
                    speed = rx_message[4] & 0x81;             // stop or nothalt
              else {
                speed = ((rx_message[4] & 0x0F) << 1) | ((rx_message[4] & 0x10) >> 4);
                speed = speed - 2;                  // map 4..31 to 2..29
                speed = speed | (rx_message[4] & 0x80);    // direction
              }
              processed = 1;
              break;
            case DCC128:
              speed = rx_message[4];
              processed = 1;
              break;
          }
          
          if (organizer_ready()) {
            unsigned char myspeed;
            myspeed = convert_speed_from_rail(speed, format); // map lenz to internal 0...127                      

            retval = do_loco_speed_f(current_slot, addr, myspeed, format);
            if (retval & (1<<ORGZ_STOLEN) ) {
              xp_send_lok_stolen(orgz_old_lok_owner , addr);
            }
            processed = 1;
            // no response is given
          }
          else {
            xp_send_busy();             // we are busy
            processed = 1;
          }
          break;
        case 0x20:
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          switch(rx_message[1] & 0x0F) {  // Lok Funktionsbefehl ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
            case 0:          // Hex : 0xE4 0x20 AH AL Gruppe 1 X-Or-Byte   (Gruppe 1: 000FFFFF) f0, f4...f1
              if (organizer_ready()) {
                retval = do_loco_func_grp0(current_slot, addr, rx_message[4]>>4); // light, f0
                retval |= do_loco_func_grp1(current_slot, addr, rx_message[4]);
                if (retval & (1<<ORGZ_STOLEN)) {
                  xp_send_lok_stolen(orgz_old_lok_owner, addr);
                } 
                processed = 1;              // no response is given
              }
              else {
                xp_send_busy();             // we are busy
                processed = 1;
              }
              break;
            case 1:          // Hex : 0xE4 0x21 AH AL Gruppe 2 X-Or-Byte   (Gruppe 2: 0000FFFF) f8...f5
              if (organizer_ready()) {
                retval = do_loco_func_grp2(current_slot, addr, rx_message[4]);
                if (retval & (1<<ORGZ_STOLEN) )
                  {
                    xp_send_lok_stolen(orgz_old_lok_owner, addr);
                  } 
                processed = 1;                // no response is given
              }
              else {
                xp_send_busy();             // we are busy
                processed = 1;
              }
              break;
            case 2:          // Hex : 0xE4 0x22 AH AL Gruppe 3 X-Or-Byte   (Gruppe 3: 0000FFFF) f12...f9
              if (organizer_ready()) {
                retval = do_loco_func_grp3(current_slot, addr, rx_message[4]);
                if (retval & (1<<ORGZ_STOLEN) )
                  {
                    xp_send_lok_stolen(orgz_old_lok_owner, addr);
                  } 
                processed = 1;// no response is given
              }
              else  {
                  xp_send_busy();             // we are busy
                  processed = 1;
              }
              break;
            case 3:          // Hex : 0xE4 0x23 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              if (organizer_ready()) {
                #if (DCC_F13_F28 == 1)
                retval = do_loco_func_grp4(current_slot, addr, rx_message[4]);
                if (retval & (1<<ORGZ_STOLEN)) {
                  xp_send_lok_stolen(orgz_old_lok_owner, addr);
                }
                #endif
                processed = 1;
                // no response is given
              }
              else {
                xp_send_busy();             // we are busy
                processed = 1;
              }
              break;
            case 4:
            case 5:
            case 6:
              //  Funktionsstatus setzen ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
              // Hex : 0xE4 0x24 AH AL Gruppe 1 (000SSSSS)  S=1: Funktion ist tastend
              // Hex : 0xE4 0x25 AH AL Gruppe 2 (0000SSSS)
              // Hex : 0xE4 0x26 AH AL Gruppe 3 (0000SSSS)
              break;

            case 7:  // set function status
              // Hex : 0xE4 0x27 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              break;
            case 8: // Hex : 0xE4 0x28 AH AL Gruppe 5 X-Or-Byte   (Gruppe 5: FFFFFFFF) f28...f21
              if (organizer_ready()) {
                #if (DCC_F13_F28 == 1)
                retval = do_loco_func_grp5(current_slot, addr, rx_message[4]);
                if (retval & (1<<ORGZ_STOLEN) )
                  {
                    xp_send_lok_stolen(orgz_old_lok_owner, addr);
                  }
                #endif
                processed = 1;
                // no response is given
              }
              else {
                xp_send_busy();             // we are busy
                processed = 1;
              }
              break;
            case 0xC: // Hex : 0xE4 0x2C AH AL Gruppe 5 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              break;
            case 0xF: // Hex : 0xE4 0x27 AH AL RF X-Or-Byte
              // (RF=Refreshmode: 0:F0..F4, 1:F0...F8, 3=F0..F12, 7=F0..F12, F=f0..F28)
              break;
            }
          break;
        case 0x30:
          // Prog. on Main Byte ab V3   0xE6 0x30 AddrH AddrL 0xEC + C CV DAT X-Or
          // Prog. on Main Bit ab V3    0xE6 0x30 AddrH AddrL 0xE8 + C CV DAT X-Or
          // Prog. on Main Read ab V3.6 0xE6 0x30 AddrH AddrL 0xEA + C CV DAT [XOR] 
          // NOTE: 0xEA seem to be an error, should be 0xE4
          // Note: Xpressnet does only PoM for Loco, no Accessory!
          // xaddr.as_uint16 = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          xaddr.as_uint8[1] = rx_message[2] & 0x3F;
          xaddr.as_uint8[0] = rx_message[3];
          unsigned int xp_cv;
          unsigned char xp_data;
          xp_cv = (rx_message[4] & 0x03) * 256 + rx_message[5];    // xp_cv has the range 0..1023!
          xp_cv++;                                                 // map to internal range
          xp_data = rx_message[6];
          if ((rx_message[4] & 0xFC) == 0xEC) {
            do_pom_loco(xaddr.as_uint16, xp_cv, xp_data);        //  program on the main (byte mode)
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xE4) {  // 02.04.2010
            do_pom_loco_cvrd(xaddr.as_uint16, xp_cv);           //  pom cvrd the main (byte mode)
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xE8) {
            //!!! bit mode unsupported
          }
          else if ((rx_message[4] & 0xFC) == 0xF0) {
            do_pom_accessory(xaddr.as_uint16, xp_cv, xp_data);
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xF4) {
            do_pom_accessory_cvrd(xaddr.as_uint16, xp_cv);
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xF8) {
            do_pom_ext_accessory(xaddr.as_uint16, xp_cv, xp_data);
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xFC) {
            do_pom_ext_accessory_cvrd(xaddr.as_uint16, xp_cv);
            processed = 1;
          }
          break; 
        case 0x40:   //Lokverwaltung (Double Header)
          // !!! Lok zu MTR hinzufügen ab V3 0xE4 0x40 + R ADR High ADR Low MTR X-Or
          // !!! Lok aus MTR entfernen ab V3 0xE4 0x42 ADR High ADR Low MTR X-Or
          // !!! DTR-Befehle ab V3 0xE5 0x43 ADR1 H ADR1 L ADR2 H ADR2 L X-Or
          // !!! Lok aus Stack löschen ab V3 0xE3 0x44 ADR High ADR Low X-Or
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          switch(rx_message[1] & 0x0f) {
            case 0x04:
                delete_from_locobuffer(addr);   // normally we don't need this - we manage it dynamically
                processed = 1;
                break;
          }
          break;
        case 0xF0:
          switch(rx_message[1] & 0x0F) {
            case 1: // Hex : 0xE? 0xF1 (Lokdatenbank, Roco)
              break;
            case 3: // Hex : 0xE4 0xF3 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              // (special version for Roco Multimaus)
              if (organizer_ready()) {
                #if (DCC_F13_F28 == 1)
                  addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
                  retval = do_loco_func_grp4(current_slot, addr, rx_message[4]);
                  if (retval & (1<<ORGZ_STOLEN)) {
                    xp_send_lok_stolen(orgz_old_lok_owner, addr);
                  }
                #endif
                processed = 1;
                // no response is given
              }
              else {
                xp_send_busy();             // we are busy
                processed = 1;
              }
              break;
          }
          break;
      }
  }
  if (!processed) 
  {
    xp_send_message_to_current_slot(tx_ptr = xp_unknown);            // unknown command
  }
} // xp_parser

//===============================================================================
//
// 4. Xpressnet Master 
//
//===============================================================================
// Timeouts: SLOT_TIMEOUT: We wait this time after an INQUIRY for the first response
//           RX_TIMEOUT:   We wait this time for the complete message (error timeout)

#define XP_TIMER_TICK    TIMER2_TICK_PERIOD         // this is 4us
#define XP_SLOT_TIMEOUT  120L
#define XP_CALL_DURATION  176L

#if (((XP_SLOT_TIMEOUT+XP_CALL_DURATION) / XP_TIMER_TICK) > 127)
   #warning Error: XP_SLOT_Timeout too large or Timertick too small! 
#endif

#define RX_TIMEOUT     10L // sds : in ms zoals millis()

// internal and static
enum xp_states { // actual state for the Xpressnet Task
  XP_INIT,
  XP_INQUIRE_SLOT,                    // schedule
  XP_WAIT_FOR_TX_COMPLETE,            // complete inquiry sent?
  XP_WAIT_FOR_REQUEST,                // client request
  XP_WAIT_FOR_REQUEST_COMPLETE,
  XP_WAIT_FOR_ANSWER_COMPLETE,        // Our answer complete sent?
  XP_CHECK_BROADCAST,                 // is there a broadcast event
  XP_CHECK_FEEDBACK,                  // is there a feedback event
  XP_CHECK_STOLEN_LOK,
  XP_CHECK_DATABASE,                  // is there a database transfer
} xp_state;

signed char slot_timeout;
uint32_t rx_timeout; // zelfde eenheid als millis()

void run_xpressnet(void)
{
  switch (xp_state) {
    case XP_INIT:
      // we supose this is done: init_timer2(); // running with 4us per tick
      xp_state = XP_INQUIRE_SLOT;
      break;

    case XP_INQUIRE_SLOT:
      current_slot = get_next_slot();
      XP_send_call_byte (0x40 | current_slot);          // this is a normal inquiry call
      xp_state = XP_WAIT_FOR_TX_COMPLETE;
      break;

    case XP_WAIT_FOR_TX_COMPLETE:
      if (XP_is_all_sent()) {
        xp_state = XP_WAIT_FOR_REQUEST;
        slot_timeout = TCNT2 + ((XP_SLOT_TIMEOUT+XP_CALL_DURATION) / XP_TIMER_TICK);
      }
      break;

    case XP_WAIT_FOR_REQUEST:
      if (XP_rx_ready()) {
        // slot is requesting -> process it (a complete message could last up to 3ms)
        rx_message[0] = XP_rx_read();           // save header
        rx_size = rx_message[0] & 0x0F;         // length is without xor
        rx_size++;                              // now including xor
        rx_index = 1;        
        rx_timeout = millis();
        xp_state = XP_WAIT_FOR_REQUEST_COMPLETE;
      }
      else if ((signed char)(TCNT2 - slot_timeout) >= 0) {
        // slot timeout reached, continue
          xp_state = XP_CHECK_BROADCAST;
      } 
      break;

    case XP_WAIT_FOR_REQUEST_COMPLETE:
      if ((millis() - rx_timeout) >= RX_TIMEOUT) {
        // message incomplete, timeout reached !
        set_slot_to_watch(current_slot);
        xp_send_message_to_current_slot(tx_ptr = xp_datenfehler);
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      else {
        if (XP_rx_ready()) {
          rx_message[rx_index] = XP_rx_read();
          if (rx_index == rx_size) {
            unsigned char i, my_check = 0;
            // all data and xor read, now check xor
            for (i=0; i<=rx_size; i++) my_check ^= rx_message[i];   
            if (my_check == 0) {
              // packet is received and okay, now parse it
              set_slot_used(current_slot);
              xp_parser();
              xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
            }
            else {
              // XOR is wrong!
              xp_send_message_to_current_slot(tx_ptr = xp_datenfehler);
              set_slot_to_watch(current_slot);
              xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
            }
          }
          else {
            rx_index++;
            if (rx_index == 17-1) rx_index = 17-1;   // overrun!
          }
        }
      }           
      break;

    case XP_WAIT_FOR_ANSWER_COMPLETE:
      if (XP_is_all_sent()) {
          xp_state = XP_CHECK_BROADCAST;
      }
      break;
    case XP_CHECK_BROADCAST:
      if (status_event.changed_xp) {
        xp_send_bc_message();                            // report any Status Change
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      #if (DCC_FAST_CLOCK == 1)
      else if (status_event.clock) {
        xp_send_fast_clock(0);    // send as broadcast   // new: 23.06.2009; possibly we need a flag
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      #endif
      else {
        xp_state = XP_CHECK_FEEDBACK;
      }
      break;

    case XP_CHECK_FEEDBACK:
      /* SDS - we gaan dit moeten vervangen door een non-S88 mechanisme!!
        if ((s88_event.hardware_change_xp) || (s88_event.turnout_change_xp))
          {
            make_s88_xpressnet_report();                         // send broadcast: feedback events
            xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
          }
      SDS */
      xp_state = XP_CHECK_STOLEN_LOK;                         // xp_state = XP_INQUIRE_SLOT;
      break;

    case XP_CHECK_STOLEN_LOK:
      if (organizer_state.lok_stolen_by_pc) {
        xp_send_stolen_loks();                              // report any Status Change on loks
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      else {
        #if (LOCO_DATABASE == NAMED)
          if (db_message_ready) xp_state = XP_CHECK_DATABASE;
          else xp_state = XP_INQUIRE_SLOT;
        #else
          xp_state = XP_INQUIRE_SLOT;
        #endif
      }
      break;

    case XP_CHECK_DATABASE:
      #if (LOCO_DATABASE == NAMED)
        if (db_message_ready == 1) {
          xp_send_message(MESSAGE_ID | 0, db_message);     // send this as MESSAGE (normal download)
        }
        else {
          xp_send_message(CALL_ID | 0, db_message);           // send this as 'CALL',
        }                                                  // it looks like coming from a client
                                                            // (Roco has done this hack!)
        db_message_ready = 0;
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      #endif
      break;
  }
} // run_xpressnet


void init_xpressnet(void)
{
  xp_state = XP_INIT;
} // init_xpressnet


#else // #if (XPRESSNET_ENABLED == 1)

void init_xpressnet(void) {};
void run_xpressnet(void) {};

#endif // #if (XPRESSNET_ENABLED == 1)
