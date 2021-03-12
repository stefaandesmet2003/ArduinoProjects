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
// file:      status.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-28 V0.01 started
//            2006-05-15 V0.02 bugfixes in LED-Control
//            2006-06-01 V0.03 debouncer for short curcuit inputs
//            2007-10-15 V0.05 key_go_at_start_pressed neu dazu.
//            2008-01-08 V0.06 short_turnoff_time is fetched from eeprom
//            2008-01-11 V0.07 prog_short_turnoff_time is fetched from eeprom
//            2008-01-16 V0.08 is_prog_state() added
//                             moved to XP
//            2008-07-18 V0.09 ext_stop added; also DMX out is parallel controlled
//                             bugfix in timer mode for atmega644p
//            2009-01-14 V0.10 all timeouts now under atomic
//            2009-03-15 V0.12 ext_stop_deadtime added
//            2009-06-23 V0.13 new handling of Timer0 - now running at 4us
//                             previous file saved to ..\backup
//                             dcc fast clock command requires an exact timebase
//            2010-04-03 V0.14 PROG_TRACK_OFF also stops any programmer task
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   control of LED via state engines
//            timertick
//            check for key strokes and output short
//            shorts are stored and and a status_event.changed is set.
//            It is up to the selected parser, to report this event
//            to the host.
//               Lenz uses a push system (event_send), IB gets polled.           
//
// 2do:       output short for prog mode not yet impl.
// 
//-----------------------------------------------------------------
/* 2021 SDS TODO
 * check of de oude implementatie van short check weg mag
 * gebruiken? :  #define BLOCK_SHORT_after_PowerOn     25    // Shorts are ignored immediately after power on, voorlopig niet gebruikt (copy from bidi-booster)

*/

#include "config.h"     // general structures and definitions
#include "status.h"

#include "organizer.h"  // for sending fast clock dcc message
#include "programmer.h" // for reset_programmer

//---------------------------------------------------------------------------
// data for timeout and debounce
// every task loads its  corresponding member with the desired timeout value
t_no_timeout no_timeout;

unsigned char main_short_ignore_time;   // time from detect of a short until disable output. is loaded during init from eeprom
unsigned char prog_short_ignore_time;   // is loaded during init from eeprom

t_opendcc_state opendcc_state;            // this is the current running state
t_status_event status_event;              // message flag for other programs

unsigned char ext_stop_enabled = 0;       // if true: external stop input is enabled (CV36) 
uint32_t ext_stop_deadtime = EXT_STOP_DEAD_TIME;      // CV37
uint32_t extStopOkLastMillis;

uint32_t runState5msLastMillis; //sds : zelfde type als millis()

// values in ms
#define FAST_RECOVER_ON_TIME        1
#define FAST_RECOVER_OFF_TIME       4
#define SLOW_RECOVER_TIME         1000  // 1s voor we opnieuw NO_SHORT melden na een kortsluiting
//sds #define BLOCK_SHORT_after_PowerOn     25    // Shorts are ignored immediately after power on, voorlopig niet gebruikt (copy from bidi-booster)

typedef enum {NO_SHORT, IGNORE_SHORT, FASTREC_OFF, FASTREC_ON, SHORT} shortState_t;
shortState_t mainShortState,progShortState;
uint32_t shortLastMillis;
uint8_t shortFastRecoverAttemptsLeft;
static bool main_short_check();
static bool prog_short_check();

void init_state(void)
{
  if (eeprom_read_byte((uint8_t *)eadr_ext_stop_enabled) != 0)
  {
    ext_stop_enabled = 1;           // enable external OC Input for emergency stop
    ext_stop_deadtime = eeprom_read_byte((uint8_t *)eadr_ext_stop_deadtime);
    if (ext_stop_deadtime > 100) ext_stop_deadtime = 100;
  }
  extStopOkLastMillis = 0;

  // load timing values from eeprom
  main_short_ignore_time = eeprom_read_byte((uint8_t *)eadr_short_turnoff_time);
  if (main_short_ignore_time == 0) main_short_ignore_time = MAIN_SHORT_DEAD_TIME; // sds : in ms

  prog_short_ignore_time = eeprom_read_byte((uint8_t *)eadr_prog_short_toff_time);
  if (prog_short_ignore_time == 0) prog_short_ignore_time = PROG_SHORT_DEAD_TIME; // sds : in ms

  // clear all timeouts
  no_timeout.parser = 0;  // sds: nog nodig in lenz_parser.cpp

  #if (DCC_FAST_CLOCK==1)
    fast_clock.ratio = eeprom_read_byte((uint8_t *)eadr_fast_clock_ratio);
    //fast_clock.ratio = 8; //SDS from config.cpp
  #endif
  mainShortState = NO_SHORT;
  progShortState = NO_SHORT;

} // init_state


// global timeout supervision
// opendcc used this for key debouncing too
// i took short supervision out for now -> <5ms monitoring
void timeout_tick_5ms(void)
{
    if (no_timeout.parser) no_timeout.parser--;  
    //if (no_timeout.main_short) no_timeout.main_short--;  
    //if (no_timeout.prog_short) no_timeout.prog_short--;  
} // timeout_tick_5ms

//---------------------------------------------------------------------------------
// set_opendcc_state(t_opendcc_state next)
//
// Einstellen des nï¿½chsten Zustand mit:
// - Stellen entsprechender Bits in der Hardware und Rï¿½cksetzen der
//   ï¿½berwachungstasks ->
//   damit wird nach "power on" wieder Strom auf den Ausgang gelegt.
// - Stellen der LEDs
// - Setzen der globalen Variablen, fallweise Nachricht an den Host
//


static void prog_on(void)
{
  PROG_TRACK_ON;
}
static void prog_off(void)
{
  PROG_TRACK_OFF;
  reset_programmer();
}

static void main_on(void)
{
  MAIN_TRACK_ON;
}
static void main_off(void)
{
  MAIN_TRACK_OFF;
}

void set_opendcc_state(t_opendcc_state next)
{
  if (next == opendcc_state) return;      // no change
  opendcc_state = next;
  status_event.changed = 1;
  status_event.changed_xp = 1;
    
  switch(next)
  {
    case RUN_OKAY:                  // DCC running
      prog_off();
      main_on();
      organizer_state.halted = 0;   // enable speed commands again
      break;
    case RUN_STOP:                  // DCC Running, all Engines Emergency Stop
      do_all_stop();
      prog_off();
      main_on();
      break;
    case RUN_OFF:                   // Output disabled (2*Taste, PC)
      prog_off();
      main_off();
      break;
    case RUN_SHORT:                 // Kurzschluss
      prog_off();
      main_off();
      break;
    case RUN_PAUSE:                 // DCC Running, all Engines Speed 0
      prog_off();
      main_on();
      break;

    case PROG_OKAY:
      prog_on();
      main_off();
      organizer_state.halted = 0;   // enable speed commands again
      break;
    case PROG_SHORT:                //
      prog_off();
      main_off();
      break;
    case PROG_OFF:
      prog_off();
      main_off();
      break;
    case PROG_ERROR:
      prog_on();
      main_off();
      break;
  }
} // set_opendcc_state

#if (DCC_FAST_CLOCK==1)

unsigned int fc_value;
t_fast_clock fast_clock =      // we start, Monday, 8:00
  {
    0,    // unsigned char minute;
    8,    // unsigned char hour;
    0,    // unsigned char day_of_week;
    8,    // unsigned char ratio;
  };

// TODO : deze is niet meer nauwkeurig; 1 fast clock minuut om de +-9sec?
// is dat omdat deze functie op een 5ms software tick hangt, en niet meer op een timer isr?
void dcc_fast_clock_step_5ms(void)
{
  if (fast_clock.ratio)
  {
    fc_value += fast_clock.ratio;
    if (fc_value >= 12000) {         // 1min = 60000ms 
      fc_value = 0;
      fast_clock.minute++;
      if (fast_clock.minute >= 60) {
        fast_clock.minute = 0;
        fast_clock.hour++;
        if (fast_clock.hour >= 24) {
          fast_clock.hour = 0;
          fast_clock.day_of_week++;
          if (fast_clock.day_of_week >=7) fast_clock.day_of_week = 0;
        }
      }
      // now send this to DCC (but not during programming or when stopped)
      if (opendcc_state == RUN_OKAY) do_fast_clock(&fast_clock);
      
      status_event.clock = 1; // send clock_event to Xpressnet
    }
  }
} // dcc_fast_clock_step_5ms
#endif // DCC_FAST_CLOCK

// Hinweis: RUN_PAUSE wird zur Zeit nicht angesprungen
void run_state(void)
{
  if ((millis() - runState5msLastMillis) > 5) {
    runState5msLastMillis = millis();
    timeout_tick_5ms();
    
    #if (DCC_FAST_CLOCK==1)
    dcc_fast_clock_step_5ms();
    #endif

    // check external stop
    // TODO : remove analogRead, want die is traag
    // is 5ms response time op EXT_STOP echt nodig?
    if ((ext_stop_enabled) && (opendcc_state != RUN_OFF))
    {
      if (!EXT_STOP_ACTIVE) extStopOkLastMillis = millis();
      else if ((millis() - extStopOkLastMillis) > ext_stop_deadtime)
      {
          // we hebben een extStop event!
        set_opendcc_state(RUN_OFF);
        if (hwEvent_Handler)
          hwEvent_Handler(HWEVENT_EXTSTOP);
      }
    }
  }
  // check main short
  if ((opendcc_state != RUN_SHORT) && (opendcc_state != PROG_SHORT)) {
    if (main_short_check() == true)
    {
      set_opendcc_state(RUN_SHORT);
      if (hwEvent_Handler)
        hwEvent_Handler(HWEVENT_MAINSHORT);
    }
    // check prog short
    if (prog_short_check() == true)
    {
      set_opendcc_state(PROG_SHORT);
      if (hwEvent_Handler)
        hwEvent_Handler(HWEVENT_PROGSHORT);
    }
  }
} // run_state

// returns true, if we are in prog state
// sds, houden, gebruikt door lenz_parser
bool is_prog_state(void)
{
  bool retval = true;
  switch(opendcc_state)
  {
    case RUN_OKAY:             // wir laufen ganz normal:
    case RUN_STOP:             // DCC Running, all Engines Emergency Stop
    case RUN_OFF:              // Output disabled (2*Taste, PC)
    case RUN_SHORT:            // Kurzschluss
    case RUN_PAUSE:            // DCC Running, all Engines Speed 0
      retval = false;
      break;
  case PROG_OKAY:
  case PROG_SHORT:
  case PROG_OFF:
  case PROG_ERROR:
    retval = true;
    break;
  }
  return(retval);
} // is_prog_state

// return : false = no_short (ook tijdens de fast recovery), true = short
// mijn eigen implementatie, kan beter want in praktijk is er nooit recovery
static bool main_short_check()
{
  bool retval = false; // no_short
  switch(mainShortState)
  {
    case NO_SHORT :
      if (MAIN_IS_SHORT) {
        mainShortState =  IGNORE_SHORT;
        shortLastMillis = millis();
      }
      break;
    case IGNORE_SHORT :
      if (!MAIN_IS_SHORT) {
        mainShortState = NO_SHORT;
        MAIN_TRACK_ON;
      }
      else {
        if ((millis() - shortLastMillis) > main_short_ignore_time) {
          mainShortState = FASTREC_OFF;
          shortFastRecoverAttemptsLeft = 3;
          MAIN_TRACK_OFF;
          shortLastMillis = millis();
        }
      }
      break;
    case FASTREC_OFF : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_OFF_TIME) { // 4ms uit, en dan opnieuw aan en zien of de short weg is
        MAIN_TRACK_ON;
        mainShortState = FASTREC_ON;
        shortLastMillis = millis();
      }
      break;
    case FASTREC_ON : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_ON_TIME) { // 1ms wachten en dan short checken
        if (MAIN_IS_SHORT) {
          mainShortState = FASTREC_OFF;
          shortLastMillis = millis();
          MAIN_TRACK_OFF;
          shortFastRecoverAttemptsLeft--;
          if (!shortFastRecoverAttemptsLeft) {
            mainShortState = SHORT;
            retval = true;
          }
        }
        else {
          mainShortState = NO_SHORT;
          MAIN_TRACK_ON;
        }
      }
      break;
    case SHORT : 
      if (MAIN_IS_SHORT) {
        shortLastMillis = millis();
        retval = true;
      }
      else if ((millis() - shortLastMillis) > SLOW_RECOVER_TIME) {
        mainShortState = NO_SHORT; 
      }
      break;
  }
  return (retval);
} // main_short_check

// return : false = no_short (ook tijdens de fast recovery), true = short
static bool prog_short_check()
{
  bool retval = false; // no_short
  switch(progShortState) {
    case NO_SHORT :
      if (PROG_IS_SHORT) {
        progShortState =  IGNORE_SHORT;
        shortLastMillis = millis();
      }
      break;
    case IGNORE_SHORT :
      if (!PROG_IS_SHORT) {
        progShortState = NO_SHORT;
        PROG_TRACK_ON;
      }
      else {
        if ((millis() - shortLastMillis) > prog_short_ignore_time) {
          progShortState = FASTREC_OFF;
          shortFastRecoverAttemptsLeft = 3;
          PROG_TRACK_OFF;
          shortLastMillis = millis();
        }
      }
      break;
    case FASTREC_OFF : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_OFF_TIME) { // 4ms uit, en dan opnieuw aan en zien of de short weg is
        PROG_TRACK_ON;
        progShortState = FASTREC_ON;
        shortLastMillis = millis();
      }
      break;
    case FASTREC_ON : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_ON_TIME) { // 1ms wachten en dan short checken
        if (PROG_IS_SHORT) {
          progShortState = FASTREC_OFF;
          shortLastMillis = millis();
          PROG_TRACK_OFF;
          shortFastRecoverAttemptsLeft--;
          if (!shortFastRecoverAttemptsLeft) {
            progShortState = SHORT;
            retval = true;
          }
        }
        else {
          progShortState = NO_SHORT;
          PROG_TRACK_ON;
        }
      }
      break;
    case SHORT : 
      if (PROG_IS_SHORT) {
        shortLastMillis = millis();
        retval = true;
      }
      else if ((millis() - shortLastMillis) > SLOW_RECOVER_TIME) {
        progShortState = NO_SHORT; 
      }
      break;
  }
  return (retval);
} // prog_short_check

/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

#if SDS_BACKUP
// dit was de opendcc implementatie met glijdend venster evaluatie van short

//===================================================================================
/// output short protection
/// this routine debounces short curcuit message from output stage
/// act on MAIN_IS_SHORT and PROG_IS_SHORT
//
// a filter is performed on MAIN_IS_SHORT:
// every scan: if MAIN is short, the mean value is increased by 4
//          

#define UNPRESSED 0
#define DEBOUNCE  1
#define PRESSED   2

#define IS_SHORT    1
#define IS_OKAY     0


unsigned char main_short = UNPRESSED;
signed char main_short_mean = 0;

unsigned char prog_short = UNPRESSED;
signed char prog_short_mean = 0;

// returns 0 if no short
// returns 1 if short is detected and debounced


// -------------------------------------------------------------------- new
// TIMER2_TICK_PERIOD         // this is 4us Tickinterval of timer2
// we asume a running Timer2 - and do a check every 52us;
// Fully shorted this means a delay of 2.5ms until we have all integrated

#define   SHORT_CHECK_PERIOD    52L

// returns 0 if no short
// returns 1 if short is detected and debounced

signed char next_short_check;

unsigned char run_main_short_check(void)
  {
    switch(main_short)
      {
        case UNPRESSED:
            if (MAIN_IS_SHORT)
              {
                main_short = DEBOUNCE;
                next_short_check = TCNT2 + (SHORT_CHECK_PERIOD / TIMER2_TICK_PERIOD);
                no_timeout.main_short = short_turnoff_time;   // default 15ms
                main_short_mean = 0;
              }
            break;
        case DEBOUNCE:
            if ((signed char)(TCNT2 - next_short_check) >= 0)
              {
                next_short_check = TCNT2 + (SHORT_CHECK_PERIOD / TIMER2_TICK_PERIOD);
                if (!MAIN_IS_SHORT)
                  {
                    main_short_mean -= 1;
                    if (main_short_mean < -100) main_short = UNPRESSED;  // nochmal von vorn
                  }
                else
                  {
                    main_short_mean += 1;
                    if (main_short_mean > 100) main_short_mean = 100;   // limit
                
                    if (no_timeout.main_short) {}
                    else
                      {
                        main_short = PRESSED;
                        return(IS_SHORT);     // exit here
                      } 
                  }
              }
            break;
        case PRESSED:
            if ((signed char)(TCNT2 - next_short_check) >= 0)
              {
                next_short_check = TCNT2 + (SHORT_CHECK_PERIOD / TIMER2_TICK_PERIOD);
                if (!MAIN_IS_SHORT)
                  {
                    main_short_mean -= 1;
                    if (main_short_mean < -100) main_short = UNPRESSED;  // nochmal von vorn
                  }
                else
                  {
                    main_short_mean += 1;
                    if (main_short_mean > 100) main_short_mean = 100;   // limit

                    return(IS_SHORT);        // still short - exit here
                  }
               }
            break;
      }
    return(IS_OKAY);
  }



unsigned char run_prog_short_check(void)
  {
    switch(prog_short)
      {
        case UNPRESSED:
            if (PROG_IS_SHORT)
              {
                prog_short = DEBOUNCE;
                no_timeout.prog_short = prog_short_turnoff_time;   // default 40ms
              }
            break;
        case DEBOUNCE:
            if (!PROG_IS_SHORT)
              {
                prog_short = UNPRESSED;  // nochmal von vorn
              }
            else
              {
                if (no_timeout.prog_short) {}
                else
                  {
                    prog_short = PRESSED;
                    return(IS_SHORT);     // exit here
                  } 
              }
            break;
        case PRESSED:
            if (!PROG_IS_SHORT)
              {
                prog_short = UNPRESSED;  // nochmal von vorn
              }
            else
              {
                return(IS_SHORT);        // still short - exit here
              }
            break;
      }
    return(IS_OKAY);
  }

#endif
