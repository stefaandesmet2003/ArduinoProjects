//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006, 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      database.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-08-25 V0.01 started
//            2008-08-27 V0.02 Transfertask for Database entries
//            2009-03-10 V0.03 added delimiters to write_loco_name
//                             LOK_NAME_LENGTH now per define
//            2009-03-11 V0.04 data base is sent as message and as call
//                             bugfix in total_data_base_size
//
//
//--------------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   handling of lok names and speed steps
//            (This is not locobuffer, this is the data base,
//            has nothing to do with the actual dcc content)
//
//            
//--------------------------------------------------------------------------

#include "config.h"                // general structures and definitions
//sds #include "status.h"                // timerval, CHECK! hebben we deze include nu nog nodig??
#include "database.h"


// mega32:   2kByte SRAM, 1kByte EEPROM --> SDS : idem atmega328p
// mega644:  4kByte SRAM, 2kByte EEPROM


t_format dcc_default_format;            // dcc default: 0=DCC14, 2=DCC28, 3=DCC128
                                        // this value is read from eeprom

unsigned char next_search_index;        // was asked continously - this is the index for the parser

unsigned char cur_database_entry;
unsigned char total_database_entry;  // all locos with a name
unsigned char db_message[17]; 
unsigned char db_message_ready;


#if (LOCO_DATABASE == NAMED)

//------------------------------------------------------------------------
//------------------------------------------------------------------------
// Database for ADDR, FORMAT and NAME
//------------------------------------------------------------------------
//------------------------------------------------------------------------
//
// The default format of OpenDCC is defined in config.h (DCC_DEFAULT_FORMAT)
// and stored in EEPROM (CV24)
//
// Rules: if a loco runs with the default format and no name, it is not stored.
//        if a loco runs at a different format, the address and the format
//        is stored in EEPROM as 16 bit value, which is defined as follows:
//        - upper 2 bits contain the loco format;
//        - lower 14 bits contain the loco address.
//        - if entry == 0x0000, the entry is void.
//        if a name is given: loco is stored
//  
// Up to 64 locos may have different format (ESIZE_LOCO_FORMAT)

// We allocate this storage to a fixed address to have access from the
// parser with a constant offset to eeprom
// (parser commands: read and write Special Option)
//
#if (EEPROM_FIXED_ADDR == 1) 
 #define EEMEM_LOCO __attribute__((section(".ee_loco")))
#else
 #define EEMEM_LOCO EEMEM
#endif
//
// please add following command to linker:
//
//   -Wl,--section-start=.ee_loco=0x810080


#if (ESIZE_LOCO_FORMAT==64)
t_locoentry locoentry[ESIZE_LOCO_FORMAT] EEMEM_LOCO =
  {
    //    addr   format       pid,  name
    { { {   98,   DCC128 } }, { 0x1001 }, {  "BBII" } },
    { { {   14,    DCC14 } }, { 0x3001 }, {  "V200" } },
    { { {   28,    DCC28 } }, { 0x4002 }, {  "E94"  } },
    { { {  123,    DCC28 } }, { 0x4006 }, {  "Taurus" } },   
  };


#else
 #warning locoformat[ESIZE_LOCO_FORMAT] is not initialized! 
#endif


// EEPROM Interface
// EEPROM byteweise lesen -> geht schneller
//
/// get_loco_format returns the stored loco format, if loco was never
//  used with a format different from default it is not stored.
//
t_format get_loco_format(unsigned int addr)
  {
    unsigned char stored_h, stored_l;
	unsigned char i;
	unsigned char addr_low, addr_high;

	addr_low = (unsigned char) addr;
	addr_high = (unsigned char) (addr>>8);

    for (i=0; i<ESIZE_LOCO_FORMAT; i++)
      {
        stored_l = eeprom_read_byte(&locoentry[i].b[0]);
        if (stored_l == addr_low)
          {
            stored_h = eeprom_read_byte(&locoentry[i].b[1]);
            if ((stored_h & 0x3f) == addr_high)
              {
                return ((t_format)(stored_h & 0xC0) >> 6);
              }
          }
      }
    return(dcc_default_format);     // not found - default format
  }

//
// loco found -> replace format
// not found:  
//
// search loco, if found:  default: clear entry
//                         non default: store it
//              not found: default: exit
//                         non default: search empty and store it


// is it default format?
// -> no:   search loco, if found: replace it
//          if not found: search empty and store it
//          if no empty found: error too many locos with extra format -> unhandled - Code 0
// -> yes:  search loco, if found: clear entry


unsigned char store_loco_format(unsigned int addr, t_format format)
  {
    unsigned char i;
    unsigned char stored_h, stored_l;
    unsigned char my_h;
    unsigned char addr_low, addr_high;

    addr_low = (unsigned char) addr;
    addr_high = (unsigned char) (addr>>8);
    my_h = (format << 6) | addr_high;
    
    if (format != dcc_default_format)
      {                                         // not the default -> store it
        // search loco, if found: replace it
        for (i=0; i<ESIZE_LOCO_FORMAT; i++)
          {
            stored_l = eeprom_read_byte(&locoentry[i].b[0]);
            if (stored_l == addr_low)
              {
                stored_h = eeprom_read_byte(&locoentry[i].b[1]);
                if ((stored_h & 0x3f) == addr_high)
                  {
                    // entry with same addr found
                    if (stored_h == my_h) return(1);     // is already equal
                    else
                      {
                        eeprom_write_byte(&locoentry[i].b[1], my_h);   // replace format
                        return(1);
                      }
                  }
              }
          }
             // if not found: search empty and store it
        for (i=0; i<ESIZE_LOCO_FORMAT; i++)
          {
            stored_h = eeprom_read_byte(&locoentry[i].b[1]);
            if (stored_h == 0x00)
              {
                stored_l = eeprom_read_byte(&locoentry[i].b[0]);
                if (stored_l == 0x00)
                  {
                    // empty found
                    eeprom_write_byte(&locoentry[i].b[1], my_h);
                    eeprom_write_byte(&locoentry[i].b[0], addr_low);
                    eeprom_write_byte(&locoentry[i].name[0], 0);      // blank name
                    return(1);
                  }
              }
          }
         // others: error, database full!!!
         // too many locos with extra format
         return(0);
       }
     else
       {
         // it is the default
         // search loco, if found: clear entry
        for (i=0; i<ESIZE_LOCO_FORMAT; i++)
          {
            stored_l = eeprom_read_byte(&locoentry[i].b[0]);
            if (stored_l == addr_low)
              {
                stored_h = eeprom_read_byte(&locoentry[i].b[1]);
                if ((stored_h & 0x3F) == addr_high)
                  {
                    // target found -> clear entry
                    if ( eeprom_read_byte(&locoentry[i].name[0]) ) return(1);   // default, but named!
                    eeprom_write_byte(&locoentry[i].b[1], 0x00);
                    eeprom_write_byte(&locoentry[i].b[0], 0x00);
                    return(1);
                  }
              }
          }
      }
    return(1);
  }

void write_loco_name(unsigned char index, unsigned char * name)
  {
    unsigned char i;
    unsigned char delimiter;

    delimiter = ' ';
    if (name[0] == '\'') 
      {
        delimiter = '\'';
        name++;
      }
    if (name[0] == '\"')
      {
        delimiter = '\"';
        name++;
      }
    for (i=0; i<LOK_NAME_LENGTH; i++)
      {
        if (name[i] == delimiter)
          {
            eeprom_write_byte(&locoentry[index].name[i], 0);
          }
        else
          {
            eeprom_write_byte(&locoentry[index].name[i], name[i]);
          }
      }
  }


unsigned char store_loco_name(unsigned int addr, unsigned char * name)
  {
    unsigned char i;
    unsigned char stored_h, stored_l;
    unsigned char addr_low, addr_high;

    addr_low = (unsigned char) addr;
    addr_high = (unsigned char) (addr>>8);

    // search loco, if found: replace name
    for (i=0; i<ESIZE_LOCO_FORMAT; i++)
      {
        stored_l = eeprom_read_byte(&locoentry[i].b[0]);
        if (stored_l == addr_low)
          {
            stored_h = eeprom_read_byte(&locoentry[i].b[1]);
            if ((stored_h & 0x3f) == addr_high)
              {
                write_loco_name(i, name);
                return(1);
              }
          }
      }

     // if not found: search empty and store name
    for (i=0; i<ESIZE_LOCO_FORMAT; i++)
      {
        stored_h = eeprom_read_byte(&locoentry[i].b[1]);
        if (stored_h == 0x00)
          {
            stored_l = eeprom_read_byte(&locoentry[i].b[0]);
            if (stored_l == 0x00)
              {
                // empty found
                addr_high = addr_high | (dcc_default_format << 6);
                eeprom_write_byte(&locoentry[i].b[1], addr_high);
                eeprom_write_byte(&locoentry[i].b[0], addr_low);
                write_loco_name(i, name);
                return(1);
              }
          }
      }
     // ohters: error!!!
     // too many locos with extra format
     return(0);
  }

void clear_loco_database(void)
  {
    unsigned char i;
    unsigned char stored_h, stored_l;

    for (i=0; i<ESIZE_LOCO_FORMAT; i++)
      {
        stored_h = eeprom_read_byte(&locoentry[i].b[1]);
        if (stored_h != 0x00)
          {
            eeprom_write_byte(&locoentry[i].b[1], 0);
          }
        stored_l = eeprom_read_byte(&locoentry[i].b[0]);
        if (stored_l != 0x00)
          {
            eeprom_write_byte(&locoentry[i].b[0], 0);
          }
      }
  }

// fills structure actual, returns true if an entry was found;
// only named locos are reported
// This routine is used in parallel for dump and Xmit, but is not reentry!
unsigned char get_loco_data(t_locoentry* actual)
  {
    unsigned char i,j;

    for (i=next_search_index; i<ESIZE_LOCO_FORMAT; i++)
      {
        actual->b[0] = eeprom_read_byte(&locoentry[i].b[0]);
        actual->b[1] = eeprom_read_byte(&locoentry[i].b[1]);
        actual->pid_b[0] = eeprom_read_byte(&locoentry[i].pid_b[0]);
        actual->pid_b[1] = eeprom_read_byte(&locoentry[i].pid_b[1]);
        if (actual->w.addr != 0x00)
          {
            if (eeprom_read_byte(&locoentry[i].name[0]))   // yes we have a name
              {
                next_search_index = i+1;
                for (j=0; j<sizeof(locoentry[0].name); j++)
                  {
                    actual->name[j] = eeprom_read_byte(&locoentry[i].name[j]);
                  }
                return(1);            
              }
          }
      }
    next_search_index = 0;
    return(0);
  }

unsigned char format_to_uint8(t_format myformat)
  {  unsigned char i;
    switch(myformat)
      {
        default:
        case DCC14:
                i = 14; break;
        case DCC27:
                i = 27; break;
        case DCC28:
                i = 28;  break;
        case DCC128:
                i = 126; break;
      }
    return(i);
  }



// #if (XPRESSNET_ENABLED == 1)
//=================================================================================
//
// run_database: multitask replacement, must be called in loop
//
// This task transports the loco data base to connected throttles
//
// create 4 messages for every entry, 50ms gap
//
// loop(i)
//   {  message(i)
//      call(i)
//      message(i)
//      call(i)
//   }
//
//=================================================================================
enum db_run_states
  {                                 // actual state
     IDLE,
     DB_XMIT,
     DB_XMIT1,
     DB_XMIT2,
     DB_XMIT3,
     DB_XMIT4
  } db_run_state;

//SDS signed char next_database_run;   // timer variable to create a update grid;
uint32_t next_database_run;   // timer variable to create a update grid; SDS : zelfde type als millis()
//#define DB_UPDATE_PERIOD  50000L  //
#define DB_UPDATE_PERIOD  50L  // SDS : in ms, zelfde eenheid als millis()

void calc_database_size(void)
  {
    unsigned char i, j0, j1;

    total_database_entry = 0;

    for (i=0; i<ESIZE_LOCO_FORMAT; i++)
      {
        j0 = eeprom_read_byte(&locoentry[i].b[0]);
        j1 = eeprom_read_byte(&locoentry[i].b[1]) & 0x3F;
        j0 = j0 | j1;
        if (j0)  // any address
          {
            if (eeprom_read_byte(&locoentry[i].name[0]) )  // any name
               total_database_entry++;
          }
      }
  }

// Note: this message must be transmitted as CALL - like if it is transmitted from another client after a call!
void xmit_locoentry(void)
  {
    t_locoentry report;
    unsigned char i; // t_format myformat; 

    if (get_loco_data(&report))
      {
        // db_message[0] = 0xE0;                // see below
		db_message[1] = 0xF1;
        db_message[2] = report.b[1] & 0x3F;     // addr high
        db_message[3] = report.b[0];            // addr low
        db_message[4] = cur_database_entry;
        db_message[5] = total_database_entry;
        for (i=0; i<sizeof(report.name); i++)
          {
            if (report.name[i] == 0)
              {
                break;
              }
            db_message[6+i] = report.name[i];
          }
        db_message[0] = 0xE0 + 5 + i;
        db_message_ready = 1;                   // tell xpressnet that we have a message
      }
  }


void run_database(void)
  {
    switch (db_run_state)
      {
        case IDLE:
            break;

        case DB_XMIT:
            if (cur_database_entry >= total_database_entry)
              {
                db_run_state = IDLE; return;        // all done
              }

            xmit_locoentry();
            cur_database_entry++;
            next_database_run = millis() + DB_UPDATE_PERIOD;    // next wake up
            db_run_state = DB_XMIT1;
            break;

        case DB_XMIT1:
            if ((next_database_run - millis())>= 0)  return;

            db_message_ready = 2;                   // send again as call 
            next_database_run = millis() + DB_UPDATE_PERIOD;    // next wake up
            db_run_state = DB_XMIT2;
            break;

        case DB_XMIT2:
            if ((next_database_run - millis())>= 0)  return;

            db_message_ready = 1;                   // send again as message 
            next_database_run = millis() + DB_UPDATE_PERIOD;    // next wake up
            db_run_state = DB_XMIT3;
            break;

        case DB_XMIT3:
            if ((next_database_run - millis())>= 0)  return;

            db_message_ready = 2;                   // send again as call
            next_database_run = millis() + DB_UPDATE_PERIOD;    // next wake up
            db_run_state = DB_XMIT4;
            break;

        case DB_XMIT4:
            if ((next_database_run - millis())>= 0)  return;
            db_run_state = DB_XMIT;
            break;
     }
  }


void rewind_database(void)
  {
    next_search_index = 0;
  }


void init_database(void)
  {
    dcc_default_format = eeprom_read_byte((uint8_t*)eadr_dcc_default_format); 
    rewind_database();
  }


void do_xmit_database(void)
  {
    if (db_run_state != IDLE) return;       // block reentry
    db_run_state = DB_XMIT;
    rewind_database();
    calc_database_size();
    cur_database_entry = 0;                 // restart
  }


#elif (LOCO_DATABASE == UNNAMED)

//------------------------------------------------------------------------
//------------------------------------------------------------------------
// Database only for ADDR and FORMAT
//------------------------------------------------------------------------
//------------------------------------------------------------------------
//
// The default format of OpenDCC is defined in config.h (DCC_DEFAULT_FORMAT)
// and stored in EEPROM (CV24)
//
// Rules: if a loco runs with the default format, it is not stored.
//        if a loco runs at a different format, the address and the format
//        is stored in EEPROM as 16 bit value, which is defined as follows:
//        - upper 2 bits contain the loco format;
//        - lower 14 bits contain the loco address.
//        - if entry == 0x0000, the entry is void.
//  
// 
// Up to 64 locos may have different format (ESIZE_LOCO_FORMAT)

// We allocate this storage to a fixed address to have access from the
// parser with a constant offset to eeprom
// (parser commands: read and write Special Option)
//
#if (EEPROM_FIXED_ADDR == 1) 
 #define EEMEM_LOCO __attribute__((section(".ee_loco")))
#else
 #define EEMEM_LOCO EEMEM
#endif
//
// please add following command to linker:
//
//   -Wl,--section-start=.ee_loco=0x810080

#if (ESIZE_LOCO_FORMAT==64)
 union
  {
    unsigned int w;
    unsigned char b[sizeof(unsigned int)];
  } locoformat[ESIZE_LOCO_FORMAT] EEMEM_LOCO =
  {
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  {0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},{0x0000},
  };

#else
 #warning locoformat[ESIZE_LOCO_FORMAT] is not initialized! 
 union
  {
    unsigned int w;
    unsigned char b[sizeof(unsigned int)];
  } locoformat[ESIZE_LOCO_FORMAT] EEMEM_LOCO;
#endif

// EEPROM Interface
// EEPROM byteweise lesen -> geht schneller
//
/// get_loco_format returns the stored loco format, if loco was never
//  used with a format different from default it is not stored.
//
t_format get_loco_format(unsigned int addr)
  {
    unsigned char stored_h, stored_l;
	unsigned char i;
	unsigned char addr_low, addr_high;

    eeprom_busy_wait();

	addr_low = (unsigned char) addr;
	addr_high = (unsigned char) (addr>>8);

    for (i=0; i<ESIZE_LOCO_FORMAT; i++)
      {
        stored_l = eeprom_read_byte(&locoformat[i].b[0]);
        if (stored_l == addr_low)
          {
            stored_h = eeprom_read_byte(&locoformat[i].b[1]);
            if ((stored_h & 0x3f) == addr_high)
              {
                return ((stored_h & 0xC0) >> 6);
              }
          }
      }
    return(dcc_default_format);     // not found - default format
  }



//
// see also: http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial#EEPROM
// is it default format?
// -> no:   search loco, if found: replace it
//          if not found: search empty and store it
//          if no empty found: error too many locos with extra format -> unhandled - Code 0
// -> yes:  search loco, if found: clear entry


unsigned char store_loco_format(unsigned int addr, t_format format)
  {
    unsigned char i;
    unsigned char stored_h, stored_l;
    unsigned char my_h;
    unsigned char addr_low, addr_high;

    my_h = (format << 6) | (unsigned char)(addr >> 8);
    addr_low = (unsigned char) addr;
    addr_high = (unsigned char) (addr>>8);

    if (format != dcc_default_format)
      {                                         // not the default -> store it
        // search loco, if found: replace it
        for (i=0; i<ESIZE_LOCO_FORMAT; i++)
          {
            stored_l = eeprom_read_byte(&locoformat[i].b[0]);
            if (stored_l == addr_low)
              {
                stored_h = eeprom_read_byte(&locoformat[i].b[1]);
                if ((stored_h & 0x3f) == addr_high)
                  {
                    if (stored_h == my_h) return(1);     // does exist
                    else
                      {
                        eeprom_write_byte(&locoformat[i].b[1], my_h);
                        return(1);
                      }
                  }
              }
          }
             // if not found: search empty and store it
        for (i=0; i<ESIZE_LOCO_FORMAT; i++)
          {
            stored_h = eeprom_read_byte(&locoformat[i].b[1]);
            if (stored_h == 0x00)
              {
                stored_l = eeprom_read_byte(&locoformat[i].b[0]);
                if (stored_l == 0x00)
                  {
                    // empty found
                    eeprom_write_byte(&locoformat[i].b[1], my_h);
                    eeprom_busy_wait();
                    eeprom_write_byte(&locoformat[i].b[0], addr_low);
                    return(1);
                  }
              }
          }
         // ohters: error!!!
         // too many locos with extra format
         return(0);
       }
     else
       {
         // it is the default
         // search loco, if found: clear entry
        for (i=0; i<ESIZE_LOCO_FORMAT; i++)
          {
            stored_l = eeprom_read_byte(&locoformat[i].b[0]);
            if (stored_l == addr_low)
              {
                stored_h = eeprom_read_byte(&locoformat[i].b[1]);
                if ((stored_h & 0x3F) == addr_high)
                  {
                    // target found -> clear entry
                    eeprom_write_byte(&locoformat[i].b[1], 0x00);
                    eeprom_write_byte(&locoformat[i].b[0], 0x00);
                    return(1);
                  }
              }
          }
      }
    return(1);
  }

void init_database(void)
  {
    dcc_default_format = eeprom_read_byte((void *)eadr_dcc_default_format); 
  }

void rewind_database(void)
  {
    next_search_index = 0;
  }

unsigned char format_to_uint8(t_format myformat)
  {  unsigned char i;
    switch(myformat)
      {
        default:
        case DCC14:
                i = 14; break;
        case DCC27:
                i = 27; break;
        case DCC28:
                i = 28;  break;
        case DCC128:
                i = 126; break;
      }
    return(i);
  }




// #define LOCO_EEPROM_TEST
#ifdef LOCO_EEPROM_TEST
#warning LOCO_EEPROM_TEST activated

void test_loco_eeprom(void)
  {
     store_loco_format(1234, DCC128);
     store_loco_format(123, DCC14);
     store_loco_format(12, DCC28);
     // now we should have 2 locos (123 and 1234) in EEPROM
     PORTA = get_loco_format(1234);
     PORTA = get_loco_format(123);
     PORTA = get_loco_format(12);
     PORTA = get_loco_format(5555);
     // remap locos
     store_loco_format(1234, DCC28);
     store_loco_format(123, DCC14);
     store_loco_format(12, DCC128);
     // now we should have 2 locos (12 and 123) in EEPROM
     PORTA = get_loco_format(1234);
     PORTA = get_loco_format(123);
     PORTA = get_loco_format(12);
     
  }

#endif // LOCO_EEPROM_TEST

#else
  #warning LOCO_DATABASE undefined
#endif // (LOCO_DATABASE == NAMED)



