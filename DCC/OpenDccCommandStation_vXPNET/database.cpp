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
// Database for ADDR, FORMAT and NAME
//------------------------------------------------------------------------
//------------------------------------------------------------------------
//
// The default format of OpenDCC is defined in config.h (DCC_DEFAULT_FORMAT)
// and stored in EEPROM (CV24)
//
// the address and the format is stored in EEPROM as 16 bit value, which is defined as follows:
//        - upper 2 bits contain the loco format;
//        - lower 14 bits contain the loco address.
//        - if entry == 0x0000, the entry is void.
// the name is stored as a 10-char string
// Up to LOCODB_NUM_ENTRIES locos in the database

/*
 * TODO SDS2021 : database transmission code aanpassen, don't like
 * - er is een func get_loco_data die op idx werkt (de global next_search_index)
 *   en er zijn funcs die zoeken op loc address (get_loco_format / get_loco_name)
 *  ---> maak 1 func get_loco_data ?
 * - er is ook een vieze vlag db_message_ready die wordt gereset vanuit xpnet, en een global db_message eventueel via intf func
 * - test ui met database entry zonder naam (bv nieuwe loc met adres 5 -> wordt in db opgeslagen met een zero string name)
*/

#include "Arduino.h"
#include "config.h"                // general structures and definitions
#include "database.h"

enum db_run_states { // actual state
  IDLE,
  DB_XMIT,
  DB_XMIT1,
  DB_XMIT2,
  DB_XMIT3,
  DB_XMIT4
} db_run_state;

t_format dcc_default_format;            // dcc default: 0=DCC14, 2=DCC28, 3=DCC128
                                        // this value is read from eeprom
unsigned char next_search_index;        // was asked continously - this is the index for the parser
unsigned char cur_database_entry;
unsigned char total_database_entry;
unsigned char db_message[17]; 
unsigned char db_message_ready;

uint32_t db_lastMillis;                 // controls min delay between xpnet messages for database transmission
#define DB_UPDATE_PERIOD  50L           // ms, like millis()

const locoentry_t locdb_defaults[] PROGMEM = {
  //  addr format        name
  {{ {  3, DCC128 }}, {"DIESEL\x00"} },
  {{ {  4, DCC128 }}, {"STOOM\x00"}  },
};

// we define locodb as a pointer, and make the compiler do all offset calculations
// locodb is an eeprom address, so we should never dereference directly!!
static locoentry_t *locodb = (locoentry_t *) LOCODB_EEPROM_OFFSET;

/****************************************************************************************************/
/*   HELPER FUNCTIONS                                                                               */
/****************************************************************************************************/
// SDS : telde enkel entries met non-null 'name'
static void calc_database_size(void) {
  unsigned char i, j0, j1;

  total_database_entry = 0;
  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    j0 = eeprom_read_byte(&locodb[i].b[0]);
    j1 = eeprom_read_byte(&locodb[i].b[1]) & 0x3F;
    j0 = j0 | j1;
    if (j0) total_database_entry++;
  }
} // calc_database_size

// Note (Kufer): this message must be transmitted as CALL - like if it is transmitted from another client after a call!
static void xmit_locoentry() {
  locoentry_t report;
  unsigned char i;

  if (get_loco_data(&report)) {
    // db_message[0] = 0xE0;                // see below
    db_message[1] = 0xF1;
    db_message[2] = report.b[1] & 0x3F;     // addr high
    db_message[3] = report.b[0];            // addr low
    db_message[4] = cur_database_entry;
    db_message[5] = total_database_entry;
    for (i=0;i<LOK_NAME_LENGTH;i++) {
      db_message[6+i] = report.name[i];
      if (report.name[i] == 0x0) break;
    }
    db_message[0] = 0xE0 + 5 + i;
    db_message_ready = 1;        // tell xpressnet that we have a message
  }
} // xmit_locoentry

static void rewind_database() {
    next_search_index = 0;
} // rewind_database

/****************************************************************************************************/
/*   EEPROM PUBLIC INTERFACE                                                                        */
/****************************************************************************************************/
/// get_loco_format returns the stored loco format, if loco was never
//  used with a format different from default it is not stored.
t_format get_loco_format(unsigned int addr) {
  unsigned char stored_h, stored_l;
	unsigned char i;
	unsigned char addr_low, addr_high;

	addr_low = (unsigned char) addr;
	addr_high = (unsigned char) (addr>>8);

  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    stored_l = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
    if (stored_l == addr_low) {
      stored_h = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
      if ((stored_h & 0x3f) == addr_high) {
          return ((t_format)(stored_h & 0xC0) >> 6);
      }
    }
  }
  return(dcc_default_format); // not found - default format
} // get_loco_format

// sds temp??
// caller provides memory for the name string
uint8_t get_loco_name(unsigned int addr, uint8_t *name) {
  unsigned char stored_h, stored_l;
	unsigned char i,j;
	unsigned char addr_low, addr_high;

	addr_low = (unsigned char) addr;
	addr_high = (unsigned char) (addr>>8);

  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    stored_l = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
    if (stored_l == addr_low) {
      stored_h = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
      if ((stored_h & 0x3f) == addr_high) {
        for (j=0;j<LOK_NAME_LENGTH;j++){
          name[j] = eeprom_read_byte((uint8_t*)&locodb[i].name[j]);
          if (name[j]== 0x0) return(1);
        }
      }
    }
  }
  // not found
  return(0);
}

// SDS : aangepast : geen onderscheid meer tussen default format of niet, elke loc wordt opgeslagen
// TODO SDS2021 : naar word size voor eeprom read gaan? code wordt dan eenvoudiger, maar trager
unsigned char store_loco_format(unsigned int addr, t_format format) {
  unsigned char i;
  unsigned char stored_h, stored_l;
  unsigned char my_h;
  unsigned char addr_low, addr_high;

  addr_low = (unsigned char) addr;
  addr_high = (unsigned char) (addr>>8);
  my_h = (format << 6) | addr_high;
  
  // search loco, if found: replace it
  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    stored_l = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
    if (stored_l == addr_low) {
      stored_h = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
      if ((stored_h & 0x3f) == addr_high) { // entry with same addr found
        eeprom_update_byte((uint8_t*)&locodb[i].b[1], my_h);   // replace format
        return(1);
      }
    }
  }
  // if not found: search empty and store it
  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    stored_h = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
    if (stored_h == 0x00) {
      stored_l = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
      if (stored_l == 0x00) { // empty found
        eeprom_write_byte((uint8_t*)&locodb[i].b[1], my_h);
        eeprom_write_byte((uint8_t*)&locodb[i].b[0], addr_low);
        eeprom_write_byte((uint8_t*)&locodb[i].name[0], 0);      // blank name for now
        return(1);
      }
    }
  }
  // others: error, database full!!!
  // too many locos with extra format
  return(0);
} // store_loco_format

// SDS delimiter stuff weggedaan, de name moet null terminated zijn
static void write_loco_name(unsigned char index, unsigned char *locName) {
  unsigned char i;
  for (i=0; i<LOK_NAME_LENGTH; i++) {
    eeprom_update_byte((uint8_t*)&locodb[index].name[i], locName[i]);
    if (locName[i] == 0x00)
      break;
  }
  eeprom_update_byte((uint8_t*)&locodb[index].name[LOK_NAME_LENGTH-1], 0x0); // make sure there is always a trailing \0
} // write_loco_name

unsigned char store_loco_name(unsigned int addr, unsigned char * locName) {
  unsigned char i;
  unsigned char stored_h, stored_l;
  unsigned char addr_low, addr_high;

  addr_low = (unsigned char) addr;
  addr_high = (unsigned char) (addr>>8);

  // search loco, if found: replace name
  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    stored_l = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
    if (stored_l == addr_low) {
      stored_h = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
      if ((stored_h & 0x3f) == addr_high) {
        write_loco_name(i, locName);
        return(1);
      }
    }
  }

  // if not found: search empty and store name & default dcc format
  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    stored_h = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
    if (stored_h == 0x00) {
      stored_l = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
      if (stored_l == 0x00) { // empty found
        addr_high = addr_high | (dcc_default_format << 6);
        eeprom_write_byte((uint8_t*)&locodb[i].b[1], addr_high);
        eeprom_write_byte((uint8_t*)&locodb[i].b[0], addr_low);
        write_loco_name(i, locName);
        return(1);
      }
    }
  }
  // too many locos with extra format -> error
  return(0);
} // store_loco_name

// SDS : clear only the address/format field, that invalidates the entry
void clear_loco_database() {
  unsigned char i;

  for (i=0; i<LOCODB_NUM_ENTRIES; i++) {
    eeprom_update_byte((uint8_t*)&locodb[i].b[0], 0);
    eeprom_update_byte((uint8_t*)&locodb[i].b[1], 0);
  }
} // clear_loco_database


// SDS : reset defaults
void reset_loco_database() {
  locoentry_t dbEntry;
  for (uint8_t i=0; i < sizeof(locdb_defaults)/sizeof(locoentry_t); i++) {
    memcpy_P(&dbEntry,&locdb_defaults[i],sizeof(locoentry_t)); // copy flash->ram first
    // byte per byte copy to eeprom
    for (uint8_t j=0; j < sizeof(locoentry_t);j++)
      eeprom_update_byte(((uint8_t*)&locodb[i])+j,*(((uint8_t*)&dbEntry) + j));
  }
} // reset_loco_database

// fills structure actual, returns true if an entry was found;
// This routine is used in parallel for dump and Xmit, but is not reentrant!
// SDS : updates 'next_search_index'
unsigned char get_loco_data(locoentry_t* actual) {
  unsigned char i,j;

  for (i=next_search_index; i<LOCODB_NUM_ENTRIES; i++) {
    actual->b[0] = eeprom_read_byte((uint8_t*)&locodb[i].b[0]);
    actual->b[1] = eeprom_read_byte((uint8_t*)&locodb[i].b[1]);
    if (actual->w.addr != 0x00) {
      next_search_index = i+1;
      for (j=0;j<LOK_NAME_LENGTH;j++){
        actual->name[j] = eeprom_read_byte((uint8_t*)&locodb[i].name[j]);
        if (actual->name[j]== 0x0) break;
      }
      return(1);            
    }
  }
  next_search_index = 0;
  return(0);
} // get_loco_data

/****************************************************************************************************/
/*   PUBLIC INTERFACE  DATABASE BROADCAST                                                           */
/****************************************************************************************************/

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

void init_database() {
    dcc_default_format = eeprom_read_byte((uint8_t*)eadr_dcc_default_format); 
    rewind_database();
} // init_database

void run_database() {
  switch (db_run_state) {
    case IDLE:
      break;

    case DB_XMIT:
      if (cur_database_entry >= total_database_entry) {
        db_run_state = IDLE;
        return;        // all done
      }
      xmit_locoentry();
      cur_database_entry++;
      db_lastMillis = millis();
      db_run_state = DB_XMIT1;
      break;

    case DB_XMIT1:
      // wait for flag reset by xpnet after transmission
      if (db_message_ready || ((millis() - db_lastMillis) < DB_UPDATE_PERIOD)) return;
      db_message_ready = 2;  // send again as call (is done by xpnet.cpp)
      db_lastMillis = millis();
      db_run_state = DB_XMIT2;
      break;

    case DB_XMIT2:
      if (db_message_ready || ((millis() - db_lastMillis) < DB_UPDATE_PERIOD)) return;
      db_message_ready = 1; // send again as message (is done by xpnet.cpp)
      db_lastMillis = millis();
      db_run_state = DB_XMIT3;
      break;

    case DB_XMIT3:
      if (db_message_ready || ((millis() - db_lastMillis) < DB_UPDATE_PERIOD)) return;
      db_message_ready = 2;  // send again as call (is done by xpnet.cpp)
      db_lastMillis = millis();
      db_run_state = DB_XMIT4;
      break;

    case DB_XMIT4:
      if (db_message_ready || ((millis() - db_lastMillis) < DB_UPDATE_PERIOD)) return;
      db_run_state = DB_XMIT;
      break;
  }
} // run_database

// deze functie start de db broadcast transmission over xpnet
void do_xmit_database() {
    if (db_run_state != IDLE) return;       // block reentry
    db_run_state = DB_XMIT;
    rewind_database();
    calc_database_size();
    cur_database_entry = 0;                 // restart
} // do_xmit_database
