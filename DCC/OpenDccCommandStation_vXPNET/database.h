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
// file:      database.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-08-25 V0.01 started
//           
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   handling of lok names and speed steps
//            (This is not locobuffer, this is the data base,
//            has nothing to do with the actual dcc content)
//
// interface upstream: 
//
//-----------------------------------------------------------------

typedef struct {
  union {
    struct {
        unsigned int addr:14;
        unsigned int format:2;
    } w;
    unsigned char b[sizeof(unsigned int)];
  };
  unsigned char name[LOK_NAME_LENGTH];          // multiMaus supports up to 5 chars
} locoentry_t;

// TODO SDS2021 : steek dit in de public interface ipv global vars
extern unsigned char db_message[17]; 
extern unsigned char db_message_ready;            // interface flag to Xpressnet

void init_database();           // at power up
void run_database();            // multitask replacement, call in loop
void do_xmit_database();        // start transfer on Xpressnet

void clear_loco_database();     // delete all entries
void reset_loco_database();     // factory reset the entries
unsigned char get_loco_data(locoentry_t *actual);
t_format get_loco_format(unsigned int addr);   // if loco not in data base - we return default format
//TODO SDS2021, is het niet beter om een get_loco_data (addr, locoentry_t*) te hebben??
// we willen de data ook voor local ui makkelijk uit de db halen
uint8_t get_loco_name(unsigned int addr, uint8_t *name); // sds temp??
unsigned char store_loco_format(unsigned int addr, t_format format);
unsigned char store_loco_name(unsigned int addr, unsigned char * name);





