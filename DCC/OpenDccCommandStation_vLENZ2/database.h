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

#if (LOCO_DATABASE == NAMED)
typedef struct
  {
    union
      {
        struct
          {
            unsigned int addr:14;
            t_format format:2;
          } w;
        unsigned char b[sizeof(unsigned int)];
      } ;
    union
      {
        unsigned int pid;                             // picture ID
        unsigned char pid_b[sizeof(unsigned int)];
      } ;
    unsigned char name[LOK_NAME_LENGTH];          // multiMaus supports up to 5 chars
  } t_locoentry;

  extern unsigned char db_message[17]; 
  extern unsigned char db_message_ready;            // interface flag to Xpressnet

  void init_database(void);           // at power up

  void rewind_database(void);         // before dump, do a rewind

  void run_database(void);            // multitask replacement, call in loop

  void do_xmit_database(void);        // start transfer on Xpressnet

  void clear_loco_database(void);     // delet all

  void dump_loco_database(void);

  unsigned char get_loco_data(t_locoentry *actual);

  t_format get_loco_format(unsigned int addr);   // if loco not in data base - we return default format

  unsigned char format_to_uint8(t_format myformat);

  unsigned char store_loco_format(unsigned int addr, t_format format);

  unsigned char store_loco_name(unsigned int addr, unsigned char * name);

#else  // (LOCO_DATABASE == NAMED)

  void init_database(void);

  t_format get_loco_format(unsigned int addr);

  unsigned char format_to_uint8(t_format myformat);

  unsigned char store_loco_format(unsigned int addr, t_format format);

#endif // (LOCO_DATABASE == NAMED)




