//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-2008 Wolfgang Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      tunnel_fifo.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-11-04 V0.01 start
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   fifo for tunneling messages on XPressnet
//
//-----------------------------------------------------------------


#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>         // put var to program memory
#include <avr/eeprom.h>
#include <string.h>

#include "config.h"               // general structures and definitions

#include "tunnel_fifo.h"           // Xpressnet extension 0x3*

#if (XPRESSNET_ENABLED == 1)
#if (XPRESSNET_TUNNEL == 1)

#define SIZE_TUNNEL_2PC     4
#define SIZE_TUNNEL_2XP     3


t_tunnel_message queue_2pc[SIZE_TUNNEL_2PC];

t_tunnel_message queue_2xp[SIZE_TUNNEL_2XP];

unsigned char tunnel_2pc_read;            // points to next read
unsigned char tunnel_2pc_write;           // points to next write
                                          // rd = wr: queue empty
                                          // rd = wr + 1: queue full

unsigned char tunnel_2xp_read;            // points to next read
unsigned char tunnel_2xp_write;           // points to next write
                                          // rd = wr: queue empty
                                          // rd = wr + 1: queue full


void init_tunnel(void)
  {
    tunnel_2pc_read = 0;
    tunnel_2pc_write = 0;

    tunnel_2xp_read = 0;
    tunnel_2xp_write = 0;
  }


//================================================== 2PC

bool tunnel_2pc_is_full(void)
  {
    unsigned char i;
    
    // check for full
    i = tunnel_2pc_write+1;
    if (i == SIZE_TUNNEL_2PC) i = 0;
    if (i == tunnel_2pc_read) return(1);         // totally full
    return(0);
  }



bool put_in_tunnel_2pc(t_tunnel_message *new_tunnel_message)
  {
    unsigned char i;

    memcpy(&queue_2pc[tunnel_2pc_write], new_tunnel_message, sizeof(t_tunnel_message));

    tunnel_2pc_write++;
    if (tunnel_2pc_write == SIZE_TUNNEL_2PC) tunnel_2pc_write = 0;

    // check for full
    i = tunnel_2pc_write+1;
    if (i == SIZE_TUNNEL_2PC) i = 0;
    if (i == tunnel_2pc_read) return(1);         // totally full
    return(0);
  }

bool tunnel_2pc_not_empty(void)
  {
    if (tunnel_2pc_write == tunnel_2pc_read) return(0);
    return(1);
  }

bool get_from_tunnel_2pc(t_tunnel_message *new_tunnel_message)
  {
    // first do a check before calling this function!
    if (tunnel_2pc_write == tunnel_2pc_read) return(0);

    memcpy(new_tunnel_message, &queue_2pc[tunnel_2pc_read], sizeof(t_tunnel_message));

    tunnel_2pc_read++;
    if (tunnel_2pc_read == SIZE_TUNNEL_2PC) tunnel_2pc_read = 0;   // advance pointer

    return(1);
  } 



//================================================== 2XP

bool tunnel_2xp_is_full(void)
  {
    unsigned char i;
    
    // check for full
    i = tunnel_2xp_write+1;
    if (i == SIZE_TUNNEL_2XP) i = 0;
    if (i == tunnel_2xp_read) return(1);         // totally full
    return(0);
  }



bool put_in_tunnel_2xp(t_tunnel_message *new_tunnel_message)
  {
    unsigned char i;

    memcpy(&queue_2xp[tunnel_2xp_write], new_tunnel_message, sizeof(t_tunnel_message));

    tunnel_2xp_write++;
    if (tunnel_2xp_write == SIZE_TUNNEL_2XP) tunnel_2xp_write = 0;

    // check for full
    i = tunnel_2xp_write+1;
    if (i == SIZE_TUNNEL_2XP) i = 0;
    if (i == tunnel_2xp_read) return(1);         // totally full
    return(0);
  }

bool tunnel_2xp_not_empty(void)
  {
    if (tunnel_2xp_write == tunnel_2xp_read) return(0);
    return(1);
  }

bool get_from_tunnel_2xp(t_tunnel_message *new_tunnel_message)
  {
    // first do a check before calling this function!
    if (tunnel_2xp_write == tunnel_2xp_read) return(0);

    memcpy(new_tunnel_message, &queue_2xp[tunnel_2xp_read], sizeof(t_tunnel_message));

    tunnel_2xp_read++;
    if (tunnel_2xp_read == SIZE_TUNNEL_2XP) tunnel_2xp_read = 0;   // advance pointer

    return(1);
  } 


#endif // (XPRESSNET_TUNNEL == 1)
#endif // (XPRESSNET_ENABLED == 1)

