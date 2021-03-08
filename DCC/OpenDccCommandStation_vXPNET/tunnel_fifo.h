//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-9 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      tunnel_fifo.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-11-03 V0.1 started
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   fifo for tunneling messages on XPressnet
//
//-----------------------------------------------------------------


#ifndef __TUNNEL_FIFO_H__
#define __TUNNEL_FIFO_H__

typedef struct
  {
    unsigned char content[17];  // the message content
  } t_tunnel_message;


extern void init_tunnel(void);

//================================================== 2XP

bool tunnel_2pc_is_full(void);
bool put_in_tunnel_2pc(t_tunnel_message *new_tunnel_message);
bool tunnel_2pc_not_empty(void);
bool get_from_tunnel_2pc(t_tunnel_message *new_tunnel_message);


//================================================== 2XP

bool tunnel_2xp_is_full(void);
bool put_in_tunnel_2xp(t_tunnel_message *new_tunnel_message);
bool tunnel_2xp_not_empty(void);
bool get_from_tunnel_2xp(t_tunnel_message *new_tunnel_message);


#endif  // __TUNNEL_FIFO_H__

