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
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-12-04 V0.2 change to multitasking

// 2do:       not completed
//            missing: notify stolen loco
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   xpressnet Interface (protocol layer)


void init_xpressnet(void);

void run_xpressnet(void);                       // multitask replacement

void xp_send_message(unsigned char slot, unsigned char *str);   // send a message to this slot

// debug only

extern unsigned char rx_message[17];             // current message from client

extern unsigned char tx_message[17];             // current message from master




