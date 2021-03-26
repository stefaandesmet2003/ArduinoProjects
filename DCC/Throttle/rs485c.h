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
// file:      rs485.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2008-07-11 V0.1 started
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for Xpressnet master (RS485)
//            here all low level routines
//
//-----------------------------------------------------------------


#ifndef __RS485C_H__
#define __RS485C_H__

typedef enum {XP_TX, XP_RX} t_X_state;

// uit hardware.h
#define RS485_DERE      4     // D4, OUT (RS485 CTRL)
#define RS485Transmit    HIGH
#define RS485Receive     LOW

//=============================================================================
// Used Hardware
//----------------------------------------------------------------------------

#define HARDWARE_SET_XP_RECEIVE    digitalWrite(RS485_DERE,RS485Receive)
#define HARDWARE_SET_XP_TRANSMIT   digitalWrite(RS485_DERE,RS485Transmit)

//=============================================================================
// Upstream Interface
//----------------------------------------------------------------------------
//
void init_rs485(void);


inline void set_XP_to_receive(void) __attribute__((always_inline));
void set_XP_to_receive(void);

inline void set_XP_to_transmit(void) __attribute__((always_inline));
void set_XP_to_transmit(void);


bool XP_send_byte (const unsigned char c);       // sends one byte with isr and fifo, bit 8 = 0
bool XP_send_call_byte (const unsigned char c);
void X_uart_puts (const char *s); //sds added here for test
inline bool XP_is_all_sent(void) __attribute__((always_inline));
//bool XP_is_all_sent(void);                       // true if fifo is empty and all data are sent

inline bool XP_is_all_sent (void) // true if fifo is empty and all data are sent
{
    return (digitalRead(RS485_DERE) != RS485Transmit);  // just look at driver bit
}


bool XP_tx_ready (void);                         // true if byte can be sent

unsigned int XP_rx_peek (void); // sds added, voor het gemak van xpc
unsigned int XP_rx_read (void);                 // reads one byte, sds : 9bits voor de client, check callbit!
bool XP_rx_ready (void);                         // true if one byte can be read

#define X_RxBuffer_Size  32              
extern unsigned int X_RxBuffer[X_RxBuffer_Size];

#define X_TxBuffer_Size  32
extern unsigned char X_TxBuffer[X_TxBuffer_Size];

extern unsigned char X_rx_read_ptr;        // point to next read
extern unsigned char X_rx_write_ptr;       // point to next write
extern unsigned char X_rx_fill;
extern unsigned char X_tx_read_ptr;
extern unsigned char X_tx_write_ptr;
extern unsigned char X_tx_fill;


#endif  // __RS485C_H__
