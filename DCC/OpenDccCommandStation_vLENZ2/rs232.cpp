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
// file:      rs232.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2006-04-28 V0.02 added set_led*** and timeout
//            2006-05-15 V0.03 removed early sei();
//            2006-10-17 V0.04 baudrate types for intellibox mode
//            2006-11-16 V0.05 added push_to_rx for easier simulation
//            2007-01-27 V0.06 changed to 2 stop bits to clear some 
//                             trouble with USB-to-Serial Converter
//            2007-06-09 V0.07 new function tx_all_sent
//                             reset txc on data transmission 
//            2008-04-04 V0.08 used for sniffer - runs on Atmega162,
//                             UART0; 
//            2008-04-17 V0.09 fixed a bug in init U2X was cleared
//                             unintentionally; added better cast!
//            2008-07-09 V0.10 back port; UART is now generic
//            2011-03-10 V0.11 rs232_is_break dazu
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for RS232
//            see also:
//            http://www.roboternetz.de/wissen/index.php/UART_mit_avr-gcc
//
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
#include "hardware.h"
#include "status.h"
#include "rs232.h"

#ifndef FALSE 
  #define FALSE  (1==0)
  #define TRUE   (1==1)
#endif

//
//=====================================================================
// resolve UART definitions
#if (__AVR_ATmega32__)
    #define my_UCSRA  UCSRA
     #define my_RXC    RXC
     #define my_TXC    TXC
     #define my_U2X    U2X
     #define my_FE     FE
     #define my_DOR    DOR
     #define my_UDRE   UDRE
    #define my_UCSRB  UCSRB
     #define my_TXB8   TXB8
     #define my_UCSZ2  UCSZ2
     #define my_UDRIE  UDRIE
     #define my_TXEN   TXEN
     #define my_RXEN   RXEN
     #define my_RXCIE  RXCIE
    #define my_UCSRC  UCSRC
     #define my_URSEL  URSEL
     #define my_UMSEL  UMSEL
     #define my_UPM1   UPM1 
     #define my_UPM0   UPM0
     #define my_USBS   USBS
     #define my_UCSZ1  UCSZ1 
     #define my_UCSZ0  UCSZ0
     #define my_UCPOL  UCPOL
    #define my_UBRRL  UBRRL
    #define my_UBRRH  UBRRH
    #define my_UDR    UDR
#elif (__AVR_ATmega644__)
    #warning - check datasheet
#elif (__AVR_ATmega644P__)
    #define my_UCSRA  UCSR0A
     #define my_RXC    RXC0
     #define my_TXC    TXC0
     #define my_U2X    U2X0
     #define my_FE     FE0
     #define my_DOR    DOR0
     #define my_UDRE   UDRE0
    #define my_UCSRB  UCSR0B
     #define my_TXB8   TXB80
     #define my_UCSZ2  UCSZ20
     #define my_UDRIE  UDRIE0
     #define my_TXEN   TXEN0
     #define my_RXEN   RXEN0
     #define my_RXCIE  RXCIE0
    #define my_UCSRC  UCSR0C
     #define my_UMSEL0 UMSEL00
     #define my_UMSEL1 UMSEL01
     #define my_UPM1   UPM01 
     #define my_UPM0   UPM00
     #define my_USBS   USBS0
     #define my_UCSZ1  UCSZ01 
     #define my_UCSZ0  UCSZ00
     #define my_UCPOL  UCPOL0
    #define my_UBRRL  UBRR0L
    #define my_UBRRH  UBRR0H
    #define my_UDR    UDR0 
#elif (__AVR_ATmega328P__) //SDS atmega328 definitions
    #define my_UCSRA  UCSR0A
     #define my_RXC    RXC0
     #define my_TXC    TXC0
     #define my_U2X    U2X0
     #define my_FE     FE0
     #define my_DOR    DOR0
     #define my_UDRE   UDRE0
    #define my_UCSRB  UCSR0B
     #define my_TXB8   TXB80
     #define my_UCSZ2  UCSZ20
     #define my_UDRIE  UDRIE0
     #define my_TXEN   TXEN0
     #define my_RXEN   RXEN0
     #define my_RXCIE  RXCIE0
    #define my_UCSRC  UCSR0C
     #define my_UMSEL0 UMSEL00
     #define my_UMSEL1 UMSEL01
     #define my_UPM1   UPM01 
     #define my_UPM0   UPM00
     #define my_USBS   USBS0
     #define my_UCSZ1  UCSZ01 
     #define my_UCSZ0  UCSZ00
     #define my_UCPOL  UCPOL0
    #define my_UBRRL  UBRR0L
    #define my_UBRRH  UBRR0H
    #define my_UDR    UDR0 
#else    
  #warning - no or wrong processor defined    
#endif


//=====================================================================
//
// RS232
//
// purpose:   send and receive messages from pc
//
// how:       uart acts with interrupt on fifos.
//            ohter programs access only the fifos.
//            hardware handshake with RTS and CTS.
//            display of status with one LED (see status.c)
//
// uses:      set_led_rs232
//            no_timeout.rs232 (see status.c)
// interface: see rs232.h
//
// 2do:       zur Zeit wird RTS nur als connected Erkennung benutzt,
//            das Senden wird NICHT angehalten.
//
//-----------------------------------------------------------------


// FIFO-Objekte und Puffer fï¿½r die Ein- und Ausgabe
// max. Size: 255

#define RxBuffer_Size  64              // mind. 16
unsigned char RxBuffer[RxBuffer_Size];


#define TxBuffer_Size  64              // on sniffer: 128
unsigned char TxBuffer[TxBuffer_Size];

unsigned char rx_read_ptr = 0;        // point to next read
unsigned char rx_write_ptr = 0;       // point to next write
unsigned char rx_fill = 0;
unsigned char tx_read_ptr = 0;
unsigned char tx_write_ptr = 0;
unsigned char tx_fill = 0;

const unsigned long baudrate[] PROGMEM =    // ordered like in Lenz Interface!
    {9600L,   //  = 0,
     19200L,  //  = 1,
     38400L,  //  = 2,
     57600L,  //  = 3,
     115200L, //  = 4,
     2400L,   //  = 5,  // used for Intellibox 
	};

t_baud actual_baudrate;                      // index to field above

volatile unsigned char rs232_break_detected = 0;      // flag, that a break was detected
                                                    // volatile, weil aus ISR bearbeitet wird.

//-------------------------------------------------------------------
// this goes to usart i

void init_rs232(t_baud new_baud)
  {
    uint16_t ubrr;
    uint8_t sreg = SREG;
    uint8_t dummy;

    cli();

    my_UCSRB = 0;                  // stop everything

    actual_baudrate = new_baud;

                                // note calculations are done at mult 100
                                // to avoid integer cast errors +50 is added
    switch(new_baud)
	  {
      //sds info : die ubrr berekeningen werken niet altijd, omdat de precompiler er een soep van maakt!!
	    default:
		  case BAUD_2400:
  		    // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*2400L) - 1);
          ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*24L) - 100L + 50L) / 100);
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
    			my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
  		    break;
  		case BAUD_4800:
	  	    // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*4800L) - 1);
          ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*48L) - 100L + 50L) / 100);
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
		    	my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
		      break;
		  case BAUD_9600:
			    // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*9600L) - 1);
          //SDS ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*96L) - 100L + 50L) / 100);
          ubrr = 103; // 19200bps @ 16.00MHz
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
  			  my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
	  	    break;
		  case BAUD_19200:
			    // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*19200L) - 1);
          //SDS ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*192L) - 100L + 50L) / 100);
          ubrr = 51; // 19200bps @ 16.00MHz
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
			    my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
		      break;
		case BAUD_38400:
			    // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*38400L) - 1);
          //SDS ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*384L) - 100L + 50L) / 100);
          ubrr = 25; // 38400bps @ 16.00MHz
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
			    my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
		      break;
		case BAUD_57600:
  		    // ubrr = (uint16_t) ((uint32_t) F_CPU/(8*57600L) - 1);
          ubrr = (uint16_t) ((uint32_t) (F_CPU/(8*576L) - 100L + 50L) / 100);
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
      		my_UCSRA = (1 << my_RXC) | (1 << my_TXC) | (1 << my_U2X);  // High Speed Mode, nur div 8
		      break;
		case BAUD_115200:
  		    // ubrr = (uint16_t) ((uint32_t) F_CPU/(8*115200L) - 1);
          ubrr = (uint16_t) ((uint32_t) (F_CPU/(8*1152L) - 100L + 50L) / 100);
          my_UBRRH = (uint8_t) (ubrr>>8);
          my_UBRRL = (uint8_t) (ubrr);
	    		my_UCSRA = (1 << my_RXC) | (1 << my_TXC) | (1 << my_U2X);  // High Speed Mode
		      break;
	  }
   
    // FIFOs fï¿½r Ein- und Ausgabe initialisieren
    rx_read_ptr = 0;      
    rx_write_ptr = 0;
    rx_fill = 0;      
    tx_read_ptr = 0;
    tx_write_ptr = 0;
    tx_fill = 0;

    #if (__AVR_ATmega32__)
        my_UCSRC = (1 << my_URSEL)        // must be one - to address this register (also on sniffer)
                 | (0 << my_UMSEL)        // 0 = asyn mode
                 | (0 << my_UPM1)         // 00 = parity disabled
                 | (0 << my_UPM0)         
                 | (1 << my_USBS)         // 1 = tx with 2 stop bits
                 | (1 << my_UCSZ1)        // 11 = 8 or 9 bits
                 | (1 << my_UCSZ0)
                 | (0 << my_UCPOL);
    #elif (__AVR_ATmega644__)
        #warning - check datasheet
    #elif (__AVR_ATmega644P__)
        my_UCSRC = (0 << my_UMSEL1)       // 00 = async. UART 
                 | (0 << my_UMSEL0)
                 | (0 << my_UPM1)         // 00 = parity disabled
                 | (0 << my_UPM0)
                 | (1 << my_USBS)         // 1 = tx with 2 stop bits
                 | (1 << my_UCSZ1)        // 11 = 8 or 9 bits
                 | (1 << my_UCSZ0)
                 | (0 << my_UCPOL);
    #elif (__AVR_ATmega328P__) //SDS added for atmega328
        my_UCSRC = (0 << my_UMSEL1)       // 00 = async. UART 
                 | (0 << my_UMSEL0)
                 | (0 << my_UPM1)         // 00 = parity disabled
                 | (0 << my_UPM0)
                 | (1 << my_USBS)         // 1 = tx with 2 stop bits
                 | (1 << my_UCSZ1)        // 11 = 8 or 9 bits
                 | (1 << my_UCSZ0)
                 | (0 << my_UCPOL);
    #else 
    #endif

    // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren
    // Data mode 8N1, asynchron
    
    #ifdef RS232_EXTRA_STOP
        my_UCSRB = (1 << my_RXEN)
                 | (1 << my_TXEN)
                 | (1 << my_RXCIE) 
                 | (1 << my_TXB8)           // set 9th bit to 1
                 | (1 << my_UCSZ2);         // set frame length to 9bit
    #else
        my_UCSRB = (1 << my_RXEN)
                 | (1 << my_TXEN)
                 | (1 << my_RXCIE);
    #endif

    // Flush Receive-Buffer
    do
      {
        dummy = my_UDR;
      }
    while (my_UCSRA & (1 << my_RXC));

    // Rï¿½cksetzen von Receive und Transmit Complete-Flags
    my_UCSRA |= (1 << my_RXC);
    my_UCSRA |= (1 << my_TXC);
    
    dummy = my_UDR; dummy = my_UCSRA;    // again, read registers

    rs232_break_detected = 0;
    
    SREG = sreg;

}

// sds : moet nog weg!!
unsigned char rs232_is_break(void)
{
  return (FALSE);
}
/*
unsigned char rs232_is_break(void)
  {
    if (PORTD & (1 << MY_RXD))
      {
        rs232_break_detected = 0;
        return(FALSE);
      }
    else return(TRUE);
  }
  */

//---------------------------------------------------------------------------
// Empfangene Zeichen werden in die Eingabgs-FIFO gespeichert und warten dort
// Wenn bis auf einen Rest von 10 gefï¿½llt ist: CTS senden.
//

#if (__AVR_ATmega32__)
    ISR(SIG_UART_RECV)              // standard opendcc
#elif (__AVR_ATmega644__)
    #warning - check datasheet
#elif (__AVR_ATmega162__)           // sniffer
    ISR(USART0_RXC_vect) 
#elif (__AVR_ATmega644P__)
    ISR(USART0_RX_vect)             // opendcc_xp
#elif (__AVR_ATmega328P__) //SDS for atmega328p
    ISR(USART_RX_vect) 
#else 
    #warning - no or wrong processor defined
#endif
{
    if (my_UCSRA & (1<< my_FE))
	  { // Frame Error 
	    rs232_break_detected = 1;      // set flag for parser and discard
		if (my_UDR == 0)                    // this is a break sent!
	      {
		     rs232_break_detected = 1;      // set flag for parser and discard
		  }
	  }
    else
      {
        if (my_UCSRA & (1<< my_DOR))
          { // DATA Overrun -> Fatal
            // !!! 
          }

       	RxBuffer[rx_write_ptr] = my_UDR;

        rx_write_ptr++;
        if (rx_write_ptr == RxBuffer_Size) rx_write_ptr=0;
        rx_fill++;
        if (rx_fill > (RxBuffer_Size - 10))
          {
            // we are full, stop remote Tx -> set CTS off !!!!
            // removed here done with polling of CTS in status.c
    		MY_CTS_HIGH;
          }
      }
}

//----------------------------------------------------------------------------
// Ein Zeichen aus der Ausgabe-FIFO lesen und ausgeben
// Ist das Zeichen fertig ausgegeben, wird ein neuer SIG_UART_DATA-IRQ getriggert
// Ist das FIFO leer, deaktiviert die ISR ihren eigenen IRQ.

// Bei 9 Bit kï¿½nnte noch ein Stopbit erzeugt werden: UCSRB |= (1<<TXB8);



#if (DEBUG == 3)
   unsigned  char tx_char;
#endif

#if (__AVR_ATmega32__)
    ISR(SIG_UART_DATA)              // standard opendcc
#elif (__AVR_ATmega644__)
    #warning - check datasheet
#elif (__AVR_ATmega162__)      // sniffer
    ISR(USART0_UDRE_vect) 
#elif (__AVR_ATmega644P__)
    ISR(USART0_UDRE_vect)           // opendcc_xp
#elif (__AVR_ATmega328P__) //SDS for atmega328p
    ISR(USART_UDRE_vect)           
#else 
    #warning - no or wrong processor defined
#endif
  {
    if (tx_read_ptr != tx_write_ptr)
      {
        my_UCSRA |= (1 << my_TXC);              // writing a one clears any existing tx complete flag
        my_UDR = TxBuffer[tx_read_ptr];
        #if (DEBUG == 3)
            tx_char = TxBuffer[tx_read_ptr];
        #endif
        tx_read_ptr++;
        if (tx_read_ptr == TxBuffer_Size) tx_read_ptr=0;
        tx_fill--;
      }
    else
    {
        my_UCSRB &= ~(1 << my_UDRIE);           // disable further TxINT
        digitalWrite(RS485_DERE,RS485Receive);
    }
  }



//=============================================================================
// Upstream Interface
//-----------------------------------------------------------------------------
// TX:
bool tx_fifo_ready (void)
{
    if (tx_fill < (TxBuffer_Size-16))     // keep space for one complete message (16)
      {
        return(1);                        // true if enough room
      }
    else
      {
        return(0);
      }
}


// ret 1 if full
bool tx_fifo_write (const unsigned char c)
{
    TxBuffer[tx_write_ptr] = c;

    tx_write_ptr++;
    if (tx_write_ptr == TxBuffer_Size) tx_write_ptr=0;

    unsigned char mysreg = SREG;
    cli();
    tx_fill++;
    SREG = mysreg;

    digitalWrite(RS485_DERE,RS485Transmit);
    my_UCSRB |= (1 << my_UDRIE);   // enable TxINT



    //if (tx_fill < (TxBuffer_Size-16)) //sds : dit is toch niet juist??? enfin, retval wordt toch niet gebruikt
    if (tx_fill > (TxBuffer_Size-16))
      {
        return(1);
      }

    return(0);
}

// ret 1 if all is sent
bool tx_all_sent (void)
{
    if (tx_fill == 0)
      {
        if (!(my_UCSRA & (1 << my_UDRE)))  return(0);    // UDR not empty
        if (!(my_UCSRA & (1 << my_TXC)))  return(0);    // TX Completed not set
        return(1);                        
      }
    else
      {
        return(0);
      }
}

void uart_puts (const char *s)
{
    do
      {
        tx_fifo_write (*s);
      }
    while (*s++);
}


//------------------------------------------------------------------------------
// RX:
bool rx_fifo_ready (void)
{
    if (rx_read_ptr != rx_write_ptr)
      {
        return(1);     // there is something
      }
    else
      {
        return(0);  
      }
}

//-------------------------------------------------------------------
// rx_fifo_read gets one char from the input fifo
//
// there is no check whether a char is ready, this must be
// done before calling with a call to rx_fifo_ready();

unsigned char rx_fifo_read (void)
  {
    unsigned char retval;

    retval = RxBuffer[rx_read_ptr];
    rx_read_ptr++;
    if (rx_read_ptr == RxBuffer_Size) rx_read_ptr=0;

    unsigned char mysreg = SREG;
    cli();
    rx_fill--;
    SREG = mysreg;

    if (rx_fill < (RxBuffer_Size - 14))
      {
        if (MY_CTS_STATE)
          {
            // CTS is high, but we are no longer full
            // enable remote Tx -> reset CTS 
            MY_CTS_LOW;
          }
      }
    return(retval);
  }


//-----------------------------------------------------------------------------------
// testroutinen zu Parsertest
// Ersatzroutine, um ein Zeichen in den Empfangsbuffer abzulegen
// Sendepuffer kann im Simulator mit einen Databreakpoint auf UDR ï¿½berwacht werden.
void push_to_rx(unsigned char d)
  {
	RxBuffer[rx_write_ptr] = d;

    rx_write_ptr++;
    if (rx_write_ptr == RxBuffer_Size) rx_write_ptr=0;
    rx_fill++;
    if (rx_fill > (RxBuffer_Size-5))
      {
        // we are full, stop remote Tx -> set CTS off !!!!
        MY_CTS_HIGH;
        // 
      }
  }



