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
// file:      main.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-10-24 V0.2 check jumpers
//            2007-01-11 V0.3 clean up
//            2008-01-16 V0.4 call to s88 only if not prog_state
//            2008-07-19 V0.5 external stop
//            2008-08-25 V0.6 database added
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
//            a replacement for Lenz, Intellibox or HSI88
//            
//            See CONFIG.H about more information and how to
//            configure the system
//
// content:   init Atmel AVR ATmega
//            check jumpers
//            init states
//            startup ISR for DCC-OUT
//            idle-loop
//
//-----------------------------------------------------------------

#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <string.h>

#include "config.h"                // general structures and definitions - make your changes here

#include "database.h"              // format and names
#include "status.h"                // led, key, state
#include "dccout.h"                // make dcc
#include "organizer.h"             // manage commands
#include "programmer.h"            // DCC service mode

#if (XPRESSNET_ENABLED == 1)
#include "rs485.h"                 // interface to xpressnet
#include "xpnet.h"                 // xpressnet parser
#endif


#if (PARSER == LENZ)
    #include "rs232.h"                 // interface to pc
    #include "lenz_parser.h"         // talk to pc (same as ibox_parser.h)
    #warning Parseremulation = LENZ
#endif
#if (PARSER == INTELLIBOX)
    #include "rs232.h"                 // interface to pc
    #include "ibox_parser.h"         
    #warning Parseremulation = INTELLIBOX
#endif
//SDS #include "s88.h"                   // s88-bus

/*********************************************************************************************************/
// sds : voor de UI
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "ui.h"
#include "keys.h"
LiquidCrystal_I2C lcd( 0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE );
// Creat a set of new characters
const uint8_t charBitmap[][8] = {
   { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x0E }, // lampke aan
   { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x0E }, // lampke uit
   { 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F }, // leeg blokske
   { 0x1F, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x1F }, // half blokske
   { 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04 }, // wissel recht
   { 0x0F, 0x03, 0x05, 0x09, 0x10, 0x10, 0x10, 0x10 }, // wissel schuin
   { 0x10, 0x0, 0x6, 0x9, 0x9, 0x6, 0, 0x0 },
   { 0x0, 0x0, 0x0, 0x6, 0x9, 0x9, 0x6, 0x0 },
};

/*********************************************************************************************************/

//-------------------------------------------------------------------------------------------
// (32L * 1000000L / F_CPU)    // we use div 64 on timer 2 -> 4us

#if  (TIMER2_TICK_PERIOD != (64L * 1000000L / F_CPU))    // we use div 64 on timer 2 -> 4us
    #warning TIMER2_TICK_PERIOD does not match divider!
#endif

void init_timer2(void)
  {
    #if (__AVR_ATmega644P__)
        // Timer/Counter 2 initialization
        // Clock source: System Clock / 32 -> 2us
        TCCR2A = (0<< COM2A1)   // 00 = normal port mode
               | (0<< COM2A0)
               | (0<< COM2B1)   // 00 = normal port mode
               | (0<< COM2B0)
               | (0<< WGM21)    // WGM = 000: normal mode
               | (0<< WGM20);
        TCCR2B = (0<< FOC2A)    // 0 = no forced compare match
               | (0<< FOC2B)    // 0 = no forced compare match
               | (0<< WGM22)    // WGM = 000: normal mode
               | (1<<CS22)      // CS = 000: stopped
               | (0<<CS21)      //      001 = run, 010 = div8, 011=div32, 100=div64, 101=div128. 
               | (0<<CS20);     //      110 = div256, 111 = div1024
        TCNT2=0x00;
    #elif (__AVR_ATmega644__)
        // Timer/Counter 2 initialization
        // Clock source: System Clock / 32 -> 2us
        // Mode: Normal top=FFh
        // OC2 output: Disconnected
        TCCR2 =  (0<<  FOC2)    // 0 = no forced compare match
               | (0<<  WGM20)   // WGM = 00: normal mode  
               | (0<<  COM21)   // 00 = normal port mode
               | (0<<  COM20)
               | (0<<  WGM21)
               | (1<<  CS22)    // CS = 000: stopped
               | (0<<  CS21)    //      001 = run, 010 = div8, 011=div32, 100=div64, 101=div128. 
               | (0<<  CS20);   //      110 = div256, 111 = div1024
        TCNT2=0x00;
    #elif (__AVR_ATmega32__)
        // Timer/Counter 2 initialization
        // Clock source: System Clock / 32 -> 2us
        // Mode: Normal top=FFh
        // OC2 output: Disconnected
        TCCR2 =  (0<<  FOC2)    // 0 = no forced compare match
               | (0<<  WGM20)   // WGM = 00: normal mode  
               | (0<<  COM21)   // 00 = normal port mode
               | (0<<  COM20)
               | (0<<  WGM21)
               | (1<<  CS22)    // CS = 000: stopped
               | (0<<  CS21)    //      001 = run, 010 = div8, 011=div32, 100=div64, 101=div128. 
               | (0<<  CS20);   //      110 = div256, 111 = div1024
        TCNT2=0x00;
    #elif (__AVR_ATmega328P__) //SDS for atmega328 - zelfde als 644P
        // Timer/Counter 2 initialization
        // Clock source: System Clock / 32 -> 2us
        TCCR2A = (0<< COM2A1)   // 00 = normal port mode
               | (0<< COM2A0)
               | (0<< COM2B1)   // 00 = normal port mode
               | (0<< COM2B0)
               | (0<< WGM21)    // WGM = 000: normal mode
               | (0<< WGM20);
        TCCR2B = (0<< FOC2A)    // 0 = no forced compare match
               | (0<< FOC2B)    // 0 = no forced compare match
               | (0<< WGM22)    // WGM = 000: normal mode
               | (1<<CS22)      // CS = 000: stopped
               | (0<<CS21)      //      001 = run, 010 = div8, 011=div32, 100=div64, 101=div128. 
               | (0<<CS20);     //      110 = div256, 111 = div1024
        TCNT2=0x00;
    #else 
       #error: counter2 undefined
    #endif
  }



void init_main(void)
    {
    // Input/Output Ports initialization

    // Port B
    pinMode(BUTTON_GREEN, INPUT); // pullup is extern 10K
    pinMode(BUTTON_RED, INPUT); // pullup is extern 10K
    pinMode(NDCC, OUTPUT); // pullup is extern 10K
    pinMode(DCC, OUTPUT); // pullup is extern 10K
    digitalWrite(DCC,HIGH);
    digitalWrite(NDCC,LOW);

    // Port C
    pinMode (NSHORT_PROG,INPUT); // uitgang van 7414, geen pullup nodig
    pinMode (NSHORT_MAIN,INPUT); // uitgang van 7414, geen pullup nodig
    pinMode (SW_ENABLE_MAIN,OUTPUT);
    pinMode (SW_ENABLE_PROG,OUTPUT);
    pinMode (EXT_STOP,INPUT); // pullup is extern 10K
    digitalWrite(SW_ENABLE_MAIN,LOW);
    digitalWrite(SW_ENABLE_PROG,LOW);

    // Port D
    pinMode (ROTENC_CLK,INPUT); // pullup zit op de ROTENC module
    pinMode (ROTENC_DT,INPUT); // pullup zit op de ROTENC module
    pinMode (ROTENC_SW,INPUT_PULLUP);
    pinMode (ACK_DETECTED,INPUT); // pullup is extern 10K
    pinMode (RS485_DERE,OUTPUT);
    digitalWrite(RS485_DERE,RS485Receive);

    // Timer/Counter 0 initialization: done in Status.c
    
    // Timer1: done in init_dccout();

    init_timer2();

    // Analog Comparator initialization
    // Analog Comparator: Off
    // Analog Comparator Input Capture by Timer/Counter 1: Off
    ACSR=0x80;
    // SFIOR=0x00;   // PUD = 0: no "pull up disabled"
    //SDS 201610 : waarom is SFIOR lijn hier gecomment?? niet in de GOLD

    invert_accessory = eeprom_read_byte((uint8_t*)eadr_invert_accessory);     // used for Lenz and XP
    xpressnet_feedback_mode = eeprom_read_byte((uint8_t*)eadr_xpressnet_feedback); 
   
  }

void init_interrupt(void)
  {
    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: Off
    // INT2: Off
    // INT3: Off
    // INT4: Off
    // INT5: Off
    // INT6: Off
    // INT7: Off
    // INT7 Mode: Low level

    // Timer(s)/Counter(s) Interrupt(s) initialization
  
    #if (__AVR_ATmega644P__)
        // TIMSK0: OCIE0B OCIE0A TOIE0
        

    #elif (__AVR_ATmega644__)
       
    #elif (__AVR_ATmega32__)
        TIMSK = (0<<OCIE2)       // Output Compare
              | (0<<TOIE2)
              | (0<<TICIE1)      // Input Capture Int
              | (1<<OCIE1A)      // Output Compare A
              | (0<<OCIE1B)      // Output Compare B
              | (0<<TOIE1)       // war 1::::: Timer1/Counter1 Overflow Int Enable
              | (0<<OCIE0)
              | (0<<TOIE0);      // Timer0 Overflow
    #elif (__AVR_ATmega328P__)
      //SDS moeten we hier iets doen voor atmega328??
    #else
      #error:  undefined
    #endif
  


    sei();              // Global enable interrupts

  }



void dcc_startup_messages (void)
  {
    t_message testmess;
    t_message *testmessptr;
    testmessptr = &testmess;

    // 20 Reset Pakete

    testmessptr = &DCC_Reset;
    set_next_message(testmessptr);
    next_message_count = 20;

    while (next_message_count > 0)      // wait
      {
        delay(1);
      }
    // 10 Idle Pakete

    testmessptr = &DCC_Idle;
    set_next_message(testmessptr);
    next_message_count = 10;

    while (next_message_count > 0)      // wait
      {
        delay(1);
      }
  }



//===============================================================================
//
//  Main Program: There are several branches - depending on jumper settings
//
//  Jumpers:
//  0  0     Run as OpenDCC, either Lenz or Intellibox mode
//
//===============================================================================


// Run as DCC central station, emulating Lenz LI101 or Uhlenbrock Intellibox
// see config.h for compile swtiches

static void setup_lcd()
{
    int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));

    lcd.begin(20,4); // initialize the lcd 
    // Switch on the backlight
    //lcd.setBacklight(1); // overbodig, want by default al aan

    // init special characters
    for ( int i = 0; i < charBitmapSize; i++ )
    {
        lcd.createChar ( i, (uint8_t *)charBitmap[i] );
    }

    ui_Init ();
    keys_Init ();

    lcd.home ();
} // setup_lcd



void setup() {
   
    init_main();                                  // all io's + globals
    init_database();                    // loco format and names
    init_interrupt();
    init_dccout();                      // timing engine for dcc    
    
    //init_rs232((t_baud)eeprom_read_byte((uint8_t *)eadr_baudrate));   // 19200 is default for Lenz 3.0
                                                                        // see also DEFAULT_BAUD

    #if (PARSER == LENZ)
        init_rs232(BAUD_19200); 
    #endif
    #if (PARSER == INTELLIBOX)
        init_rs232(BAUD_19200); 
    #endif
    #if (PARSER == SDS_BOX)
        Serial.begin(115200);
    #endif

    #if (XPRESSNET_ENABLED == 1)
      init_rs485();
      init_xpressnet();
    #endif
       
    init_state();                      // 5ms timer tick 
    #if (PARSER == LENZ) 
        init_parser(); // command parser
    #endif        
    #if (PARSER == INTELLIBOX) 
        init_parser(); // command parser
    #endif        
    

    init_organizer();                   // engine for command repetition, 
                                        // memory of loco speeds and types
    init_programmer();             // State Engine des Programmers
    
    #if (S88_ENABLED == 1)
     init_s88(READ_FROM_EEPROM);  
    #endif

    if (eeprom_read_byte((void *)eadr_OpenDCC_Version) != OPENDCC_VERSION)
    {
        // oops, no data loaded or wrong version! 
        // sds : todo :add something
    }

    //set_opendcc_state(RUN_OFF);         // start up with power off
    set_opendcc_state(RUN_OKAY);         // start up with power enabled
    
    // dcc_startup_messages();          // issue defined power up sequence on tracks (sds: vreemd dat dit ook in de GOLD uitgecomment is..)
    setup_lcd();

} // setup

void loop() 
{
    // put your main code here, to run repeatedly:
    run_state();                     // check short and keys
    run_organizer();                 // run command organizer, depending on state,
                                   // it will execute normal track operation
                                   // or programming
    run_programmer();
    #if ((XPRESSNET_ENABLED == 1) && (LOCO_DATABASE == NAMED))
        run_database();                  // check transfer of loco database 
    #endif
    #if (PARSER == LENZ) //sds 
        run_parser();                    // check commands from pc
    #endif
    #if (PARSER == INTELLIBOX) //sds 
        run_parser();                    // check commands from pc
    #endif

    #if (S88_ENABLED == 1)
        if (!is_prog_state()) run_s88(); // s88 has busy loops, we block it when programming
                                       // this is no longer true - but keep it blocked
    #endif

    #if (XPRESSNET_ENABLED == 1)
        run_xpressnet();
    #endif

    keys_Update();
    ui_Update();   


} // loop
