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
//
//-----------------------------------------------------------------

#include "config.h"                // general structures and definitions - make your changes here
#include "hardware.h"              // hardware definitions
#include "database.h"              // format and names
#include "status.h"                // led, key, state
#include "dccout.h"                // make dcc
#include "organizer.h"             // manage commands
#include "programmer.h"            // DCC service mode

// op atmega328 is het of LENZ of XPNET
#if (XPRESSNET_ENABLED == 1)
#include "rs485.h"                 // interface to xpressnet
#include "xpnet.h"                 // xpressnet parser
#endif

#if (PARSER == LENZ)
  #include "rs232.h"               // interface to pc
  #include "lenz_parser.h"         // talk to pc (same as ibox_parser.h)
#endif

/*********************************************************************************************************/
// sds : voor de UI
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "ui.h"
#include "keys.h"
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// Create a set of new characters
const uint8_t char1[] PROGMEM = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x0E }; // lampke aan
const uint8_t char2[] PROGMEM = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x0E }; // lampke uit
const uint8_t char3[] PROGMEM = { 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F }; // leeg blokske
const uint8_t char4[] PROGMEM = { 0x1F, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x1F }; // half blokske
const uint8_t char5[] PROGMEM = { 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x04, 0x04 }; // wissel recht
const uint8_t char6[] PROGMEM = { 0x0F, 0x03, 0x05, 0x09, 0x10, 0x10, 0x10, 0x10 }; // wissel schuin
const uint8_t char7[] PROGMEM = { 0x10, 0x0, 0x6, 0x9, 0x9, 0x6, 0, 0x0 };
const uint8_t char8[] PROGMEM = { 0x0, 0x0, 0x0, 0x6, 0x9, 0x9, 0x6, 0x0 };
const uint8_t *const charBitmap[] PROGMEM = {char1,char2,char3,char4,char5,char6,char7,char8};

/*********************************************************************************************************/

#if  (TIMER2_TICK_PERIOD != (64L * 1000000L / F_CPU))    // we use div 64 on timer 2 -> 4us
    #warning TIMER2_TICK_PERIOD does not match divider!
#endif

// SDS TODO 2021
// in mijn CS is timer2 enkel nog nodig voor de xpnet slot timing
// kan dat niet met micros() ?, dan hebben we timer2 geheel niet nodig
void timer2_Init() {
  // Timer/Counter 2 initialization
  // Clock source: System Clock / 64 -> 4us (komt overeen met TIMER2_TICK_PERIOD in config.h)
  TCCR2A = (0<< COM2A1)   // 00 = normal port mode
          | (0<< COM2A0)
          | (0<< COM2B1)  // 00 = normal port mode
          | (0<< COM2B0)
          | (0<< WGM21)   // WGM = 000: normal mode
          | (0<< WGM20);
  TCCR2B = (0<< FOC2A)    // 0 = no forced compare match
          | (0<< FOC2B)   // 0 = no forced compare match
          | (0<< WGM22)   // WGM = 000: normal mode
          | (1<<CS22)     // CS = 000: stopped
          | (0<<CS21)     //      001 = run, 010 = div8, 011=div32, 100=div64, 101=div128. 
          | (0<<CS20);    //      110 = div256, 111 = div1024
  TCNT2=0x00;
} // timer2_Init

void hardware_Init() {
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
  digitalWrite(SW_ENABLE_MAIN,LOW); // main track off
  digitalWrite(SW_ENABLE_PROG,LOW); // prog track off

  // Port D
  pinMode (ROTENC_CLK,INPUT); // pullup zit op de ROTENC module
  pinMode (ROTENC_DT,INPUT); // pullup zit op de ROTENC module
  pinMode (ROTENC_SW,INPUT_PULLUP);
  pinMode (ACK_DETECTED,INPUT); // pullup is extern 10K
  pinMode (RS485_DERE,OUTPUT);

  // with xpnet the TX/RX direction will be switched by the rs485 driver
  // since CS is xpnet-master, TX-direction as init is the right choice
  // with lenz pc interface, the direction will never change.
  // but since the RS485 chip is connected to the uart, setting RS485Transmit disables the RS485 bus for receiving, 
  // so we can receive properly over usb-uart
  digitalWrite(RS485_DERE,RS485Transmit); 

  // Timer1: done in dccout_Init();
  timer2_Init();

  // Analog Comparator: Off (SDS: stond hier zo, is allicht al default in arduino)
  // Analog Comparator Input Capture by Timer/Counter 1: Off
  ACSR=0x80;

  // A7 as current measurement, will not exceed 1.1V
  analogReference(INTERNAL);
} // hardware_Init

static void lcd_Init() {
  int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));

  lcd.begin(20,4); // initialize the lcd 
  // Switch on the backlight
  //lcd.setBacklight(1); // overbodig, want by default al aan

  // init special characters
  for (int i = 0; i < charBitmapSize; i++) {
    uint8_t aBuffer[8];
    memcpy_P ((void*)aBuffer,(void*)pgm_read_word(&(charBitmap[i])),8); // eerst data copiëren van flash->sram
    lcd.createChar (i, aBuffer);
  }
  lcd.home ();
} // lcd_Init

void setup() {
  
  hardware_Init();          // all io's + globals
  database_Init();      // loco format and names
  dccout_Init();        // timing engine for dcc    

  //SDS20160823-eeprom data voorlopig niet gebruiken in test arduino
  //init_rs232((t_baud)eeprom_read_byte((uint8_t *)eadr_baudrate));   // 19200 is default for Lenz 3.0
  #if (PARSER == LENZ)
    init_rs232(BAUD_19200); 
  #endif

  #if (XPRESSNET_ENABLED == 1)
    rs485_Init();
    xpnet_Init();
  #else
    #if (PARSER == LENZ) 
      init_parser(); // command parser
    #else 
      Serial.begin(115200);
      Serial.println("CS zonder XPNET");
    #endif        
  #endif

  status_Init();         // status.cpp
  organizer_Init();     // engine for command repetition, 
                        // memory of loco speeds and types
  programmer_Init();    // State Engine des Programmers
  
  if (eeprom_read_byte(eadr_OpenDCC_Version) != OPENDCC_VERSION) {
    // oops, no data loaded or wrong version! 
    // sds : todo :add something
  }

  status_SetState(RUN_OKAY);  // start up with power enabled (or RUN_OFF, to start with power off)
  organizer_SendDccStartupMessages();   // issue defined power up sequence on tracks (sds: vreemd dat dit ook in de GOLD uitgecomment is..)
  
  lcd_Init();
  ui_Init();
  keys_Init();

} // setup

void loop() {
  // SDS TODO 2021 : zijn er time-consuming zaken die we niet willen doen tijdens programming?
  //if (!status_IsProgState())

  status_Run();            // check short and keys
  organizer_Run();        // run command organizer, depending on state,
                          // it will execute normal track operation
                          // or programming
  programmer_Run();
  #if (XPRESSNET_ENABLED == 1)
    database_Run();                  // check transfer of loco database 
  #endif
  #if (PARSER == LENZ)
    run_parser();                    // check commands from pc
  #endif

  #if (XPRESSNET_ENABLED == 1)
    xpnet_Run();
  #endif

  keys_Update();
  ui_Update();   
} // loop
