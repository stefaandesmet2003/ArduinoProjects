#include "NmraDcc.h"
#include "AccessoryDecoder.h"
#include "TurnoutDecoder.h"
#include "Timer.h"

// This Example shows how to use the library as a DCC Accessory Decoder or a DCC Signalling Decoder
// It responds to both the normal DCC Turnout Control packets and the newer DCC Signal Aspect packets 
// You can also print every DCC packet by uncommenting the "#define NOTIFY_DCC_MSG" line below

/*
// dcc.process() verwerkt exact 1 DCC packet. Vanuit dcc.process wordt de notify functie aangeroepen, die het packet verwerkt
// als de processing langer dan 1 DCC packet duurt, dan gaan er packetten verloren. NmraDcc heeft een buffer van 1 pakket!! (dccRx.PacketCopy)
De Rx ISR gaat gewoon door, en gaat op het einde van het pakket de bitbuffer in PacketCopy copiëren, ongeacht dataReady al op 0 stond

*/
// DCC signaal ontvangen op pin D2
// als je FLAGS_MY_ADDRESS_ONLY gebruikt worden enkel voor dit adres notifyDccAccState callbacks gedaan
// de DCC isr ontvangt nog steeds alle pakketten, en notifyDccMsg dus ook (indien niet NULL)
// zonder FLAGS_MY_ADDRESS_ONLY kan je voor alle accessory decoders  notifyDccAccState ontvangen
// ihb accessory decoder pakketten voor 2 verschillende adressen, om bv een decoder voor 8 wissels te maken
// om een factory reset te doen moet je een CV write naar CV8 doen
// om het adres te wijzigen doe je CV write (direct byte) naar CV1 en/of CV9

#define VersionId 0x1 // versie van deze software

NmraDcc  Dcc ;
DCC_MSG  Packet ; //SDS : nodig?

enum {DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM } decoderState;
uint8_t decoderSoftwareMode;

//factory defaults
  
static CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_29_CONFIG,0x80},
  {CV_SoftwareMode, 0} 
};

uint8_t accessory_FactoryResetCV( void );

// keyhandling
#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 1000
#define NUMBER_OF_KEYS 4
typedef enum {UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP} keyState_t;
typedef struct
{
  uint8_t Pin;
  keyState_t State;
  uint32_t LastMillis;
} key_t;

key_t keys[NUMBER_OF_KEYS]; // 4 keys, key1 = progkey

void init_keys (void);
void detect_keys (void);
void keyHandler (uint8_t keyEvent);

// timers

Timer timer;
int8_t ledTimer;

// 0 = DONE, 1 = NOT DONE (eeprom not ready)
uint8_t accessory_FactoryResetCV( void )
{
  int i;
  uint8_t swMode; // we gebruiken hier een read uit eeprom, niet de global var
  
  if (!Dcc.isSetCVReady())
    return 1;
  // factory reset de algemene CV's (die niet afhangen van de software mode
  for (i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++)
  {
    Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  }
  swMode = Dcc.getCV(CV_SoftwareMode);
  switch(swMode)
  {
    case SOFTWAREMODE_TURNOUT_DECODER :
      turnout_FactoryResetCV();
      break;
    case SOFTWAREMODE_LIGHT_DECODER :
      //todo ! 
      break;
    default :
      // er staat een unsupported software mode in CV33 --> we gaan die terugzetten naar TURNOUT_DECODER en de TURNOUT_DECODER factory defaults zetten
      Dcc.setCV(CV_SoftwareMode,SOFTWAREMODE_TURNOUT_DECODER);
      turnout_FactoryResetCV();
      break;
  }
  
  return 0;
  
} // accessory_FactoryResetCV

uint16_t getMyAddr(void); // uit nmraDcc.cpp --> in de Dcc class steken?

void setup()
{
  decoderState = DECODER_INIT;

  Serial.begin(115200);
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( PIN_ACKOUT, OUTPUT );
  digitalWrite( PIN_ACKOUT, LOW );
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, PIN_DCCIN, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  //SDS  Dcc.init( FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER | FLAGS_MY_ADDRESS_ONLY, 0 );
  Dcc.init( FLAGS_OUTPUT_ADDRESS_MODE | FLAGS_DCC_ACCESSORY_DECODER, 0 );

  // progled, progkey
  pinMode (PIN_PROGKEY, INPUT_PULLUP);
  // key2-4
  pinMode (PIN_KEY2, INPUT_PULLUP);
  pinMode (PIN_KEY3, INPUT_PULLUP);
  pinMode (PIN_KEY4, INPUT_PULLUP);
  // led
  pinMode (PIN_PROGLED, OUTPUT);
  digitalWrite(PIN_PROGLED, LOW); //LED uit

  // check if factory defaults are written in eeprom
  if (!Dcc.isSetCVReady())
  {
    // we kunnen geen CV's inlezen, dus weten we niet wat te doen!!
    // goto safe mode --> alles outputs uit en wachten op progkey
    return;
  }
  if (Dcc.getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY)
  {
    decoderState = DECODER_FACTORY_RESET;
  }
  else
  {
    decoderState = DECODER_INIT;
  }
  // init key handling
  init_keys ();
  
} // setup

void loop()
{
  switch (decoderState)
  {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0)
      {
        // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      Serial.print("my address is : ");
      Serial.println(getMyAddr());
      decoderSoftwareMode = Dcc.getCV(CV_SoftwareMode);
      switch (decoderSoftwareMode)
      {
        case SOFTWAREMODE_TURNOUT_DECODER :
          turnout_Init ();
          break;
         case SOFTWAREMODE_LIGHT_DECODER :
          // SDS : TODO!
          break;
      }
      decoderState = DECODER_RUNNING;
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      Dcc.process();
      break;
  }
  
  // check key input
  detect_keys();

  // run timers
  timer.update();
  
} // loop


/******************************************************************/

void init_keys (void)
{
    keys[0].Pin = PIN_PROGKEY;
    keys[0].State = UP;
    keys[1].Pin = PIN_KEY2;
    keys[1].State = UP;
    keys[2].Pin = PIN_KEY3;
    keys[2].State = UP;
    keys[3].Pin = PIN_KEY4;
    keys[3].State = UP;
    // keys[0].Lastmillis = 0; //init overbodig
} // init_keys

void detect_keys (void)
{
  int i;
  for (i=0;i<NUMBER_OF_KEYS;i++)
  {
    switch(keys[i].State)
    {
      case UP : 
        if (digitalRead(keys[i].Pin) == LOW )
        {
          keys[i].LastMillis = millis();
          keys[i].State = DEBOUNCING_DOWN;
        }
        break;
      case DEBOUNCING_DOWN :
        if (digitalRead(keys[i].Pin) != LOW )
        {
          keys[i].State = UP;
        }
        else if ( (millis() - keys[i].LastMillis) > DEBOUNCE_DELAY )
        {
          keys[i].State = DOWN;
          keyHandler ((i<<2) | KEYEVENT_DOWN); // gebruik index in de keys[] array als keycode
        }
        break;
      case DOWN :
        if (digitalRead(keys[i].Pin) != LOW )
        {
          keys[i].State = DEBOUNCING_UP;
          keys[i].LastMillis = millis();
        }
        else if ( (millis() - keys[i].LastMillis) > LONGPRESS_DELAY )
        {
          keys[i].State = LONG_DOWN;
          keyHandler ((i<<2) | KEYEVENT_DOWN | KEYEVENT_LONGDOWN );
        }
        break;
      case LONG_DOWN :
        if (digitalRead(keys[i].Pin) != LOW )
        {
          keys[i].State = DEBOUNCING_UP;
          keys[i].LastMillis = millis();
        }
        break;
      case DEBOUNCING_UP :
        if (digitalRead(keys[i].Pin) == LOW )
        {
          keys[i].LastMillis = millis();
        }
        else if ( (millis() - keys[i].LastMillis) > DEBOUNCE_DELAY )
        {
          keys[i].State = UP;
          keyHandler ((i<<2) | KEYEVENT_UP);
        }
        break;
    }
  }
} // detect_keys

void keyHandler (uint8_t keyEvent)
{
    // debug info
    switch (keyEvent & 0x3)
    {
        case KEYEVENT_UP :
            Serial.print("key up");
            break;
        case KEYEVENT_DOWN :
            Serial.print("key down");
            break;
        case KEYEVENT_LONGDOWN :
            Serial.print("long key down");
            break;
    }
    Serial.print (" on key : ");
    Serial.println(keyEvent>>2);
    
    if (keyEvent & KEYEVENT_LONGDOWN)
    {
        if (decoderState != DECODER_PROGRAM)
        {
            decoderState = DECODER_PROGRAM;
            ledTimer = timer.oscillate(PIN_PROGLED, LED_SLOW_FLASH, HIGH, -1); //forever
        }
    }
    else if (keyEvent & KEYEVENT_DOWN)
    {
        if (decoderState == DECODER_PROGRAM)
        {
            decoderState = DECODER_INIT;
            timer.stop (ledTimer); //stop de slow flashing led
        }
        else
        {
            // we gebruiken de knoppen om wissels te bedienen
            turnout_ManualToggle (keyEvent >> 2);
        }
    }
    else // key up
    {
    }
    
} // keyHandler


/**********************************************************************************************/
/* notify functies uit Nmra layer */
/**********************************************************************************************/

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void)
{
  Serial.println("notifyCVAck") ;
  
  digitalWrite( PIN_ACKOUT, HIGH );
  delay( 6 );  
  digitalWrite( PIN_ACKOUT, LOW );
}

//SDS added 20160824
void notifyDccReset(uint8_t hardReset )
{
  //Serial.println("notifyDccReset") ;
}

//#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg[ ") ;
  Serial.print(millis());
  Serial.print("] : ");
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State)
{
  if (decoderSoftwareMode == SOFTWAREMODE_TURNOUT_DECODER)
  {
    turnout_Handler (Addr, BoardAddr, OutputAddr, State);
  }
  else if (decoderSoftwareMode == SOFTWAREMODE_LIGHT_DECODER)
  {
    Serial.println("unsupported software mode!");
    // TODO
  }
  else
  {
    Serial.println("unsupported software mode!");
    //goto safe mode??
  }
}

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState( uint16_t Addr, uint8_t OutputIndex, uint8_t State)
{
  Serial.print("notifyDccSigState: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(OutputIndex,DEC) ;
  Serial.print(',');
  Serial.println(State, HEX) ;
}

// enkel CV's schrijven als decoder niet 'live' is (INIT, FACTORY_RESET, PROGRAM)
uint8_t notifyCVWrite( uint16_t CV, uint8_t Value)
{
  // sds : for debug
    Serial.print ("CV Write ");
    Serial.print(CV,DEC);
    Serial.print(" = ");
    Serial.println(Value,DEC);
  if (decoderState == DECODER_RUNNING)
  {
    Serial.println("no CVs written while decoder RUNNING");
  }
  
  if (decoderState != DECODER_RUNNING)
  {
    if( eeprom_read_byte( (uint8_t*) CV ) != Value )
    {
      eeprom_write_byte( (uint8_t*) CV, Value ) ;
  
      if( notifyCVChange )
        notifyCVChange( CV, Value) ;
    }
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
  }

  return eeprom_read_byte( (uint8_t*) CV ) ;
  
} // notifyCVWrite

// 0 = ongeldige schrijfactie gevraagd naar CV
// 0 = lezen naar niet-geïmplementeerde CV's : todo SDS!
// 1 = geldige actie
uint8_t notifyCVValid( uint16_t CV, uint8_t Writable )
{
  // write-request naar CV_MANUFACTURER_ID triggert een factory reset
  if( (CV == CV_MANUFACTURER_ID )  && Writable )
    decoderState = DECODER_FACTORY_RESET;

  uint8_t Valid = 1 ;

  if( CV > E2END )
    Valid = 0 ;

  if( Writable && ( ( CV ==CV_VERSION_ID ) || (CV == CV_MANUFACTURER_ID ) ) )
    Valid = 0 ;
  // TODO : uitbreiden!! (ook afhankelijk van swMode CV33)

  return Valid ;  
} // notifyCVValid


