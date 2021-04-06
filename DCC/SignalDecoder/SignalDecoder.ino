#include "NmraDcc.h"

#define VersionId (0x1)
#define PIN_DCCIN 2
typedef struct {
  uint8_t pin;
  uint8_t ledOn;
  uint8_t ledOff;
  uint8_t fOnCycle;
  uint8_t fOffCycle;
  uint8_t fTotalCycles;
  uint8_t fCycle; //0->15
} output_t;

typedef struct {
  output_t outputs[5];
  uint8_t ms;
  uint8_t bCycle; // 0->9
} head_t;

typedef enum {
  DECODER_FACTORY_RESET,DECODER_INIT,DECODER_RUNNING, DECODER_PROGRAM 
} decoderState_t;

decoderState_t decoderState;
head_t head;
NmraDcc  Dcc;

typedef struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
} CVPair;

static CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 1},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_VERSION_ID, VersionId},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_29_CONFIG,0x80}
};

uint16_t getMyAddr(void); // uit nmraDcc.cpp --> in de Dcc class steken?


// timer interrupt used for 1ms interrupt
ISR (TIMER2_COMPA_vect) {
  head.ms++;
  if (head.ms == 250) {
    head.ms = 0;
  }
  // check flashing at every start of a 250ms cycle
  if (head.ms==0) {
    for (uint8_t output=0;output<5;output++) {
      if (head.outputs[output].pin) {
        head.outputs[output].fCycle++;
        if (head.outputs[output].fCycle == head.outputs[output].fTotalCycles) 
          head.outputs[output].fCycle=0;
      }
    }
  }
  head.bCycle++;
  if (head.bCycle == 10) head.bCycle = 0;

  for (uint8_t output=0;output<5;output++) {
    if (head.outputs[output].pin) {
      if ((head.outputs[output].fCycle < head.outputs[output].fOnCycle) || 
         (head.outputs[output].fCycle >= head.outputs[output].fOffCycle))
         digitalWrite(head.outputs[output].pin,HIGH); // led off (flashing off cycle)
      else { // flashing on cycle
        // brightness control
        if (head.bCycle < head.outputs[output].ledOn) {
          digitalWrite(head.outputs[output].pin,HIGH); // led off
        }
        else if (head.bCycle < head.outputs[output].ledOff) {
          digitalWrite(head.outputs[output].pin,LOW); // led on
        }
        else digitalWrite(head.outputs[output].pin,HIGH); // led off
      }
    }
  }
} // ISR

static uint8_t accessory_FactoryResetCV() {
  // factory reset de algemene CV's (die niet afhangen van de software mode)
  for (int i=0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++) {
    Dcc.setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
  }
  /*
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
  */
  return 0;
} // accessory_FactoryResetCV


static void head_init(uint8_t aspectId) {
  for (uint8_t output=0;output<5;output++) {
    head.outputs[output].pin = 0;
  }
  // alle leds uit
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);

  if (aspectId == 0) {
    head.outputs[0].pin = 5; // rood
    head.outputs[0].ledOn = 0;
    head.outputs[0].ledOff = 8;
    head.outputs[0].fOnCycle = 0;
    head.outputs[0].fOffCycle = 1;
    head.outputs[0].fTotalCycles = 1;
    head.outputs[0].fCycle = 0;
  }
  else if (aspectId == 1) {
    head.outputs[0].pin = 4; // groen
    head.outputs[0].ledOn = 0;
    head.outputs[0].ledOff = 8;
    head.outputs[0].fOnCycle = 0;
    head.outputs[0].fOffCycle = 1;
    head.outputs[0].fTotalCycles = 1;
    head.outputs[0].fCycle = 0;
  }
  else if (aspectId == 2) { // dubbel geel
    head.outputs[0].pin = 6;
    head.outputs[0].ledOn = 0;
    head.outputs[0].ledOff = 8;
    head.outputs[0].fOnCycle = 0;
    head.outputs[0].fOffCycle = 1;
    head.outputs[0].fTotalCycles = 1;
    head.outputs[0].fCycle = 0;
    head.outputs[1].pin = 3;
    head.outputs[1].ledOn = 0;
    head.outputs[1].ledOff = 8;
    head.outputs[1].fOnCycle = 0;
    head.outputs[1].fOffCycle = 1;
    head.outputs[1].fTotalCycles = 1;
    head.outputs[1].fCycle = 0;
  }
  else if (aspectId == 3) { // rood wit
    head.outputs[0].pin = 7; // wit
    head.outputs[0].ledOn = 0;
    head.outputs[0].ledOff = 1;
    head.outputs[0].fOnCycle = 0;
    head.outputs[0].fOffCycle = 4;
    head.outputs[0].fTotalCycles = 8;
    head.outputs[0].fCycle = 0;
    head.outputs[1].pin = 5; // rood
    head.outputs[1].ledOn = 3;
    head.outputs[1].ledOff = 10;
    head.outputs[1].fOnCycle = 0;
    head.outputs[1].fOffCycle = 4;
    head.outputs[1].fTotalCycles = 8;
    head.outputs[1].fCycle = 0;  
  }
  else if (aspectId == 4) { // dubbel geel knipperend
    head.outputs[2].pin = 6; // geel midden
    head.outputs[2].ledOn = 1;
    head.outputs[2].ledOff = 10;
    head.outputs[2].fOnCycle = 0;
    head.outputs[2].fOffCycle = 2;
    head.outputs[2].fTotalCycles = 4;
    head.outputs[2].fCycle = 0;
    head.outputs[3].pin = 3; // geel boven
    head.outputs[3].ledOn = 1;
    head.outputs[3].ledOff = 10;
    head.outputs[3].fOnCycle = 0;
    head.outputs[3].fOffCycle = 2;
    head.outputs[3].fTotalCycles = 4;
    head.outputs[3].fCycle = 0;  
  }
  else  {// alles uit
  }
  // test patterns
  /*
  // rood/wit knipperen samen
  head.outputs[0].pin = 7; // wit
  head.outputs[0].ledOn = 0;
  head.outputs[0].ledOff = 1;
  head.outputs[0].fOnCycle = 0;
  head.outputs[0].fOffCycle = 4;
  head.outputs[0].fTotalCycles = 8;
  head.outputs[0].fCycle = 0;
  head.outputs[1].pin = 5; // rood
  head.outputs[1].ledOn = 3;
  head.outputs[1].ledOff = 10;
  head.outputs[1].fOnCycle = 0;
  head.outputs[1].fOffCycle = 4;
  head.outputs[1].fTotalCycles = 8;
  head.outputs[1].fCycle = 0;
  
  // dubbel geel knipperen afwisselend
  head.outputs[2].pin = 6; // geel midden
  head.outputs[2].ledOn = 1;
  head.outputs[2].ledOff = 10;
  head.outputs[2].fOnCycle = 0;
  head.outputs[2].fOffCycle = 2;
  head.outputs[2].fTotalCycles = 4;
  head.outputs[2].fCycle = 0;
  head.outputs[3].pin = 3; // geel boven
  head.outputs[3].ledOn = 1;
  head.outputs[3].ledOff = 10;
  head.outputs[3].fOnCycle = 2;
  head.outputs[3].fOffCycle = 4;
  head.outputs[3].fTotalCycles = 4;
  head.outputs[3].fCycle = 0;

  // groen brandt vast
  head.outputs[4].pin = 4; // groen
  head.outputs[4].ledOn = 1;
  head.outputs[4].ledOff = 10;
  head.outputs[4].fOnCycle = 0;
  head.outputs[4].fOffCycle = 1;
  head.outputs[4].fTotalCycles = 1;
  head.outputs[4].fCycle = 0;
  */

  head.ms = 0;
  head.bCycle = 0;
} // head_init

void timer2_init(void)
{
  // configure timer 2 for 1ms interrupt :
  // 16MHz : prescale /128 = 125kHz -> 8us per tick
  // 8Mhz : prescale /64 = 125kHz ->8us per tich
  // -> gebruik 125x8us als output compare
  TCCR2A = (1 << WGM21);  // CTC mode of operation (tot OCR2A tellen)
  OCR2A = 124;            // 125x 8us per output compare
  TIFR2 = 0x7;            // clear pending interrupts
  TIMSK2 = (1 << OCIE2A); // enable OC channel A interrupt
  TCNT2 = 0;
  #if (F_CPU==8000000L)
    TCCR2B = (1 << CS22); // prescaler /64 voor 8MHz pro Mini
  #else
    TCCR2B = (1 << CS22) | (1 << CS20);   // prescaler /128 (opgelet : niet dezelfde bits voor TIMER0!!! table 19-10 vs. 22-10)
  #endif  
} // timer2_init

void setup() 
{
  decoderState = DECODER_INIT;
  Serial.begin(115200);

  digitalWrite(8,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);
  pinMode(8,OUTPUT); // VCC pin op het sein
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

  // Configure the DCC CV Programing ACK pin for an output
  /*
  pinMode( PIN_ACKOUT, OUTPUT );
  digitalWrite( PIN_ACKOUT, LOW );
  */  
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, PIN_DCCIN, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( FLAGS_DCC_ACCESSORY_DECODER, 0 );

  /*
  // progled, progkey
  pinMode (PIN_PROGKEY, INPUT_PULLUP);
  // key2-4
  pinMode (PIN_KEY2, INPUT_PULLUP);
  pinMode (PIN_KEY3, INPUT_PULLUP);
  pinMode (PIN_KEY4, INPUT_PULLUP);
  // led
  pinMode (PIN_PROGLED, OUTPUT);
  digitalWrite(PIN_PROGLED, LOW); //LED uit
  */

  if (Dcc.getCV(CV_MANUFACTURER_ID) != MAN_ID_DIY) {
    decoderState = DECODER_FACTORY_RESET;
  }
  else {
    decoderState = DECODER_INIT;
  }

  /*
  // init key handling
  init_keys ();
  */
  Serial.println("start");
} // setup

void loop() 
{
  switch (decoderState) {
    case DECODER_FACTORY_RESET : 
      if (accessory_FactoryResetCV () == 0) {
        // factory reset gelukt --> DECODER_INIT
        decoderState = DECODER_INIT;
      }
      break;
    case DECODER_INIT :
      Serial.print("my address is : ");
      Serial.println(getMyAddr());

      head_init(0); // rood - for test
      timer2_init();
      
      decoderState = DECODER_RUNNING;
      break;
    case DECODER_RUNNING :
    case DECODER_PROGRAM : 
      // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function 
      // for correct library operation
      Dcc.process();
      break;
  }
} // loop

/**********************************************************************************************/
/* notify functies uit Nmra layer */
/**********************************************************************************************/

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void)
{
  Serial.println("notifyCVAck") ;
  /*
  digitalWrite( PIN_ACKOUT, HIGH );
  delay( 6 );  
  digitalWrite( PIN_ACKOUT, LOW );
  */
}

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccState( uint16_t Addr, uint16_t BoardAddr, uint8_t OutputAddr, uint8_t State)
{
  Serial.println("notifyDccAccState - turnout packet received, not handled for now");
} // notifyDccAccState

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigState( uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect)
{
  Serial.print("notifyDccSigState: ");
  Serial.print(decoderAddress,DEC);
  Serial.print(',');
  Serial.print(signalId,DEC);
  Serial.print(',');
  Serial.println(signalAspect, DEC);
  // voorlopig signalId ignored, 0..3 heads TODO
  head_init(signalAspect);
} // notifyDccSigState

// enkel CV's schrijven als decoder niet 'live' is (INIT, FACTORY_RESET, PROGRAM)
uint8_t notifyCVWrite( uint16_t CV, uint8_t Value) {
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
    /*
    timer.oscillate(PIN_PROGLED, LED_FAST_FLASH, LOW, 3); // 3 led flashes ter bevestiging van een CV write
    */
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
