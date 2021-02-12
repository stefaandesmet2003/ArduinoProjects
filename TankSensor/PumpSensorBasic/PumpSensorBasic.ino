// compile for AVR
// basic version of PumpSensor to test the remote tank sensor
// without NRF24 radio
#include "Opentherm.h"

/*
32-bit dataframe :
data[0] : TankEmpty
data[1] : charging
data[2:11] : batVoltage, 0..1023 (10-bit ADC)
data[12:23] : distanceCm, 0..4095 (12-bit)
data[24:31] : XOR of bytes 0..2
charging
**********
the remote sensor indicates whether it needs charging
The pump sensor is always on, and doesn't need charging.
The pump sensor provides active level opentherm current (20mA) for charging the remote sensor
When no communication is received from the remote sensor, the pump sensor turns on charging
(in an attempt to revive a potentially dead battery)
tank volume calculation
*************************
using defines for tank geometry for now :
#define TANK_VOLUME             10000
#define TANK_HEIGHT_CM          250
#define TANKVOLUME_PER_CM       (TANK_VOLUME / TANK_HEIGHT_CM) // tank 10000L with 250cm depth -> 1cm of water level diff = 40L
The sensor is mounted above the max water level; this is the minimum distanceCm that will be measured
#define SENSOR_OFFSET_CM        20 // height of the sensor above the max water level
The actual water volume in the tank is then:
tankLiters = TANK_VOLUME - (distanceCm - SENSOR_OFFSET_CM)*TANKVOLUME_PER_CM
TODO: can we make these values configurable over mysensors, and stored in eeprom?
so we don't need firmware update after install
TODO : idem comms interval
TODO : discard out-of-whack measurements, or average? 
TODO : this should be done on the remote sensor
TODO : possibility to push config from pump sensor -> remote sensor
like sleep interval, or charging thresholds
*/

// mysensors uses pins 9-13 for the nrf24 radio
#define PIN_OUTPUT_LEVEL_SENSOR       6 // HIGH = contact closed (opto on) = tank not empty, LOW = contact open (opto off) = tank empty
#define PIN_INPUT_PUMP_ON             5 // TBD, a current sensor for sensing pump activity
#define PIN_INPUT_FLOW_SENSOR         4 // TBD, reading flow sensor pulses to count water consumption

// and additionally for opentherm comms
//#define T_RX_PIN  7
//#define T_TX_PIN  8

// tank & sensor setup config
#define TANK_VOLUME             10000
#define TANK_HEIGHT_CM          250
#define TANKVOLUME_PER_CM       (TANK_VOLUME / TANK_HEIGHT_CM) // tank 10000L with 250cm depth -> 1cm of water level diff = 40L
#define SENSOR_OFFSET_CM        20 // height of the sensor above the max water level


// we expect a comms from the remote sensor every interval; 
// this could be 3x the remote sensor measurement interval to account for transmission fails
// or we could implement retries on the remote..
#define COMMS_TIMEOUT_INTERVAL  15000 // 60s on final setup, smaller for testing; 
// TODO : conversion batVoltage analogRead to batLevel for mysensors
// for now use the raw analogRead

bool handleNewData = false;
bool commsTimeout = false;
uint32_t txMillis;

// todo : make a struct with these, that's our complete state to exchange on MySensors
uint16_t batVoltage, distanceCm, tankLiters;
bool tankEmpty = false;
bool charging = true;

// return : true : XOR OK, false : XOR NOK
static bool parseDataFrame(uint32_t data, bool *tankEmpty, bool *charging, uint16_t *batVoltage, uint16_t *distanceCm) {
  uint8_t xorVal;
  bool xorOK;
  xorVal = (data & 0xFF) ^ ((data >> 8) & 0xFF) ^ ((data >> 16) & 0xFF);
  xorOK = xorVal == ((data >> 24) & 0xFF);
  if (!xorOK) return false;

  // xor = ok -> continue parsing data
  *tankEmpty = data & 0x1;
  *charging = (data >> 1) & 0x1;
  *batVoltage = (data >> 2) & 0x3FF; // 10-bit field
  *distanceCm = (data >> 12) & 0xFFF; // 12-bit field
  return true;

} // parseDataFrame

void setup() {
  pinMode(LED_BUILTIN,OUTPUT); // indicate charging for now

  digitalWrite(PIN_OUTPUT_LEVEL_SENSOR, !tankEmpty); // is this the good choice? if there is no comms with the remote sensor, we make the DAB control believe tank is not empty
  pinMode(PIN_OUTPUT_LEVEL_SENSOR, OUTPUT);

  // TBD
  //pinMode(PIN_INPUT_PUMP_ON, INPUT);
  //pinMode(PIN_INPUT_FLOW_SENSOR, INPUT);

  OT_init();

  charging = true;
  digitalWrite(LED_BUILTIN,charging);
  OT_setTxState(charging);

  Serial.begin(115200);
  Serial.println("Start PumpSensor");

} // setup

void loop() {
  uint32_t otData, otResponse;
  //uint16_t distanceCm; // stored in global state for reference

  if (OT_available()) {
    commsTimeout = false;
    otData = OT_readFrame();
    Serial.print("Received data : "); Serial.println(otData,HEX);
    if (parseDataFrame(otData, &tankEmpty, &charging, &batVoltage, &distanceCm)) { // XOR check OK
      tankLiters = TANK_VOLUME - (distanceCm - SENSOR_OFFSET_CM)*TANKVOLUME_PER_CM;
      digitalWrite(PIN_OUTPUT_LEVEL_SENSOR, !tankEmpty);
      // sending a dummy response - response not used by remote sensor for now    
      otResponse = 0xAA55;
      txMillis = millis();
      handleNewData = true;
      OT_writeFrameInt(otResponse);

      Serial.print("tankEmpty:");Serial.println(tankEmpty);
      Serial.print("charging:");Serial.println(charging);
      Serial.print("distanceCm:");Serial.println(distanceCm);
      Serial.print("tankLiters:");Serial.println(tankLiters);
      Serial.print("batVoltage:");Serial.println(batVoltage);
    }
    else {
      Serial.println("XOR error in received data! Discarding ..");
    }
  }

  // in interrupt mode we don't know when Tx is complete, but a frame takes 1ms/bit, 32bits + start/stop, so roughly 50ms
  // value is not critical, but may not interfere with an ongoing TX
  if (handleNewData && (millis() - txMillis) > 100) { // tx is complete now, we can re-enable the charging if necessary
    handleNewData = false;
    digitalWrite(LED_BUILTIN,charging);
    OT_setTxState(charging); // charging=true->set active level on OT-Tx = high current (20mA)
  }
  if ((millis() - txMillis) > COMMS_TIMEOUT_INTERVAL) {
    if (!commsTimeout) { // do this only once per timeout
      Serial.println("haven't seen remote sensor in a while");
      // enable charging as a precaution, maybe the battery of the remote sensor is low
      charging = true;
      digitalWrite(LED_BUILTIN,charging);
      OT_setTxState(charging);
    }
    commsTimeout = true;
  }

  // our TX state might be active when we receive a startbit from remote
  // not strictly according to opentherm, but no problem, because 
  // it doesn't interfere with the remote sensor Tx, and we generate a valid startbit on reply
  
} // loop