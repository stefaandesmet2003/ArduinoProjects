// compile for AVR
// TODO : if radio init fails, Mysensors hangs before setup() and loop() is not executed

// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_RADIO_RF24 // Enable and select radio type attached
#define MY_NODE_ID 2
#include <MySensors.h>

#include "Opentherm.h"

/*
Opentherm communication with remote tank sensor : 32-bit dataframe :
data[0] : TankEmpty
data[1] : charging
data[2:11] : batAnalogRead, 0..1023 (10-bit ADC)
data[12:23] : distanceCm, 0..4095 (12-bit)
data[24:31] : XOR of bytes 0..2 XOR 0xAA -> the last XOR avoids that 0xFFFFFFFF is seen as valid frame

charging
**********
the remote sensor indicates whether it needs charging
The pump sensor is always on, and doesn't need charging.
The pump sensor provides active level opentherm current (20mA) for charging the remote sensor
When no communication is received from the remote sensor, the pump sensor turns on charging
(in an attempt to revive a potentially dead battery)

This means TX state might be active when we receive a startbit from the remote sensor.
not strictly according to opentherm, but it doesn't interfere with the remote sensor Tx, 
and we generate a valid startbit on reply

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

MySensors
***********
uses MySensors, but without NodeManager framework

*/

#define PIN_OUTPUT_LEVEL_SENSOR       6 // HIGH = contact closed (opto on) = tank not empty, LOW = contact open (opto off) = tank empty
#define PIN_INPUT_PUMP_ON_TBD         5 // TBD, a current sensor for sensing pump activity
#define PIN_INPUT_FLOW_SENSOR_TBD     4 // TBD, reading flow sensor pulses to count water consumption
#define PIN_INPUT_PUMP_CURRENT_TBD    A0 // TBD, reading flow sensor pulses to count water consumption
// opentherm comms :
//#define T_RX_PIN  7
//#define T_TX_PIN  8
// mysensors uses pins 9-13 for the nrf24 radio

// tank & sensor setup config
#define TANK_VOLUME             10000
#define TANK_HEIGHT_CM          250
#define TANKVOLUME_PER_CM       (TANK_VOLUME / TANK_HEIGHT_CM) // tank 10000L with 250cm depth -> 1cm of water level diff = 40L
#define SENSOR_OFFSET_CM        20 // height of the sensor above the max water level

// we expect a comms from the remote sensor every interval; 
// this could be 3x the remote sensor measurement interval to account for transmission fails
// or we could implement retries on the remote..
#define COMMS_TIMEOUT_INTERVAL  15000 // 60s on final setup, smaller for testing; 
 
// MySensors config
// MY_NODE_ID needs to be defined before #include
// sensor node child sensorIDs
#define REMOTE_SENSOR_OK_CHILD_ID 1
#define RAINTANK_VOLUME_CHILD_ID  2
#define LEVEL_SENSOR_CHILD_ID     3
#define BAT_CHARGING_CHILD_ID     4
// and the internal I_BATTERY_LEVEL to indicate battery status of the remote sensor

bool openthermNewData = false;
bool openthermTimeout = false;
uint32_t openthermTxMillis;

typedef struct {
  uint16_t tankLiters;
  uint16_t distanceCm; // kept for internal ref, not transmitted over MySensors
  bool tankEmpty;
  uint16_t batAnalogRead; // kept for internal ref, not transmitted over MySensors
  bool batCharging;
  uint32_t packetsOK;
  uint32_t packetsNOK;
} sensorData_t;

sensorData_t curSensorData;
MyMessage sensorMsg; // one structure reused for all messages
uint32_t radioTxInterval = 30000; // in ms
uint32_t radioTxLastMillis;

// return : true : XOR OK, false : XOR NOK
static bool parseDataFrame(uint32_t data, sensorData_t *sensorData) {
  uint8_t xorVal;
  bool xorOK;
  xorVal = (data & 0xFF) ^ ((data >> 8) & 0xFF) ^ ((data >> 16) & 0xFF) ^ 0xAA;
  xorOK = xorVal == ((data >> 24) & 0xFF);
  if (!xorOK) return false;

  // xor = ok -> continue parsing data
  sensorData->tankEmpty = data & 0x1;
  sensorData->batCharging = (data >> 1) & 0x1;
  sensorData->batAnalogRead = (data >> 2) & 0x3FF; // 10-bit field
  sensorData->distanceCm = (data >> 12) & 0xFFF; // 12-bit field
  return true;
} // parseDataFrame

static uint8_t convBatAnalogRead2BatLevel(uint16_t batAnalogReadValue) {
  uint32_t batVoltage = 3600000 / (uint32_t) batAnalogReadValue; // experimental
  uint8_t batLevel = (uint8_t) ((100.0*(batVoltage - 2700)) / (4200-2700)); // dropout = 2700, max = 4200
  return batLevel;
} // convBatAnalogRead2BatLevel

static printSensorData() {
  Serial.print("tankLiters:");Serial.println(curSensorData.tankLiters);
  Serial.print("tankEmpty:");Serial.println(curSensorData.tankEmpty);
  Serial.print("distanceCm:");Serial.println(curSensorData.distanceCm);
  Serial.print("batAnalogRead:");Serial.println(curSensorData.batAnalogRead);
  Serial.print("batLevel:");Serial.println(convBatAnalogRead2BatLevel(curSensorData.batAnalogRead));
  Serial.print("batCharging:");Serial.println(curSensorData.batCharging);
  Serial.print("packetsOK:");Serial.println(curSensorData.packetsOK);
  Serial.print("packetsNOK:");Serial.println(curSensorData.packetsNOK);
} // printSensorData

static updatePacketCounters (bool txOK) {
  if (txOK) curSensorData.packetsOK ++;
  else curSensorData.packetsNOK++;
} // updatePacketCounters

void presentation() { // MySensors
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Pump Sensor", "0.1");
  // present child sensors
	present(REMOTE_SENSOR_OK_CHILD_ID, S_BINARY, "RemoteOK");
	present(RAINTANK_VOLUME_CHILD_ID, S_WATER, "RainVolume");
	present(LEVEL_SENSOR_CHILD_ID, S_BINARY, "TankEmpty");
	present(BAT_CHARGING_CHILD_ID, S_BINARY, "BatCharging");

} // presentation

void receive(const MyMessage &message) { // MySensors - not used for now
  uint8_t msgType = message.getType();
  Serial.print(F("receiving msgType:"));Serial.println(msgType);
} // receive

// function from MySensors, called before setting up the radio
void before() {

  digitalWrite(PIN_OUTPUT_LEVEL_SENSOR, curSensorData.tankEmpty); // is this the good choice? if there is no comms with the remote sensor, we make the DAB control believe tank is not empty
  pinMode(PIN_OUTPUT_LEVEL_SENSOR, OUTPUT);
  // TBD
  //pinMode(PIN_INPUT_PUMP_ON, INPUT);
  //pinMode(PIN_INPUT_FLOW_SENSOR, INPUT);
  OT_init();
  curSensorData.batCharging = true;
  OT_setTxState(curSensorData.batCharging);
} // before

void setup() {
  //Serial.begin(115200); // already done by MySensors (hwInit)
  Serial.println("Start PumpSensor");
} // setup

void loop() {
  uint32_t otData, otResponse;

  // 1. communication with RemoteSensor
  if (OT_available()) {
    openthermTimeout = false;
    otData = OT_readFrame();
    Serial.print("Received data : "); Serial.println(otData,HEX);
    if (parseDataFrame(otData, &curSensorData)) { // XOR check OK
      // a basic sanity check on distanceCm for now; if check fails we don't update the tankLiters value and await a new reading
      if ((curSensorData.distanceCm > 0) && (curSensorData.distanceCm < (TANK_HEIGHT_CM + SENSOR_OFFSET_CM))) {
        curSensorData.tankLiters = TANK_VOLUME - (curSensorData.distanceCm - SENSOR_OFFSET_CM)*TANKVOLUME_PER_CM;
      }
      // no sanity check on tankEmpty, there could be a fault on the sensor, that we want to diagnose!
      digitalWrite(PIN_OUTPUT_LEVEL_SENSOR, !curSensorData.tankEmpty);
      // sending a dummy response - response not used by remote sensor for now    
      otResponse = 0xAA55;
      openthermTxMillis = millis();
      openthermNewData = true;
      OT_writeFrameInt(otResponse);
      printSensorData();
    }
    else {
      Serial.println("XOR error in received data! Discarding ..");
    }
  }

  // in interrupt mode we don't know when Tx is complete, but a frame takes 1ms/bit, 32bits + start/stop, so roughly 50ms
  // value is not critical, but may not interfere with an ongoing TX
  if (openthermNewData && (millis() - openthermTxMillis) > 100) { // tx is complete now, we can re-enable the charging if necessary
    openthermNewData = false;
    OT_setTxState(curSensorData.batCharging); // charging=true->set active level on OT-Tx = high current (20mA)
  }
  if ((millis() - openthermTxMillis) > COMMS_TIMEOUT_INTERVAL) {
    if (!openthermTimeout) { // do this only once per timeout
      Serial.println("haven't seen remote sensor in a while");
      // enable charging as a precaution, maybe the battery of the remote sensor is low
      curSensorData.batCharging = true;
      OT_setTxState(curSensorData.batCharging);
    }
    openthermTimeout = true;
  }

  // 2. MySensors communication
  // sending data to the gateway/controller
  if (( millis() - radioTxLastMillis) >= radioTxInterval )
  {
    radioTxLastMillis = millis();

    // TODO? split tx over time?
    Serial.print("sending data to gateway...");
    sensorMsg.setSensor(REMOTE_SENSOR_OK_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(!openthermTimeout);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setSensor(RAINTANK_VOLUME_CHILD_ID);
    sensorMsg.setType(V_VOLUME);
    sensorMsg.set(curSensorData.tankLiters);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setSensor(LEVEL_SENSOR_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.tankEmpty);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setSensor(BAT_CHARGING_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.batCharging);
    updatePacketCounters(send(sensorMsg));

    updatePacketCounters(sendBatteryLevel(convBatAnalogRead2BatLevel(curSensorData.batAnalogRead)));
    Serial.println("done!");
  }
} // loop
