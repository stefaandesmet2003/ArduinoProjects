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
#define PIN_INPUT_FLOW_SENSOR_TBD     4 // TBD, reading flow sensor pulses to count water consumption
#define PIN_INPUT_PUMP_CURRENT        A0 // analog reading from current transformer
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

// pump current sensing
#define ANALOGREAD_ZERO_CURRENT       561 // calibration depends on voltage divider
// pump current is 3.7A nominal, test measurement gave analog delta (vs ANALOGREAD_ZERO_CURRENT) = 90
// that is 90/1023*1.1V = 96.8mV, =4.84A peak, = 3.42A rms
#define PUMPCURRENT_THRESHOLD         50 // amplitude that defines pump on/off
#define PUMPCURRENT_SAMPLE_INTERVAL   1000 // ms
bool pumpOn = false;      // pump on/off detection
uint32_t pumpStartMillis; // for counting pump on-time
uint32_t pumpSampleCurrentMillis;
 
// MySensors config
// MY_NODE_ID needs to be defined before #include
// sensor node child sensorIDs
#define REMOTE_SENSOR_OK_CHILD_ID 1
#define RAINTANK_VOLUME_CHILD_ID  2
#define LEVEL_SENSOR_CHILD_ID     3
#define BAT_CHARGING_CHILD_ID     4
#define PUMPSENSOR_CHILD_ID       5 // custom sensor with pump stats

// and the internal I_BATTERY_LEVEL to indicate battery status of the remote sensor

bool openthermNewData = false;
bool openthermTimeout = false;
uint32_t openthermTxMillis;

typedef struct {
  uint16_t tankLiters;
  uint16_t distanceCm;      // kept for internal ref, not transmitted over MySensors
  bool tankEmpty;
  uint16_t batAnalogRead;   // kept for internal ref, not transmitted over MySensors
  bool batCharging;
  uint32_t pumpStartsCount;   // number of pump starts, V_VAR1
  uint32_t pumpOnTimeInSeconds; // total on-time, V_VAR2
  uint32_t packetsCountOK;    // V_VAR3
  uint32_t packetsCountNOK;   // V_VAR4
} sensorData_t;

sensorData_t curSensorData;
MyMessage sensorMsg; // one structure reused for all messages
uint32_t radioTxInterval = 15000; // in ms
uint32_t radioTxLastMillis;
uint8_t radioTxStep; // sending the curSensorData over 2 Tx bursts (total refresh time = 2*radioTxInterval)

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
  Serial.print("packetsCountOK:");Serial.println(curSensorData.packetsCountOK);
  Serial.print("packetsCountNOK:");Serial.println(curSensorData.packetsCountNOK);
} // printSensorData

static void updatePacketCounters (bool txOK) {
  if (txOK) curSensorData.packetsCountOK++;
  else curSensorData.packetsCountNOK++;
} // updatePacketCounters

//sample during +- 10ms, or 1/2 50Hz-period
// the highest value is an estimate for current amplitude
// blocking implementation for now
static int samplePumpCurrent() {
  int samples[10];
  int retval = 0;

  // take 10 samples over roughly 10ms
  for (int i=0;i<10;i++) {
    samples[i] = analogRead(PIN_INPUT_PUMP_CURRENT);
    delay(1);
  }

  // convert to current amplitude
  for (int i=0;i<10;i++) {
    if (samples[i] > ANALOGREAD_ZERO_CURRENT) 
      samples[i] = samples[i] - ANALOGREAD_ZERO_CURRENT;
    else samples[i] = ANALOGREAD_ZERO_CURRENT - samples[i];
  }

  // find the highest value (estimate for sine amplitude)
  for (int i=0;i<10;i++) {
    if (samples[i] > retval) 
      retval = samples[i];
  }
  return retval;
} // samplePumpCurrent

void presentation() { // MySensors
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Pump Sensor", "0.1");
  // present child sensors
	present(REMOTE_SENSOR_OK_CHILD_ID, S_BINARY, "RemoteOK");
	present(RAINTANK_VOLUME_CHILD_ID, S_WATER, "RainVolume");
	present(LEVEL_SENSOR_CHILD_ID, S_BINARY, "TankEmpty");
	present(BAT_CHARGING_CHILD_ID, S_BINARY, "BatCharging");
	present(PUMPSENSOR_CHILD_ID, S_CUSTOM, "Pump");
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

  // using 50A/1V current transformer, set zero at +- 0.6V and expecting much less than +-1V amplitude
  // so we can use the 1.1V bandgap reference for more accuracy
  analogReference(INTERNAL);

  Serial.println("Start PumpSensor");
} // setup

void loop() {
  uint32_t otData, otResponse;
  int pumpCurrent;

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

  // 2. pump current sampling
  if (( millis() - pumpSampleCurrentMillis) >= PUMPCURRENT_SAMPLE_INTERVAL)
  {
    pumpSampleCurrentMillis = millis();
    pumpCurrent = samplePumpCurrent();
    if ((pumpCurrent > PUMPCURRENT_THRESHOLD) && (!pumpOn)) {
      pumpStartMillis = pumpSampleCurrentMillis;
      pumpOn = true;
      curSensorData.pumpStartsCount += 1;
      Serial.println("pump on!");
    }
    if ((pumpCurrent < PUMPCURRENT_THRESHOLD) && (pumpOn)) {
      pumpOn = false;
      curSensorData.pumpOnTimeInSeconds = curSensorData.pumpOnTimeInSeconds + (pumpSampleCurrentMillis - pumpStartMillis) / 1000;
      Serial.println("pump off!");
      Serial.print("pumpOnTimeInSeconds = ");Serial.println(curSensorData.pumpOnTimeInSeconds);
    }
  }

  // 3. MySensors communication
  // sending data to the gateway/controller
  if (( millis() - radioTxLastMillis) >= radioTxInterval )
  {
    radioTxLastMillis = millis();

    // TODO? split tx over time?
    Serial.print("sending data to gateway...");

    if (radioTxStep == 0) { // remote sensor data
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
    }
    else if (radioTxStep == 1) { // local sensor data
      // todo : send pump info
      sensorMsg.setSensor(PUMPSENSOR_CHILD_ID);
      sensorMsg.setType(V_VAR1);
      sensorMsg.set(curSensorData.pumpStartsCount);
      updatePacketCounters(send(sensorMsg));

      sensorMsg.setSensor(PUMPSENSOR_CHILD_ID);
      sensorMsg.setType(V_VAR2);
      sensorMsg.set(curSensorData.pumpOnTimeInSeconds);
      updatePacketCounters(send(sensorMsg));

      sensorMsg.setSensor(PUMPSENSOR_CHILD_ID);
      sensorMsg.setType(V_VAR3);
      sensorMsg.set(curSensorData.packetsCountOK);
      updatePacketCounters(send(sensorMsg));

      sensorMsg.setSensor(PUMPSENSOR_CHILD_ID);
      sensorMsg.setType(V_VAR4);
      sensorMsg.set(curSensorData.packetsCountNOK);
      updatePacketCounters(send(sensorMsg));
    }
    radioTxStep = (radioTxStep + 1) % 2;
    Serial.println("done!");
  }
} // loop
