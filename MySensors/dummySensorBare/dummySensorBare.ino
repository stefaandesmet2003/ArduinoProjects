/*
 * uses MySensors, but without NodeManager framework
 * 
 * dummy is presented as a S_MULTIMETER device to show the solar voltage

 * A0 : solar cell voltage over a resistor divider
 * vcc = lipo battery on a 5V nano/pro mini, read via readVcc function (internal reference)

 */
// Enable debug prints to serial monitor
//#define MY_DEBUG
#define MY_RADIO_RF24 // Enable and select radio type attached

#define MY_NODE_ID 1
#include <MySensors.h>

#define CHILD_ID  1   // Id of the sensor child
#define solarPin  A0

// msgs to send to the gateway/controller
// can be a single MyMessage instance that's reconfigured before every tx, in order to save memory
// solarValue, vccValue, packetsCountOK, packetsCountNOK
MyMessage solarValueMsg(CHILD_ID,V_VOLTAGE);
MyMessage packetsCountOKMsg(CHILD_ID,V_VAR1);
MyMessage packetsCountNOKMsg(CHILD_ID,V_VAR2);
// todo : battery via internal message type?

// aanpasbare parameters ? bv. TxInterval, sleepMode, NodeName : TODO
uint32_t radioTxInterval = 3000; // 3s default
uint32_t radioTxLastMillis;
bool stayAwake = false; // sleeping on/off between tx intervals

bool initPacketsOK = false;
bool initPacketsNOK = false;
bool initLocalTime = false;

typedef struct {
  uint16_t solarValue;
  uint16_t vccValue;
  uint16_t packetsCountOK;
  uint16_t packetsCountNOK;
} sensorData_t;
sensorData_t curSensorData;

static void updatePacketCounters (bool txOK) {
  if (txOK) curSensorData.packetsCountOK++;
  else curSensorData.packetsCountNOK++;
} // updatePacketCounters

void setup(void)
{
  bool txOK;
  // Fetch last known packet counter values from gw
  // -> moved to loop
  radioTxLastMillis = millis();
} // setup

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	updatePacketCounters(sendSketchInfo("Dummy Sensor", "0.2"));

	// Register this device as a multimeter device
	updatePacketCounters(present(CHILD_ID, S_MULTIMETER, "DUMMY"));
} // presentation

void receiveTime(uint32_t localTime) {
  Serial.print("local time = ");
  Serial.println(localTime);
  initLocalTime = true;
}

void receive(const MyMessage &message)
{
  uint8_t msgType = message.getType();
  Serial.print(F("receiving msgType:"));Serial.println(msgType);

	if (msgType==V_VAR1) { // packetsCountOK
		uint16_t packetsCountOK = message.getUInt();
		Serial.print("Received last packetsCountOK from gw/ctrl:");
		Serial.println(packetsCountOK);
    if (!initPacketsOK){
      curSensorData.packetsCountOK += packetsCountOK;
      initPacketsOK = true;
    } 
	}
	else if (msgType==V_VAR2) { // packetsCountNOK
		uint16_t packetsCountNOK = message.getUInt();
		Serial.print("Received last packetsCountNOK from gw/ctrl:");
		Serial.println(packetsCountNOK);
    if (!initPacketsNOK) {
      curSensorData.packetsCountNOK += packetsCountNOK;
      initPacketsNOK = true;
    }
	}
  else {
    Serial.print("received unknown type : "); Serial.println(msgType);
  }
} // receive

void loop() {
  bool txOK;

  // sending data to the gateway/controller
  if (((stayAwake) && (radioTxInterval != 0xFFFFFFFF) && (millis()-radioTxLastMillis >=radioTxInterval)) 
   || (!stayAwake)) // if sleeping between transmits
  {
    radioTxLastMillis = millis();
    // dummy sensor data
    curSensorData.solarValue = (uint16_t) analogRead(solarPin);
    curSensorData.vccValue = readVcc();
    // calculate in uint32_t, report in uint8_t
    Serial.print("solarValue:");Serial.println(curSensorData.solarValue);
    Serial.print("vccValue:");Serial.println(curSensorData.vccValue);
    uint8_t batteryLevel = (uint8_t) ((100.0*(curSensorData.vccValue - 2700)) / (4200-2700)); // dropout = 2700, max = 4200
    Serial.print("batteryLevel:");Serial.println(batteryLevel);
    Serial.print("packetsCountOK : "); Serial.print(curSensorData.packetsCountOK);
    Serial.print(", packetsCountNOK : "); Serial.println(curSensorData.packetsCountNOK);

    Serial.println("sending data!");
    updatePacketCounters(send(solarValueMsg.set(curSensorData.solarValue)));
    if (initPacketsOK) {
      updatePacketCounters(send(packetsCountOKMsg.set(curSensorData.packetsCountOK)));
    }
    else {
      // request initial value from gw/ctrl
      Serial.println(F("request V_VAR1 initial value"));
      txOK = request(CHILD_ID, V_VAR1); // packetsCountOK
      if (!txOK) Serial.println(F("request V_VAR1 failed"));
      updatePacketCounters(txOK);
    }
    if (initPacketsNOK){
      updatePacketCounters(send(packetsCountNOKMsg.set(curSensorData.packetsCountNOK)));
    }
    else {
      // request initial value from gw/ctrl
      Serial.println(F("request V_VAR2 initial value"));
      txOK = request(CHILD_ID, V_VAR2); // packetsCountNOK      
      if (!txOK) Serial.println(F("request V_VAR2 failed"));
      updatePacketCounters(txOK);
    }
    updatePacketCounters(sendBatteryLevel(batteryLevel));

    if (!initLocalTime) { // request local time
      Serial.println(F("request local time"));
      updatePacketCounters(requestTime()); 
    }
    // TODO : we kunnen nog andere dinges sturen  (MySensorsCore.h): sendSignalStrength, sendTXPowerLevel
  }

  if (!stayAwake) {
    Serial.flush();
    sleep(radioTxInterval,true); // smart sleep for 3 seconds, dan is er eerst een 500ms wait zodat de controller nog wat kan sturen
    Serial.println("woke up!");
  }
} // loop

// returns vcc in mV
uint16_t readVcc() {
  uint32_t result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return (uint16_t) result;
} // readVcc
