/*
 * uses MySensors, but without NodeManager framework

 * A0 : solar cell voltage over a resistor divider
 * vcc = lipo battery on a 5V nano/pro mini, read via readVcc function (internal reference)

 */
// Enable debug prints to serial monitor
//#define MY_DEBUG
#define MY_RADIO_RF24 // Enable and select radio type attached

#define MY_NODE_ID 1
#include <MySensors.h>

#define CHILD_ID 1                              // Id of the sensor child

// wat wil je sturen naar controller?
// let's present this dummy as a S_MULTIMETER to show the solar voltage
// solarValue, vccValue, packetsOK, packetsNOK
MyMessage solarValueMsg(CHILD_ID,V_VOLTAGE);
MyMessage packetsOKMsg(CHILD_ID,V_VAR1);
MyMessage packetsNOKMsg(CHILD_ID,V_VAR2);
// todo : battery via internal message type?

// aanpasbare parameters ? bv. TxInterval, sleepMode, NodeName : TODO

uint32_t radioTxInterval = 3000; // 3s default
uint32_t radioTxLastMillis;
bool stayAwake = false; // sleeping on/off
uint32_t lastSleepMillis;

bool initPacketsOK, initPacketsNOK = false;

int solarPin = A0;

struct sensorData_t {
  uint16_t solarValue;
  uint16_t vccValue;
  uint16_t packetsOK;
  uint16_t packetsNOK;
};
sensorData_t curSensorData;

void setup(void)
{
  bool txOK;
  // Fetch last known packet counter values from gw
	txOK = request(CHILD_ID, V_VAR1); // packetsOK
  if (txOK) Serial.println(F("setup:request V_VAR1 ok"));
	txOK = request(CHILD_ID, V_VAR2); // packetsNOK
  if (txOK) Serial.println(F("setup:request V_VAR2 ok"));
} // setup

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Dummy Sensor", "0.2");

	// Register this device as a multimeter device
	present(CHILD_ID, S_MULTIMETER, "DUMMY");
} // presentation

void receive(const MyMessage &message)
{
  uint8_t msgType = message.getType();
  Serial.print(F("receiving msgType:"));Serial.println(msgType);

	if (msgType==V_VAR1) { // packetsOK
		uint16_t packetsOK=message.getUInt();
		Serial.print("Received last packetsOK from gw:");
		Serial.println(packetsOK);
    curSensorData.packetsOK += packetsOK;
    initPacketsOK = true;
	}
	else if (msgType==V_VAR2) { // packetsNOK
		uint16_t packetsNOK=message.getUInt();
		Serial.print("Received last packetsNOK from gw:");
		Serial.println(packetsNOK);
    curSensorData.packetsNOK += packetsNOK;
    initPacketsNOK = true;
	}
  else {
    Serial.print("received unknown type : "); Serial.println(msgType);
  }
} // receive

void loop() {
  bool txOK;

  // sending data to the gateway/controller
  if ((radioTxInterval != 0xFFFFFFFF) && ( millis() - radioTxLastMillis >= radioTxInterval ))
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

    Serial.println("sending data :");
    txOK = send (solarValueMsg.set(curSensorData.solarValue));
    if (txOK) curSensorData.packetsOK ++;
    else curSensorData.packetsNOK++;

    Serial.print("packetsOK : "); Serial.print(curSensorData.packetsOK);
    Serial.print(", packetsNOK : "); Serial.println(curSensorData.packetsNOK);
    if (initPacketsOK) {
      txOK = send (packetsOKMsg.set(curSensorData.packetsOK));
      if (txOK) curSensorData.packetsOK ++;
      else curSensorData.packetsNOK++;
    }
    else {
      // request again
      Serial.println(F("retry request V_VAR1"));
      txOK = request(CHILD_ID, V_VAR1); // packetsOK
      if (!txOK) Serial.println(F("request V_VAR1 retry failed"));
    }
    if (initPacketsNOK){
      txOK = send (packetsNOKMsg.set(curSensorData.packetsNOK));
      if (txOK) curSensorData.packetsOK ++;
      else curSensorData.packetsNOK++;
    }
    else {
      // request again
      Serial.println(F("retry request V_VAR2"));
      txOK = request(CHILD_ID, V_VAR2); // packetsNOK      
      if (!txOK) Serial.println(F("request V_VAR2 retry failed"));
    }
    txOK  = sendBatteryLevel(batteryLevel);
    if (txOK) curSensorData.packetsOK ++;
    else curSensorData.packetsNOK++;

    // TODO : we kunnen nog andere dinges sturen  (MySensorsCore.h): sendSignalStrength, sendTXPowerLevel

    // stay awake for another 1000 ms after sending data
    lastSleepMillis = millis();

  }

  // stay awake for another 1000 ms after sending data
  if ((millis() - lastSleepMillis) > 1000) {
    if (!stayAwake) {
      lastSleepMillis = millis();
      sleep(3000,true); // smart sleep for 3 seconds, dan is er eerst een 500ms wait zodat de controller nog wat kan sturen
    }
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
