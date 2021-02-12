/*
uses MySensors, but without NodeManager framework
sends dummy data 

*/
// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_RADIO_RF24 // Enable and select radio type attached

#define MY_NODE_ID 2
#include <MySensors.h>

// sensor node child sensorIDs
#define RAINTANK_CHILD_ID     1
#define LEVELSENSOR_CHILD_ID  2
#define CHARGING_CHILD_ID     3
// and the internal I_BATTERY_LEVEL to indicate battery status of the remote sensor

//MyMessage sensorMsg(RAINTANK_CHILD_ID,V_VOLUME);
MyMessage sensorMsg; // fill in details later

uint32_t radioTxInterval = 10000; // in ms
uint32_t radioTxLastMillis;

typedef struct {
  uint16_t tankLiters;
  uint16_t batAnalogRead;
  bool batCharging;
  bool tankEmpty;
  uint32_t packetsOK;
  uint32_t packetsNOK;

} sensorData_t;
sensorData_t curSensorData;

static void updateFakeData() {
  // bat level : 870 = full, 970 = empty
  if (curSensorData.batCharging) curSensorData.batAnalogRead -= 5;
  else curSensorData.batAnalogRead += 5;
  if ((curSensorData.batAnalogRead > 970) && !curSensorData.batCharging) curSensorData.batCharging = true;
  else if ((curSensorData.batAnalogRead < 870) && curSensorData.batCharging) curSensorData.batCharging = false;
  // rain tank : -100 until empty then full
  if (curSensorData.tankEmpty) curSensorData.tankLiters += 1000; // one shot refill whoohoo
  else curSensorData.tankLiters -= 100; // slow empty
  curSensorData.tankEmpty = curSensorData.tankLiters < 200;

} // updateFakeData

static uint8_t convBatAnalogRead2BatLevel(uint16_t batAnalogReadValue) {
  uint32_t batVoltage = 3600000 / (uint32_t) batAnalogReadValue; // experimental
  uint8_t batLevel = (uint8_t) ((100.0*(batVoltage - 2700)) / (4200-2700)); // dropout = 2700, max = 4200
  return batLevel;
} // convBatAnalogRead2BatLevel

static printSensorData() {
  Serial.print("tankLiters:");Serial.println(curSensorData.tankLiters);
  Serial.print("tankEmpty:");Serial.println(curSensorData.tankEmpty);
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

void setup(void)
{
  curSensorData.tankLiters = 1000;
  curSensorData.tankEmpty = false;
  curSensorData.batAnalogRead = 870;
  curSensorData.batCharging = false;
} // setup

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Pump Sensor", "0.1");
  // present child sensors
	present(RAINTANK_CHILD_ID, S_WATER, "Raintank");
	present(LEVELSENSOR_CHILD_ID, S_BINARY, "TankEmpty");
	present(CHARGING_CHILD_ID, S_BINARY, "BatCharging");

} // presentation

void receive(const MyMessage &message)
{
  uint8_t msgType = message.getType();
  Serial.print(F("receiving msgType:"));Serial.println(msgType);
} // receive

void loop() {

  // sending data to the gateway/controller
  if (( millis() - radioTxLastMillis) >= radioTxInterval )
  {
    updateFakeData();
    printSensorData();
    radioTxLastMillis = millis();

    Serial.print("sending data...");
    sensorMsg.setSensor(RAINTANK_CHILD_ID);
    sensorMsg.setType(V_VOLUME);
    sensorMsg.set(curSensorData.tankLiters);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setSensor(LEVELSENSOR_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.tankEmpty);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setSensor(CHARGING_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.batCharging);
    updatePacketCounters(send(sensorMsg));

    updatePacketCounters(sendBatteryLevel(convBatAnalogRead2BatLevel(curSensorData.batAnalogRead)));
    Serial.println("done!");
  }

} // loop
