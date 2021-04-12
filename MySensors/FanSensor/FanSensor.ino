// controls the speed of the home ventilation fan motor using 2 relays
// compile for AVR
// TODO : if radio init fails, Mysensors hangs before setup() and loop() is not executed
/*
 * fanOn=true from remote -> switch to high speed
 * fanOn=false from remote -> switch off
 * auto high->low speed after 15 minutes
 * remote sends C_SET with ack=true -> MySensors sends auto reply
*/

// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_RADIO_RF24 // Enable and select radio type attached
#define MY_NODE_ID 5

#include <MySensors.h>

/*
MySensors
***********
uses MySensors, but without NodeManager framework
*/

// hardware configuration
#define PIN_RELAY_LOWSPEED  4
#define PIN_RELAY_HIGHSPEED 5
#define PIN_FEEDBACK_RELAY1 6
#define PIN_FEEDBACK_RELAY2 7
#define PIN_TESTBUTTON      A1
// + mysensors uses pins 9-13 for the nrf24 radio

// digitalWrite values for the relays 
// inverse logic on the relay module : input=low -> relay powered
#define RELAY_ON    LOW
#define RELAY_OFF   HIGH

#define DEBOUNCE_TIME   100 // for the local control button
#define FAN_HIGH_AUTO_LOW_DELAY   15*60*1000 // after 15 minutes, fan switches back to low speed

// MySensors config
// MY_NODE_ID needs to be defined before #include
// sensor node child sensorIDs
// TODO : eventueel V_VAR1 voor config data
#define FAN_CHILD_ID    1 // V_STATUS for fan on/off, V_HVAC_SPEED for fan speed
#define ALARM_CHILD_ID  2 // V_STATUS for alarm on/off
char *strFanSpeed[]= {"Min","Normal","Max","Auto"};
#define FAN_SPEED_LOW 0
#define FAN_SPEED_MED 1
#define FAN_SPEED_HIGH 2
#define FAN_SPEED_AUTO 2 // will be handled as "Max" & returned as "Max" too

typedef struct {
  bool fanOn;  // FAN_CHILD_ID, V_STATUS : on/off
  uint8_t fanSpeed; // FAN_CHILD_ID, V_HVAC_SPEED : "Min","Normal","Max", but "Normal" not supported, using uint8_t for the moment iso. string
  bool alarmActive; // ALARM_CHILD_ID, V_STATUS : error in relays
  uint32_t packetsCountOK;    // V_VAR3
  uint32_t packetsCountNOK;   // V_VAR4
  // TODO : count alarms?
} sensorData_t;

sensorData_t curSensorData;
MyMessage sensorMsg; // one structure reused for all messages
uint32_t radioTxInterval = 30000; // in ms
uint32_t radioTxLastMillis;

typedef enum {  
  FANSTATE_IS_OFF, 
  FANSTATE_IS_LOW, 
  FANSTATE_IS_HIGH, 
  FANSTATE_TO_OFF, 
  FANSTATE_ERROR
} FanState_t;
static FanState_t fanState;
uint32_t fanStateMillis; // respect minimum of 2 seconds in each fanState to avoid too many relay switches
uint32_t debounceMillis;

static void updatePacketCounters (bool txOK) {
  if (txOK) curSensorData.packetsCountOK++;
  else curSensorData.packetsCountNOK++;
} // updatePacketCounters

void presentation() { // MySensors
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Fan Sensor", "0.1");
  // present child sensors
	present(FAN_CHILD_ID, S_HVAC, "FanMotor");
	present(ALARM_CHILD_ID, S_BINARY, "FanAlarm");
} // presentation

void receive(const MyMessage &message) { // MySensors
  uint8_t sensorId = message.getSensor();
  if (message.getCommand() == C_SET)
  {
    if (sensorId == FAN_CHILD_ID) {
      if (message.getType() == V_STATUS) {
        curSensorData.fanOn = message.getBool();
        curSensorData.fanSpeed = FAN_SPEED_HIGH; // for now because remote only sends on/off
        Serial.print("req:FanOn: ");Serial.println(curSensorData.fanOn);
      }
      else if (message.getType() == V_HVAC_SPEED) {
        String fanSpeed = message.getString();
        Serial.print("req:FanSpeed: ");Serial.println(fanSpeed);
        if (strcmp(fanSpeed.c_str(),"Min")) curSensorData.fanSpeed = FAN_SPEED_LOW;
        else curSensorData.fanSpeed = FAN_SPEED_HIGH;
      }
    }
    // don't accept C_SET on alarm child
  }

  else if (message.getCommand() == C_REQ) {
    // prefill response message
    sensorMsg.setType(message.getType());
    sensorMsg.setSensor(sensorId);
    sensorMsg.setDestination(message.getSender()); // send reply to the sender, not gateway (default)
    if (sensorId == FAN_CHILD_ID) {
      if (message.getType() == V_STATUS) send(sensorMsg.set(curSensorData.fanOn));
      else if (message.getType() == V_HVAC_SPEED) send(sensorMsg.set(strFanSpeed[curSensorData.fanSpeed]));
    }
    else if (sensorId == ALARM_CHILD_ID) {
      if (message.getType() == V_STATUS) send(sensorMsg.set(curSensorData.alarmActive));
    }
  }
  
} // receive

// function from MySensors, called before setting up the radio
void before() {
    digitalWrite (PIN_RELAY_LOWSPEED,RELAY_OFF);
    digitalWrite (PIN_RELAY_HIGHSPEED,RELAY_OFF);
    pinMode(PIN_RELAY_LOWSPEED, OUTPUT);
    pinMode(PIN_RELAY_HIGHSPEED, OUTPUT);
    pinMode(PIN_FEEDBACK_RELAY1, INPUT);
    pinMode(PIN_FEEDBACK_RELAY2, INPUT);
    pinMode(PIN_TESTBUTTON, INPUT_PULLUP);
    curSensorData.fanOn = false;
    curSensorData.alarmActive = false;
    curSensorData.fanSpeed = FAN_SPEED_HIGH; // TODO : via config?
    fanState = FANSTATE_IS_OFF;
} // before

void setup() {
  //Serial.begin(115200); // already done by MySensors (hwInit)
  Serial.println("Start FanSensor");
} // setup

void fanProcess() {
  if ((millis() - fanStateMillis) < 2000) {
    return; // do nothing
  }
  switch (fanState) {
    case FANSTATE_IS_OFF: 
      if (curSensorData.fanOn) { // got request to turn on the fan
        if (curSensorData.fanSpeed == FAN_SPEED_HIGH) {
          digitalWrite(PIN_RELAY_HIGHSPEED,RELAY_ON);
          fanState = FANSTATE_IS_HIGH;
        }
        else {
          digitalWrite(PIN_RELAY_LOWSPEED,RELAY_ON);
          fanState = FANSTATE_IS_LOW;
        }
        fanStateMillis = millis();
      }
      break;
    case FANSTATE_IS_LOW:
      if ((!curSensorData.fanOn) || (curSensorData.fanSpeed != FAN_SPEED_LOW)) {
        // first switch off fan, and then switch speed
        digitalWrite(PIN_RELAY_LOWSPEED, RELAY_OFF);
        fanState = FANSTATE_TO_OFF;
        fanStateMillis = millis();
      }
      break;
    case FANSTATE_IS_HIGH:
      if ((millis() - fanStateMillis) > FAN_HIGH_AUTO_LOW_DELAY) { // auto switch to low speed
        curSensorData.fanSpeed = FAN_SPEED_LOW;
        // and the next if-block will do the rest
      }
      if ((!curSensorData.fanOn) || (curSensorData.fanSpeed != FAN_SPEED_HIGH)) {
        // first switch off fan, and then switch speed
        digitalWrite(PIN_RELAY_HIGHSPEED, RELAY_OFF);
        fanState = FANSTATE_TO_OFF;
        fanStateMillis = millis();
      }
      break;
    case FANSTATE_TO_OFF:
      // TODO FANSTATE_ERROR! voorlopig all good
      fanState = FANSTATE_IS_OFF;
      /*
      // check feedback pin 1, check if fan correctly switched off
      if ((digitalRead(PIN_FEEDBACK_RELAY1) == HIGH) && 
          (digitalRead(PIN_FEEDBACK_RELAY2) == HIGH)) { // feedback pin OK
        fanState = FANSTATE_IS_OFF;
      }
      else {
        fanState = FANSTATE_ERROR;
        curSensorData.alarmActive = true;
      }
      */
      break;
    case FANSTATE_ERROR : 
      // TODO
      break;
  }
} // fanProcess

static void toggleFanSpeed() {
  if (curSensorData.fanOn && curSensorData.fanSpeed == FAN_SPEED_HIGH) {
    curSensorData.fanOn = false;
    curSensorData.fanSpeed = FAN_SPEED_LOW;
  }
  else if (curSensorData.fanOn && curSensorData.fanSpeed == FAN_SPEED_LOW) {
    curSensorData.fanSpeed = FAN_SPEED_HIGH;
  }
  else if (!curSensorData.fanOn) {
    curSensorData.fanOn = true;
    curSensorData.fanSpeed = FAN_SPEED_LOW;
  }
} // toggleFanSpeed

void loop() {

  // 1. handle fan states
  fanProcess();

  // 2. check button
  // TODO : make non-blocking because maybe nrf won't like this
  if ( digitalRead(PIN_TESTBUTTON) == LOW)
  {
    debounceMillis = millis();
    while ((digitalRead(PIN_TESTBUTTON) == LOW) && 
          ((millis() - debounceMillis) < DEBOUNCE_TIME)) ;
    if (digitalRead(PIN_TESTBUTTON) == LOW) // we hebben een key
    {
      // manually toggle fan speed between off-low-high
      Serial.println("toggleFanSpeed");
      toggleFanSpeed();
    }
    while (digitalRead(PIN_TESTBUTTON) == LOW); // wait for key up
  }

  // 3. MySensors communication
  // sending data to the gateway/controller
  if (( millis() - radioTxLastMillis) >= radioTxInterval )
  {
    radioTxLastMillis = millis();

    Serial.print("sending status to gateway...");

    sensorMsg.setDestination(GATEWAY_ADDRESS);

    sensorMsg.setSensor(FAN_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.fanOn);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setType(V_HVAC_SPEED);
    sensorMsg.set(strFanSpeed[curSensorData.fanSpeed]);
    updatePacketCounters(send(sensorMsg));

    sensorMsg.setSensor(ALARM_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.alarmActive);
    updatePacketCounters(send(sensorMsg));

    Serial.println("done!");
  }
} // loop
