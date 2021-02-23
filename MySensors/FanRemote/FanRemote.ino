// remote control for the home ventilation fan
// only on/off supported for now
// prototype code for AVR, final version on STM8 (FanRemoteSTM8)

// Enable debug prints to serial monitor
#define MY_DEBUG
#define MY_RADIO_RF24 // Enable and select radio type attached
#define MY_NODE_ID 6

#include <MySensors.h>

// hardware configuration
#define PIN_BUTTON  4
// + mysensors uses pins 9-13 for the nrf24 radio

#define DEBOUNCE_TIME   100 // for the remote control button

// MySensors config
// MY_NODE_ID needs to be defined before #include

// fan sensor config that we want to control
#define FAN_NODE_ID     5
#define FAN_CHILD_ID    1 // V_STATUS voor fan on/off, V_HVAC_SPEED for fan speed
#define ALARM_CHILD_ID  2 // V_STATUS for alarm on/off
char *strFanSpeed[]= {"Min","Normal","Max","Auto"};


typedef struct {
  bool fanOn;  // FAN_CHILD_ID, V_STATUS : on/off
  char fanSpeed[7]; // FAN_CHILD_ID, V_HVAC_SPEED : "Min","Normal","Max", but "Normal" not supported
  bool alarmActive; // ALARM_CHILD_ID, V_STATUS : error in relays
  bool fanWaitResponse; // true if we are waiting for a response from the sensor after a command
} sensorData_t;

sensorData_t curSensorData;
MyMessage sensorMsg; // one structure reused for all messages
uint32_t radioResponseTimeoutInterval = 2000; // in ms
uint32_t radioTxLastMillis; // for response timeout
uint32_t debounceMillis;

void presentation() { // MySensors
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("Fan Remote", "0.1");
  // present child sensors -> no child sensors on remote
} // presentation

// we only handle a response from the FanSensor (nodeId==FAN_NODE_ID)
void receive(const MyMessage &message) { // MySensors
  uint8_t nodeId = message.getSender();
  uint8_t sensorId = message.getSensor();
  if ((nodeId == FAN_NODE_ID) && (message.getCommand() == C_SET))
  {
    if (sensorId == FAN_CHILD_ID) {
      if (message.getType() == V_STATUS) {
        bool fanOn = message.getBool();
        Serial.print("msg:FanOn: ");Serial.println(fanOn);
        if (curSensorData.fanWaitResponse) {
          if (fanOn == curSensorData.fanOn) curSensorData.fanWaitResponse = false; // reset flag, we got our response &
        }
        else // not waiting for a response, only updating our internal state
          curSensorData.fanOn = fanOn;
      }
      else if (message.getType() == V_HVAC_SPEED) {
        strncpy(curSensorData.fanSpeed,message.getString(),7);
        Serial.print("msg:FanSpeed: ");Serial.println(curSensorData.fanSpeed);
      }
    }
    if (sensorId == ALARM_CHILD_ID) {
      if (message.getType() == V_STATUS) {
        curSensorData.alarmActive = message.getBool();
        Serial.print("msg:AlarmActive: ");Serial.println(curSensorData.alarmActive);
      }
    }
  }
} // receive

// function from MySensors, called before setting up the radio
void before() {
    digitalWrite (PIN_BUTTON,HIGH);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    curSensorData.fanOn = false;
    curSensorData.alarmActive = false;
    curSensorData.fanWaitResponse = false;
} // before

void setup() {
  //Serial.begin(115200); // already done by MySensors (hwInit)
  Serial.println("Start FanRemote");
} // setup

void loop() {

  // 1. check button
  // TODO : make non-blocking because maybe nrf won't like this
  if ( digitalRead(PIN_BUTTON) == LOW)
  {
    debounceMillis = millis();
    while ((digitalRead(PIN_BUTTON) == LOW) && 
          ((millis() - debounceMillis) < DEBOUNCE_TIME)) ;
    if (digitalRead(PIN_BUTTON) == LOW) // we hebben een key
    {
      // toggle fan on/off
      curSensorData.fanOn = !curSensorData.fanOn;
      curSensorData.fanWaitResponse = true;
      Serial.println("Button pressed!");
      // request is sent & resent below , until a response received
    }
    while (digitalRead(PIN_BUTTON) == LOW); // wait for key up
  }

  // 2. MySensors communication
  // sending request to fan
  if ((curSensorData.fanWaitResponse) && ((millis()-radioTxLastMillis) >= radioResponseTimeoutInterval))
  {
    Serial.print("sending request to fan : ");Serial.println(curSensorData.fanOn);
    sensorMsg.setDestination(FAN_NODE_ID);
    sensorMsg.setSensor(FAN_CHILD_ID);
    sensorMsg.setType(V_STATUS);
    sensorMsg.set(curSensorData.fanOn);
    send(sensorMsg, true); // request echo
    radioTxLastMillis = millis();
    Serial.println("done!");
  }
} // loop
