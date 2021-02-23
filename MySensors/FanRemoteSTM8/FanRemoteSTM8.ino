// remote control for the home ventilation fan
// only on/off supported for now
// STM8 wakes up on button press, toggles the fan on/off
// and goes back to sleep after an ack is received from the fan sensor

// compile for STM8

/* 
 * Radio connection : 
 * CE : PC4
 * CS : SS (A3)
 * SCK : PC5
 * MOSI : PC6
 * MISO : PC7
 * 
 * remote button on PD3
 */
#include "MySensorsLight.h"

#define MY_NODE_ID 100
//#define SDS_USE_SERIAL

// hardware configuration
#define PIN_BUTTON  PD3

#define DEBOUNCE_TIME   100 // for the remote control button

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
bool buttonPressed = false;

static void portd_irq (void) {
  buttonPressed = true;
}

// SDCC doesn't know weak functions
void MySensors_receiveTime(uint32_t controllerTime){(void) controllerTime;}

void MySensors_presentation() { // MySensors
	// Send the sketch version information to the gateway and Controller
	MySensors_sendSketchInfo("Fan Remote STM8", "0.1",false);
  // present child sensors -> no child sensors on remote
} // presentation

// we only handle a response from the FanSensor (nodeId==FAN_NODE_ID)
void MySensors_receive(const MyMessage *message) { // MySensors
  uint8_t nodeId = MyMessage_getSender(message);
  uint8_t sensorId = MyMessage_getSensor(message);
  if ((nodeId == FAN_NODE_ID) && (MyMessage_getCommand(message) == C_SET))
  {
    if (sensorId == FAN_CHILD_ID) {
      if (MyMessage_getType(message) == V_STATUS) {
        bool fanOn = MyMessage_getBool(message);
#ifdef SDS_USE_SERIAL
        Serial_print_s("msg:FanOn: ");Serial_println_u(fanOn);
#endif        
        if (curSensorData.fanWaitResponse) {
          if (fanOn == curSensorData.fanOn) curSensorData.fanWaitResponse = false; // reset flag, we got our response &
        }
        else // not waiting for a response, only updating our internal state
          curSensorData.fanOn = fanOn;
      }
      else if (MyMessage_getType(message) == V_HVAC_SPEED) {
        strncpy(curSensorData.fanSpeed,MyMessage_getString(message),7);
#ifdef SDS_USE_SERIAL
        Serial_print_s("msg:FanSpeed: ");Serial_println_s(curSensorData.fanSpeed);
#endif
      }
    }
    if (sensorId == ALARM_CHILD_ID) {
      if (MyMessage_getType(message) == V_STATUS) {
        curSensorData.alarmActive = MyMessage_getBool(message);
#ifdef SDS_USE_SERIAL
        Serial_print_s("msg:AlarmActive: ");Serial_println_u(curSensorData.alarmActive);
#endif
      }
    }
  }
} // receive

void setup() {
#ifdef SDS_USE_SERIAL
  Serial_begin(115200);
  Serial_println_s("Start FanRemote STM8");
#endif
  bool ok;
  pinMode (LED_BUILTIN,OUTPUT);
  digitalWrite (PIN_BUTTON,HIGH);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  attachInterrupt(3,portd_irq, 0); // mode parameter is not implemented in sduino, but we do the config manually hereafter
  disableInterrupts(); // EXTI->CR1 kan je maar schrijven onder disabled interrupts (CCR=3); manual nog eens goed lezen, maar zo lijkt het wel te werken
  EXTI->CR1 = 0x80; // set falling interrupt for all pins on port D
  GPIOD->CR2 = 0x08; // enable ext interrupt on pin PD3
  enableInterrupts();

  curSensorData.fanOn = false;
  curSensorData.alarmActive = false;
  curSensorData.fanWaitResponse = false;
  MyMessage_init2(&sensorMsg,0,0); // TODO! : MyMessage_init()
  ok = MySensors_init(MY_NODE_ID);
  digitalWrite(LED_BUILTIN, !ok);

} // setup

void loop() {

  MySensors_process(); // call this as much as possible

  // 1. check button
  if (buttonPressed) { // got button press from irq handler
    if ((millis() - debounceMillis) < DEBOUNCE_TIME) return;
    debounceMillis = millis();
    buttonPressed = false;
    // toggle fan on/off
    curSensorData.fanOn = !curSensorData.fanOn;
    curSensorData.fanWaitResponse = true;
#ifdef SDS_USE_SERIAL
    Serial_println_s("Button pressed!");
#endif
    // request is sent & resent below , until a response received
  }

  // 2. MySensors communication
  // sending request to fan
  if ((curSensorData.fanWaitResponse) && ((millis()-radioTxLastMillis) >= radioResponseTimeoutInterval))
  {
#ifdef SDS_USE_SERIAL
    Serial_print_s("sending request to fan : ");Serial_println_u(curSensorData.fanOn);
#endif
    MyMessage_setDestination(&sensorMsg, FAN_NODE_ID);
    MyMessage_setSensor(&sensorMsg, FAN_CHILD_ID);
    MyMessage_setType(&sensorMsg, V_STATUS);
    MyMessage_setByte(&sensorMsg, curSensorData.fanOn);
    MySensors_send(&sensorMsg, true); // request echo
    radioTxLastMillis = millis();
#ifdef SDS_USE_SERIAL
    Serial_println_s("done!");
#endif
  }

  if (!curSensorData.fanWaitResponse) {
    // go back to sleep
    // enkel nodig in active halt, maar hier niet het geval, want geen AWU
    //CLK->ICKR |= CLK_ICKR_SWUAH; // MVR off during halt = lower power, but slower wake-up from halt
    //FLASH->CR1 |= FLASH_CR1_AHALT; // Flash in power-down when MCU is in Active-halt mode
    for (int i=0;i<5;i++) {
      digitalWrite(LED_BUILTIN,HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN,LOW);
      delay(100);
    }
    pinMode(LED_BUILTIN,INPUT);
    RF24_sleep(); // == transportDisable
    halt();
    //wakeup
    pinMode(LED_BUILTIN,OUTPUT);
    RF24_standBy(); // == transportReInitialise
  }
} // loop
