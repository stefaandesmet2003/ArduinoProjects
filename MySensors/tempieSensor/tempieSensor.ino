/**********************************
* MySensors node configuration
*/

// General settings
#define SKETCH_NAME "TempieSensor"
#define SKETCH_VERSION "0.2"
#define MY_BAUD_RATE 115200
#define MY_NODE_ID 3

//#define MY_DEBUG

// NRF24 radio settings
#define MY_RADIO_NRF24
#define MY_SPLASH_SCREEN_DISABLED
#define MY_RF24_PA_LEVEL RF24_PA_MAX
//#define MY_RF24_DATARATE RF24_250KBPS // = default


/***********************************
* NodeManager configuration
*/
#define NODEMANAGER_DEBUG OFF // default = ON
#define NODEMANAGER_SLEEP ON
#define NODEMANAGER_OTA_CONFIGURATION ON
#define NODEMANAGER_RECEIVE ON 		//default is also ON

// import NodeManager library (a nodeManager object will be then made available)
#include <MySensors_NodeManager.h>

// Add a battery sensor
#include <sensors/SensorBattery.h>
SensorBattery battery;

#include <sensors/SensorBMP280.h>
//SensorBMP280 bmp280; // met auto child_id : 1,2,3
SensorBMP280 bmp280(1); // met manuele child_id 1 = V_TEMP, 2 = V_PRESSURE, 3 = V_FORECAST

#include <sensors/SensorSI7021.h>
//SensorSI7021 si7021; // met auto child_id, wordt 4+5 als de constructor na SensorBMP280 wordt aangeroepen
SensorSI7021 si7021(4); // child_id 4 = V_TEMP, 5 = V_HUM

void before() {
  /***********************************
  * Configure your sensors  */
  // elke 10 seconden efkes wakker, zo kan de controller iets vragen
  // als de controller niets vraagt gaat sensor na ongeveer 500ms terug slapen (smart sleep)
  // elke 60seconden sturen we een report van de sensors, dat duurt ongeveer 100ms (timing uit de serial log)
  
  battery.setBatteryPin(A0);
  battery.setBatteryInternalVcc(false);
  battery.setMinVoltage(3.2);
  battery.setMaxVoltage(4.2);
  battery.setBatteryVoltsPerBit(0.0045);
  battery.setReportIntervalSeconds(60);

	// set reporting interval for all the sensors
	//sds nodeManager.setReportIntervalMinutes(10);
	nodeManager.setReportIntervalSeconds(60);
  #if NODEMANAGER_SLEEP == ON
	// set sleep interval
	nodeManager.setSleepSeconds(10); // sds : was 10 minutes
  #endif
  
	// call NodeManager before routine
	nodeManager.before();
}

void presentation() {
	// call NodeManager presentation routine
	nodeManager.presentation();
}

void setup() {
	// call NodeManager setup routine
	nodeManager.setup();
}

void loop() {
	// call NodeManager loop routine
	nodeManager.loop();
}

#if NODEMANAGER_RECEIVE == ON
void receive(const MyMessage &message) {
	// call NodeManager receive routine
	nodeManager.receive(message);
}
#endif

#if NODEMANAGER_TIME == ON
void receiveTime(unsigned long ts) {
	// call NodeManager receiveTime routine
	nodeManager.receiveTime(ts);
}
#endif
