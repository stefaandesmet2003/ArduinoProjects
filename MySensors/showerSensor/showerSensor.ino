/**********************************
* MySensors node configuration
*/
//hw config van de sensor
#define LDR_PIN             2
#define IR_PIN              3
#define DHT22_DATA_PIN      4
#define DHT_IR_POW_OUT_PIN  5
#define LED_OUT_PIN         8
#define NRF_CSN_PIN         9
#define NRF_CE_PIN          10
#define LDR_CALIBRATION_PIN A3

// General settings for MySensors
#define SKETCH_NAME "ShowerSensor"
#define SKETCH_VERSION "0.1"
#define MY_BAUD_RATE 115200
#define MY_NODE_ID 4

//#define MY_DEBUG
#define NODEMANAGER_DEBUG OFF // staat default aan

// NRF24 radio settings
#define MY_RADIO_NRF24
#define MY_SPLASH_SCREEN_DISABLED
#define MY_RF24_PA_LEVEL RF24_PA_MAX
//#define MY_RF24_DATARATE RF24_250KBPS // = default, but just to be sure
// soldeerfoutje rechtzetten:
#define MY_RF24_CE_PIN NRF_CE_PIN
#define MY_RF24_CS_PIN NRF_CSN_PIN

/***********************************
* NodeManager configuration
*/

#define NODEMANAGER_SLEEP ON
#define NODEMANAGER_OTA_CONFIGURATION ON
#define NODEMANAGER_RECEIVE ON 		//default is also ON
#define NODEMANAGER_POWER_MANAGER ON

// import NodeManager library (a nodeManager object will be then made available)
#include <MySensors_NodeManager.h>

// Add a battery sensor
#include <sensors/SensorBattery.h>
SensorBattery battery;

#include <sensors/SensorDHT22.h>
SensorDHT22 dht22(4);

PowerManager power(-1,DHT_IR_POW_OUT_PIN);

#include <sensors/SensorDigitalOutput.h>
SensorDigitalOutput led(LED_OUT_PIN);

#include <sensors/SensorLDR.h>
SensorLDR ldr(LDR_CALIBRATION_PIN);

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
	battery.setReportIntervalSeconds(10);

  ldr.setRangeMin(50); // adc value onder burolamp
  ldr.setRangeMax(850); // adc value pitch dark

	// set reporting interval for all the sensors
	//sds nodeManager.setReportIntervalMinutes(10);
	nodeManager.setReportIntervalSeconds(30);
  #if NODEMANAGER_SLEEP == ON
	// set sleep interval
	nodeManager.setSleepSeconds(30); // sds : was 10 minutes
  #endif
	nodeManager.setPowerManager(power);

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
