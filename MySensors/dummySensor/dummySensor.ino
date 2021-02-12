/*
 maakt gebruik van MySensors + NodeManager framework

// dummy sensor connections :
A0 : solar cell voltage over a resistor divider
vcc = lipo battery on a 5V nano

*/
#define SOLARPIN A0
/**********************************
* MySensors node configuration
*/

// General settings
#define SKETCH_NAME "DummyWithNM"
#define SKETCH_VERSION "1.0"
#define MY_BAUD_RATE 115200
#define MY_NODE_ID 99

//#define MY_DEBUG
// NRF24 radio settings
#define MY_RADIO_NRF24
#define MY_SPLASH_SCREEN_DISABLED
#define MY_RF24_PA_LEVEL RF24_PA_HIGH  // default = RF24_PA_HIGH
//#define MY_RF24_DATARATE RF24_250KBPS // = default

/***********************************
* NodeManager configuration
*/
#define NODEMANAGER_HOOKING ON 	// default = OFF, need this for bug fix in Node Manager
#define NODEMANAGER_DEBUG ON		// default = ON
#define NODEMANAGER_SLEEP ON
#define NODEMANAGER_OTA_CONFIGURATION ON
#define NODEMANAGER_RECEIVE ON 		//default is also ON

// import NodeManager library (a nodeManager object will be then made available)
#include <MySensors_NodeManager.h>

/***********************************
* Add your sensors
*/

#include <sensors/SensorBattery.h>


SensorBattery battery;
// modified SensorBattery in library to allow modifying name
SensorBattery solar(202,"SOLARVOLT"); // een andere child id dan de batterij

// a counter for test
class SensorTestCounter : public Sensor {
	protected:
		int counterValue;

	public:
		SensorTestCounter (char *name): Sensor(-1) {
			_name = name;
			children.allocateBlocks(1);
			new Child(this,INT,nodeManager.getAvailableChildId(0),S_CUSTOM,V_VAR1,_name);
			counterValue = 0;
		};
		
		void onLoop(Child* child) {
			counterValue++;
			child->setValue(counterValue);
		};
};
SensorTestCounter loopCounter("LOOPS");

void checkAdcReference(Sensor *me) {
	// the solar voltage must be measured with INTERNAL adc voltage reference
	// after a SensorBattery loop the voltage reference is set to DEFAULT
	// for measuring VCC via bandgap (hwCPUVoltage function)
  (void) me;
	uint8_t vref = (ADMUX >> 6) & 0x3;
	if (vref != INTERNAL) {
		analogReference(INTERNAL);
		analogRead(SOLARPIN); // dummy read to force switching the voltage reference in hardware
		wait(10); // 5ms looks enough from testing, but just to be sure
	}
} // checkAdcReference

/***********************************
* Main Sketch
*/

void before() {
  /***********************************
  * Configure your sensors  */
  // dummy sensor has fast reporting times for testing

	// set reporting interval for all the sensors
	nodeManager.setReportIntervalSeconds(60); // nodeManager.setReportIntervalMinutes(10);
	nodeManager.setSleepSeconds(11); // set sleep interval

  battery.setMinVoltage(3.2);
  battery.setMaxVoltage(4.2);
	battery.setReportIntervalSeconds(30); // default in class = 60minutes

	solar.setBatteryPin(SOLARPIN);
  solar.setBatteryInternalVcc(false);
  solar.setBatteryVoltsPerBit(0.0069);
	solar.setReportIntervalSeconds(10); // default in class = 60minutes
	solar.setSendBatteryLevel(false); // het is geen batterij
	solar.setPreLoopHook(checkAdcReference); // because Node Manager doesn't handle correctly the switching of ADC voltage reference

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
