// compile for AVR
// should work, but not tested after copying over latest changes from STM8 code
#include "Opentherm.h"
#include <NewPing.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

/*
32-bit dataframe :
data[0] : TankEmpty
data[1] : charging
data[2:11] : batVoltage, 0..1023 (10-bit ADC)
data[12:23] : distanceCm, 0..4095 (12-bit)
data[24:31] : XOR of bytes 0..2

battery voltage measurement
****************************
based on the 15V zener reference (in reality it's 14.7V), which gives a active bus voltage of 15.3V
15.3V over voltage divider 330k//1M = 3.77V on A0
the measured value changes with the battery voltage 
-> same principle as on AVR, where battery voltage is measured using the 1.1V bandgap ref, and back-calculate the VCC
but in practice the analogRead values deviate from the theoretical values, TO CHECK!
3.72V bat = 945 analogRead, 
3.97V bat = 894 analogRead, 
4.10V bat = 870 analogRead

charging with hysteresis
*************************
we take 870 & 970 as thresholds
< 870 : stop charging, > 970 start charging

*/

#define PIN_OUTPUT_TRIG               2
#define PIN_OUTPUT_TURNOFF_CHARGING   3
#define PIN_INPUT_ECHO                4
#define PIN_INPUT_LEVEL_SENSOR        5
#define PIN_INPUT_BATVOLTAGE          A0
// and additionally for opentherm comms
//#define T_RX_PIN  7
//#define T_TX_PIN  8

#define OPENTHERM_RESP_TIMEOUT_INTERVAL 1000 // expecting reply in 1s

NewPing sonar(PIN_OUTPUT_TRIG, PIN_INPUT_ECHO, 500); // NewPing setup of pins and maximum distance (cm)
uint32_t txMillis;
bool waitResponse = false;
// charging as default, in case we can't communicate, at least we can build up charge, and other side should do the same
// is mcu = dead, the opentherm-TX output will be low, and bus voltage will be 15V and facilitate charging
bool charging = true; 
int batVoltage = 1023; // need this combination at startup to force charging at startup

static uint32_t buildDataFrame(bool tankEmpty, bool charging, int batVoltage, uint32_t distanceCm) {
  uint32_t retVal;
  uint32_t xorVal;
  charging &= 0x1; // to be sure
  distanceCm = distanceCm & 0xFFF; // keep 12 bits, 0..4095 is more than enough, range is 500cm
  retVal = (distanceCm << 12) | (batVoltage << 2) | (charging << 1) | (tankEmpty & 0x1); // batVoltage is 10bits, so we can shift left without losing data
  xorVal = (retVal & 0xFF) ^ ((retVal >> 8) & 0xFF) ^ ((retVal >> 16) & 0xFF);
  retVal = retVal | ((xorVal & 0xFF) << 24);
  return retVal;

} // buildDataFrame

// Watchdog Timer interrupt service routine. This routine is required
// to allow automatic WDIF and WDIE bit clearance in hardware.
ISR (WDT_vect)
{
}

static void hwPowerDown(const uint8_t wdto)
{
	// Let serial prints finish (debug, log etc)
	Serial.flush();

	// disable ADC for power saving
	ADCSRA &= ~(1 << ADEN);
	// save WDT settings
	const uint8_t WDTsave = WDTCSR;
	wdt_enable(wdto);
	// enable WDT interrupt before system reset
	WDTCSR |= (1 << WDCE) | (1 << WDIE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	cli();
	sleep_enable();
	sleep_bod_disable();
	// Enable interrupts & sleep until WDT or ext. interrupt
	sei();
	// Directly sleep CPU, to prevent race conditions!
	// Ref: chapter 7.7 of ATMega328P datasheet
	sleep_cpu();
	sleep_disable();
	// restore previous WDT settings
	cli();
	wdt_reset();
	// enable WDT changes
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	// restore saved WDT settings
	WDTCSR = WDTsave;
	sei();
	// enable ADC
	ADCSRA |= (1 << ADEN);
} // hwPowerDown

static void hwSleep(uint32_t ms)
{
	// Sleeping with watchdog only supports multiples of 16ms.
	// Round up to next multiple of 16ms, to assure we sleep at least the
	// requested amount of time. Sleep of 0ms will not sleep at all!
	ms += 15u;

	while (ms >= 16) {
		for (uint8_t period = 9u; ; --period) {
			const uint16_t comparatorMS = 1 << (period + 4);
			if ( ms >= comparatorMS) {
				hwPowerDown(period); // 8192ms => 9, 16ms => 0
				ms -= comparatorMS;
				break;
			}
		}
	}
} // hwSleep

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  OT_init();
  pinMode(PIN_OUTPUT_TURNOFF_CHARGING, OUTPUT);
  pinMode(PIN_INPUT_LEVEL_SENSOR, INPUT_PULLUP); // on final hardware this should be no pullup (external pullup)
  // ultrasonic sensor pins are configured by newping lib
  Serial.println("Start Remote Sensor");

} // setup

void loop() {
  uint32_t distanceCm;
  bool tankEmpty;
  uint32_t otData,otResponse;

  if (waitResponse) {
    if (OT_available()) {
      otResponse = OT_readFrame();
      Serial.print("Received response : "); Serial.println(otResponse,HEX);
      // for the moment no need to do anything with the response
      waitResponse = false;
    }
    else if ((millis() - txMillis) > OPENTHERM_RESP_TIMEOUT_INTERVAL) {
      Serial.println("response timeout");
      waitResponse = false;
      // TODO : we could implement a retry mechanism here.
      // for now, we are happy with a retry on the next measurement
    }
  }
  else {
    // set OT-TX pin LOW during sleep, to avoid opto current (2mA)
    // this will set 15V active voltage on the OpenTherm bus, which helps charging if needed
    OT_setTxState(true);
    // for now : only turn on charger if needed
    // if we always keep charging on during sleep, the battery may never discharge, because of permanent low current from PumpSensor?
    // like permanent trickle charging
    // drawback : the chg_off circuit consumes 0.2mA (R7 too small:/)
    if (charging) {
      digitalWrite(PIN_OUTPUT_TURNOFF_CHARGING, false); // turn on charging
    }
    digitalWrite(LED_BUILTIN,HIGH);
    hwSleep(10000);
    // awake again
    digitalWrite(LED_BUILTIN,LOW); // builtin led on while awake

    // we measure the batVoltage based on the 15V bus voltage (zener reference)
    // therefore disable charging here, because charging pulls bus voltage down
    // set active bus voltage 15V, and hope it's stable after the delay
    digitalWrite(PIN_OUTPUT_TURNOFF_CHARGING, true); // turn off charging
    OT_setTxState(true);
    delay(20); // nodig? (1000ms maakte geen verschil)

    batVoltage = analogRead(PIN_INPUT_BATVOLTAGE);
    distanceCm = (uint32_t) sonar.ping_cm(); // why is return value a long, when the max range is 500??
    tankEmpty = digitalRead(PIN_INPUT_LEVEL_SENSOR); // sensor contact closed (reads LOW) = tank not empty, open (reads HIGH) = tank empty
    // charging with hysteresis (see intro)
    // < 870 : stop charging, > 970 start charging
    if (charging && (batVoltage < 870)) charging = false;
    else if (!charging && (batVoltage > 970)) charging = true;

    Serial.print("tankEmpty = ");Serial.println(tankEmpty);
    Serial.print("charging:");Serial.println(charging);
    Serial.print("batVoltage = ");Serial.println(batVoltage);
    Serial.print("distanceCm = ");Serial.println(distanceCm);

    // send these values over opentherm
    OT_setTxState(false);
    digitalWrite(PIN_OUTPUT_TURNOFF_CHARGING, true); // turn off charging, to enable modem function
    // delay not required here, tested
    otData = buildDataFrame(tankEmpty,charging,batVoltage,distanceCm);
    Serial.print("sending now : ");Serial.println(otData,HEX);
    txMillis = millis();
    OT_writeFrameInt(otData);
    waitResponse = true;
  }
  
} // loop
