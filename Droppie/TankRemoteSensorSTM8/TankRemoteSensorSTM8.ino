#include "Opentherm.h"
#include "sonar.h"

/*
32-bit dataframe :
data[0] : TankEmpty
data[1] : charging
data[2:11] : batAnalogRead, 0..1023 (10-bit ADC)
data[12:23] : distanceCm, 0..4095 (12-bit)
data[24:31] : XOR of bytes 0..2 XOR 0xAA -> the last XOR avoids that 0xFFFFFFFF is seen as valid frame

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

// keep PD4/PD5 free (uart for debug)
// keep PD1 free (SWIM, avoid interference with stlink)
// we keep I2C & SPI free as well
#define PIN_OUTPUT_TRIG               PA3
#define PIN_INPUT_ECHO                PA2
#define PIN_OUTPUT_TURNOFF_CHARGING   PC3
#define PIN_INPUT_LEVEL_SENSOR        PD4
#define PIN_INPUT_BATREFERENCE          A0 // ==PC4
// and additionally for opentherm comms
//#define T_RX_PIN  PD2
//#define T_TX_PIN  PD3

#define OPENTHERM_RESP_TIMEOUT_INTERVAL 1000 // expecting reply in 1s

uint32_t txMillis;
bool waitResponse = false;
// charging as default, in case we can't communicate, at least we can build up charge, and other side should do the same
// is mcu = dead, the opentherm-TX output will be low, and bus voltage will be 15V and facilitate charging
bool batCharging = true; 
int batAnalogRead = 1023; // need this combination at startup to force charging at startup

volatile uint8_t dummy;
INTERRUPT_HANDLER(AWU_IRQHandler, ITC_IRQ_AWU) {
  dummy = AWU->CSR; // read to clear int flag
}

static uint32_t buildDataFrame(bool tankEmpty, bool batCharging, int batAnalogRead, uint32_t distanceCm) {
  uint32_t retVal;
  uint32_t xorVal;
  batCharging &= 0x1; // to be sure
  distanceCm = distanceCm & 0xFFF; // keep 12 bits, 0..4095 is more than enough, range is 500cm
  retVal = (distanceCm << 12) | (batAnalogRead << 2) | (batCharging << 1) | (tankEmpty & 0x1); // batAnalogRead is 10bits, so we can shift left without losing data
  xorVal = (retVal & 0xFF) ^ ((retVal >> 8) & 0xFF) ^ ((retVal >> 16) & 0xFF) ^ 0xAA;
  retVal = retVal | ((xorVal & 0xFF) << 24);
  return retVal;

} // buildDataFrame

// using AWU on STM8
// temp : for now just implement 10s sleep, because that's what we need
static void hwSleep(uint32_t ms)
{
  (void) ms;
  Serial_flush();
  //delay(100); //omdat flush niet correct lijkt te werken
  disableInterrupts();
  AWU->CSR = 0;

  CLK->ICKR |= CLK_ICKR_SWUAH; // MVR off during halt = lower power, but slower wake-up from halt
  FLASH->CR1 |= FLASH_CR1_AHALT; // Flash in power-down when MCU is in Active-halt mode
  
  AWU->APR = 21;
  AWU->TBR = 15;
  AWU->CSR |= AWU_CSR_AWUEN;
  dummy = AWU->CSR; // read to clear int flag

  // als de PumpSensor de charging aanzet, krijgen we een pin change van idle->active, en worden we gewekt!
  // todo: OT_DeInit(), zodat we niet kunnen wakker worden van activity op de OT-RX line
  // voorlopig quick fix hier
  GPIOD->CR2 = 0x0; // disable ext interrupt on pin PD2

  enableInterrupts();
  halt();
  // todo: OT_Init()?, of toch minstens de pin change int weer aanzetten
  GPIOD->CR2 = 0x04; // enable ext interrupt on pin PD2

} // hwSleep

void setup() {
  Serial_begin(115200); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  sonar_init(PIN_OUTPUT_TRIG,PIN_INPUT_ECHO);

  OT_init();
  pinMode(PIN_OUTPUT_TURNOFF_CHARGING, OUTPUT);
  pinMode(PIN_INPUT_LEVEL_SENSOR, INPUT_PULLUP); // on final hardware this should be no pullup (external pullup)
  Serial_println_s("Start Remote Sensor");

} // setup

void loop() {
  uint32_t distanceCm;
  bool tankEmpty;
  uint32_t otData,otResponse;

  if (waitResponse) {
    if (OT_available()) {
      otResponse = OT_readFrame();
      Serial_print_s("Received response : "); Serial_println_ub(otResponse,HEX);
      // for the moment no need to do anything with the response
      waitResponse = false;
    }
    else if ((millis() - txMillis) > OPENTHERM_RESP_TIMEOUT_INTERVAL) {
      Serial_println_s("response timeout");
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
    if (batCharging) {
      digitalWrite(PIN_OUTPUT_TURNOFF_CHARGING, false); // turn on charging
    }
    digitalWrite(LED_BUILTIN,HIGH);
    hwSleep(10000);
    // awake again
    digitalWrite(LED_BUILTIN,LOW); // builtin led on while awake
    
    // we measure the battery voltage based on the 15V bus voltage (zener reference)
    // therefore disable charging here, because charging pulls bus voltage down
    // set active bus voltage 15V, and hope it's stable after the delay
    digitalWrite(PIN_OUTPUT_TURNOFF_CHARGING, true); // turn off charging
    OT_setTxState(true);
    delay(20); // nodig? (1000ms maakte geen verschil)

    batAnalogRead = analogRead(PIN_INPUT_BATREFERENCE);
    distanceCm = (uint32_t) sonar_ping_cm();
    tankEmpty = digitalRead(PIN_INPUT_LEVEL_SENSOR); // sensor contact closed (reads LOW) = tank not empty, open (reads HIGH) = tank empty
    // charging with hysteresis (see intro)
    // < 870 : stop charging, > 970 start charging
    // < 800 : impossible value (probably error on bus voltage, for instance PumpSensor not powered) -> ignore
    if (batCharging && (batAnalogRead < 870) && (batAnalogRead > 800)) batCharging = false;
    else if (!batCharging && (batAnalogRead > 970)) batCharging = true;

    Serial_print_s("tankEmpty = ");Serial_println_u(tankEmpty);
    Serial_print_s("batCharging:");Serial_println_u(batCharging);
    Serial_print_s("batAnalogRead = ");Serial_println_u(batAnalogRead);
    Serial_print_s("distanceCm = ");Serial_println_u(distanceCm);

    // send these values over opentherm
    OT_setTxState(false);
    digitalWrite(PIN_OUTPUT_TURNOFF_CHARGING, true); // turn off charging, to enable modem function
    // delay not required here, tested
    otData = buildDataFrame(tankEmpty,batCharging,batAnalogRead,distanceCm);
    Serial_print_s("sending now : ");Serial_println_ub(otData,HEX);
    txMillis = millis();
    OT_writeFrameInt(otData);
    waitResponse = true;
  }
  
} // loop
