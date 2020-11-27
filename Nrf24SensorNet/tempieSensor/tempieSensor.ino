/*
// dummy sensor connections :
A0 : solar cell voltage over a resistor divider
vcc = lipo battery, read via trick (readVcc function)

// TODO : find a clean way to use this code with a specific RF24 & RF24Network configuration
// for now change this in RF24Network_config.h

#define ENABLE_SLEEP_MODE // include code for sleepNode
#define DISABLE_FRAGMENTATION //exclude code for complex long packets

SERIAL_DEBUG can be defined in RF24_config.h

TODO : 
- reboot function doesn't work, nano ends up in a bootloader loop

*/
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <EEPROM.h>
#include "radioProtocol.h"
#include <avr/wdt.h> // for rebooting via watchdog & watchdog constants
#include <OneWire.h> // DS18B20
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Si7021.h>



#ifdef SERIAL_DEBUG
  #include <printf.h>
#endif

/************************************************************************************/
/* CONFIG SETUP                                                                     */
/************************************************************************************/
// parameter id's
#define PARAM_COUNT               4
#define PARAM_NODE_ADDRESS        0x0
#define PARAM_NODE_NAME           0x1
#define PARAM_RADIO_TX_INTERVAL   0x2
#define PARAM_DUMMY1              0x3 // bv. tellerstand die een reboot moet overleven

// parameter default values - writing these to eeprom on the first boot
#define DEFAULT_NODE_ADDRESS      0x2         // 2 bytes
#define DEFAULT_NODE_NAME         "TEMPIE"    // 8-char max name
#define DEFAULT_RADIO_TX_INTERVAL 5000        // 0xFFFFFFFF=no transmission, otherwise 4 bytes value in ms
#define DEFAULT_DUMMY1            0x12345678  // dummy, 4 bytes

// eeprom addresses for parameters
#define PARAM_BASE_EEPROM_ADDRESS 0x0
uint8_t paramSizes[PARAM_COUNT] = {2,8,4,4};
// the param eeprom addresses are then PARAM_BASE_EEPROM_ADDRESS, PARAM_BASE_EEPROM_ADDRESS+ paramSizes[0], ...

// parameter names go in flash, not in eeprom anymore
static const char PROGMEM paramNames[PARAM_COUNT][9] = {"Address","NodeName","TxIntval","Dummy1"};

#define CONFIG_INITCONFIG_EEPROM_ADDRESS          511 // last byte in eeprom, set 1 = valid eeprom contents, else invalid, not initialized

/************************************************************************************/
/* RADIO SETUP                                                                      */
/************************************************************************************/
RF24 radio(7,8);                      // nRF24L01(+) radio attached using Getting Started board 
RF24Network network(radio);           // Network uses that radio
const uint16_t baseNodeAddress = 00;  // Address of the base node

// these parameters will be copied from eeprom to RAM for use at runtime
uint16_t myNodeAddress = DEFAULT_NODE_ADDRESS; // our address
uint32_t radioTxInterval = DEFAULT_RADIO_TX_INTERVAL; // periodic data transmission in ms

uint32_t radioTxLastMillis;
bool stayAwake = true; // sleeping on/off
uint32_t lastSleepMillis;

/************************************************************************************/
/* SENSOR SPECIFIC                                                                  */
/************************************************************************************/
// sensor specifics
#define READING_INTERVAL 60000 // in ms
// TEMP : until we devise a more flexible way to define payload types per sensor
#define PAYLOADTYPE_SENSORDATA    1
#define PAYLOADTYPE_SENSORSTATUS  2
OneWire  ds(14);  // on gpio14=D5 (a 4.7K resistor is necessary)
byte addr[8];
byte present = 0;
byte type_s;

// BMP280 Pressure + Temperature Sensor
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
Adafruit_Si7021 si7021 = Adafruit_Si7021();

bool ds18b20Present, bmpPresent, si7021Present;

uint32_t sensorReadingMillis = -READING_INTERVAL; // reading interval

struct payloadTempieData_t {
  uint8_t payloadType = PAYLOADTYPE_SENSORDATA;
  float ds18b20Temperature;
  float si7021Temperature;
  float si7021Humidity;
  float bmp280Temperature;
  float bmp280Pressure;
};

struct payloadTempieStatus_t {
  uint8_t payloadType = PAYLOADTYPE_SENSORSTATUS;
  uint16_t vccValue;
  uint16_t packetsOK;
  uint16_t packetsNOK;
};

payloadTempieData_t curSensorData;
payloadTempieStatus_t curSensorStatus;
uint8_t payloadIdToTx = 0; // send the different payload types in round robin, not all during 1 TxInterval, seems like hub misses data more often then

// TODO : modify approach to support multiple payload types per sensor
#define PAYLOAD_ITEM_COUNT  4
uint8_t payloadItemTypes[PAYLOAD_ITEM_COUNT] = {PAYLOAD_TYPE_UINT16,PAYLOAD_TYPE_UINT16,
                                                PAYLOAD_TYPE_UINT16,PAYLOAD_TYPE_UINT16};

/************************************************************************************/
/* CONFIG HELPERS                                                                   */
/************************************************************************************/
static void readParameterValueFromEeprom(uint8_t paramId, uint8_t *paramValue) {
  uint8_t paramEepromAddress = PARAM_BASE_EEPROM_ADDRESS;
  for (int i=0; i<paramId;i++) paramEepromAddress += paramSizes[i];
  for (int i=0;i<paramSizes[paramId];i++) paramValue[i] = EEPROM.read(paramEepromAddress+i);
}// readParameterValueFromEeprom

static void writeParameterValueToEeprom(uint8_t paramId, uint8_t *paramValue) {
  uint8_t paramEepromAddress = PARAM_BASE_EEPROM_ADDRESS;
  for (int i=0; i<paramId;i++) paramEepromAddress += paramSizes[i];
  for (int i=0;i<paramSizes[paramId];i++) EEPROM.update(paramEepromAddress+i,paramValue[i]);
}// writeParameterValueToEeprom

// load config settings from eeprom
static void loadParametersFromEeprom() {
  Serial.println("loading parameters from eeprom : ");

  // PARAM_NODE_ADDRESS
  uint16_t eepromNodeAddress;
  readParameterValueFromEeprom(PARAM_NODE_ADDRESS, (uint8_t*) &eepromNodeAddress);
  Serial.print(F("got this node address from eeprom : ")); Serial.println(eepromNodeAddress, HEX);
  if ((eepromNodeAddress != 0xFFFF) && (eepromNodeAddress != 0)) {
    myNodeAddress = eepromNodeAddress;
    Serial.print("setting myNodeAddress to ");Serial.println(myNodeAddress);
  }
  // PARAM_RADIO_TX_INTERVAL
  uint32_t eepromRadioTxInterval;
  readParameterValueFromEeprom(PARAM_RADIO_TX_INTERVAL, (uint8_t*) &eepromRadioTxInterval);
  Serial.print(F("got this radioTxInterval from eeprom : ")); Serial.println(eepromRadioTxInterval, HEX);
  if (eepromRadioTxInterval > 1000) { // max frequency 1Hz for TX not to spam our network when eeprom error
    radioTxInterval = eepromRadioTxInterval;
    Serial.print("setting radioTxInterval to ");Serial.println(radioTxInterval);
  }
  // example param1
  uint32_t param1;
  readParameterValueFromEeprom(PARAM_DUMMY1, (uint8_t*) &param1);
  Serial.print(F("got dummy1 from eeprom : ")); Serial.println(param1, HEX);

} // loadParametersFromEeprom

// fill eeprom with initial boot settings
static void initEeprom() {
  uint8_t eepromInitialized = EEPROM.read(CONFIG_INITCONFIG_EEPROM_ADDRESS);
  if (eepromInitialized!= 1) {
    // let's initialize the eeprom with the default contents
    char aValue[8];
    Serial.println(F("initializing eeprom!"));
    *(uint16_t*)aValue = DEFAULT_NODE_ADDRESS;
    writeParameterValueToEeprom (PARAM_NODE_ADDRESS, aValue);
    memcpy (aValue, DEFAULT_NODE_NAME,paramSizes[PARAM_NODE_NAME]);
    writeParameterValueToEeprom (PARAM_NODE_NAME, aValue);
    *(uint32_t*)aValue = DEFAULT_RADIO_TX_INTERVAL;
    writeParameterValueToEeprom (PARAM_RADIO_TX_INTERVAL, aValue);
    *(uint32_t*)aValue = DEFAULT_DUMMY1;
    writeParameterValueToEeprom (PARAM_DUMMY1, aValue);

    EEPROM.update(CONFIG_INITCONFIG_EEPROM_ADDRESS,1); // and now eeprom is initialized
  }
  else {
    Serial.println(F("found intialized eeprom!"));
  }
} // initEeprom

/************************************************************************************/
/* RADIO CONFIG INTERFACE                                                           */
/************************************************************************************/
void handleRadioCommand () {
  RF24NetworkHeader inHeader, outHeader;
  commandReplyPayload_t inCommand, outReply;
  network.read(inHeader,&inCommand,sizeof(inCommand));
  outHeader.to_node = inHeader.from_node;
  outHeader.type = HEADER_REPLY;
  outReply.id = inCommand.id;
  for (int i=0;i<8;i++) outReply.data[i] = inCommand.data[i]; // TEMP echo back received data for commands that don't reply data

  switch (inCommand.id) {
    uint8_t itemId, paramId;
    case CMD_STAY_AWAKE :
      stayAwake = inCommand.param ? true : false;
      Serial.print(F("stayAwake is now : ")); Serial.println(stayAwake);
      lastSleepMillis = millis(); // if stayAwake = false, we go to sleep in app. 1000ms
      break;
    case CMD_REBOOT :
      Serial.println(F("rebooting now!"));
      network.write(outHeader,&outReply,sizeof(outReply));
      reboot(); //TODO FIX doesn't work
      break;
    case CMD_RELOAD_EEPROM:
      Serial.println(F("reload eeprom settings"));
      loadParametersFromEeprom();
      break;
    case CMD_RESET_EEPROM :
        Serial.println(F("doing RESET_EEPROM!"));
      EEPROM.update(CONFIG_INITCONFIG_EEPROM_ADDRESS,0); // make initEeprom believe eeprom is uninitialized
      initEeprom();
      break;
    case CMD_GET_PAYLOAD_ITEM_COUNT :
      outReply.param = PAYLOAD_ITEM_COUNT;
      break;
    case CMD_GET_PAYLOAD_ITEM_TYPE :
      itemId = inCommand.param;
      if (itemId < PAYLOAD_ITEM_COUNT) 
        outReply.param = payloadItemTypes[itemId];
      else 
        outReply.param = REPLY_INVALID;
      break;
    case CMD_GET_PARAMETER_COUNT :
      outReply.param = PARAM_COUNT;
      break;
    case CMD_GET_PARAM_NAME :
      paramId = inCommand.param;
      if (paramId < PARAM_COUNT) {
        outReply.param = paramId;
        memcpy_P(outReply.data,paramNames[paramId],8);
      }
      else {
        outReply.param = REPLY_INVALID;
      }
      break;
    case CMD_GET_NODE_NAME :
      readParameterValueFromEeprom(PARAM_NODE_NAME, outReply.data);
      break;
    case CMD_SET_NODE_NAME :
      writeParameterValueToEeprom(PARAM_NODE_NAME,inCommand.data);
      break;
    case CMD_GET_PARAM_VALUE :
      paramId = inCommand.param;
      if (paramId < PARAM_COUNT) {
        outReply.param = paramId;
        readParameterValueFromEeprom(paramId, outReply.data);
      }
      else {
        outReply.param = REPLY_INVALID;
      }
      break;
    case CMD_SET_PARAM_VALUE :
      paramId = inCommand.param;
      if (paramId < PARAM_COUNT) {
        outReply.param = paramId;
        writeParameterValueToEeprom(paramId, inCommand.data);
      }
      else {
        outReply.param = REPLY_INVALID;
      }
      break;
  }
  
  Serial.print(F("sending reply to:"));Serial.println(outHeader.to_node);
  Serial.print(F("id:"));Serial.println(outReply.id);
  Serial.print(F("param:"));Serial.println(outReply.param);
  Serial.print("data:");
  for (int i=0;i<8;i++) { Serial.print(outReply.data[i],HEX); Serial.print(" "); }
  Serial.println();
  
  bool ok = network.write(outHeader,&outReply,sizeof(outReply));
  if (ok) curSensorStatus.packetsOK ++;
  else curSensorStatus.packetsNOK++;

} // handleRadioCommand

/************************************************************************************/
/* SENSOR SPECIFIC                                                                  */
/************************************************************************************/
void setupBMP280Sensor() {
  bmpPresent = bmp.begin(0x76);
  if (!bmpPresent) return;

  /* copied from bmp280_sensortest */
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  bmp_temp->printSensorDetails();
} // setupBMP280Sensor

void setupSi7021Sensor() {
  si7021Present = si7021.begin();
} // setupSi7021Sensor

// demo code -> TODO : cleanup
void setupDS18B20TemperatureSensor() {
  ds18b20Present = false;
  if ( !ds.search(addr)) {
    Serial.println("DS18B20: sensor not found");
    // todo : notify web intf
    ds.reset_search();
    return;
  }
  
  Serial.print("ROM =");
  for( int i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("DS18B20: CRC is not valid!");
      return;
  }
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }
  ds18b20Present = true;
} // setupDS18B20TemperatureSensor

// todo : make non-blocking!
float getDS18B20Temperature() {
  byte data[12];
  float celsius;

  digitalWrite(LED_BUILTIN, LOW); // led on during conversion

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  digitalWrite(LED_BUILTIN, HIGH); // led on during conversion

  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( int i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print("CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  //fahrenheit = celsius * 1.8 + 32.0;
  return celsius;
  
} // getDS18B20Temperature

/************************************************************************************/
/* MISC                                                                             */
/************************************************************************************/
// trigger reboot from SW
static void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
} // reboot

static uint16_t readVcc() {
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

/************************************************************************************/
/* MAIN                                                                             */
/************************************************************************************/
void setup(void)
{
  // TODO FIX - this doesn't work
  MCUSR = 0;// Clear the WD System Reset Flag
  wdt_disable(); // on atmega168 without optiboot we get stuck in bootloader anyway after a watchdog reset, never get to here
  // END TO FIX
  #ifdef SERIAL_DEBUG
    printf_begin(); //sds test for SERIAL_DEBUG
  #endif
  Serial.begin(115200);
  Serial.println(F("TempieSensor!"));

  // eeprom setup
  initEeprom(); // initial boot settings
  loadParametersFromEeprom();

  // radio setup
   SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ myNodeAddress);
  network.setup_watchdog(WDTO_1S); // configuration for sleep mode -> 1 cycle = 1second

  // sensors setup
  setupDS18B20TemperatureSensor();
  setupBMP280Sensor();
  setupSi7021Sensor();

} // setup

void loop() {
  uint32_t currentMillis = millis();  

  // Check for commands from the hub
  network.update(); 
  while ( network.available() ) {
    RF24NetworkHeader header;
    uint16_t nbrBytesRx = network.peek(header);
    // eventueel nog checken dat dit command van node 0 komt, de enige die commands mag sturen
    if (header.type == HEADER_COMMAND) { // reply to a command
      handleRadioCommand();
    }
    else if (header.type == HEADER_REPLY) {
      commandReplyPayload_t payload;
      network.read(header,&payload,sizeof(payload));
      // TODO : do something wih the data
    }
    else if (header.type == HEADER_DATA) {
      // we don't expect any data for now
      network.read(header,NULL,0); // consume the payload & remove it from the receive Q
    }
    else {
      // unknown header type
      network.read(header,NULL,0); // consume the payload & remove it from the receive Q
    }
  }

  // reading sensor data every READING_INTERVAL
  if ((currentMillis - sensorReadingMillis) > READING_INTERVAL) {
    curSensorStatus.vccValue = readVcc();
  
    if (ds18b20Present){
      curSensorData.ds18b20Temperature = getDS18B20Temperature();
    }
    else { // fake some data
      curSensorData.ds18b20Temperature = 18.2;
      Serial.print(F("DS18B20 : ")); Serial.print(curSensorData.ds18b20Temperature);Serial.println("°C");
    }
    if (bmpPresent) {
      sensors_event_t temp_event, pressure_event;
      bmp_temp->getEvent(&temp_event);
      bmp_pressure->getEvent(&pressure_event);
      curSensorData.bmp280Temperature = temp_event.temperature;
      curSensorData.bmp280Pressure = pressure_event.pressure;
      Serial.print(F("BMP280 Temperature = "));Serial.print(curSensorData.bmp280Temperature);Serial.println(" °C");
      Serial.print(F("BMP280 Pressure = "));Serial.print(curSensorData.bmp280Pressure);Serial.println(" hPa");
    }
    else { // fake some data
      curSensorData.bmp280Temperature = 18.28;
      curSensorData.bmp280Pressure = 1013.0;
    }
    if (si7021Present) {
      curSensorData.si7021Temperature = si7021.readTemperature();
      curSensorData.si7021Humidity = si7021.readHumidity();
      Serial.print(F("SI7021 Temperature: ")); Serial.print(curSensorData.si7021Temperature, 2); Serial.println("°C");
      Serial.print(F("SI7021 Humidity: ")); Serial.print(curSensorData.si7021Humidity, 2);Serial.println("%");
    }
    else { // fake some data
      curSensorData.si7021Temperature = 18.7021;
      curSensorData.si7021Humidity = 70.21;
    }
    sensorReadingMillis = currentMillis;
  }

  // sending data to the hub
  if ((radioTxInterval != 0xFFFFFFFF) && ( millis() - radioTxLastMillis >= radioTxInterval ))
  {
    bool txOK;
    radioTxLastMillis = millis();

    RF24NetworkHeader outHeader(baseNodeAddress,HEADER_DATA);

    if (payloadIdToTx) {
      txOK = network.write(outHeader,&curSensorData,sizeof(curSensorData));
      if (txOK) curSensorStatus.packetsOK ++;
      else curSensorStatus.packetsNOK++;
      for (int i=0;i<sizeof(curSensorData);i++) {
        Serial.print(*((uint8_t*)&curSensorData + i), HEX);Serial.print(' ');
      }
      Serial.println();
    }
    else {
      txOK = network.write(outHeader,&curSensorStatus,sizeof(curSensorStatus));
      if (txOK) curSensorStatus.packetsOK ++;
      else curSensorStatus.packetsNOK++;
    }
    payloadIdToTx = (payloadIdToTx + 1) % 2;

  }

  // stay awake for a minimum of 1000 ms after a sleep, so hub can send a command like STAY_AWAKE
  if ((millis() - lastSleepMillis) > 1000) {
    if (!stayAwake) {
      uint8_t nbrSleepCycles = 4;
      lastSleepMillis = millis();
      radio.powerDown();
      network.sleepNode(nbrSleepCycles,255); // x cycles sleep, no wake-up from interrupt
      // and here we wake up
      // during sleep the millis stand still, so compensate here for the sleep time
      radioTxLastMillis -= nbrSleepCycles*1000; // 1000ms per sleep cycle configured at setup
      sensorReadingMillis -= nbrSleepCycles*1000; // 1000ms per sleep cycle configured at setup
      radio.powerUp();
    }
  }
} // loop
