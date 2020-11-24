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


#ifdef SERIAL_DEBUG
  #include <printf.h>
#endif

// parameter id's
#define PARAM_COUNT               4
#define PARAM_NODE_ADDRESS        0x0
#define PARAM_NODE_NAME           0x1
#define PARAM_RADIO_TX_INTERVAL   0x2
#define PARAM_DUMMY1              0x3 // bv. tellerstand die een reboot moet overleven

// parameter default values - writing these to eeprom on the first boot
#define DEFAULT_NODE_ADDRESS      0x5         // 2 bytes
#define DEFAULT_NODE_NAME         "DUMMY001"  // 8-char name
#define DEFAULT_RADIO_TX_INTERVAL 0xFFFFFFFF  // 0xFFFFFFFF=no transmission, otherwise 4 bytes value in ms
#define DEFAULT_DUMMY1            0x12345678  // dummy, 4 bytes

// eeprom addresses for parameters
#define PARAM_BASE_EEPROM_ADDRESS 0x0
uint8_t paramSizes[PARAM_COUNT] = {2,8,4,4};
// the param eeprom addresses are then PARAM_BASE_EEPROM_ADDRESS, PARAM_BASE_EEPROM_ADDRESS+ paramSizes[0], ...

// parameter names go in flash, not in eeprom anymore
static const char PROGMEM paramNames[PARAM_COUNT][9] = {"Address","NodeName","TxIntval","Dummy1"};

#define CONFIG_INITCONFIG_EEPROM_ADDRESS          511 // last byte in eeprom, set 1 = valid eeprom contents, else invalid, not initialized


RF24 radio(7,8);                      // nRF24L01(+) radio attached using Getting Started board 
RF24Network network(radio);           // Network uses that radio

const uint16_t baseNodeAddress = 00;  // Address of the base node

// these parameters will be copied from eeprom to RAM for use at runtime
uint16_t myNodeAddress = DEFAULT_NODE_ADDRESS; // our address
uint32_t radioTxInterval = DEFAULT_RADIO_TX_INTERVAL; // periodic data transmission in ms

uint32_t radioTxLastMillis;
bool stayAwake = true; // sleeping on/off
uint32_t lastSleepMillis;

int solarPin = A0;

uint16_t solarValue = 0; 
uint16_t vccValue = 0;
uint16_t packetsOK = 0;
uint16_t packetsNOK = 0;

struct dataPayload_t {
  uint16_t solarValue;
  uint16_t vccValue;
  uint16_t packetsOK;
  uint16_t packetsNOK;
};

#define PAYLOAD_ITEM_COUNT  4
uint8_t payloadItemTypes[PAYLOAD_ITEM_COUNT] = {PAYLOAD_TYPE_UINT16,PAYLOAD_TYPE_UINT16,
                                                PAYLOAD_TYPE_UINT16,PAYLOAD_TYPE_UINT16};

// helpers
void readParameterValueFromEeprom(uint8_t paramId, uint8_t *paramValue) {
  uint8_t paramEepromAddress = PARAM_BASE_EEPROM_ADDRESS;
  for (int i=0; i<paramId;i++) paramEepromAddress += paramSizes[i];
  for (int i=0;i<paramSizes[paramId];i++) paramValue[i] = EEPROM.read(paramEepromAddress+i);
}// readParameterValueFromEeprom

void writeParameterValueToEeprom(uint8_t paramId, uint8_t *paramValue) {
  uint8_t paramEepromAddress = PARAM_BASE_EEPROM_ADDRESS;
  for (int i=0; i<paramId;i++) paramEepromAddress += paramSizes[i];
  for (int i=0;i<paramSizes[paramId];i++) EEPROM.update(paramEepromAddress+i,paramValue[i]);
}// writeParameterValueToEeprom

// load config settings from eeprom
void loadParametersFromEeprom() {
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
void initEeprom() {
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
  /*
  Serial.print(F("sending reply to:"));Serial.println(outHeader.to_node);
  Serial.print(F("id:"));Serial.println(outReply.id);
  Serial.print(F("param:"));Serial.println(outReply.param);
  Serial.print("data:");
  for (int i=0;i<8;i++) { Serial.print(outReply.data[i],HEX); Serial.print(" "); }
  Serial.println();
  */
  bool ok = network.write(outHeader,&outReply,sizeof(outReply));
  if (ok) packetsOK ++;
  else packetsNOK++;

} // handleRadioCommand

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

  initEeprom(); // initial boot settings
  loadParametersFromEeprom();
 
  SPI.begin();
  radio.begin();
  network.begin(/*channel*/ 90, /*node address*/ myNodeAddress);
  network.setup_watchdog(WDTO_1S); // configuration for sleep mode -> 1 cycle = 1second

} // setup

void loop() {

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

  // sending data to the hub
  if ((radioTxInterval != 0xFFFFFFFF) && ( millis() - radioTxLastMillis >= radioTxInterval ))
  {
    radioTxLastMillis = millis();
    // dummy sensor data
    solarValue = (uint16_t) analogRead(solarPin);
    vccValue = readVcc();

    dataPayload_t payload = { solarValue, vccValue, packetsOK, packetsNOK };
    RF24NetworkHeader header(baseNodeAddress,HEADER_DATA);
    bool ok = network.write(header,&payload,sizeof(payload));
    if (ok) packetsOK ++;
    else packetsNOK++;
  }

  // stay awake for another 1000 ms after sending data, zodat de hub nog een command kan sturen, bv STAY AWAKE
  if ((millis() - lastSleepMillis) > 1000) {
    if (!stayAwake) {
      lastSleepMillis = millis();
      radio.powerDown();
      network.sleepNode(3,255); // x cycles sleep, no wake-up from interrupt
      // and here we wake up; during sleep the millis stand still
      // so the radioTxInterval doesn't count the sleep time!
      radio.powerUp();
    }
  }
} // loop

// trigger reboot from SW
void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
} // reboot

uint16_t readVcc() {
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
