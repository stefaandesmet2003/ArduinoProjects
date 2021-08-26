// a 16 button keypad to control DCC accessories
// a connected 16x WS2812 addressable led strip indicates the accessory status
// off = unknown, red = thrown, green = closed
// control & status happens over xpnet
// TODO : toggle signals?

// help vscode
/*
#undef __AVR__
#define ARDUINO_ARCH_STM8
#define STM8S105
*/
#if defined(__AVR__)
  #include <Keypad.h>
  #include <Adafruit_NeoPixel.h>

  #define WS2812LED_PIN         13
  #define RS485_DIRECTION_PIN   2 // xpnet rs485

#elif defined(ARDUINO_ARCH_STM8)
  #include <Keypad_stm8.h>
  #include "NeoPixel.h"

  #if defined(STM8S103)
    #define WS2812LED_PIN     (PA3)
  #elif defined (STM8S105) // for testing
    #define WS2812LED_PIN     (PD7)
  #endif

  #define RS485_DIRECTION_PIN PD4 // xpnet rs485

#endif

#include "rs485c.h"
#include "xpc.h"

#define ROWS          4
#define COLS          4
#define NUMPIXELS     16
#define XPNET_ADDRESS 7

/*
char keys[ROWS][COLS] = {
    {'1','2','3','4'},
    {'5','6','7','8'},
    {'9','A','B','C'},
    {'D','E','F','0'}
};
*/
// keys can't start from 0, because 0 = NO_KEY
const char keys[ROWS][COLS] = {
  {1 , 2 , 3 , 4},
  {5 , 6 , 7 , 8},
  {9 , 10, 11, 12},
  {13, 14, 15, 16}
};

// a local copy of turnout positions (1byte/decoder in xpnet accessory format)
// DECODERBUFFER_SIZE consecutive decoder addresses starting at firstDecoderAddress
// led strip can show 4 consecutive addresses so DECODERBUFFER_SIZE = 4 is sufficient
#define DECODERBUFFER_SIZE          8 // keep multiple of 2, or this rubbish code won't work properly
#define DECODERBUFFER_REFRESH_DELAY 30000
#define MODE_TURNOUTS         0   // leds show turnout status, keys modify turnouts
#define MODE_FEEDBACK_INPUTS  1   // leds show feedback inputs, keys no effect

typedef struct {
  uint8_t firstDecoderAddress; // 8-bit in xpnet, decoderBuffer stores 4 successive decoder addresses' data
  uint8_t data[DECODERBUFFER_SIZE]; // keep state for 16 accessories : 0=unknown, 1=green,2=red, we could do with 2 bits/turnout, but don't care for now
  uint32_t lastRefreshMillis;
  uint8_t mode;
  bool dataChanged;
} decoderBuffer_t;

// organizing regular xpnet requests to command station
#define XPC_REQUESTS_RETRY_DELAY  500
// list of requests that throttle will systematically send to CS
typedef enum {
  REQ_IDLE, /*REQ_CS_STATUS, REQ_LOC_INFO,*/ REQ_ACC_INFO
} xpcRequestState_t;

typedef struct {
  xpcRequestState_t state;
  uint32_t reqLastMillis;
  uint32_t respLastMillis;
  uint8_t idx; // multi-use
} xpcRequests_t;

static decoderBuffer_t decoderBuffer;
static xpcRequests_t xpcRequests;

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR LOCAL TURNOUT BUFFER                              */
/*****************************************************************************/
static uint8_t turnout_GetStatus (uint16_t turnoutAddress) {
  uint8_t decoderAddress;
  uint8_t turnoutState;
  uint8_t bitPos;

  decoderAddress = turnoutAddress >> 2;
  if ((decoderAddress < decoderBuffer.firstDecoderAddress) ||
      (decoderAddress >= (decoderBuffer.firstDecoderAddress + DECODERBUFFER_SIZE))) {
    // don't have data in the decoderBuffer to show
    return TURNOUT_STATE_INVALID;
  }
  bitPos = (turnoutAddress & 0x3) << 1;
  turnoutState = decoderBuffer.data[decoderAddress - decoderBuffer.firstDecoderAddress]; // 4 turnouts
  turnoutState = (turnoutState >> bitPos) & 0x3; // i & 0x3 : turnoutId 0,1,2,3 in the decoderAddress
  return turnoutState;
} // turnout_GetStatus

// update turnoutbuffer with this status
// return true if status has changed
static bool turnout_SetStatus (uint16_t turnoutAddress, uint8_t turnoutStatus) {
  uint8_t decoderAddress;
  uint8_t bitPos, mask;
  uint8_t curData, newData;
  bool retval = false;
  decoderAddress = turnoutAddress >> 2;
  // turnoutAddress not in decoderBuffer -> shouldn't happen
  if ((decoderAddress < decoderBuffer.firstDecoderAddress) ||
      (decoderAddress >= (decoderBuffer.firstDecoderAddress + DECODERBUFFER_SIZE))) {
    return false;
  }

  bitPos = (turnoutAddress & 0x3) << 1;
  mask = ~(0x3 << bitPos);
  curData = decoderBuffer.data[decoderAddress - decoderBuffer.firstDecoderAddress]; // current byte
  newData = (curData & mask) | (turnoutStatus << bitPos);
  if (newData != curData) retval = true;
  decoderBuffer.data[decoderAddress - decoderBuffer.firstDecoderAddress] = newData;

  return retval;
} //turnout_SetStatus

// clear data in turnout buffer
static void turnout_ClearBuffer () {
  for (uint8_t i=0;i<DECODERBUFFER_SIZE;i++)
    decoderBuffer.data[i] = 0x00;
} // turnout_ResetBuffer

static void turnout_TriggerBufferRefresh() {
  decoderBuffer.lastRefreshMillis = millis() - DECODERBUFFER_REFRESH_DELAY;
} // turnout_TriggerBufferRefresh


#if defined(ARDUINO_ARCH_STM8)
  byte rowPins[ROWS] = {PC6,PC5,PC4,PC3}; //connect to the row pinouts of the keypad
  byte colPins[COLS] = {PC7,PD1,PD2,PD3}; //connect to the column pinouts of the keypad

void keypadEventHandler(KeypadEvent key){
  (void) key;
  switch (Keypad_getState()) {
    case PRESSED:
      break;
    case RELEASED:
      break;
    case HOLD:
      if (key == 16) { // switch mode
        decoderBuffer.mode = !decoderBuffer.mode;
        decoderBuffer.dataChanged = true;
        NeoPixel_clear(); // TODO, for now just clear the leds
        // TODO : trigger a series of requests from command station
      }
      break;
  }
} // keypadEventHandler

void setup() {
  Keypad_init (makeKeymap(keys), rowPins, colPins, ROWS, COLS);
  Keypad_addEventListener(keypadEventHandler); // Add an event listener for this keypad

  NeoPixel_init (NUMPIXELS, WS2812LED_PIN, NEO_GRB + NEO_KHZ800);
  NeoPixel_begin();
  NeoPixel_clear();
  NeoPixel_show();

  rs485_Init(RS485_DIRECTION_PIN);
  xpc_Init(XPNET_ADDRESS);

  xpcRequests.state = REQ_IDLE;
  decoderBuffer.firstDecoderAddress = 0; // show decoder addresses 0..(DECODERBUFFERSIZE-1)
  turnout_TriggerBufferRefresh();

} // setup

void loop() {
  char key;

  // handle keypad
  key = Keypad_getKey();
  if (decoderBuffer.mode == MODE_TURNOUTS) {
    uint16_t turnoutAddress;
    uint8_t turnoutPosition = 0; // 0/1  = coil to be activated
    uint8_t turnoutStatus;

    if (key) {
      turnoutAddress = (decoderBuffer.firstDecoderAddress<<2) + key - 1;
      turnoutStatus = turnout_GetStatus(turnoutAddress);
      if (turnoutStatus == TURNOUT_STATE_CLOSED) {
        turnoutStatus = TURNOUT_STATE_THROWN;
        turnoutPosition = 1;
      }
      else // THROWN, UNKNOWN, INVALID -> CLOSED
        turnoutStatus = TURNOUT_STATE_CLOSED;
      turnout_SetStatus(turnoutAddress, turnoutStatus); // update local turnout buffer
      xpc_send_SetTurnoutRequest(turnoutAddress, turnoutPosition); // send command to CS
      decoderBuffer.dataChanged = true; // trigger led strip update
    }

  }
  else {
    // geen keys af te handelen hier, de HOLD switcht mode in de key event handler
  }

  // run the expressnet
  xpc_Run();

  // xpnet data refresh/polling in a simple fixed queue fashion
  switch (xpcRequests.state) {
    case REQ_IDLE:
      if ((millis() - decoderBuffer.lastRefreshMillis) > DECODERBUFFER_REFRESH_DELAY) {
        xpcRequests.state = REQ_ACC_INFO;
        xpcRequests.reqLastMillis = millis();
        xpcRequests.idx = 0;
        xpc_send_AccessoryDecoderInfoRequest(decoderBuffer.firstDecoderAddress, 0);
      }
      break;
    case REQ_ACC_INFO : 
      if (xpcRequests.respLastMillis > xpcRequests.reqLastMillis) { // response received
        xpcRequests.idx+= 1; 
        // we need 2 requests per decoder address (idx & 0x1) : even/odd nibble
        if ((xpcRequests.idx>>1) < DECODERBUFFER_SIZE) {
          xpcRequests.reqLastMillis = millis();
          xpc_send_AccessoryDecoderInfoRequest(decoderBuffer.firstDecoderAddress +(xpcRequests.idx>>1), xpcRequests.idx & 0x1);
        }
        else { // all done
          xpcRequests.state = REQ_IDLE;
        }
      }
      else if ((millis() - xpcRequests.reqLastMillis) > XPC_REQUESTS_RETRY_DELAY) { // need a retry
        xpcRequests.reqLastMillis = millis();
        xpc_send_AccessoryDecoderInfoRequest(decoderBuffer.firstDecoderAddress +(xpcRequests.idx>>1), xpcRequests.idx & 0x1);
      }
      break;
  }

  // led strip update
  if (decoderBuffer.dataChanged) {
    if (decoderBuffer.mode == MODE_TURNOUTS) { // rood/groen
      uint8_t turnoutStatus;
      uint16_t turnoutAddress;
      for (uint8_t i=0;i< NUMPIXELS;i++) {
        turnoutAddress = (decoderBuffer.firstDecoderAddress<<2) + i;
        turnoutStatus = turnout_GetStatus(turnoutAddress);
        if (turnoutStatus == TURNOUT_STATE_CLOSED)
          NeoPixel_setPixelColor(i,0,10,0); // green
        else if (turnoutStatus == TURNOUT_STATE_THROWN)
          NeoPixel_setPixelColor(i,10,0, 0); // red
      }
    }
    else { // TODO
      // for now just set all leds white
      for (uint8_t i=0;i< NUMPIXELS;i++) {
        NeoPixel_setPixelColor(i,10,10,10);
      }
    }
    NeoPixel_show();
    decoderBuffer.dataChanged = false;
  }

} // loop

#elif defined(__AVR__)
  byte rowPins[ROWS] = {6, 5, 4, 3}; //connect to the row pinouts of the keypad
  byte colPins[COLS] = {7, 8, 9, 10}; //connect to the column pinouts of the keypad

  Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
  Adafruit_NeoPixel pixels(NUMPIXELS, WS2812LED_PIN, NEO_GRB + NEO_KHZ800);

void keypadEventHandler(KeypadEvent key){
  switch (keypad.getState()){
    case PRESSED:
      break;
    case RELEASED:
      break;
    case HOLD:
      break;
  }
} // keypadEventHandler

void setup() {
  Serial.begin(115200);
  Serial.println("START!");
  keypad.addEventListener(keypadEventHandler); // Add an event listener for this keypad
  
  pixels.begin();
  pixels.clear();
  pixels.show();
  init_rs485(RS485_DIRECTION_PIN);
  xpc_Init(XPNET_ADDRESS);
}

void loop() {
  char key;

  // handle keypad  
  key = keypad.getKey();
  if (key) {
    // toggle accessory acc[key-1]
    // toggle led key-1
    if (decoderBuffer.data[key-1] == 1) { // green -> red
      decoderBuffer.data[key-1] = 2;
      pixels.setPixelColor(key-1,pixels.Color(10,0,0));
    }
    else { // red/white -> green
      decoderBuffer.data[key-1] = 1;
      pixels.setPixelColor(key-1,pixels.Color(0,10,0));
    }
    pixels.show();
    Serial.println(key);
  }

  // run the expressnet
  xpc_Run();

} // loop

#endif

/*****************************************************************************/
/*  XPNET NOTIFY FUNCTIONS                                                   */
/*****************************************************************************/
void xpc_FastClockNotify (xpcFastClock_t *newFastClock) {(void)newFastClock;}

void xpc_MessageNotify (uint8_t *msg) {
  (void) msg;
} // xpc_MessageNotify

void xpc_EventNotify (xpcEvent_t xpcEvent) {
  (void) xpcEvent;
} // xpc_EventNotify

// here arrives all info concerning turnout changes & feedback decoder input changes
// turnouts : only storing the 8 turnouts shown on display
// if curStartTurnout changes, we request the data for these turnouts with xpc_send_AccessoryDecoderInfoRequest
void xpc_AccessoryDecoderInfoResponse (uint8_t decoderAddress, uint8_t decoderBits, uint8_t decoderBitsMask, uint8_t decoderType) {
  if (decoderType == DECODERTYPE_FEEDBACK_DECODER)
    return; // TEMP, not interested for now
  
  xpcRequests.respLastMillis = millis(); // the callback could also be a broadcast, but OK for now (we could check if the decoderAddress corresponds with the last request..)
  decoderBuffer.lastRefreshMillis = millis();
  
  // so we have info about a turnout decoder
  // for turnouts we store the 2bits/turnout in decoderBuffer = 1 byte/accessory decoder 
  // if decoderAddress is in the range defined by decoderBuffer.startAddress + DECODERBUFFER_SIZE
  if ((decoderAddress >= decoderBuffer.firstDecoderAddress) && 
      (decoderAddress < (decoderBuffer.firstDecoderAddress + DECODERBUFFER_SIZE))) {
    uint8_t curTurnoutBufferData, newTurnoutBufferData;
    curTurnoutBufferData = decoderBuffer.data[decoderAddress - decoderBuffer.firstDecoderAddress];
    newTurnoutBufferData = (curTurnoutBufferData & ~decoderBitsMask) | decoderBits;

    // trigger a led strip update
    if (newTurnoutBufferData != curTurnoutBufferData) {  // data changed
      decoderBuffer.dataChanged = true; // set flag to trigger display update
    }
    decoderBuffer.data[decoderAddress - decoderBuffer.firstDecoderAddress] = newTurnoutBufferData;
  }
} // xpc_AccessoryDecoderInfoResponse




