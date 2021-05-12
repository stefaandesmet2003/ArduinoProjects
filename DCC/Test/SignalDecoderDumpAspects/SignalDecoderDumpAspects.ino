
#include <EEPROM.h>
// CV(eeprom) configuration
#define INDEX_TABLE_START_ADDRESS       33
#define INDEX_TABLE_SIZE                40
#define ASPECT_TABLE_START_ADDRESS (INDEX_TABLE_START_ADDRESS + INDEX_TABLE_SIZE) // 65
#define ASPECT_BLOCK_SIZE               10

#define NUM_HEADS             1 // for test
#define NUM_OUTPUTS_PER_HEAD  5   // max 5 supported by CV configuration, but we can use less to save RAM
#define NUM_OUTPUTS           19
#define PIN_UNUSED            0xFF

typedef struct {
  uint8_t pin;
  uint8_t ledOn;
  uint8_t ledOff;
  uint8_t fOnCycle;
  uint8_t fOffCycle;
  uint8_t fTotalCycles;
  uint8_t fCycle; //0->15
} output_t;

typedef struct {
  output_t outputs[NUM_OUTPUTS_PER_HEAD]; // save mem, max [3] outputs active per heads[]
  uint8_t ms;
  uint8_t bCycle; // 0->9
} head_t;
volatile head_t heads[NUM_HEADS]; // a decoder supports up to 4 signal heads

// convert a brightness value in the head definition CVs to a ledOn/ledOff value (0..10)
uint8_t brightnessLUT[8][2] = {{0,10},{2,10},{4,10},{6,10},{8,10},{0,3},{0,2},{0,1}};



void printHead(uint8_t head) {
  Serial_print_s("head : "); Serial_println_u(head);
  for (int i=0;i<NUM_OUTPUTS_PER_HEAD;i++) {
    Serial_print_s("output[");Serial_print_u(i);Serial_print_s("].pin = ");Serial_println_u(heads[head].outputs[i].pin);
    Serial_print_s("output[");Serial_print_u(i);Serial_print_s("].ledOn = ");Serial_println_u(heads[head].outputs[i].ledOn);
    Serial_print_s("output[");Serial_print_u(i);Serial_print_s("].ledOff = ");Serial_println_u(heads[head].outputs[i].ledOff);
    Serial_print_s("output[");Serial_print_u(i);Serial_print_s("].fOnCycle = ");Serial_println_u(heads[head].outputs[i].fOnCycle);
    Serial_print_s("output[");Serial_print_u(i);Serial_print_s("].fOffCycle = ");Serial_println_u(heads[head].outputs[i].fOffCycle);
    Serial_print_s("output[");Serial_print_u(i);Serial_print_s("].fTotalCycles = ");Serial_println_u(heads[head].outputs[i].fTotalCycles);
  }
}

void setup() {
  Serial_begin(115200);
  Serial_println_s("dump all aspects in signal decoder EEPROM");

  uint8_t idx;
  uint16_t cvAddress;
  uint8_t cvValue, cvHead, cvAspect;
  uint8_t outputPin;
  uint8_t brightness;
  uint8_t blinkTimeOn, blinkTimeOff, blinkInverse;
  uint8_t idxOutput;

  for (int idx=0;idx < INDEX_TABLE_SIZE;idx++) {
    Serial_print_s("idx:");Serial_println_u(idx);
    cvValue = EEPROM_read(INDEX_TABLE_START_ADDRESS + idx);
    if (cvValue & 0x80) { // line in index table is in use
      cvHead = (cvValue >> 5) & 0x3;
      cvAspect = cvValue & 0x1F;
      Serial_print_s("head:");Serial_println_u(cvHead);
      Serial_print_s("aspect:");Serial_println_u(cvAspect);
    }
    else {
      Serial_println_s("geen active head op deze index");
      continue;
    }
    // lamp definitions voor deze index 
    cvAddress = ASPECT_TABLE_START_ADDRESS + ASPECT_BLOCK_SIZE*idx;
    idxOutput = 0;
    for (uint8_t i=0;i<NUM_OUTPUTS_PER_HEAD;i++) {
      heads[0].outputs[i].pin = PIN_UNUSED;
    }
    for (uint8_t i=0;i<NUM_OUTPUTS_PER_HEAD;i++) {
      cvValue = EEPROM_read(cvAddress++); // output definition byte
      outputPin = cvValue & 0x1F;
      brightness = (cvValue >> 5) & 0x7;
      cvValue = EEPROM_read(cvAddress++); // blinking definition byte
      blinkTimeOn = cvValue & 0x7;
      blinkTimeOff = (cvValue >> 3) & 0x7;
      blinkInverse = cvValue & 0x40; //bit6
      if (outputPin) {
        heads[0].outputs[idxOutput].pin = outputPin - 1;
        heads[0].outputs[idxOutput].ledOn = brightnessLUT[brightness][0];
        heads[0].outputs[idxOutput].ledOff = brightnessLUT[brightness][1];
        heads[0].outputs[idxOutput].fCycle = 0;
        heads[0].outputs[idxOutput].fOnCycle = 0;
        heads[0].outputs[idxOutput].fOffCycle = blinkTimeOn;
        heads[0].outputs[idxOutput].fTotalCycles = blinkTimeOn + blinkTimeOff;
        if (blinkInverse) {
          heads[0].outputs[idxOutput].fOnCycle+= blinkTimeOff;
          heads[0].outputs[idxOutput].fOffCycle+= blinkTimeOff; 
        }
        idxOutput++;
      }
    }
    printHead(0);
    Serial_println_s("************************");

  }

}

void loop () {
}
