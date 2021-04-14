
/* 
 * accessoryBuffer[] : 8 bits per accessory adres
 *  -> 8 outputs if accessory  = turnout decoder, light decoder etc
 *  -> extended accessories (signal decoders) are not stored, must preferably use upper accessory address range
 *  -> 8 inputs if accessory = feedback decoder
 *  -> both use a combined address space
 *  -> different from opendcc, here we store 2 bits per turnout (-), but we can also store light outputs (+)
 * turnoutFeedbackBuffer[] : only storing the feedback bits for a feedback decoder that works
 * in conjunction with a turnout decoder to provide feedback for its outputs
 * these feedback decoders have the same accessory address as the turnout decoder they're feeding back
 *  
 */

#include "Arduino.h"
//#include "accessories.h"

#define NUM_ACCESSORY_ADDRESSES       32 // = accessory decoder addresses 0..31
//#define NUM_TURNOUTS_WITH_FEEDBACK    16
#define NUM_TURNOUTS_WITH_FEEDBACK    16
#define NUM_TURNOUTFEEDBACK_ADDRESSES (NUM_TURNOUTS_WITH_FEEDBACK >> 2)
// NUM_TURNOUTS_WITH_FEEDBACK moet < NUM_ACCESSORY_ADDRESSES*8, & multiple van 4

/* dus NUM_TURNOUTS_WITH_FEEDBACK/4 accessory decoders 
 * zijn gelinkt met feedback decoders; de feedback decoders feedback over de positie van de overeenkomstige wissel
 * mapt naar NUM_TURNOUTS_WITH_FEEDBACK/4 bytes in turnoutFeedbackBuffer
 */

static uint8_t accessoryBuffer[NUM_ACCESSORY_ADDRESSES]; // for inputs (feedback) and outputs (turnouts/lights), 1 bit per I/O
static uint8_t turnoutFeedbackBuffer[NUM_TURNOUTFEEDBACK_ADDRESSES]; // store feedback bits from feedbacked turnouts separately, 4 turnouts/address
// need this for the TT bits
static uint8_t feedbackDecoderAddresses[NUM_ACCESSORY_ADDRESSES>>3]; // true/false bit per accessory address

/*****************************************************************************/
/*    HELPERS                                                                */
/*****************************************************************************/

// returns the 2 bits for the turnout from the accessoryBuffer, 
// or 0 if turnout not stored (=ZZ format)
static uint8_t getTurnoutCommandPosition(uint16_t turnoutAddress) {
  uint16_t bufferIdx;
  uint8_t bitPos, retval;
  bufferIdx = turnoutAddress >> 2; // 4 wissels per byte in accessoryBuffer
  bitPos = ((uint8_t)(turnoutAddress) & 0x3) << 1; // turnout bits position in the byte
  if (bufferIdx < NUM_ACCESSORY_ADDRESSES)
    retval = (accessoryBuffer[bufferIdx] >> bitPos) & 0x3;
  else retval = 0; // position not stored, we don't know
  return retval;
} // getTurnoutCommandPosition

// returns the 2 bits from the turnoutFeedbackBuffer, or 0 if turnout not feedbacked
static uint8_t getTurnoutFeedbackPosition(uint16_t turnoutAddress) {
  uint16_t bufferIdx;
  uint8_t bitPos, retval;

  if (turnoutAddress < NUM_TURNOUTS_WITH_FEEDBACK) {
    bufferIdx = turnoutAddress >> 2; // 4 wissels per byte in turnoutFeedbackBuffer
    bitPos = ((uint8_t)(turnoutAddress) & 0x3) << 1; // turnout bits position in the byte
    retval = (turnoutFeedbackBuffer[bufferIdx] >> bitPos) & 0x3;
  }
  else retval = 0; // no feedback info for this turnout
  return retval;
} // getTurnoutFeedbackPosition

static bool isFeedbackDecoderAddress(uint8_t decoderAddress) {
  uint8_t bitPos, bitVal;
  bitPos = decoderAddress & 0x7;
  bitVal = (feedbackDecoderAddresses[decoderAddress>>3] >> bitPos) & 0x1;
  return (bitVal!=0);
} // isFeedbackDecoderAddress

/*****************************************************************************/
/*    PUBLIC FUNCTIONS                                                       */
/*****************************************************************************/

// store de data die van de sds feedback decoders komt
// mijn eigen feedback decoders geven 8 bits tegelijk
// dus niet in ITTNZZZZ formaat !!
// return : TRUE = data changed, so xpnet knows it needs to broadcast the changes
uint8_t feedback_update(uint8_t decoderAddress, uint8_t data) {
  bool dataChanged = false;
  uint8_t bitPos = decoderAddress & 0x7;
  uint8_t oldData;
  feedbackDecoderAddresses[decoderAddress>>3] |= (1 << bitPos); // mark this address as a feedback decoder address
  // TODO : maak onderscheid ts de feedbacked turnouts en niet
  if (decoderAddress < NUM_TURNOUTFEEDBACK_ADDRESSES){
    oldData = turnoutFeedbackBuffer[decoderAddress];
    turnoutFeedbackBuffer[decoderAddress] = data; // feedback from turnouts -> store separately for later
  }
  else {
    oldData = accessoryBuffer[decoderAddress];
    accessoryBuffer[decoderAddress] = data;
  }
  /* 
   * hier nog een *msg vullen met de wijzigingen om te broadcasten?
   * of laten we de caller 2x turnout_getInfo oproepen?
   */
  return oldData;
} // feedback_update

// vervangt save_turnout uit organizer
// 2 bits per wissel in accessoryBuffer
// even bit = coil 0 laatst actief, oneven bit = coil 1 laatst actief
void turnout_update(uint16_t turnoutAddress, uint8_t coil) {
  uint16_t bufferIdx;
  uint8_t bitPos, bitsVal;

  bufferIdx = turnoutAddress >> 2; // 4 wissels per byte in accessoryBuffer
  bitPos = ((uint8_t) (turnoutAddress) & 0x3) << 1; // even bit position for the 2 turnout bits
  bitsVal = (0x1 << coil); // the 2 turnout bits, 0b01 for coil 0, 0b10 for coil 1
  if (bufferIdx < NUM_ACCESSORY_ADDRESSES) { // overwrite the 2 bits in the buffer
    accessoryBuffer[bufferIdx] = (accessoryBuffer[bufferIdx] & ~(0x3 << bitPos)) | (bitsVal << bitPos);
  }
} // turnout_update

// fills *msg with decoderAddress + ITTNZZZZ xpnet data (2bytes)
// ofwel op decoderAddress niveau en 4 bytes sturen?
// -> da werkt niet voor een accessory info request
// msg = ptr naar header byte vd message, dus data byte 1 = msg[1]
// vervangt create_xpressnet_schaltinfo
void accessory_getInfo (uint8_t decoderAddress, uint8_t nibble, uint8_t *msg) {
  /*
   * if turnoutAddress < NUM_TURNOUTS_WITH_FEEDBACK
   * - TT=01
   * vgl turnoutBuffer(=cmd) met turnoutFeedbackBuffer
   * cmd<>feedback -> I=1, ZZ uit turnoutFeedbackBuffer
   * cmd==feedback -> I=0, ZZ uit turnoutFeedbackBuffer
   * 
   * else (turnoutAddress < NUM_TURNOUTS_WITH_FEEDBACK)
   * - TT=00 (turnout zonder feedback)
   * - I=0 altijd
   * - ZZZZ uit turnoutbuffer, want feedbackbuffer zegt '00'
   * idem met het gepairde turnoutAddress
   */
  uint16_t turnoutAddress;
  uint8_t turnoutCommandPosition[2];
  uint8_t turnoutFeedbackPosition[2];

  turnoutAddress = ((uint16_t)decoderAddress << 2) + ((nibble & 0x1) << 1);
  // we need an even & odd turnout address for the reply
  turnoutCommandPosition[0] = getTurnoutCommandPosition(turnoutAddress & 0xFFFE);
  turnoutFeedbackPosition[0] = getTurnoutFeedbackPosition(turnoutAddress & 0xFFFE);
  turnoutCommandPosition[1] = getTurnoutCommandPosition(turnoutAddress | 0x0001);
  turnoutFeedbackPosition[1] = getTurnoutFeedbackPosition(turnoutAddress | 0x0001);

  *msg++ = decoderAddress;
  if (turnoutAddress < NUM_TURNOUTS_WITH_FEEDBACK) { // turnout with feedback
    // ITTN ZZZZ, TT=01
    if (nibble == 0)  *msg = 0b00100000;
    else              *msg = 0b00110000;
    // the ZZZZ bits
    *msg = *msg + turnoutFeedbackPosition[0];
    *msg = *msg + (turnoutFeedbackPosition[1] << 2);
    // the I-bit
    if ((turnoutCommandPosition[0] != turnoutFeedbackPosition[0]) 
       || (turnoutCommandPosition[1] != turnoutFeedbackPosition[1])) {
         *msg |= 0x80; // set the I-bit
    }
  }
  else if (isFeedbackDecoderAddress(decoderAddress)) { // feedback decoder
    // ITTN ZZZZ, TT=10,I=0
    if (nibble == 0) { // lower nibble
      *msg = 0b01000000;
      *msg = *msg + (accessoryBuffer[decoderAddress] & 0xF);
    } 
    else { // upper nibble
      *msg = 0b01010000;
      *msg = *msg + ((accessoryBuffer[decoderAddress] >> 4) & 0xF);
    }
  }
  else { // turnout without feedback
    // ITTN ZZZZ, TT=00, I=0
    if (nibble == 0)  *msg = 0b00000000;
    else              *msg = 0b00010000;
    // the ZZZZ bits
    *msg = *msg + turnoutCommandPosition[0];
    *msg = *msg + (turnoutCommandPosition[1] << 2);
  }
} // accessory_getInfo

// is hetzelfde als accessory_getInfo
void turnout_getInfo (uint16_t turnoutAddress, unsigned char *msg) {
  unsigned char decoderAddress, nibble;

  decoderAddress = (unsigned char) (turnoutAddress>>2);
  nibble = (unsigned char) turnoutAddress & 0x2;
  accessory_getInfo(decoderAddress,nibble,msg);
} // turnout_getInfo



/***************************************************************************************************/
static void printBuffers() {
  Serial.println("accessoryBuffer:");
  for (int i=0;i<NUM_ACCESSORY_ADDRESSES;i++) {
    if (i%10 == 0) {
      Serial.print(i); Serial.print(" :");
    }
    Serial.print(accessoryBuffer[i],HEX);Serial.print(" ");
    if (i%10 == 9) {
      Serial.println();
    }
  }
  Serial.println(); Serial.println("turnoutFeedbackBuffer:");
  for (int i=0;i<NUM_TURNOUTFEEDBACK_ADDRESSES;i++) {
    if (i%10 == 0) {
      Serial.print(i); Serial.print(" :");
    }
    Serial.print(turnoutFeedbackBuffer[i],HEX);Serial.print(" ");
    if (i%10 == 9) {
      Serial.println();
    }
  }
  Serial.println();
  Serial.println("***************************");
  Serial.println();
} // printBuffers

static void printMsg(uint8_t *msg) {
  Serial.print("msg : ");
  for (int i=0;i<10;i++) {
    Serial.print(msg[i],HEX);Serial.print(' ');
  }
  Serial.println();
} // printMsg

void setup() {
  // put your setup code here, to run once:
  uint8_t msg[10];
  uint8_t oldData;
  Serial.begin(115200);
  Serial.println("start!");
  Serial.println("buffers at startup:");
  printBuffers();
  accessory_getInfo(0,0,msg); // decoderAddress,nibble,msg
  printMsg(msg);
  Serial.println("turnout_update(0,0)");
  turnout_update(0,0); // wissel, coil
  printBuffers();
  accessory_getInfo(0,0,msg); // decoderAddress,nibble,msg
  printMsg(msg);
  
  Serial.println("feedback_update(0,0x1) -> feedback voor de verzette wissel");
  oldData = feedback_update(0,0x1); // decoderAddress, data
  Serial.print("feedback_update, old data = ");Serial.println(oldData);
  oldData = feedback_update(0,0x1); // decoderAddress, data
  Serial.print("feedback_update, old data = ");Serial.println(oldData);
  Serial.println("check buffer contents");
  printBuffers();
  Serial.println("check if feedbacked turnout reports OK now:");
  accessory_getInfo(0,0,msg); // decoderAddress,nibble,msg
  printMsg(msg);

  /*
  turnout_update(0,1);
  turnout_update(1,1);
  turnout_update(3,1);
  printBuffers();
  accessory_getInfo(0,0,msg); // decoderAddress,nibble,msg
  printMsg(msg);
  accessory_getInfo(0,1,msg); // decoderAddress,nibble,msg
  printMsg(msg);

  Serial.println("turnout_update(4,1)");

  turnout_update(4,1);
  printBuffers();
  accessory_getInfo(1,0,msg); // decoderAddress,nibble,msg
  printMsg(msg);
  accessory_getInfo(1,1,msg); // decoderAddress,nibble,msg
  printMsg(msg);
  */
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
