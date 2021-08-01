//------------------------------------------------------------------------
//
// OpenDCC - NmraDcc.cpp
//
// Copyright (c) 2011 Alex Shepherd
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			  and build DCC Mobile and Stationary Decoders
//------------------------------------------------------------------------
#include "NmraDcc.h"
#include <avr/eeprom.h>

/* handling of standard CV variables done here instead of user code (CV29, addresses vs POM etc)
 * CV27 (autostop) : moet altijd 0 voor MOB
 * CV28 (bidir) : altijd 0
 * 
 * CV29 defaults:
 * bit0=0 : loco direction, allow writing (TODO), MOB only
 * bit1=1 : 0=light function,1=speed bit -> always speed bit, don't do DCC14, MOB only
 * bit2=0 : NMRA only, MOB only
 * bit3=0 : bidir off
 * bit4=0 : speed table (unused), MOB only
 * bit5  : MOB : extended addressing : updated by lib based on decoder address (writeDccAddress)
 * bit5  : ACC : extended accessory (signal decoder), allow writing
 * bit6 : used for ACC decoder
 * bit7 : 0=mobile decoder, 1=accessory decoder : updated by lib
 * 
 * CV31+32 : indexed CV : TODO check if supported, if not disallow writing
 */

#define CV29_ACCESSORY_DECODER 0x80   // CV 29/541 bit 7
#define CV29_EXTENDED_ADDRESSING 0x20 // CV 29/541 bit 5

#define CV28_DEFAULT 0
#ifdef NMRA_DCC_MOBILE_DECODER
#define CV27_DEFAULT 0
#define CV29_DEFAULT 0x2
#endif
#ifdef NMRA_DCC_ACCESSORY_DECODER
#define CV29_DEFAULT CV29_ACCESSORY_DECODER // basic accessory
#endif

//------------------------------------------------------------------------
// DCC Receive Routine
//
// Howto:    uses two interrupts: a rising edge in DCC polarity triggers INTx
//           in INTx handler, Timer0 CompareB with a delay of 80us is started.
//           On Timer0 CompareB Match the level of DCC is evaluated and
//           parsed.
//
//                           |<-----116us----->|
//
//           DCC 1: _________XXXXXXXXX_________XXXXXXXXX_________
//                           ^-INTx
//                           |----87us--->|
//                                        ^Timer-INT: reads zero
//
//           DCC 0: _________XXXXXXXXXXXXXXXXXX__________________
//                           ^-INTx
//                           |----------->|
//                                        ^Timer-INT: reads one
//
//------------------------------------------------------------------------

// The Timer0 prescaler is hard-coded in wiring.c
#define TIMER_PRESCALER 64

// We will use a time period of 80us and not 87us as this gives us a bit more time to do other stuff
#define DCC_BIT_SAMPLE_PERIOD (F_CPU * 70L / TIMER_PRESCALER / 1000000L)

#if (DCC_BIT_SAMPLE_PERIOD > 254)
#error DCC_BIT_SAMPLE_PERIOD too big, use either larger prescaler or slower processor
#endif
#if (DCC_BIT_SAMPLE_PERIOD < 8)
#error DCC_BIT_SAMPLE_PERIOD too small, use either smaller prescaler or faster processor
#endif

typedef enum
{
  WAIT_PREAMBLE = 0,
  WAIT_START_BIT,
  WAIT_DATA,
  WAIT_END_BIT
} DccRxWaitState;

typedef struct
{
  DccRxWaitState State;
  uint8_t DataReady;
  uint8_t BitCount;
  uint8_t TempByte;
  DCC_MSG PacketBuf;
  DCC_MSG PacketCopy;
} DccRx_t;

typedef struct
{
  bool inServiceMode;
  long LastServiceModeMillis;
  uint8_t PageRegister; // Used for Paged Operations in Service Mode Programming
  uint8_t DuplicateCount;
  DCC_MSG LastMsg;
  uint8_t ExtIntNum;
  uint8_t ExtIntPinNum;
} DCC_PROCESSOR_STATE;

static DccRx_t DccRx;
static DCC_PROCESSOR_STATE DccProcState;
static uint16_t dccFilterAddress = 0; // 0 = no address filtering, all packets are callbacked

// keep track of fast clock
static dccFastClock_t dccFastClock;

void ExternalInterruptHandler()
{
  OCR0B = TCNT0 + DCC_BIT_SAMPLE_PERIOD;

#if defined(TIMSK0)
  TIMSK0 |= (1 << OCIE0B); // Enable Timer0 Compare Match B Interrupt
  TIFR0 |= (1 << OCF0B);   // Clear  Timer0 Compare Match B Flag
#elif defined(TIMSK)
  TIMSK |= (1 << OCIE0B); // Enable Timer0 Compare Match B Interrupt
  TIFR |= (1 << OCF0B);   // Clear  Timer0 Compare Match B Flag
#endif
} // ExternalInterruptHandler

ISR(TIMER0_COMPB_vect)
{
  uint8_t DccBitVal;

  // Read the DCC input value, if it's low then its a 1 bit, otherwise it is a 0 bit
  DccBitVal = !digitalRead(DccProcState.ExtIntPinNum);

  // Disable Timer0 Compare Match B Interrupt
#if defined(TIMSK0)
  TIMSK0 &= ~(1 << OCIE0B);
#elif defined(TIMSK)
  TIMSK &= ~(1 << OCIE0B);
#endif

  DccRx.BitCount++;

  switch (DccRx.State)
  {
  case WAIT_PREAMBLE:
    if (DccBitVal)
    {
      if (DccRx.BitCount > 10)
        DccRx.State = WAIT_START_BIT;
    }
    else
      DccRx.BitCount = 0;
    break;

  case WAIT_START_BIT:
    if (!DccBitVal)
    {
      DccRx.State = WAIT_DATA;
      DccRx.PacketBuf.Size = 0;
      DccRx.PacketBuf.PreambleBits = 0;
      for (uint8_t i = 0; i < MAX_DCC_MESSAGE_LEN; i++)
        DccRx.PacketBuf.Data[i] = 0;

      // We now have 1 too many PreambleBits so decrement before copying
      DccRx.PacketBuf.PreambleBits = DccRx.BitCount - 1;

      DccRx.BitCount = 0;
      DccRx.TempByte = 0;
    }
    break;

  case WAIT_DATA:
    DccRx.TempByte = (DccRx.TempByte << 1);
    if (DccBitVal)
      DccRx.TempByte |= 1;

    if (DccRx.BitCount == 8)
    {
      if (DccRx.PacketBuf.Size == MAX_DCC_MESSAGE_LEN)
      { // Packet is too long - abort
        DccRx.State = WAIT_PREAMBLE;
        DccRx.BitCount = 0;
      }
      else
      {
        DccRx.State = WAIT_END_BIT;
        DccRx.PacketBuf.Data[DccRx.PacketBuf.Size++] = DccRx.TempByte;
      }
    }
    break;

  case WAIT_END_BIT:
    if (DccBitVal)
    { // End of packet?
      DccRx.State = WAIT_PREAMBLE;
      DccRx.PacketCopy = DccRx.PacketBuf;
      DccRx.DataReady = 1;
    }
    else // Get next Byte
      DccRx.State = WAIT_DATA;
    DccRx.BitCount = 0;
    DccRx.TempByte = 0;
  }
} // ISR(TIMER0_COMPB_vect)

/*****************************************************************************/
/***  LOCAL FUNCTIONS  *******************************************************/
/*****************************************************************************/
// for prog track, implement a 60ms current pulse
// for main track, this should be railcom: not supported
static void ackCV()
{
  if (!DccProcState.inServiceMode)
    return; // don't do ops mode ack for now

  if (notifyCVAck)
    notifyCVAck();
} // ackCV

// writable = true: function returns true if the CV can be written/modified
// writable = false : function returns true if the CV can be read
static bool validCV(uint16_t cv, bool writable)
{
  if (notifyCVValid)
    return notifyCVValid(cv, writable);

  // and below a default implementation, when callback is not implemented
  bool isValidCV = true;

  if (cv > MAXCV)
    isValidCV = false;
  if (writable && ((cv == CV_VERSION_ID) || (cv == CV_MANUFACTURER_ID)))
    isValidCV = false;

  return isValidCV;
} // validCV

#ifdef NMRA_DCC_PROCESS_POM
static bool validPoM(uint16_t decoderAddress, uint16_t cv, bool writable)
{
  if (notifyPoMValid)
    return notifyPoMValid(decoderAddress, cv, writable);

  // and below a default implementation, when callback is not implemented
  if (dccFilterAddress == 0)
    return false; // can't tell if this PoM is for us -> play safe
  else
    return validCV(cv, writable);
} // validPoM
#endif // NMRA_DCC_PROCESS_POM

uint8_t readCV(uint16_t cv)
{
  uint8_t cvValue;

  if (notifyCVRead)
    return notifyCVRead(cv);

  // and below a default implementation, when callback is not implemented
  cvValue = eeprom_read_byte((uint8_t *)cv);
  return cvValue;
} // readCV

uint8_t writeCV(uint16_t cv, uint8_t cvValue)
{
  if ((cv == 27) || (cv == CV_RAILCOM_CONFIGURATION))
    return 0;
  // TODO : can be extended to disallow writing to certain standard CVs, or certain CV29 bits

  if (notifyCVWrite)
    return notifyCVWrite(cv, cvValue);

  // and below a default implementation, when callback is not implemented
  if (eeprom_read_byte((uint8_t *)cv) != cvValue)
  {
    eeprom_write_byte((uint8_t *)cv, cvValue);
    if (notifyCVChange)
      notifyCVChange(cv, cvValue);
  }
  return eeprom_read_byte((uint8_t *)cv);
} // writeCV

// decoderAddress : 0=service mode programming (broadcast), !=0 = POM
// cmd : 1110CCVV
// CC : 01 : verify byte, 11 : write byte, 10 : bit manipulation
// VV : 2 MSb of cv-address, but caller has already assembled all bits in parameter [cv]
// we only support PoM write for now (no railcom)
static void processCvAccessInstruction(uint16_t decoderAddress, uint8_t cmd, uint16_t cv, uint8_t cvValue)
{
  bool validAccess = false;
  cmd = cmd & 0x0C;
  if (cmd == 0xC)
  { // byte write operation
    if (decoderAddress == 0)
      validAccess = validCV(cv, true);
#ifdef NMRA_DCC_PROCESS_POM
    else
      validAccess = validPoM(decoderAddress, cv, true);
#endif
    if (validAccess)
    {
      if (writeCV(cv, cvValue) == cvValue)
        ackCV();
    }
  }
  else if (cmd == 0x4)
  {
    if (validCV(cv, false))
    {
      if (readCV(cv) == cvValue)
        ackCV();
    }
  }

  // Perform the Bit-Wise Operation
  // special format for the data byte : 111CDBBB
  // BBB : bit position within the CV
  // D : value of bit to be verified/written
  // C : 1= write bit, 0=verify bit
  else
  {
    uint8_t BitMask = (1 << (cvValue & 0x07));
    uint8_t BitValue = cvValue & 0x08;
    uint8_t BitWrite = cvValue & 0x10;
    uint8_t tempValue = readCV(cv); // Read the Current CV value

    if (BitWrite)
    { // Perform the Bit Write Operation
      if (validCV(cv, true))
      {
        if (BitValue)
          tempValue |= BitMask; // Turn the Bit On
        else
          tempValue &= ~BitMask; // Turn the Bit Off
        if (writeCV(cv, tempValue) == tempValue)
          ackCV();
      }
    }

    else
    { // Perform the Bit Verify Operation
      if (validCV(cv, false))
      {
        if (BitValue)
        {
          if (tempValue & BitMask)
            ackCV();
        }
        else
        {
          if (!(tempValue & BitMask))
            ackCV();
        }
      }
    }
  }
} // processCvAccessInstruction

#ifdef NMRA_DCC_MOBILE_DECODER
// mobAddress is either 7-bit or 14-bit, pre-parsed before coming here
static void processMultiFunctionMessage(uint16_t mobAddress, uint8_t cmd, uint8_t data1, uint8_t data2)
{
  uint8_t speed;
  uint16_t cv;

  uint8_t cmdMasked = cmd & 0b11100000;

  // we are filtering addresses and it's not our address nor broadcast -> discard
  if ((dccFilterAddress != 0) && (mobAddress != dccFilterAddress) && (mobAddress != 0))
    return;

  switch (cmdMasked)
  {
  case 0b00000000: // Decoder Control
    // TODO SDS : dit is niet duidelijk in de spec. decoder reset : is dit als DDDDDDDD=00000000, of als F=0?
    // de originele implementatie kijkt hier niet naar de D-bits, en stuurt hard reset als F=1, en stuurt geen soft reset
    switch (cmd & 0b00001110)
    {
    case 0b00000000:
      if (notifyDccReset && (cmd & 0b00000001)) // Hard Reset
        notifyDccReset(1);
      break;
    case 0b00000010: // Factory Test
      break;
    case 0b00000110: // Set Decoder Flag
      break;
    case 0b00001010: // Set Advanced Addressing
      break;
    case 0b00001110: // Decoder Acknowledgment
      break;
    default: // Reserved
      break;
    }
    break;

  // TODO : consist control
  // according nmra spec CV19 is written on consist activation?
  case 0b00010000:
    // consist activation & deactivation
    // 0001.CCCC 0AAA.AAAA
    // AAA.AAAA = 7-bit consist address, 0 = deactivate consist on this mobAddress
    // CCCC = activate consist instruction : 0010 or 0011
    break;

  // DSSS.SSSS : 7-bit speed + direction
  case 0b00100000: // Advanced Operations
    switch (cmd & 0b00011111)
    {
    case 0b00011111:
      if (notifyDccSpeed)
      {
        notifyDccSpeed(mobAddress, data1 & 0x7F, data1 & 0x80, DCC_SPEED_128STEPS);
      }
    }
    break;

  // no support for DCC14, we don't check CV29 bit 1, too old to care
  // 01DS.SSSS -> 5-bits speed + direction
  // map similar to DCC128 : 0 = STOP (slow down), 1 = EmergencySTOP, 2..29 : 28 steps
  case 0b01000000: // speed & direction instructions
  case 0b01100000:
    if (notifyDccSpeed)
    {
      cmd = cmd & 0b00011111;
      switch (cmd)
      {
      case 0b00000000:
      case 0b00010000:
        speed = 0; // stop
        break;

      case 0b00000001:
      case 0b00010001:
        speed = 1; // emergency stop
        break;

      default:
        // This speed is not quite right as 14 bit mode can happen and we should check CV29 but...
        speed = (((cmd & 0b00001111) << 1) | ((cmd & 0b00010000) >> 4)) - 2;
      }
      notifyDccSpeed(mobAddress, speed, cmd & 0x20, DCC_SPEED_28STEPS);
    }
    break;

  case 0b10000000: // Function Group 0..4
    if (notifyDccFunc)
    {
      // for convenience we put F0 at bit0, F1 at bit1 etc
      cmd = ((cmd >> 4) & 0x1) | ((cmd & 0xF) << 1);
      notifyDccFunc(mobAddress, FN_0_4, cmd);
    }
    break;

  case 0b10100000: // Function Group 5..8
    if (notifyDccFunc)
    {
      if (cmd & 0b00010000)
        notifyDccFunc(mobAddress, FN_5_8, cmd & 0b00001111);
      else
        notifyDccFunc(mobAddress, FN_9_12, cmd & 0b00001111);
    }
    break;

  // SDS: we don't do binary state control instructions
  case 0b11000000: // Feature Expansion Instruction
    switch (cmd & 0b00011111)
    {
    case 0b00011110: // CCCCC=11110 F13-F20 function control
      if (notifyDccFunc)
        notifyDccFunc(mobAddress, FN_13_20, data1);
      break;

    case 0b00011111: // CCCCC=11111 F21-F28 function control
      if (notifyDccFunc)
        notifyDccFunc(mobAddress, FN_21_28, data1);
      break;
    }
    break;

#ifdef NMRA_DCC_PROCESS_POM
  // 1110CCVV VVVVVVVV DDDDDDDD
  // CC : 01 : verify byte, 11 : write byte, 10 : bit manipulation
  // V : 10-bit cv address
  // D : 8-bit value to write or verify
  case 0b11100000: // CV Access
    cv = (((cmd & 0x03) << 8) | data1) + 1;
    processCvAccessInstruction(mobAddress, cmd, cv, data2);
    break;
#endif // NMRA_DCC_PROCESS_POM
  }
} // processMultiFunctionMessage
#endif // NMRA_DCC_MOBILE_DECODER

#ifdef NMRA_DCC_ACCESSORY_DECODER
// 10AAAAAA 1AAACDDD : basic accessory packet
// 10AAAAAA 0AAA0AA1 000XXXXX : extended accessory packet
static void processAccessoryMessage(DCC_MSG *pDccMsg)
{
  uint16_t decoderAddress; // SDS: 9-bit accessory decoder address 0..511
  decoderAddress = (((~pDccMsg->Data[1]) & 0b01110000) << 2) | (pDccMsg->Data[0] & 0b00111111);

  // we are filtering addresses and it's not our address nor broadcast -> discard
  if ((dccFilterAddress != 0) && (decoderAddress != dccFilterAddress) && (decoderAddress != 511))
    return;

  if ((pDccMsg->Size == 3) && (pDccMsg->Data[1] & 0x80))
  {                   // 3 Byte Packets are basic accessory commands
    uint8_t OutputId; // SDS : 3bits output 0..7
    OutputId = pDccMsg->Data[1] & 0x07;
    if (notifyDccAccState)
      notifyDccAccState(decoderAddress, OutputId, pDccMsg->Data[1] & 0b00001000);
  }

  else if (pDccMsg->Size == 4)
  { // 4 Byte Packets are extended accessory commands
    // TODO check : is nop ook voor MOB decoders?
    // RCN-213 : voorziet ook een NOP packet, we gaan dat voorlopig hier enkel capteren
    // voor als we ooit railcom implementeren
    if (pDccMsg->Data[1] & 0x08)
    { // NOP packet
      // pDccMsg->Data[1] & 0x1 is T-bit, to indicate basic/extended accessory, not sure if we need this in the callback
      if (notifyDccNop)
      {
        notifyDccNop(decoderAddress);
      }
    }
    else
    {
      // TODO : nog 1 bits checken?
      // NMRA-DCC : signal aspect is 5 bits dus pDccMsg->Data[2] & 0x1F
      // RCN-213 :  laat nu wel 8 bits signal aspect toe
      // en zelfde packet ook voor basic accessories waarbij pDccMsg->Data[2] de schakeltijd is voor de output (§2.3)
      // 4 signals per decoder address (compare turnoutId for basic accessory decoder)
      uint8_t signalId = (pDccMsg->Data[1] >> 1) & 0x03;
      if (notifyDccSigState)
        notifyDccSigState(decoderAddress, signalId, pDccMsg->Data[2]);
    }
  }
#ifdef NMRA_DCC_PROCESS_POM
  else if (pDccMsg->Size == 6)
  { // 6 Byte Packets are POM Accessory commands (CV access instructions for accessory decoders)
    // 10AAAAAA 1AAACDDD 1110CCVV VVVVVVVV DDDDDDDD xor
    // CDDD : only 0000 = entire decoder is supported here
    uint16_t cv = (((uint16_t)pDccMsg->Data[2] & 0x03L) << 8 | (uint16_t)pDccMsg->Data[3]) + 1;
    processCvAccessInstruction(decoderAddress, pDccMsg->Data[2], cv, pDccMsg->Data[4]);
  }
#endif // NMRA_DCC_PROCESS_POM

} // processAccessoryMessage
#endif // NMRA_DCC_ACCESSORY_DECODER

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
static void processServiceModeOperation(DCC_MSG *pDccMsg)
{
  uint16_t cv;
  uint8_t cvValue;

  if (pDccMsg->Size == 3)
  { // 3 Byte Packets are for Address Only, Register and Paged Mode
    uint8_t RegisterAddr;

    RegisterAddr = pDccMsg->Data[0] & 0x07;
    cvValue = pDccMsg->Data[1];

    if (RegisterAddr == 5)
    {
      DccProcState.PageRegister = cvValue;
      ackCV();
    }

    else
    {
      if (RegisterAddr == 4)
        cv = CV_DECODER_CONFIGURATION;

      else if ((RegisterAddr <= 3) && (DccProcState.PageRegister > 0))
        cv = ((DccProcState.PageRegister - 1) * 4) + RegisterAddr + 1;
      else
        cv = RegisterAddr + 1;

      if (pDccMsg->Data[0] & 0x08)
      { // Perform the Write Operation
        if (validCV(cv, true))
        {
          if (writeCV(cv, cvValue) == cvValue)
            ackCV();
        }
      }

      else
      { // Perform the Verify Operation
        if (validCV(cv, false))
        {
          if (readCV(cv) == cvValue)
            ackCV();
        }
      }
    }
  }

  else if (pDccMsg->Size == 4)
  { // 4 Byte Packets are for Direct Byte & Bit Mode
    cv = (((pDccMsg->Data[0] & 0x03) << 8) | pDccMsg->Data[1]) + 1;
    cvValue = pDccMsg->Data[2];
    processCvAccessInstruction(0, pDccMsg->Data[0] & 0b00001100, cv, cvValue);
  }
} // processServiceModeOperation

static void resetServiceModeTimer(bool inServiceMode)
{
  // Set the Service Mode
  DccProcState.inServiceMode = inServiceMode;
  DccProcState.LastServiceModeMillis = inServiceMode ? millis() : 0;
} // resetServiceModeTimer

static void setServiceMode(bool inServiceMode)
{
  resetServiceModeTimer(inServiceMode);

  // Set the Page Register to it's default of 1 only on the first Reset
  DccProcState.PageRegister = 1;

  // Clear the LastMsg buffer and DuplicateCount in preparation for possible CV programming
  DccProcState.DuplicateCount = 0;
  memset(&DccProcState.LastMsg, 0, sizeof(DCC_MSG));
} // setServiceMode

#endif // NMRA_DCC_PROCESS_SERVICEMODE

static void execDccProcessor(DCC_MSG *pDccMsg)
{
  if ((pDccMsg->Data[0] == 0) && (pDccMsg->Data[1] == 0))
  {
    if (notifyDccReset)
      notifyDccReset(0);

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    // If this is the first Reset then perform some one-shot actions as we maybe about to enter service mode
    if (DccProcState.inServiceMode)
      resetServiceModeTimer(true);
    else
      setServiceMode(true);
#endif // NMRA_DCC_PROCESS_SERVICEMODE
    return;
  }

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
  if (DccProcState.inServiceMode && (pDccMsg->Data[0] >= 112) && (pDccMsg->Data[0] < 128))
  {
    resetServiceModeTimer(true);
    if (memcmp(pDccMsg, &DccProcState.LastMsg, sizeof(DCC_MSG)))
    {
      DccProcState.DuplicateCount = 0;
      memcpy(&DccProcState.LastMsg, pDccMsg, sizeof(DCC_MSG));
    }
    else
    { // Wait until you see 2 identical packets before acting on a Service Mode Packet
      DccProcState.DuplicateCount++;
      if (DccProcState.DuplicateCount == 1) //SDS : je hoeft het maar 1 keer uit voeren, ook al stuurt het commandstation 5x hetzelfde
        processServiceModeOperation(pDccMsg);
    }
    return;
  }
  // if we are in service mode and fall thru here, there is an unexpected packet in service mode
  // we handle it by leaving service mode and parse it as ops mode packet
  if (DccProcState.inServiceMode)
    setServiceMode(false);
#endif // NMRA_DCC_PROCESS_SERVICEMODE

  if ((pDccMsg->Data[0] == 0b11111111) && (pDccMsg->Data[1] == 0))
  { // idle packet
    if (notifyDccIdle)
      notifyDccIdle();
  }

  else if ((pDccMsg->Data[0] == 0x00) && (pDccMsg->Data[1] == 0xC1))
  { // fast clock message
    if (notifyDccFastClock)
    {
      uint8_t aValue;
      for (uint8_t i = 2; i < pDccMsg->Size - 1; i++)
      {
        switch (pDccMsg->Data[i] & 0xC0)
        {
        case 0x00:
          aValue = pDccMsg->Data[i] & 0x3F;
          if (aValue < 60)
            dccFastClock.minute = aValue;
          break;
        case 0x80:
          aValue = pDccMsg->Data[i] & 0x3F;
          if (aValue < 24)
            dccFastClock.hour = aValue;
          break;
        case 0x40:
          aValue = pDccMsg->Data[i] & 0x3F;
          if (aValue < 7)
            dccFastClock.day_of_week = aValue;
          break;
        case 0xC0:
          aValue = pDccMsg->Data[i] & 0x3F;
          if (aValue < 32)
            dccFastClock.ratio = aValue;
          break;
        }
      }
      notifyDccFastClock(&dccFastClock);
    }
  }

#ifdef NMRA_DCC_MOBILE_DECODER
  // Multi Function Decoders (7-bit address)
  else if (pDccMsg->Data[0] < 128)
    processMultiFunctionMessage(pDccMsg->Data[0], pDccMsg->Data[1], pDccMsg->Data[2], pDccMsg->Data[3]);
#endif // NMRA_DCC_MOBILE_DECODER

#ifdef NMRA_DCC_ACCESSORY_DECODER
  // Basic Accessory Decoders (9-bit) & Extended Accessory Decoders (11-bit)
  else if ((pDccMsg->Data[0] >= 128) && (pDccMsg->Data[0] < 192))
  {
    processAccessoryMessage(pDccMsg);
  }
#endif // NMRA_DCC_ACCESSORY_DECODER

#ifdef NMRA_DCC_MOBILE_DECODER
  // Multi Function Decoders (14-bit address)
  else if (pDccMsg->Data[0] < 232)
  {
    uint16_t mobAddress14;
    mobAddress14 = (pDccMsg->Data[0] << 8) | pDccMsg->Data[1];
    processMultiFunctionMessage(mobAddress14, pDccMsg->Data[2], pDccMsg->Data[3], pDccMsg->Data[4]);
  }
#endif // NMRA_DCC_MOBILE_DECODER
} // execDccProcessor

/*****************************************************************************/
/***  EXTERNAL INTERFACE *****************************************************/
/*****************************************************************************/

NmraDcc::NmraDcc()
{
}

void NmraDcc::init(uint8_t ExtIntNum, uint8_t ExtIntPinNum, uint8_t EnablePullup)
{
  // Clear all the static member variables
  memset(&DccRx, 0, sizeof(DccRx));

  // setup external interrupt
  DccProcState.ExtIntNum = ExtIntNum;
  DccProcState.ExtIntPinNum = ExtIntPinNum;

  pinMode(ExtIntPinNum, INPUT);
  if (EnablePullup)
    digitalWrite(ExtIntPinNum, HIGH);

#ifdef TCCR0A
  // Change Timer0 Waveform Generation Mode from Fast PWM back to Normal Mode
  TCCR0A &= ~((1 << WGM01) | (1 << WGM00));
#else
#error NmraDcc Library requires a processor with Timer0 Output Compare B feature
#endif
  attachInterrupt(DccProcState.ExtIntNum, ExternalInterruptHandler, RISING);

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
  setServiceMode(false);
#endif // NMRA_DCC_PROCESS_SERVICEMODE

  // TODO check default values for standard CVs?

} // NmraDcc::init

uint8_t NmraDcc::getCV(uint16_t cv)
{
  return readCV(cv);
} // NmraDcc::getCV

uint8_t NmraDcc::setCV(uint16_t cv, uint8_t cvValue)
{
  return writeCV(cv, cvValue);
} // NmraDcc::setCV

bool NmraDcc::isSetCVReady()
{
  if (notifyIsSetCVReady)
    return notifyIsSetCVReady();

  return eeprom_is_ready();
}

// let the lib filter this particular dcc address
void NmraDcc::setFilterAddress(uint16_t dccAddress)
{
  dccFilterAddress = dccAddress;
} // setFilterAddress

uint8_t NmraDcc::process()
{
#ifdef NMRA_DCC_PROCESS_SERVICEMODE
  if (DccProcState.inServiceMode)
  {
    if ((millis() - DccProcState.LastServiceModeMillis) > 20L)
      setServiceMode(false);
  }
#endif // NMRA_DCC_PROCESS_SERVICEMODE

  if (DccRx.DataReady)
  {
    // We need to do this check with interrupts disabled
    cli();
    Msg = DccRx.PacketCopy;
    DccRx.DataReady = 0;
    sei();

    uint8_t xorValue = 0;

    for (uint8_t i = 0; i < DccRx.PacketCopy.Size; i++)
      xorValue ^= DccRx.PacketCopy.Data[i];

    if (xorValue)
      return 0;
    else
    {
      if (notifyDccMsg)
        notifyDccMsg(&Msg);
      execDccProcessor(&Msg);
    }
    return 1;
  }
  return 0;
} // NmraDcc::process

// helper functions to handle the CV address reading/writing
uint16_t NmraDcc::readDccAddress()
{
  uint16_t dccAddress;
  uint8_t cv29;

  cv29 = readCV(CV_DECODER_CONFIGURATION);
  if (cv29 & CV29_ACCESSORY_DECODER) // Accessory Decoder?
    dccAddress = (readCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) << 6) | readCV(CV_ACCESSORY_DECODER_ADDRESS_LSB);
  else
  {                                      // Multi-Function Decoder?
    if (cv29 & CV29_EXTENDED_ADDRESSING) // Two Byte Address?
      dccAddress = (readCV(CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB) << 8) | readCV(CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB);
    else
      dccAddress = readCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS);
  }
  return dccAddress;
} // NmraDcc::readDccAddress

void NmraDcc::writeDccAddress(uint16_t dccAddress)
{
  // take the decoderAddress & program it as our own
  // mobile & accessory decoder have different incompatible uses of address CVs
  // if both types are defined, we configure as mobile decoder
  if (dccAddress == 0)
    return; // invalid address

#ifdef NMRA_DCC_MOBILE_DECODER
  uint8_t cv29 = readCV(CV_DECODER_CONFIGURATION);
  if (dccAddress <= 127)
  {                                          // one byte addressing
    cv29 = cv29 & ~CV29_EXTENDED_ADDRESSING; // reset CV29.5
    writeCV(CV_MULTIFUNCTION_PRIMARY_ADDRESS, (uint8_t)dccAddress);
    writeCV(CV_DECODER_CONFIGURATION, cv29);
  }
  else
  {                                                                                               // extended addressing
    cv29 = cv29 | CV29_EXTENDED_ADDRESSING;                                                       // set CV29.5
    writeCV(CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, (uint8_t)(dccAddress & 0xFF));                 // lower 8-bits
    writeCV(CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, (uint8_t)(0xC0 + ((dccAddress >> 8) & 0x3F))); // upper 6-bits, highest bits=1
    writeCV(CV_DECODER_CONFIGURATION, cv29);
  }
#else
#ifdef NMRA_DCC_ACCESSORY_DECODER
  // decoderAddress=9-bits, bits 0..5 go to CV_ACCESSORY_DECODER_ADDRESS_LSB, bits 6..8 to CV_ACCESSORY_DECODER_ADDRESS_MSB
  writeCV(CV_ACCESSORY_DECODER_ADDRESS_LSB, dccAddress & 0x3F);
  writeCV(CV_ACCESSORY_DECODER_ADDRESS_MSB, (dccAddress >> 6) & 0x7);
#endif
#endif
} // NmraDcc::writeDccAddress
