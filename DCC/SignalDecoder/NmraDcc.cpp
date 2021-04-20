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
// file:      NmraDcc.cpp
// author:    Alex Shepherd
// webpage:   http://opendcc.org/
// history:   2011-06-26 Initial Version copied in from OpenDCC
//
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			  and build DCC Mobile and Stationary Decoders
//------------------------------------------------------------------------
#include "NmraDcc.h"
#include <avr/eeprom.h>

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

typedef enum {
  WAIT_PREAMBLE = 0,
  WAIT_START_BIT,
  WAIT_DATA,
  WAIT_END_BIT
} DccRxWaitState;

typedef struct {
  DccRxWaitState  State;
  uint8_t         DataReady;
  uint8_t         BitCount;
  uint8_t         TempByte;
  DCC_MSG         PacketBuf;
  DCC_MSG         PacketCopy;
} DccRx_t;

typedef struct
{
  uint8_t   Flags;
  uint8_t   OpsModeAddressBaseCV;
  bool      inServiceMode;
  long      LastServiceModeMillis;
  uint8_t   PageRegister;  // Used for Paged Operations in Service Mode Programming
  uint8_t   DuplicateCount;
  DCC_MSG   LastMsg;
  uint8_t	ExtIntNum; 
  uint8_t	ExtIntPinNum;
} DCC_PROCESSOR_STATE;

static DccRx_t DccRx;
static DCC_PROCESSOR_STATE DccProcState;

void ExternalInterruptHandler()
{
  OCR0B = TCNT0 + DCC_BIT_SAMPLE_PERIOD;

#if defined(TIMSK0)
  TIMSK0 |= (1<<OCIE0B);  // Enable Timer0 Compare Match B Interrupt
  TIFR0  |= (1<<OCF0B);   // Clear  Timer0 Compare Match B Flag 
#elif defined(TIMSK)
  TIMSK |= (1<<OCIE0B);  // Enable Timer0 Compare Match B Interrupt
  TIFR  |= (1<<OCF0B);   // Clear  Timer0 Compare Match B Flag 
#endif
}

ISR(TIMER0_COMPB_vect)
{
  uint8_t DccBitVal;

  // Read the DCC input value, if it's low then its a 1 bit, otherwise it is a 0 bit
  DccBitVal = !digitalRead(DccProcState.ExtIntPinNum);

  // Disable Timer0 Compare Match B Interrupt
#if defined(TIMSK0)
  TIMSK0 &= ~(1<<OCIE0B);
#elif defined(TIMSK)
  TIMSK &= ~(1<<OCIE0B);
#endif

  DccRx.BitCount++;

  switch(DccRx.State) {
  case WAIT_PREAMBLE:
    if (DccBitVal) {
      if (DccRx.BitCount > 10)
        DccRx.State = WAIT_START_BIT;
    }
    else
      DccRx.BitCount = 0;
    break;

  case WAIT_START_BIT:
    if (!DccBitVal) {
      DccRx.State = WAIT_DATA;
      DccRx.PacketBuf.Size = 0;
      DccRx.PacketBuf.PreambleBits = 0;
      for(uint8_t i = 0; i< MAX_DCC_MESSAGE_LEN; i++)
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

    if (DccRx.BitCount == 8) {
      if (DccRx.PacketBuf.Size == MAX_DCC_MESSAGE_LEN) { // Packet is too long - abort
        DccRx.State = WAIT_PREAMBLE;
        DccRx.BitCount = 0;
      }
      else {
        DccRx.State = WAIT_END_BIT;
        DccRx.PacketBuf.Data[ DccRx.PacketBuf.Size++ ] = DccRx.TempByte;
      }
    }
    break;

  case WAIT_END_BIT:
    if (DccBitVal) { // End of packet?
      DccRx.State = WAIT_PREAMBLE;
      DccRx.PacketCopy = DccRx.PacketBuf;
      DccRx.DataReady = 1;
    }
    else  // Get next Byte
    DccRx.State = WAIT_DATA;
    DccRx.BitCount = 0;
    DccRx.TempByte = 0;
  }
} // ISR(TIMER0_COMPB_vect)

/*****************************************************************************/
/***  LOCAL FUNCTIONS  *******************************************************/
/*****************************************************************************/
// for prog track, implement a 60ms current pulse
// for main track, this should be railcom: TODO
static void ackCV() {
  if (notifyCVAck)
    notifyCVAck();
} // ackCV

static uint8_t readCV(uint16_t cv) {
  uint8_t cvValue;

  if (notifyCVRead)
    return notifyCVRead(cv);

  // and below a default implementation, when callback is not implemented
  cvValue = eeprom_read_byte((uint8_t*) cv);
  return cvValue;
} // readCV

// writable = true: function returns true if the CV can be written/modified
// writable = false : function returns true if the CV can be read
static bool validCV(uint16_t cv, bool writable) {
  if (notifyCVValid)
    return notifyCVValid(cv, writable);
    
  // and below a default implementation, when callback is not implemented
  bool isValidCV = true;

  if (cv > MAXCV)
    isValidCV = false;
  if (writable && ((cv ==CV_VERSION_ID) || (cv == CV_MANUFACTURER_ID)))
    isValidCV = false;

  return isValidCV;
} // validCV


static uint8_t writeCV(uint16_t cv, uint8_t cvValue) {
  if (notifyCVWrite)
    return notifyCVWrite(cv, cvValue);

  // and below a default implementation, when callback is not implemented
  if (eeprom_read_byte((uint8_t*) cv) != cvValue) {
    eeprom_write_byte((uint8_t*) cv, cvValue);
    if (notifyCVChange)
      notifyCVChange(cv, cvValue);
  }

  return eeprom_read_byte((uint8_t*) cv);
} // writeCV


// SDS : TODO 2021 : deze code doet ook gewoon ackCV (60mA stroompuls) in POM, dat mag toch niet?
// dat is toch enkel voor programming track ??
// nog checken want processDirectOpsOperation wordt ook vanuit processServiceModeOperation en processMultiFunctionMessage gecalled!!!
// ack in POM moet via railcom
static void processDirectOpsOperation(uint8_t Cmd, uint16_t cv, uint8_t cvValue)
{
  // is it a Byte Operation
  if (Cmd & 0x04)
  {
    // Perform the Write Operation
    if (Cmd & 0x08)
    {
      if (validCV(cv, true))
      {
        if (writeCV(cv, cvValue) == cvValue)
          ackCV();
      }
    }

    else  // Perform the Verify Operation
    {  
      if (validCV(cv, false))
      {
        if (readCV(cv) == cvValue)
          ackCV();
      }
    }
  }
  // Perform the Bit-Wise Operation
  else
  {
    uint8_t BitMask = (1 << (cvValue & 0x07));
    uint8_t BitValue = cvValue & 0x08;
    uint8_t BitWrite = cvValue & 0x10;

    uint8_t tempValue = readCV(cv);  // Read the Current CV value

    // Perform the Bit Write Operation
    if (BitWrite)
    {
      if (validCV(cv, true))
      {
        if (BitValue)
          tempValue |= BitMask;     // Turn the Bit On

        else
          tempValue &= ~BitMask;  // Turn the Bit Off

        if (writeCV(cv, tempValue) == tempValue)
          ackCV();
      }
    }

    // Perform the Bit Verify Operation
    else
    {
      if (validCV(cv, false))
      {
        if (BitValue) 
        {
          if (tempValue & BitMask)
            ackCV();
        }
        else
        {
          if (!(tempValue & BitMask) )
            ackCV();
        }
      }
    }
  }
} // processDirectOpsOperation

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
static void processMultiFunctionMessage(uint16_t Addr, uint8_t Cmd, uint8_t Data1, uint8_t Data2)
{
  uint8_t  speed;
  uint16_t cv;

  uint8_t  CmdMasked = Cmd & 0b11100000;

  // If we are an Accessory Decoder
  if (DccProcState.Flags & FLAGS_DCC_ACCESSORY_DECODER)
  {
    // and this isn't an Ops Mode Write or we are NOT faking the Multifunction Ops mode address in CV 33+34 or
    // it's not our fake address, then return
    if ((CmdMasked != 0b11100000) || (DccProcState.OpsModeAddressBaseCV == 0))
      return;

    uint16_t FakeOpsAddr = readCV(DccProcState.OpsModeAddressBaseCV) | (readCV(DccProcState.OpsModeAddressBaseCV + 1) << 8);
    uint16_t OpsAddr = Addr & 0x3FFF;

    if (OpsAddr != FakeOpsAddr)
      return;
  }

  // We are looking for FLAGS_MY_ADDRESS_ONLY but it does not match and it is not a Broadcast Address then return
  else if ((DccProcState.Flags & FLAGS_MY_ADDRESS_ONLY) && (Addr != getMyAddr()) && (Addr != 0)) 
    return;

  switch(CmdMasked)
  {
  case 0b00000000:  // Decoder Control
    switch(Cmd & 0b00001110)
    {
    case 0b00000000:  
      if (notifyDccReset && (Cmd & 0b00000001)) // Hard Reset
        if (notifyDccReset)
          notifyDccReset(1);
      break;

    case 0b00000010:  // Factory Test
      break;

    case 0b00000110:  // Set Decoder Flasg
      break;

    case 0b00001010:  // Set Advanced Addressing
      break;

    case 0b00001110:  // Decoder Acknowledgment
      break;

    default:    // Reserved
      break;
    }
    break;

  case 0b00100000:  // Advanced Operations
    switch (Cmd & 0b00011111)
    {
    case 0b00011111:
      if (notifyDccSpeed) {
        switch(Data1 & 0b01111111)
        {
        case 0b00000000:  
          speed = 1;
          break;

        case 0b00000001:  
          speed = 0;
          break;

        default:
          speed = (Data1 & 0b01111111) - 1;
        }

        notifyDccSpeed(Addr, speed, Data1 & 0b10000000, 127);
      }
    }
    break;

  case 0b01000000:
  case 0b01100000:
    if (notifyDccSpeed) {
      switch(Cmd & 0b00011111)
      {
      case 0b00000000:  
      case 0b00010000:
        speed = 1;
        break;

      case 0b00000001:  
      case 0b00010001:
        speed = 0;
        break;

      default:
        // This speed is not quite right as 14 bit mode can happen and we should check CV29 but... 
        speed = (((Cmd & 0b00001111) << 1) | ((Cmd & 0b00010000) >> 4)) - 2;
      }

      notifyDccSpeed(Addr, speed, Cmd & 0b00100000, 28);
    }

    break;

  case 0b10000000:  // Function Group 0..4
    if (notifyDccFunc)
      notifyDccFunc(Addr, FN_0_4, Cmd & 0b00011111);
    break;

  case 0b10100000:  // Function Group 5..8
    if (notifyDccFunc) {
      if (Cmd & 0b00010000)
        notifyDccFunc(Addr, FN_5_8,  Cmd & 0b00001111);
      else
        notifyDccFunc(Addr, FN_9_12, Cmd & 0b00001111);
    }
    break;

  case 0b11000000:  // Feature Expansion Instruction
  	switch(Cmd & 0b00011111)
  	{
  	case 0B00011110:
  	  if (notifyDccFunc)
	    notifyDccFunc(Addr, FN_13_20, Data1);
	  break;
	  
  	case 0B00011111:
  	  if (notifyDccFunc)
	    notifyDccFunc(Addr, FN_21_28, Data1);
	  break;
  	}
    break;

  case 0b11100000:  // CV Access
    cv = (((Cmd & 0x03) << 8) | Data1) + 1;

    processDirectOpsOperation(Cmd, cv, Data2);
    break;
  }
} // processMultiFunctionMessage
#endif // NMRA_DCC_PROCESS_MULTIFUNCTION

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
static void processServiceModeOperation(DCC_MSG * pDccMsg)
{
  uint16_t cv;
  uint8_t cvValue;

  if (pDccMsg->Size == 3) { // 3 Byte Packets are for Address Only, Register and Paged Mode
    uint8_t RegisterAddr;

    RegisterAddr = pDccMsg->Data[0] & 0x07;
    cvValue = pDccMsg->Data[1];

    if (RegisterAddr == 5) {
      DccProcState.PageRegister = cvValue;
      ackCV();
    }

    else {
      if (RegisterAddr == 4)
        cv = CV_DECODER_CONFIGURATION;

      else if ((RegisterAddr <= 3) && (DccProcState.PageRegister > 0))
        cv = ((DccProcState.PageRegister - 1) * 4) + RegisterAddr + 1;
      else
        cv = RegisterAddr + 1;

      if (pDccMsg->Data[0] & 0x08) { // Perform the Write Operation
        if (validCV(cv, true)) {
          if (writeCV(cv, cvValue) == cvValue)
            ackCV();
        }
      }

      else { // Perform the Verify Operation
        if (validCV(cv, false)) {
          if (readCV(cv) == cvValue)
            ackCV();
        }
      }
    }
  }

  else if (pDccMsg->Size == 4) { // 4 Byte Packets are for Direct Byte & Bit Mode
    cv = (((pDccMsg->Data[0] & 0x03) << 8) | pDccMsg->Data[1]) + 1;
    cvValue = pDccMsg->Data[2];
    processDirectOpsOperation(pDccMsg->Data[0] & 0b00001100, cv, cvValue);
  }
}
#endif // NMRA_DCC_PROCESS_SERVICEMODE

static void resetServiceModeTimer(bool inServiceMode) {
  // Set the Service Mode
  DccProcState.inServiceMode = inServiceMode;
  DccProcState.LastServiceModeMillis = inServiceMode ? millis() : 0;
} // resetServiceModeTimer

// TODO 2021: hernoem naar setServiceMode true/false of zoiets
static void clearDccProcState(bool inServiceMode) {
  resetServiceModeTimer(inServiceMode);

  // Set the Page Register to it's default of 1 only on the first Reset
  DccProcState.PageRegister = 1;

  // Clear the LastMsg buffer and DuplicateCount in preparation for possible CV programming
  DccProcState.DuplicateCount = 0;
  memset(&DccProcState.LastMsg, 0, sizeof(DCC_MSG));
} // clearDccProcState

static void execDccProcessor(DCC_MSG * pDccMsg) {
  if ((pDccMsg->Data[0] == 0) && (pDccMsg->Data[1] == 0)) {
    if (notifyDccReset)
      notifyDccReset(0);

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    // If this is the first Reset then perform some one-shot actions as we maybe about to enter service mode
    if (DccProcState.inServiceMode)
      resetServiceModeTimer(1);
    else
      clearDccProcState(1);
#endif
  }

  else {
#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    if (DccProcState.inServiceMode && (pDccMsg->Data[0] >= 112) && (pDccMsg->Data[0] < 128)) {
      resetServiceModeTimer(1);
      if (memcmp(pDccMsg, &DccProcState.LastMsg, sizeof(DCC_MSG))) {
        DccProcState.DuplicateCount = 0;
        memcpy(&DccProcState.LastMsg, pDccMsg, sizeof(DCC_MSG));
      }
      else { // Wait until you see 2 identical packets before acting on a Service Mode Packet 
        DccProcState.DuplicateCount++;
        if (DccProcState.DuplicateCount == 1) //SDS : je hoeft het maar 1 keer uit voeren, ook al stuurt het commandstation 5x hetzelfde
          processServiceModeOperation(pDccMsg);
      }
    }

    else { // SDS : not service mode
      if (DccProcState.inServiceMode)
        clearDccProcState(0);	
#endif
      // Idle Packet
      if ((pDccMsg->Data[0] == 0b11111111) && (pDccMsg->Data[1] == 0)) {
        if (notifyDccIdle)
          notifyDccIdle();
      }

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
      // Multi Function Decoders (7-bit address)
      else if (pDccMsg->Data[0] < 128)
        processMultiFunctionMessage(pDccMsg->Data[0], pDccMsg->Data[1], pDccMsg->Data[2], pDccMsg->Data[3]);  

      // Basic Accessory Decoders (9-bit) & Extended Accessory Decoders (11-bit)
      else if (pDccMsg->Data[0] < 192)
#else
      else if ((pDccMsg->Data[0] >= 128) && (pDccMsg->Data[0] < 192)) {
#endif
        if (DccProcState.Flags & FLAGS_DCC_ACCESSORY_DECODER) {
          uint16_t DecoderAddress; // SDS: 9-bit accessory decoder address 0..511
          uint8_t  OutputId; // SDS : 3bits output 0..7
          DecoderAddress = (((~pDccMsg->Data[1]) & 0b01110000) << 2) | (pDccMsg->Data[0] & 0b00111111);

          // If we're filtering was it my board address or a broadcast address
          if ((DccProcState.Flags & FLAGS_MY_ADDRESS_ONLY) && (DecoderAddress != getMyAddr()) && (DecoderAddress != 511))
            return;

          OutputId = pDccMsg->Data[1] & 0b00000111;
          // geen idee wat dit is -> weg ermee
          //Address = (((DecoderAddress - 1) << 2) | OutputIndex) + 1;
          if (pDccMsg->Data[1] & 0b10000000) { // basic accessory packet
            if (notifyDccAccState)
              notifyDccAccState(DecoderAddress, OutputId, pDccMsg->Data[1] & 0b00001000);
          }

          else { // not basic accessory packet
            // dit staat in de extended packet formats spec, maar toch hier, handig
            // zo hebben we NMRA_DCC_PROCESS_MULTIFUNCTION niet nodig (enkel locdecoder stuff)?
            // of zit POM voor accessories daar ook onder? JA
            // RCN-213 : voorziet ook een NOP packet, we gaan dat voorlopig hier enkel capteren
            // voor als we ooit railcom implementeren
            if (pDccMsg->Data[1] & 0b00001000) { // NOP packet
              // pDccMsg->Data[1] & 0x1 is T-bit, to indicate basic/extended accessory, not sure if we need this in the callback
              if (notifyDccNop) {
                notifyDccNop(DecoderAddress);
              } 
            }
            else { // extended accessory packet
              // NMRA-DCC : signal aspect is 5 bits dus pDccMsg->Data[2] & 0x1F
              // RCN-213 :  laat nu wel 8 bits signal aspect toe
              // en zelfde packet ook voor basic accessories waarbij pDccMsg->Data[2] de schakeltijd is voor de output (§2.3)
              // 4 signals per decoder address (compare turnoutId for basic accessory decoder)
              uint8_t signalId = OutputId >> 1;
              if (notifyDccSigState)
                notifyDccSigState(DecoderAddress, signalId, pDccMsg->Data[2]);
            }
          }
        }
      }

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
      // Multi Function Decoders (14-bit address)
      else if (pDccMsg->Data[0] < 232) {
        uint16_t Address;
        Address = (pDccMsg->Data[0] << 8) | pDccMsg->Data[1];
        processMultiFunctionMessage(Address, pDccMsg->Data[2], pDccMsg->Data[3], pDccMsg->Data[4]);  
      }
#endif
#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    }
#endif
  }
} // execDccProcessor

/*****************************************************************************/
/***  EXTERNAL INTERFACE *****************************************************/
/*****************************************************************************/


NmraDcc::NmraDcc()
{
}

void NmraDcc::pin(uint8_t ExtIntNum, uint8_t ExtIntPinNum, uint8_t EnablePullup) {
  DccProcState.ExtIntNum = ExtIntNum;
  DccProcState.ExtIntPinNum = ExtIntPinNum;
	
  pinMode(ExtIntPinNum, INPUT);
  if (EnablePullup)
    digitalWrite(ExtIntPinNum, HIGH);
} // NmraDcc::pin

void NmraDcc::init(uint8_t Flags, uint8_t OpsModeAddressBaseCV) {
  // Clear all the static member variables
  memset(&DccRx, 0, sizeof(DccRx));

#ifdef TCCR0A
  // Change Timer0 Waveform Generation Mode from Fast PWM back to Normal Mode
  TCCR0A &= ~((1<<WGM01)|(1<<WGM00));
#else  
#error NmraDcc Library requires a processor with Timer0 Output Compare B feature 
#endif
  attachInterrupt(DccProcState.ExtIntNum, ExternalInterruptHandler, RISING);

  DccProcState.Flags = Flags;
  DccProcState.OpsModeAddressBaseCV = OpsModeAddressBaseCV;
  clearDccProcState(0);
} // NmraDcc::init

uint8_t NmraDcc::getCV(uint16_t cv) {
  return readCV(cv);
}

uint8_t NmraDcc::setCV(uint16_t cv, uint8_t cvValue) {
  return writeCV(cv,cvValue);
}

bool NmraDcc::isSetCVReady() {
  if (notifyIsSetCVReady)
	  return notifyIsSetCVReady();
	
  return eeprom_is_ready();
}

//SDS : te optimaliseren : voor elke DCC message wordt nu uit eeprom gelezen --> overbodig
// is enkel nodig na een factory reset of wanneer de adresCVs worden gewijzigd in service mode
uint16_t getMyAddr() {
  uint16_t  Addr;
  uint8_t   CV29Value;

  CV29Value = readCV(CV_DECODER_CONFIGURATION);

  if (CV29Value & 0b10000000)  // Accessory Decoder? 
    Addr = (readCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) << 6) | readCV(CV_ACCESSORY_DECODER_ADDRESS_LSB);
  else { // Multi-Function Decoder?
    if (CV29Value & 0b00100000)  // Two Byte Address?
      Addr = (readCV(CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB) << 8) | readCV(CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB);
    else
      Addr = readCV(1);
  }
  return Addr;
} // getMyAddr

uint8_t NmraDcc::process() {
  if (DccProcState.inServiceMode) {
    if ((millis() - DccProcState.LastServiceModeMillis) > 20L)
      clearDccProcState(0);
  }

  if (DccRx.DataReady) {
    // We need to do this check with interrupts disabled
    cli();
    Msg = DccRx.PacketCopy;
    DccRx.DataReady = 0;
    sei();
    
    uint8_t xorValue = 0;
    
    for(uint8_t i = 0; i < DccRx.PacketCopy.Size; i++)
      xorValue ^= DccRx.PacketCopy.Data[i];

    if (xorValue)
      return 0;
    else {
			if (notifyDccMsg)
				notifyDccMsg(&Msg);
      execDccProcessor(&Msg);
    }
    return 1;
  }
  return 0;  
} // NmraDcc::process
