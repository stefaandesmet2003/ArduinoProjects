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
//            2021 modified for STM8
//------------------------------------------------------------------------
//
// purpose:   Provide a simplified interface to decode NMRA DCC packets
//			      and build DCC Mobile and Stationary Decoders
//------------------------------------------------------------------------
#include "NmraDcc.h"
#include "EEPROM.h"

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
// DCC_BIT_SAMPLE_PERIOD = nbr of TIMER0 ticks in 70us (TIMER0 = 4us/tick @ 16MHz)
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

typedef struct DccRx_t
{
  DccRxWaitState  State;
  uint8_t         DataReady;
  uint8_t         BitCount;
  uint8_t         TempByte;
  DCC_MSG         PacketBuf;
  DCC_MSG         PacketCopy;
} DccRx_t;

typedef struct
{
  uint8_t   Flags ;
  uint8_t   OpsModeAddressBaseCV ;
  bool      inServiceMode ;
  long      LastServiceModeMillis ;
  uint8_t   PageRegister ;  // Used for Paged Operations in Service Mode Programming
  uint8_t   DuplicateCount ;
  DCC_MSG   LastMsg ;
} DCC_PROCESSOR_STATE;

static DccRx_t DccRx;
static DCC_PROCESSOR_STATE DccProcState;
static DCC_MSG Msg;

/*****************************************************************************/
/***  ISR  *******************************************************************/
/*****************************************************************************/
void ExternalInterruptHandler(void)
{
  // de DCC_BIT_SAMPLE_PERIOD zit verwerkt in de ARRH/ARRL
  // start timer2 & continue dcc processing after the 70us timeout
  TIM2->CNTRH = 0;
  TIM2->CNTRL = 0;
  TIM2->CR1 = 1; // counter enable met URS=0 (CR1=5 met URS=1)

} // ExternalInterruptHandler

INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, ITC_IRQ_TIM2_OVF) {
  uint8_t DccBitVal ;
  TIM2->SR1 = 0;
  TIM2->CR1 = 0; // counter disable until next external interrupt

  // Read the DCC input value, if it's low then its a 1 bit, otherwise it is a 0 bit
  DccBitVal = !digitalRead(PIN_DCCIN);
  DccRx.BitCount++;

  switch( DccRx.State )
  {
  case WAIT_PREAMBLE:
    if( DccBitVal )
    {
      if( DccRx.BitCount > 10 )
        DccRx.State = WAIT_START_BIT ;
    }
    else
      DccRx.BitCount = 0 ;

    break;

  case WAIT_START_BIT:
    if( !DccBitVal )
    {
      DccRx.State = WAIT_DATA ;
      DccRx.PacketBuf.Size = 0;
      DccRx.PacketBuf.PreambleBits = 0;
      for(uint8_t i = 0; i< MAX_DCC_MESSAGE_LEN; i++ )
        DccRx.PacketBuf.Data[i] = 0;

      // We now have 1 too many PreambleBits so decrement before copying
      DccRx.PacketBuf.PreambleBits = DccRx.BitCount - 1 ;

      DccRx.BitCount = 0 ;
      DccRx.TempByte = 0 ;
    }
    break;

  case WAIT_DATA:
    DccRx.TempByte = ( DccRx.TempByte << 1 ) ;
    if( DccBitVal )
      DccRx.TempByte |= 1 ;

    if( DccRx.BitCount == 8 )
    {
      if( DccRx.PacketBuf.Size == MAX_DCC_MESSAGE_LEN ) // Packet is too long - abort
      {
        DccRx.State = WAIT_PREAMBLE ;
        DccRx.BitCount = 0 ;
      }
      else
      {
        DccRx.State = WAIT_END_BIT ;
        DccRx.PacketBuf.Data[ DccRx.PacketBuf.Size++ ] = DccRx.TempByte ;
      }
    }
    break;

  case WAIT_END_BIT:
    if( DccBitVal ) // End of packet?
    {
      DccRx.State = WAIT_PREAMBLE ;
      DccRx.PacketCopy = DccRx.PacketBuf ;
      DccRx.DataReady = 1 ;
    }
    else  // Get next Byte
    DccRx.State = WAIT_DATA ;

    DccRx.BitCount = 0 ;
    DccRx.TempByte = 0 ;
  }
} // TIM2_UPD_OVF_BRK_IRQHandler

/*****************************************************************************/
/***  LOCAL FUNCTIONS  *******************************************************/
/*****************************************************************************/
// for prog track, implement a 60ms current pulse
// for main track, this should be railcom: TODO
static void ackCV(void) {
  notifyCVAck();
} // ackCV

static uint8_t readCV(uint16_t CV) {
  uint8_t Value ;
  // STM8 SDCC doesn't know weak functions, and we are happy with the default
  /*
  if( notifyCVRead )
    return notifyCVRead( CV ) ;
  */
  // a default implementatie when the callback is not implemented
  Value = EEPROM_read( (int) CV ) ;
  return Value;
} // readCV

static uint8_t validCV(uint16_t CV, uint8_t Writable) {
  // STM8 SDCC doesn't know weak functions, and we are happy with the default
  return notifyCVValid(CV, Writable);
  // a default implementatie when the callback is not implemented -> not used for STM8
  /*
  uint8_t Valid = 1 ;
  if( CV > E2END )
    Valid = 0 ;
  if( Writable && ( ( CV ==CV_VERSION_ID ) || (CV == CV_MANUFACTURER_ID ) ) )
    Valid = 0 ;
  return Valid ;
  */
} // validCV

static uint8_t writeCV( uint16_t CV, uint8_t Value) {
  return notifyCVWrite(CV, Value);

  // a default implementatie when the callback is not implemented -> not used for STM8
  /*
  if (EEPROM_read((int) CV ) != Value) {
    EEPROM_write((int) CV, Value);
    if (notifyCVChange)
      notifyCVChange(CV, Value);
  }
  return EEPROM_read( (int) CV ) ;
  */
} // writeCV

// SDS : TODO 2021 : deze code doet ook gewoon ackCV (60mA stroompuls) in POM, dat mag toch niet?
// dat is toch enkel voor programming track ??
// ack in POM moet via railcom
static void processDirectOpsOperation( uint8_t Cmd, uint16_t CVAddr, uint8_t Value )
{
  // is it a Byte Operation
  if( Cmd & 0x04 )
  {
    // Perform the Write Operation
    if( Cmd & 0x08 )
    {
      if( validCV( CVAddr, 1 ) )
      {
        if( writeCV( CVAddr, Value ) == Value )
          ackCV();
      }
    }

    else  // Perform the Verify Operation
    {  
      if( validCV( CVAddr, 0 ) )
      {
        if( readCV( CVAddr ) == Value )
          ackCV();
      }
    }
  }
  // Perform the Bit-Wise Operation
  else
  {
    uint8_t BitMask = (1 << (Value & 0x07) ) ;
    uint8_t BitValue = Value & 0x08 ;
    uint8_t BitWrite = Value & 0x10 ;

    uint8_t tempValue = readCV( CVAddr ) ;  // Read the Current CV Value

    // Perform the Bit Write Operation
    if( BitWrite )
    {
      if( validCV( CVAddr, 1 ) )
      {
        if( BitValue )
          tempValue |= BitMask ;     // Turn the Bit On

        else
          tempValue &= ~BitMask ;  // Turn the Bit Off

        if( writeCV( CVAddr, tempValue ) == tempValue )
          ackCV() ;
      }
    }

    // Perform the Bit Verify Operation
    else
    {
      if( validCV( CVAddr, 0 ) )
      {
        if( BitValue ) 
        {
          if( tempValue & BitMask )
            ackCV() ;
        }
        else
        {
          if( !( tempValue & BitMask)  )
            ackCV() ;
        }
      }
    }
  }
} // processDirectOpsOperation

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
static void processMultiFunctionMessage( uint16_t Addr, uint8_t Cmd, uint8_t Data1, uint8_t Data2 )
{
  uint8_t  speed ;
  uint16_t CVAddr ;

  uint8_t  CmdMasked = Cmd & 0b11100000 ;

  // If we are an Accessory Decoder
  if( DccProcState.Flags & FLAGS_DCC_ACCESSORY_DECODER )
  {
    // and this isn't an Ops Mode Write or we are NOT faking the Multifunction Ops mode address in CV 33+34 or
    // it's not our fake address, then return
    if( ( CmdMasked != 0b11100000 ) || ( DccProcState.OpsModeAddressBaseCV == 0 ) )
      return ;

    uint16_t FakeOpsAddr = readCV( DccProcState.OpsModeAddressBaseCV ) | ( readCV( DccProcState.OpsModeAddressBaseCV + 1 ) << 8 ) ;
    uint16_t OpsAddr = Addr & 0x3FFF ;

    if( OpsAddr != FakeOpsAddr )
      return ;
  }

  // We are looking for FLAGS_MY_ADDRESS_ONLY but it does not match and it is not a Broadcast Address then return
  else if( ( DccProcState.Flags & FLAGS_MY_ADDRESS_ONLY ) && ( Addr != getMyAddr() ) && ( Addr != 0 ) ) 
    return ;

  switch( CmdMasked )
  {
  case 0b00000000:  // Decoder Control
    switch( Cmd & 0b00001110 )
    {
    case 0b00000000:  
      if(Cmd & 0b00000001) // Hard Reset
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
      switch( Data1 & 0b01111111 )
      {
      case 0b00000000:  
        speed = 1 ;
        break ;

      case 0b00000001:  
        speed = 0 ;
        break ;

      default:
        speed = (Data1 & 0b01111111) - 1 ;
      }

      notifyDccSpeed( Addr, speed, Data1 & 0b10000000, 127 ) ;
    }
    break;

  case 0b01000000:
  case 0b01100000:
    switch( Cmd & 0b00011111 )
    {
    case 0b00000000:  
    case 0b00010000:
      speed = 1 ;
      break ;

    case 0b00000001:  
    case 0b00010001:
      speed = 0 ;
      break ;

    default:
      // This speed is not quite right as 14 bit mode can happen and we should check CV29 but... 
      speed = (((Cmd & 0b00001111) << 1 ) | ((Cmd & 0b00010000) >> 4)) - 2 ;
    }

    notifyDccSpeed( Addr, speed, Cmd & 0b00100000, 28 ) ;

  break;

  case 0b10000000:  // Function Group 0..4
    notifyDccFunc( Addr, FN_0_4, Cmd & 0b00011111 ) ;
    break;

  case 0b10100000:  // Function Group 5..8
    if (Cmd & 0b00010000 )
      notifyDccFunc( Addr, FN_5_8,  Cmd & 0b00001111 ) ;
    else
      notifyDccFunc( Addr, FN_9_12, Cmd & 0b00001111 ) ;
    break;

  case 0b11000000:  // Feature Expansion Instruction
  	switch(Cmd & 0b00011111)
  	{
  	case 0B00011110:
	    notifyDccFunc( Addr, FN_13_20, Data1 ) ;
  	  break;
	  
  	case 0B00011111:
	    notifyDccFunc( Addr, FN_21_28, Data1 ) ;
  	  break;
  	}
    break;

  case 0b11100000:  // CV Access
    CVAddr = ( ( ( Cmd & 0x03 ) << 8 ) | Data1 ) + 1 ;

    processDirectOpsOperation( Cmd, CVAddr, Data2 ) ;
    break;
  }
} // processMultiFunctionMessage
#endif // NMRA_DCC_PROCESS_MULTIFUNCTION

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
void processServiceModeOperation( DCC_MSG * pDccMsg )
{
  uint16_t CVAddr ;
  uint8_t Value ;

  if( pDccMsg->Size == 3) // 3 Byte Packets are for Address Only, Register and Paged Mode
  {
    uint8_t RegisterAddr ;

    RegisterAddr = pDccMsg->Data[0] & 0x07 ;
    Value = pDccMsg->Data[1] ;

    if( RegisterAddr == 5 )
    {
      DccProcState.PageRegister = Value ;
      ackCV();
    }

    else
    {
      if( RegisterAddr == 4 )
        CVAddr = CV_29_CONFIG ;

      else if( ( RegisterAddr <= 3 ) && ( DccProcState.PageRegister > 0 ) )
        CVAddr = ( ( DccProcState.PageRegister - 1 ) * 4 ) + RegisterAddr + 1 ;

      else
        CVAddr = RegisterAddr + 1 ;

      if( pDccMsg->Data[0] & 0x08 ) // Perform the Write Operation
      {
        if( validCV( CVAddr, 1 ) )
        {
          if( writeCV( CVAddr, Value ) == Value )
            ackCV();
        }
      }

      else  // Perform the Verify Operation
      {  
        if( validCV( CVAddr, 0 ) )
        {
          if( readCV( CVAddr ) == Value )
            ackCV();
        }
      }
    }
  }

  else if( pDccMsg->Size == 4) // 4 Byte Packets are for Direct Byte & Bit Mode
  {
    CVAddr = ( ( ( pDccMsg->Data[0] & 0x03 ) << 8 ) | pDccMsg->Data[1] ) + 1 ;
    Value = pDccMsg->Data[2] ;

    processDirectOpsOperation( pDccMsg->Data[0] & 0b00001100, CVAddr, Value ) ;
  }
}
#endif // NMRA_DCC_PROCESS_SERVICEMODE

static void resetServiceModeTimer(bool inServiceMode)
{
  // Set the Service Mode
  DccProcState.inServiceMode = inServiceMode ;
  DccProcState.LastServiceModeMillis = inServiceMode ? millis() : 0 ;
} // resetServiceModeTimer

// TODO 2021: hernoem naar setServiceMode true/false of zoiets
static void clearDccProcState(bool inServiceMode)
{
  resetServiceModeTimer( inServiceMode ) ;

  // Set the Page Register to it's default of 1 only on the first Reset
  DccProcState.PageRegister = 1 ;

  // Clear the LastMsg buffer and DuplicateCount in preparation for possible CV programming
  DccProcState.DuplicateCount = 0 ;
  memset( &DccProcState.LastMsg, 0, sizeof( DCC_MSG ) ) ;
} // clearDccProcState

static void execDccProcessor( DCC_MSG * pDccMsg )
{
  if( ( pDccMsg->Data[0] == 0 ) && ( pDccMsg->Data[1] == 0 ) )
  {
    notifyDccReset( 0 ) ;

#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    // If this is the first Reset then perform some one-shot actions as we maybe about to enter service mode
    if( DccProcState.inServiceMode )
      resetServiceModeTimer( 1 ) ;
    else
      clearDccProcState( 1 );
#endif
  }

  else
  {
#ifdef NMRA_DCC_PROCESS_SERVICEMODE
    if( DccProcState.inServiceMode && ( pDccMsg->Data[0] >= 112 ) && ( pDccMsg->Data[0] < 128 ) )
    {
      resetServiceModeTimer( 1 ) ;
      if( memcmp( pDccMsg, &DccProcState.LastMsg, sizeof( DCC_MSG ) ) )
      {
        DccProcState.DuplicateCount = 0 ;
        memcpy( &DccProcState.LastMsg, pDccMsg, sizeof( DCC_MSG ) ) ;
      }
      // Wait until you see 2 identical packets before acting on a Service Mode Packet 
      else
      {
        DccProcState.DuplicateCount++ ;
        if (DccProcState.DuplicateCount == 1) //SDS : je hoeft het maar 1 keer uit voeren, ook al stuurt het commandstation 5x hetzelfde
          processServiceModeOperation( pDccMsg ) ;
      }
    }

    else // SDS : not service mode
    {
      if( DccProcState.inServiceMode )
        clearDccProcState( 0 );	
#endif

      // Idle Packet
      if( ( pDccMsg->Data[0] == 0b11111111 ) && ( pDccMsg->Data[1] == 0 ) )
      {
        notifyDccIdle() ; //SDS : eventueel te gebruiken om bus activiteit te tonen met een led?
      }

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
      // Multi Function Decoders (7-bit address)
      else if( pDccMsg->Data[0] < 128 )
        processMultiFunctionMessage( pDccMsg->Data[0], pDccMsg->Data[1], pDccMsg->Data[2], pDccMsg->Data[3] ) ;  

      // Basic Accessory Decoders (9-bit) & Extended Accessory Decoders (11-bit)
      else if( pDccMsg->Data[0] < 192 )
#else
      else if( ( pDccMsg->Data[0] >= 128 ) && ( pDccMsg->Data[0] < 192 ) ) {
#endif
        if( DccProcState.Flags & FLAGS_DCC_ACCESSORY_DECODER )
        {
          uint16_t DecoderAddress ; // SDS: 9-bit accessory decoder address 0..511
          uint8_t  OutputAddress ; // SDS : 3bits output 0..7
          uint8_t  OutputIndex ; // SDS : wissel 0..3
          uint16_t Address ; // SDS : wadisda??? soort nr van de wissel, waarom nodig??

          DecoderAddress = ( ( (~pDccMsg->Data[1]) & 0b01110000 ) << 2 ) | ( pDccMsg->Data[0] & 0b00111111 ) ;

          // If we're filtering was it my board address or a broadcast address
          if( ( DccProcState.Flags & FLAGS_MY_ADDRESS_ONLY ) && ( DecoderAddress != getMyAddr() ) && ( DecoderAddress != 511 ) )
            return;

          OutputAddress = pDccMsg->Data[1] & 0b00000111 ;
          OutputIndex = OutputAddress >> 1;
          Address = ( ( ( DecoderAddress - 1 ) << 2 ) | OutputIndex ) + 1 ;
          if(pDccMsg->Data[1] & 0b10000000) {
            notifyDccAccState( Address, DecoderAddress, OutputAddress, pDccMsg->Data[1] & 0b00001000 ) ;
          }

          else  { // SDS 2021 : signal aspect is 5 bits dus pDccMsg->Data[2] & 0x1F
            // dit staat in de extended packet formats spec, maar toch hier, handig
            // zo hebben we NMRA_DCC_PROCESS_MULTIFUNCTION niet nodig (enkel locdecoder stuff)?
            // of zit POM voor accessories daar ook onder? JA
            notifyDccSigState( Address, OutputIndex, pDccMsg->Data[2] & 0x1F) ;
          }
        }
      }

#ifdef NMRA_DCC_PROCESS_MULTIFUNCTION
      // Multi Function Decoders (14-bit address)
      else if( pDccMsg->Data[0] < 232 )
      {
        uint16_t Address ;
        Address = ( pDccMsg->Data[0] << 8 ) | pDccMsg->Data[1];
        processMultiFunctionMessage( Address, pDccMsg->Data[2], pDccMsg->Data[3], pDccMsg->Data[4] ) ;  
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
void DCC_init (uint8_t Flags, uint8_t OpsModeAddressBaseCV, bool ExtIntPinPullupEnable) {
  // Clear all the static member variables
  memset( &DccRx, 0, sizeof( DccRx) );

  // setup external interrupt on PB4 = fixed pin for dcc input
  if (ExtIntPinPullupEnable)
    pinMode (PIN_DCCIN, INPUT_PULLUP);
  else
    pinMode (PIN_DCCIN, INPUT);
  attachInterrupt(1,ExternalInterruptHandler, 0); // mode is not implemented in sduino, but we did the config manually

  disableInterrupts(); // EXTI->CR1 kan je maar schrijven onder disabled interrupts (CCR=3); manual nog eens goed lezen, maar zo lijkt het wel te werken
  EXTI->CR1 = 0x04; // set rising interrupt for all pins on port B
  GPIOB->CR2 = 0x10; // enable ext interrupt on pin PB4
  enableInterrupts();

  // configure timer2
  // lichtjes anders dan avr versie van nmradcc, omdat we hier timer2 voor onszelf hebben
  // we gebruiken de overflow, maar starten de timer pas als we hem nodig hebben (na een ext int)
  // voor avr gebruikt nmradcc de output compare interrupt van timer0 om millis() niet te verstoren
  // we zouden dat voor de gein ook op sduino kunnen doen met TIM4
  
  TIM2->CR1 = TIM2_CR1_URS; // =4, counter disable & URS=1
  TIM2->PSCR = 5; // prescale 2^5 = 16Mhz/32 = 500kHz=2us
  TIM2->ARRH= 0; // write MSB first
  TIM2->ARRL= 35; // overflow after 70us
  TIM2->CNTRH = 0;
  TIM2->CNTRL = 0;
  TIM2->EGR = 1; // UG, forced update of ARRH & PSCR, not waiting for UEV, URS=1, so no update interrupt after this manual update
  TIM2->SR1 = 0; // clear all int flags
  TIM2->IER = 1; // update interrupt enable, IRQ triggered on UIF flag in SR1

  DccProcState.Flags = Flags ;
  DccProcState.OpsModeAddressBaseCV = OpsModeAddressBaseCV ;
  clearDccProcState( 0 );    

} // DCC_init

uint8_t DCC_getCV( uint16_t CV )
{
  return readCV(CV);
}

uint8_t DCC_setCV( uint16_t CV, uint8_t Value)
{
  return writeCV(CV,Value);
}

bool DCC_isSetCVReady(void)
{
  // STM8 SDCC doesn't know weak functions, and we are happy with the default
  /*
  if(notifyIsSetCVReady)
	  return notifyIsSetCVReady();
  */
  return true; // STM8 eeprom is always ready for read
}

//TODO : te optimaliseren : voor elke DCC message wordt nu uit eeprom gelezen --> overbodig
// is enkel nodig na een factory reset of wanneer de adresCVs worden gewijzigd in service mode
uint16_t getMyAddr(void)
{
  uint16_t  Addr;
  uint8_t   CV29Value;

  CV29Value = readCV( CV_29_CONFIG ) ;
  if( CV29Value & 0b10000000 )  // Accessory Decoder? 
    Addr = ( readCV( CV_ACCESSORY_DECODER_ADDRESS_MSB ) << 6 ) | readCV( CV_ACCESSORY_DECODER_ADDRESS_LSB ) ;
  else {  // Multi-Function Decoder?
    if( CV29Value & 0b00100000 )  // Two Byte Address?
      Addr = ( readCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB ) << 8 ) | readCV( CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB ) ;

    else
      Addr = readCV( 1 ) ;
  }
  return Addr ;
} // getMyAddr

uint8_t DCC_process()
{
  if( DccProcState.inServiceMode ) {
    if( (millis() - DccProcState.LastServiceModeMillis ) > 20L )
      clearDccProcState( 0 );
  }

  if (DccRx.DataReady) {
    // We need to do this check with interrupts disabled
    disableInterrupts();
    Msg = DccRx.PacketCopy;
    DccRx.DataReady = 0;
    enableInterrupts();

    uint8_t xorValue = 0;
    for(uint8_t i = 0; i < DccRx.PacketCopy.Size; i++)
      xorValue ^= DccRx.PacketCopy.Data[i];

    if(xorValue)
      return 0;
    else
		{
		  notifyDccMsg( &Msg );
      execDccProcessor( &Msg );
    }
    return 1 ;
  }

  return 0 ;  
} // DCC_process
