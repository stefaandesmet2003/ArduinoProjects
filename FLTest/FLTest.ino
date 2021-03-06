#define PIN PD3
typedef volatile uint8_t RwReg;
typedef volatile const uint8_t RoReg;
static RwReg sPinMask;
static volatile RwReg *sPort;
static volatile RoReg *sInPort;
static void pin_init() {
		sPinMask = digitalPinToBitMask(PIN);
		sPort = portOutputRegister(digitalPinToPort(PIN));
		sInPort = portInputRegister(digitalPinToPort(PIN));
}
// todo add always inline in sdcc speak
inline static void hi() { *sPort |= sPinMask; }
inline static void lo() { *sPort &= ~sPinMask; }

enum EOrder {
	RGB=0012,
	RBG=0021,
	GRB=0102,
	GBR=0120,
	BRG=0201,
	BGR=0210
};
/// Representation of an RGB pixel (Red, Green, Blue)
typedef struct {
	union {
		struct {
            union {
                uint8_t r;
                uint8_t red;
            };
            union {
                uint8_t g;
                uint8_t green;
            };
            union {
                uint8_t b;
                uint8_t blue;
            };
        };
		uint8_t raw[3];
	};
} CRGB;

typedef struct {
        const uint8_t *mData;
        int mLen,mLenRemaining;
        uint8_t d[3];
        uint8_t e[3];
        CRGB mScale;
        int8_t mAdvance;
        int mOffsets[1];
} PixelController;

PixelController myPixels;
#define RGB_ORDER GRB
#define RO(X) RGB_BYTE(RGB_ORDER, X)
#define RGB_BYTE(RO,X) (((RO)>>(3*(2-(X)))) & 0x3)

#define _STM8_SCRATCH_REGS 8
volatile unsigned char VR[ _STM8_SCRATCH_REGS ];

//#define OPTIMIZE_SPEED  _Pragma("optimize=speed")
#define OPTIMIZE_SPEED  #pragma opt_code_speed


//#ifndef __INC_CLOCKLESS_STM8_H
#define __INC_CLOCKLESS_STM8_H

// Definition for a single channel clockless controller for the STM8
// family of microcontrollers

#define FASTLED_HAS_CLOCKLESS 1

//FASTLED_NAMESPACE_BEGIN

// SDS TODO
// #define _NOPS(T,USED) _STM8_F(nops)< (USED < T) ? ((int)T) - ((int)USED) : 0 >();
#define _NOPS(T,USED) nop();


static size_t showRGBInternal(PixelController *pixels)
  {
    const uint8_t RO0 = RO(0);
    const uint8_t RO1 = RO(1);
    const uint8_t RO2 = RO(2);

    VR[0] = 0;          // 0, combined with the following byte to a word
    VR[1] = RO0;        // 1st, often G
    VR[2] = RO1;        // 2nd, often R
    VR[3] = RO2;        // 3rd, often B
//    VR[4];            // LANE0 / CURRENT BYTE
//    VR[5];            // TMP / dither
//    VR[6];            // LOOPS.msb
//    VR[7];            // LOOPS.lsb

// sdcc kent geen offsetof
/*
    #define _OFF(m, d) ( offsetof( pixels, m) == d )
    static_assert(                // struct PixelController {
      _OFF(mData, 0) &&           //   const uint8_t *mData;
      _OFF(mLen, 2) &&            //   const size_t mLen;
      _OFF(mLenRemaining, 4) &&   //   size_t mLenRemaining;
      _OFF(d, 6)  &&              //   uint8_t d[3];
      _OFF(e, 9) &&               //   uint8_t e[3];
      _OFF(mScale, 12) &&         //   const CRGB mScale;
      _OFF(mAdvance, 15),         //   const int8_t mAdvance;
                                  //   ptrdiff_t mOffsets[LANES];
    #undef _OFF                   // }
      "Data layout of PixelController is unexpected");
*/
    // save interrupts, register Y
    __asm__("PUSH  CC                 \n"
        "PUSHW Y                  \n"
        "LDW   Y, X               \n"); // *pixels

    __asm__("LD    A, (s:15,X)            \n"   // mAdvance (signed byte, can be zero)
//        // need to sign extend here: (x^80)-80, 7 cycles
//        "XOR   A, #$80            \n"
//        "CLRW  X                  \n"
//        "RLWA  X, A               \n"
//        "SUBW  X, #$80            \n"
//        "PUSHW X                  \n"   // ==> (3,SP)
        // 3-44 cycles
        "PUSH  A                  \n"
        "PUSH  #$00               \n"
        "JRPL  apos               \n"
        "CPL   (1,SP)             \n"
        "apos:                    \n");   // ==> (3,SP)

    __asm__("LD    A, (s:1,X)         \n"   // mData byte*
        "PUSH  A                  \n"
        "LD    A, (s:0,X)         \n"
        "PUSH  A                  \n"); // ==> (1, SP)

    __asm__("LDW   X, (s:4,X)         \n"
        "SLAW  X                  \n"   // LOOPS = nLeds * 4
        "SLAW  X                  \n"   // we DEC a 2nd time whenever
        "DECW  X                  \n"   // LOOPS is divideable by 4
        "LDW   s:VR+6, X          \n"); //

    //if( 0xFFFF == *reinterpret_cast<__tiny volatile uint16_t*>(VR+6) ) goto epilogue;

    __asm__("LDW   X, Y               \n"); // *pixels
    __asm__("ADDW  X, s:VR            \n"); //   + RGB offset from              2cy

    __asm__("LD    A, (s:9,X)         \n"   // load struct.e[i]                 1cy
        "SUB   A, (s:6,X)         \n"   // d[i] = e[i] - d[i]               1cy
        "LD    (s:6,X), A         \n"   // store struct.d[i]                1cy
        "LD    s:VR+5, A          \n"); // and backup d[i]                  1cy

    __asm__("LD    A, (s:12,X)        \n"   // load struct.mScale.raw[i]        1cy
        "PUSH  A                  \n"); // keep scale[i]                    1cy

    __asm__("LDW   X, (1+1,SP)        \n"   // get data ptr                     1cy
        "ADDW  X, s:VR            \n"   //   + RGB offset from              2cy
        "LD    A, (X)             \n"   //  *(mData+offset)                 1cy
        "JREQ  __is_null          \n"   // ADDW has cleared carry ...       1cy
        "ADD   A, s:VR+5          \n"   //                    add d[i]      1cy
        "__is_null:               \n"   // ... so we skip again ...
        "JRNC  __done             \n"   //    ...here                       1cy
        "LD    A, #$FF            \n"   //             saturate at max $FF  1cy
        "__done:                  \n"); //

    __asm__("EXG   A, XL              \n"   // our precious little byte         1cy
        "POP   A                  \n"   // load scale                       1cy
        "MUL   X, A               \n"   //                                  4cy
        "ADDW  X, #$FF            \n"   // round up                         2cy
        "LD    A, XH              \n"   // return high 8 bits               1cy
      //"LD    s:VR, A            \n"   // and done :))
    );

    // prefer to insert NOPs at possible jump targets to help pre-fetching
    #define _HI()    hi();
    #define _HL_A()  __asm__("SLA A"); __asm__("BCCM PD_ODR, #3 \n"); 
    #define _HL_VR() __asm__("SLA s:VR+4"); __asm__("BCCM PD_ODR, #3 \n"); 
    #define _LO()    lo();

    #define ____________FIRST_BIT()                              _HI() _NOPS(T1,2); _HL_A()
    #define ____________NEXT_BIT_DESTROY_CC(ADJ) _NOPS(T3,1+ADJ) _HI() _NOPS(T1,2); _HL_VR()
    #define ____________COMPLETE_BIT(ADJ)        _NOPS(T2,1+ADJ) _LO()
    #define ____________ADJUST_LOOP(ADJ)         _NOPS(T3,1+ADJ)

    // In an ideal world, the minimum timing would be: T1=2, T2=4, T3=5
    // i.e. 4 mandatory + 7 spare instructions per cycle
    // however, as we need to use jumps to skip certain instructions without
    // losing the timing, we might jump to misaligned instructions that span
    // two of the 4-byte prefetch buffers and thus cause a pipeline stall.
    // This is OK if perfect timing for every bit doesn't matter and still
    // results in the fastest time to clock out data to the LEDs.
    // For a more exact timing, T2 and T3 need to be incresed by 1.
    do
    {

#if FASTLED_ALLOW_INTERRUPTS == 1
      enableInterrupts();  // RIM
      disableInterrupts(); // SIM
      ____________ADJUST_LOOP(6) // loop + interrupts
#else
      // put this in front to ease prefetching of the 4 byte BSET instruction
      // in case of misalignment. Ideally, the BSET would be aligned to 4 bytes.
      ____________ADJUST_LOOP(4) // looping takes 4cy
#endif

      //////////////////////////////////////////////////////////////////////////
      // BIT 0 / 1 / 2
      // on entry: current byte in A
      //           VR[6/7] = LOOPS
      // actions:  --LOOPS
      //           if( LOOPS % 4 == 0):
      //               advance data pointer
      //               --LOOPS
      //           rotate RGB channel offsets
      // on exit:  VR[4] = current left-shifted data byte
      //           X = data ptr
      //           VR[6/7] = LOOPS remaining, or $FFFF when finished
      //           VR[0/1] = RGB byte offset

      ____________FIRST_BIT() //___________________________________
      // NOTE: we decrement the LSB again whenever (LSB&3==0),
      // so decrementing the MSB when LSB==0 is 'looking ahead'
      __asm__("DEC   s:VR+7           \n"   //  --LOOPS.lsb         1cy
          "JRNE  __no_borrow      \n"   //                      1cy
          "DEC   s:VR+6           \n"   // --LOOPS.msb          1cy
          "__no_borrow:           \n"); //
      //might stall if misaligned, unless there is at least one NOP in T2
      ____________COMPLETE_BIT(3) //_______________________________
      __asm__("LDW   X, (3,SP)        \n"); // mAdvance             2cy
      __asm__("LD    s:VR+4, A        \n"); // current byte         1cy
      __asm__("LD    A, s:VR+7        \n"); //  LOOPS.lsb           1cy
      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      /* rot RGB 1/4 */ __asm__("push  s:VR+1           \n"); //    1cy
      /* rot RGB 2/4 */ __asm__("mov   s:VR+1, s:VR+2   \n"); //    1cy
      __asm__("BCP   A, #3            \n"); //                      1cy
      ____________COMPLETE_BIT(3) //_______________________________   alt:
      __asm__("JREQ  __next           \n"   // if(! LOOPS & 0x03 )  1cy   2cy
          "CLRW  X                \n"   //   don't advance ptr  1cy
          "JRA   __no_adv         \n"   //   ...                2cy
          "__next:                \n"   //  else:
          "NOP                    \n"   //  NOP elim. stall           1cy
          "DEC   s:VR+7           \n"   //  --LOOPS.lsb               1cy
          "__no_adv:              \n"); //
      //might stall if misaligned, unless there is at least one NOP in T3
      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      __asm__("TNZ   s:VR+6           \n"   //                      1cy
          "JRPL  __not_exit       \n"   //                      1cy
          "CLRW  X                \n"   // stay within bounds   1cy
          "__not_exit:            \n"); //                      1cy
      //might stall if misaligned, unless there is at least one NOP in T2
      ____________COMPLETE_BIT(3) //_______________________________
      // NOTE: the rotating RGB offset is currently on the stack!
      __asm__("ADDW  X, (1+1,SP)      \n"); //  data ptr            2cy
      __asm__("LDW   (1+1,SP), X      \n"); //  save data ptr       2cy

      //////////////////////////////////////////////////////////////////////////
      // BIT 3 / 4 / 5
      // on entry: Y = struct
      // actions:  get scale, push on STACK (8-bit)
      //           compute d[i], store only if not exiting
      //           load next byte
      //           qadd(byte, d[i]) (i.e. max out at $FF)
      // on exit:  A = dithered byte

      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      __asm__("LDW   X, Y             \n");  //                     1cy
      __asm__("ADDW  X, s:VR          \n");  // RGB byte offset     2cy
      ____________COMPLETE_BIT(3) //_______________________________
      __asm__("LD   A, (s:12,X)       \n"   // struct.mScale[i]     1cy
          "PUSH A                 \n"); // see you later...     1cy
      __asm__("LD   A, (s:9,X)        \n"   // load struct.e[i]     1cy
          "SUB  A, (s:6,X)        \n"); // d[i] = e[i] - d[i]   1cy
      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      __asm__("TNZ  s:VR+6            \n"   // exiting loop?        1cy             TODO: move out of loop, simply reverse effect after loop
          "JRMI __discard         \n"   // ...yes, discard      1cy
          "LD   (s:6,X), A        \n"   // store struct.d[i]    1cy
          "__discard:             \n");
      //might stall if misaligned, unless there is at least one NOP in T2
      ____________COMPLETE_BIT(3) //_______________________________
      /* rot RGB 3/4 */ __asm__("mov   s:VR+2, s:VR+3   \n"); //    1cy
      __asm__("LD   s:VR+5, A         \n"); // backup to VR[5]      1cy
      // NOTE: rotating RGB offset and scale currently on stack!
      __asm__("LDW  X, (1+1+1,SP)     \n"); // get data ptr         2cy
      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      __asm__("ADDW X, s:VR           \n"   //   + RGB byte offset  2cy
          "LD   A, (X)            \n"); //  *(mData+offset)     1cy
      ____________COMPLETE_BIT(3) //_______________________________
      __asm__("JREQ __no_dither       \n"   // .                    1cy
          "ADD  A, s:VR+5         \n"   // dither / add d[i]    1cy
          "__no_dither:           \n"); // ...so we skip
//STALL??
      __asm__("JRNC __no_carry        \n"   // ...twice             1cy
          "LD   A, #$FF           \n"   // max at $FF           1cy
          "__no_carry:            \n"); //

      //////////////////////////////////////////////////////////////////////////
      // BIT 6 / 7
      // on entry: A = dithered data byte
      //           STACK: scale (8-bit)
      // actions:  scale_video( byte, scale)

      //might stall if misaligned, unless there is at least one NOP in T3
      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      __asm__("EXG   A, XL            \n"   // our precious byte    1cy
          "POP   A                \n"); // load scale factor    1cy
      /* rot RGB 4/4 */ __asm__("pop   s:VR+3           \n"); //    1cy
      // NOTE: stack is 'clean' again now
      ____________COMPLETE_BIT(3) //_______________________________
      __asm__("MUL   X, A             \n"); // scale_video          4cy
      ____________NEXT_BIT_DESTROY_CC(4) //________________________
      __asm__("ADDW  X, #$FF          \n"   // round up             2cy
          "LD    A, XH            \n"); // return high 8 bits   1cy
      ____________COMPLETE_BIT(3) //_______________________________

      //////////////////////////////////////////////////////////////////////////

      // BTJT VR+3, #7, epilogue;                               2cy
      // JP loop                                                2cy

    } while ( ! ( VR[6] & 0x80 ) );

//epilogue:

    // assume we have processed all data
    __asm__("LDW  X, Y         \n"
        "CLR  (4,X)        \n"   // mLenRemaining.MSB
        "CLR  (5,X)        \n"); // mLenRemaining.LSB

    // clean up stack, restore Y, restore interrupts
    __asm__("ADD   SP, #4       \n"
        "POPW  Y            \n"
        "POP   CC           \n");
  }


#undef _NOPS
#undef _HI
#undef _HL_A
#undef _HL_VR
#undef _LO



void setup() {
  pin_init();
  showRGBInternal(&myPixels);
}

void loop() {

}
