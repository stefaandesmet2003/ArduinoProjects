#ifndef _keys_h_
#define _keys_h_

// ui keys
// key bestaat uit de 6-bit key-code en 2-bit event-type

// configuratie
#define PIN_ROT_CLK     3               // Used for generating interrupts using CLK signal (D3=INT1 on throttle, D2=INT0 on CS)
#define PIN_ROT_DT      4               // Used for reading DT signal
#define PIN_ROT_SW      5               // Used for the push button switch (dit is een gedebouncete key, hieronder)

#define KEYPINS  {5, 9, 8, 7, 6, 18 }   // throttle hardware setup

#define NUMBER_OF_DEBOUNCED_KEYS 6
#define KEY_ENTER   0
#define KEY_1       1
#define KEY_2       2
#define KEY_3       3
#define KEY_4       4
#define KEY_RED     5
#define KEY_ROTARY  6
#define NUMBER_OF_DEBOUNCED_KEYS 6

#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 1000

// key events
#define EVENT_NULL          0
#define EVENT_KEY_DOWN      1
#define EVENT_KEY_UP        2
#define EVENT_KEY_LONGDOWN  3
#define EVENT_ROTARY_UP     4
#define EVENT_ROTARY_DOWN   5
#define EVENT_KEY_LASTEVENT 5 // app can add events after this

typedef enum {
  UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP
} keyState_t;

void keys_Init ();
void keys_Update ();
keyState_t keys_GetState(const uint8_t keyCode);
extern void keys_Handler(uint8_t keyEvent, uint8_t keyCode) __attribute__ ((weak));

#endif // _keys_h_
