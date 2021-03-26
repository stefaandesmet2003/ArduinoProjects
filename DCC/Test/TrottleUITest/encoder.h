#ifndef __ENCODER_H_
#define __ENCODER_H_

#define EVENT_UP      1
#define EVENT_DOWN    2
#define EVENT_BUTTON  3
#define BUTTON_ENCODER_SWITCH (1 << 2)
#define BUTTON_RED            (2 << 2)
#define BUTTON_F1             (3 << 2)
#define BUTTON_F2             (4 << 2)
#define BUTTON_F3             (5 << 2)
#define BUTTON_F4             (6 << 2)

typedef void (*encoder_CallBackFunc)(uint8_t event);
void encoder_setup();
void encoder_loop();
void encoder_setCallback(encoder_CallBackFunc cb);
#endif
