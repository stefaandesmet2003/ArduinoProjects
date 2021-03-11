#ifndef _status_h_
#define _status_h_

typedef struct
  {
    unsigned char minute; 
    unsigned char hour;
    unsigned char day_of_week;
    unsigned char ratio;
  } t_fast_clock;

extern t_fast_clock fast_clock;

#endif // _status_h_
