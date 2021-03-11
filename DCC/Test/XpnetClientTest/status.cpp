#include "status.h"


// de clock wordt via xpnet messages geupdatet

t_fast_clock fast_clock =      // we start, Monday, 8:00
  {
    0,    // unsigned char minute;
    8,    // unsigned char hour;
    0,    // unsigned char day_of_week;
    8,    // unsigned char ratio;
  };
