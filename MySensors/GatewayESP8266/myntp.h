#ifndef _MYNTP_H_
#define _MYNTP_H_

void startNTP();
void runNTP();

#define NTP_UTC_OFFSET  1 // +1 uur tov GMT
#define DST_OFFSET      1 // +1h from start to end of daylight savings time (last sunday of march till last sunday of october in BE)
                          // set this to 0 to disable automatic correction for daylight saving time

// return : 0 if no ntp time obtained yet, else utc offset corrected unix time
uint32_t ntp_GetDateTime();
uint32_t ntp_GetUnixDateTime();

// server handler
void handleNTPRequest();

#endif // _MYNTP_H_
