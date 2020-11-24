#include "Arduino.h"
#include "myntp.h"
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include "TimeLib.h" // voor de functies day(), month(), year() etc

/**************************************************************/
/*   NTP TIME STUFF */
/**************************************************************/

#define ONE_HOUR    3600000UL
#define ONE_MINUTE  60000UL
#define ONE_DAY     24*ONE_HOUR
#define NTP_UPDATE_INTERVAL ONE_DAY

WiFiUDP UDP;                   // Create an instance of the WiFiUDP class to send and receive UDP messages
IPAddress timeServerIP;        
const char* ntpServerName = "0.pool.ntp.org";

const int NTP_PACKET_SIZE = 48;          // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];      // A buffer to hold incoming and outgoing packets
typedef enum {NTP_SEND, NTP_WAIT, NTP_VALID} ntpState_t;
ntpState_t ntpState;
uint32_t lastNtpResponseMillis, lastNtpRequestMillis;
uint32_t lastNtpUnixTime = 0;  // The most recent timestamp received from the time server
uint8_t dstOffset = 0; // 0 or DST_OFFSET in h : extra offset during daylight savings time (hardcoded from last sunday of march till last sunday of october)

extern ESP8266WebServer server;

static unsigned long ntpbuf2long(byte *buffer);
static void sendNTPpacket(IPAddress& address);

uint32_t ntp_GetDateTime()
{
  if (!lastNtpUnixTime)
    return (0);
  else
    return (lastNtpUnixTime + (millis() - lastNtpResponseMillis) / 1000 + (NTP_UTC_OFFSET + dstOffset)*3600); // local time!
} // ntp_GetDateTime

// UTC time for the logging - browser already corrects for local time
uint32_t ntp_GetUnixDateTime()
{
  if (!lastNtpUnixTime)
    return (0);
  else
    return (lastNtpUnixTime + (millis() - lastNtpResponseMillis) / 1000); // no offset!
} // ntp_GetUnixDateTime

void startNTP() 
{
  Serial.println("Starting UDP");
  UDP.begin(123);                          // Start listening for UDP messages to port 123
  Serial.print("Local port:\t");
  Serial.println(UDP.localPort());
  ntpState = NTP_SEND;

} // startNTP

void runNTP()
{
  unsigned long currentMillis = millis();
  int packetSize;
  
  switch (ntpState)
  {
    case NTP_SEND :
      WiFi.hostByName(ntpServerName, timeServerIP); // Get the IP address of the NTP server
      lastNtpRequestMillis = millis();
      sendNTPpacket(timeServerIP);
      ntpState = NTP_WAIT;
      break;
    case NTP_WAIT :
      // Check if the time server has responded, if so, get the UNIX time
      packetSize = UDP.parsePacket();
      if (packetSize != 0) // packet must be 48 bytes
      {
        UDP.read(packetBuffer, packetSize); // read the packet into the buffer
        if ( (packetSize == NTP_PACKET_SIZE) // packet must be 48 bytes
        && (packetBuffer[24] == (lastNtpRequestMillis & 0xFF)))
        // check if the packet corresponds to the last request
        // the tx timestamp is copied in the origin timestamp by the NTP server
        {
          Serial.println("received NTP response");
          // Combine the 4 Transmit timestamp bytes into one 32-bit number
          uint32_t NTPTime = ntpbuf2long(&packetBuffer[40]);
          // Convert NTP time to a UNIX timestamp:
          // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
          const uint32_t seventyYears = 2208988800UL;
          // subtract seventy years:
          lastNtpUnixTime = NTPTime - seventyYears;
          // calculate correction
          // uint32_t networkDelay; // = ((lastNtpReponse - lastNtpRequest) - (TxTS - RxTS)) / 2
          uint32_t fractionTxTS = (packetBuffer[44] << 24) | (packetBuffer[45] << 16) | (packetBuffer[46] << 8) | packetBuffer[47];
          fractionTxTS = fractionTxTS / 4294967; // fraction in ms
          // lastNtpResponseMillis = (millis()+lastNtpRequestMillis) >>1; // with networkDelay correction
          lastNtpResponseMillis = millis(); // no networkDelay correction
          ntpState = NTP_VALID;

          // auto-correction for daylight savings time
          int yr = year(lastNtpUnixTime) -1970;
          
          time_t dstStart, dstEnd; // start & end of daylight savings in year yr
          tmElements_t tm;

          // start of daylight saving time in year yr
          tm.Hour = 2;
          tm.Minute = 0;
          tm.Second = 0;
          tm.Day = 1;
          tm.Month = 4;
          tm.Year = yr;
          
          dstStart = makeTime(tm);
          // find the first sunday of april and then subtract a week
          dstStart = dstStart + (((8-weekday(dstStart)) % 7) - 7)* SECS_PER_DAY;
          
          // end of daylight saving time in year yr
          tm.Month = 11;
          dstEnd = makeTime(tm);
          // find the first sunday of november and then subtract a week
          dstEnd = dstEnd + (((8-weekday(dstEnd))% 7) - 7)* SECS_PER_DAY;
          
          if ((lastNtpUnixTime >= dstStart) && (lastNtpUnixTime <= dstEnd)) {
            // we are in daylight savings time period
            dstOffset = DST_OFFSET;
          }
          else {
            dstOffset = 0;
          }
        }
        else // else discard this packet
        {
          ntpState = NTP_SEND; // send a new request immediately, we don't need to wait for a timeout
        }
      }
      else
      {
        // no response yet
        if ((currentMillis - lastNtpRequestMillis) > 10000)
        {
          // TODO : try another time server?
          ntpState = NTP_SEND;
        }
      }
      break;
    case NTP_VALID :
      if ((currentMillis - lastNtpResponseMillis)> NTP_UPDATE_INTERVAL)
      {
        // Request the time from the time server every NTP_UPDATE_INTERVAL
        ntpState = NTP_SEND;
      }
      break;
  }
} // runNTP

// web interface
// stuur via json string de actuele tijd op de module door, en de laatste ntp update
void handleNTPRequest()
{
  uint32_t actualTime = ntp_GetDateTime(); // incl UTC OFFSET
  String json = "{";
  json += "\"curTime\":"+String(actualTime);
  json += ", \"lastNtpUpdate\":"+String(lastNtpUnixTime);
  json += "}";
  server.send(200, "text/json", json);
  json = String();
} // handleNTPRequest


// convert 4 timestamp bytes into a 32-bit value
static unsigned long ntpbuf2long(byte *buffer)
{
  unsigned long tmp;
  tmp  = ((unsigned long) buffer[0]) << 24;
  tmp |= ((unsigned long) buffer[1]) << 16;
  tmp |= ((unsigned long) buffer[2]) << 8;
  tmp |= ((unsigned long) buffer[3]);
  return (tmp);
} // ntpbuf2long

static void sendNTPpacket(IPAddress& address) 
{
  Serial.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // sds test : fill in millis in the tx ts - use this as a signature
  packetBuffer[40] = lastNtpRequestMillis & 0xFF;
  packetBuffer[41] = (lastNtpRequestMillis >> 8) & 0xFF;
  packetBuffer[42] = (lastNtpRequestMillis >> 16) & 0xFF;
  packetBuffer[43] = (lastNtpRequestMillis >> 24) & 0xFF;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  UDP.beginPacket(address, 123); //NTP requests are to port 123
  UDP.write(packetBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
} // sendNTPpacket
