#include "Arduino.h"
#include "sonar.h"

#define PING_OVERHEAD 5         // Ping overhead in microseconds (uS). Default=5
#define MAX_SENSOR_DELAY 5800   // Maximum uS it takes for sensor to start the ping. Default=5800
#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57

static uint8_t _triggerPin;
static uint8_t _echoPin;

void sonar_init(uint8_t triggerPin, uint8_t echoPin) {
  _triggerPin = triggerPin;
  _echoPin = echoPin;
  pinMode(_triggerPin, OUTPUT);
  pinMode(_echoPin, INPUT);
} // sonar_init

uint16_t sonar_ping_cm() {
  uint32_t _maxEchoTime = (MAX_SENSOR_DISTANCE + 1) * US_ROUNDTRIP_CM; // Calculate the maximum distance in uS (rounded)
  uint32_t _max_time, echoTime;

  digitalWrite(_triggerPin, LOW);   // Set the trigger pin low, should already be low, but this will make sure it is.
	delayMicroseconds(4);             // Wait for pin to go low.
	digitalWrite(_triggerPin, HIGH);  // Set trigger pin high, this tells the sensor to send out a ping.
	delayMicroseconds(10);            // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
	digitalWrite(_triggerPin, LOW);   // Set trigger pin back to low.

  if (digitalRead(_echoPin)) return NO_ECHO;                // Previous ping hasn't finished, abort.
  _max_time = micros() + _maxEchoTime + MAX_SENSOR_DELAY; // Maximum time we'll wait for ping to start (most sensors are <450uS, the SRF06 can take up to 34,300uS!)
  while (!digitalRead(_echoPin))                          // Wait for ping to start.
    if (micros() > _max_time) return NO_ECHO;             // Took too long to start, abort.
	_max_time = micros() + _maxEchoTime; // Ping started, set the time-out.

  while (digitalRead(_echoPin))                 // Wait for the ping echo.
    if (micros() > _max_time) return NO_ECHO; // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.

	echoTime = (micros() - (_max_time - _maxEchoTime) - PING_OVERHEAD); // Calculate ping time, include overhead.
  return (echoTime / US_ROUNDTRIP_CM); // echo distance in cm
} // sonar_ping_cm
