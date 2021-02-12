#ifndef _SONAR_H_
#define _SONAR_H_

#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance. Default=0
#define MAX_SENSOR_DISTANCE 500 // Maximum sensor distance can be as high as 500cm, no reason to wait for ping longer than sound takes to travel this distance and back. Default=500

void sonar_init(uint8_t triggerPin, uint8_t echoPin);
uint16_t sonar_ping_cm();

#endif // _SONAR_H_