/*
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */

/*  * * * * * * * * * * * * * * * * * * * * * * * * * * *
 Code by Simon Monk
 http://www.simonmonk.org
* * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "Arduino.h"
#include "Timer.h"
#include "Event.h"

static Event _events[MAX_NUMBER_OF_EVENTS];

static int8_t findFreeEventIndex(void)
{
	for (int8_t i = 0; i < MAX_NUMBER_OF_EVENTS; i++)
	{
		if (_events[i].eventType == EVENT_NONE)
		{
			return i;
		}
	}
	return NO_TIMER_AVAILABLE;
} // findFreeEventIndex

void Timer_init(void) {
	for (int i=0;i<MAX_NUMBER_OF_EVENTS;i++) {
		_events[i].eventType = EVENT_NONE;
	}
} // Timer_init

int8_t Timer_oscillate(uint8_t pin, unsigned long period, uint8_t startingValue, int repeatCount)
{
	int8_t i = findFreeEventIndex();
	if (i == NO_TIMER_AVAILABLE) return NO_TIMER_AVAILABLE;

	_events[i].eventType = EVENT_OSCILLATE;
	_events[i].pin = pin;
	_events[i].period = period;
	_events[i].pinState = startingValue;
	digitalWrite(pin, startingValue);
	_events[i].repeatCount = repeatCount * 2; // full cycles not transitions
	_events[i].lastEventTime = millis();
	_events[i].count = 0;
	return i;
}

void Timer_stop(int8_t id)
{
	if (id >= 0 && id < MAX_NUMBER_OF_EVENTS) {
		_events[id].eventType = EVENT_NONE;
   // sds : reset output pin if used
	 // SDS terug weg voor STM8, wat is dat voor kak library
   //if (_events[id].pin != 0)
   // digitalWrite(_events[id].pin,LOW);
	}
}

void Timer_update(void)
{
	unsigned long now = millis();
  for (int8_t i = 0; i < MAX_NUMBER_OF_EVENTS; i++)
  {
    if (_events[i].eventType != EVENT_NONE)
    {
      Event_update2(&_events[i],now);
    }
  }
}
