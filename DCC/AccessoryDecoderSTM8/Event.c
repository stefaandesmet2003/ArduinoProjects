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
#include "Event.h"

void Event_init(Event *event)
{
	event->eventType = EVENT_NONE;
}

void Event_update(Event *event)
{
    unsigned long now = millis();
    Event_update2(event,now);
}

void Event_update2(Event *event, unsigned long now)
{
	if (now - event->lastEventTime >= event->period)
	{
		switch (event->eventType)
		{
			case EVENT_EVERY:
				(*event->callback)();
				break;

			case EVENT_OSCILLATE:
				event->pinState = ! event->pinState;
				digitalWrite(event->pin, event->pinState);
				break;
		}
		event->lastEventTime = now;
		event->count++;
	}
	if (event->repeatCount > -1 && event->count >= event->repeatCount)
	{
		event->eventType = EVENT_NONE;
	}
}
