/*
 * EP_Sound.cpp
 *
 * Created: 10/30/2013 11:15:02 PM
 *  Author: Alexey
 */ 

#include "Arduino.h"
#include "EP_Configuration.h"

///////////////////////////////////////////////////////////////////////////////////////
void EP_Sound(unsigned long duration, unsigned int frequency)
{
    tone(SOUND_PIN, frequency, duration);
}
