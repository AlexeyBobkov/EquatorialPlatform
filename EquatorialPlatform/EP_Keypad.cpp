/*
 * EP_Keypad.cpp
 *
 * Created: 10/30/2013 8:25:42 AM
 *  Author: Alexey
 */ 

#include "Arduino.h"
#include "EP_Keypad.h"
#include "EP_Configuration.h"

#include "ArduinoKeypadModified/Keypad.h"

///////////////////////////////////////////////////////////////////////////////////////
const byte RBITS = 2;//two row address bits
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] =
{
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

///////////////////////////////////////////////////////////////////////////////////////
Keypad kpd = Keypad( makeKeymap(keys), KPD_COUNT_OPIN, KPD_RESET_OPIN, KPD_VALUE_IPIN, RBITS, ROWS, COLS );

///////////////////////////////////////////////////////////////////////////////////////
char EP_GetKey()
{
    return kpd.getKey();
}
