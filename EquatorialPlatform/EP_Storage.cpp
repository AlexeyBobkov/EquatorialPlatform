/*
 * EP_Storage.cpp
 *
 * Created: 12/1/2013 7:28:13 PM
 *  Author: Alexey
 */ 

#include "Arduino.h"
#include <EEPROM.h>

#include <stddef.h>
#include "EP_Motor.h"
#include "EP_Storage.h"

// EEPROM layout
struct Layout
{
    uint8_t uFastRPS;
    uint8_t uFastStp;
    uint8_t uSlowStp;

    double  dCurrentAnglePos;
    uint8_t uCapabilities;
    uint8_t uSessionId;
    double  dMotorStartAngle;
    double  dMotorEndAngle;
};


///////////////////////////////////////////////////////////////////////////////////////
template <typename T1, typename T2> struct StorageFn;
template <typename T> struct StorageFn<T, T>
{
    static void Write(const T &x, size_t offset)
    {
        for(int i = sizeof(T); --i >= 0;)
            EEPROM.write(offset+i, ((const uint8_t*)&x)[i]);
    }
    static void Read(T *px, size_t offset)
    {
        for(int i = sizeof(T); --i >= 0;)
            ((uint8_t*)px)[i] = EEPROM.read(offset+i);
    }
    static T Read(size_t offset)
    {
        T x;
        for(int i = sizeof(T); --i >= 0;)
            ((uint8_t*)&x)[i] = EEPROM.read(offset+i);
        return x;
    }
};

///////////////////////////////////////////////////////////////////////////////////////
template <typename T1, typename T2>
void WriteEEPROM(const T1 &x, const T2*, size_t offset)
{
    StorageFn<T1, T2>::Write(x, offset);
}
template <typename T1, typename T2>
void ReadEEPROM(T1 *px, const T2*, size_t offset)
{
    StorageFn<T1, T2>::Read(px, offset);
}

template <typename T>
T GetEEPROM(size_t offset)
{
    return StorageFn<T, T>::Read(offset);
}

///////////////////////////////////////////////////////////////////////////////////////
#define WRITE_EEPROM(x,f)   WriteEEPROM(x, &((Layout*)0)->f, offsetof(Layout,f))
#define READ_EEPROM(px,f)   ReadEEPROM(px, &((Layout*)0)->f, offsetof(Layout,f))
#define GET_EEPROM(T,f)     GetEEPROM<T>(offsetof(Layout,f))

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteCurrentAnglePos(double angle)
{
    WRITE_EEPROM(angle, dCurrentAnglePos);
}

///////////////////////////////////////////////////////////////////////////////////////
bool EP_ReadCurrentAnglePos(double *pAngle)
{
    READ_EEPROM(pAngle, dCurrentAnglePos);
    if(!isfinite(*pAngle) && *pAngle != 0)
    {
        *pAngle = 0;
        return false;
    }
    else
        return true;    
}


///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteCapabilities(uint8_t capabilities)
{
    WRITE_EEPROM(capabilities, uCapabilities);
}

///////////////////////////////////////////////////////////////////////////////////////
bool EP_ReadCapabilities(uint8_t *pCapabilities)
{
    READ_EEPROM(pCapabilities, uCapabilities);
    return true;
}


///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteSessionId(uint8_t sessionId)
{
    WRITE_EEPROM(sessionId, uSessionId);
}
bool EP_ReadSessionId(uint8_t *pSessionId)
{
    READ_EEPROM(pSessionId, uSessionId);
    return true;
}


///////////////////////////////////////////////////////////////////////////////////////
#define EP_DEFAULT_RPS  5.

///////////////////////////////////////////////////////////////////////////////////////
double EP_ReadFastRPS()
{
    uint8_t b = GET_EEPROM(uint8_t, uFastRPS);
    return (b == 0 || b > 18) ? EP_DEFAULT_RPS : double(b)/2.;
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteFastRPS(double val)
{
    WRITE_EEPROM(uint8_t(floor(val*2)), uFastRPS);
}

///////////////////////////////////////////////////////////////////////////////////////
static uint8_t ReadStepping(int addr, uint8_t uDef)
{
    uint8_t b = GetEEPROM<uint8_t>(addr);
    switch(b)
    {
    case EP_FULLSTEP:
    case EP_HALFSTEP:
    case EP_1_4_STEP:   return b;
    case EP_1_8_STEP:
    default:            return uDef;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
uint8_t EP_ReadFastStepping()
{
    return ReadStepping(offsetof(Layout, uFastStp), EP_1_8_STEP);
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteFastStepping(uint8_t val)
{
    WRITE_EEPROM(val, uFastStp);
}

///////////////////////////////////////////////////////////////////////////////////////
uint8_t EP_ReadSlowStepping()
{
    return ReadStepping(offsetof(Layout, uSlowStp), EP_1_32_STEP);
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteSlowStepping(uint8_t val)
{
    WRITE_EEPROM(val, uSlowStp);
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_WriteMotoAngle(bool fStart, double angle)
{
    if(fStart)
        WRITE_EEPROM(angle, dMotorStartAngle);
    else
        WRITE_EEPROM(angle, dMotorEndAngle);
}

///////////////////////////////////////////////////////////////////////////////////////
bool EP_ReadMotoAngle(bool fStart, double *pAngle, double defaultVal)
{
    if(fStart)
        READ_EEPROM(pAngle, dMotorStartAngle);
    else
        READ_EEPROM(pAngle, dMotorEndAngle);
    if(!isfinite(*pAngle) && *pAngle != 0)
    {
        *pAngle = defaultVal;
        return false;
    }
    else
        return true;
}
