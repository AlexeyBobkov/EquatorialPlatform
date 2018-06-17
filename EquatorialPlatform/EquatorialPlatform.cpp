/*
 * EquatorialPlatform.cpp
 *
 * Created: 10/28/2013 7:12:41 PM
 *  Author: Alexey
 */ 


#include <Arduino.h>

#include "EP_Motor.h"
#include "EP_Keypad.h"
#include "EP_Sound.h"
#include "EP_Storage.h"
#include "EP_Encoders.h"
#include "EP_Configuration.h"

///////////////////////////////////////////////////////////////////////////////////////
#define GEAR_RATIO              5 //40      // gear ratio
#define THREAD_PER_INCH         13      // treads per inch

//#define TOTAL_MOTOR_ROTATION    168310.795
//#define MOTOR_START_ANGLE       (-85373.55)

#define TOTAL_MOTOR_ROTATION    167755.05
#define MOTOR_START_ANGLE       (-85373.55)

#define MOTOR_END_ANGLE         (MOTOR_START_ANGLE+TOTAL_MOTOR_ROTATION)

double dMotorStartAngle = MOTOR_START_ANGLE;
double dMotorEndAngle   = MOTOR_END_ANGLE;

// angle calibration
double dMotorLastStartAngle, dMotorLastEndAngle;

///////////////////////////////////////////////////////////////////////////////////////
bool fMotorAngleSync = false;
double fastRPS = 5.;
uint8_t fastStepping = EP_1_8_STEP, slowStepping = EP_1_32_STEP;

///////////////////////////////////////////////////////////////////////////////////////
bool fMotorFastMotion = false;

///////////////////////////////////////////////////////////////////////////////////////
static void PrintMotorAngle(EP_Motor *m)
{
    //Serial.println(m->GetAngle());
}

///////////////////////////////////////////////////////////////////////////////////////
static void SetMotorAngle(EP_Motor *m, double angle)
{
    m->SetAngle(angle);
    EP_WriteCurrentAnglePos(angle);
    fMotorAngleSync = true;
}

///////////////////////////////////////////////////////////////////////////////////////
class FwdMotion : public EP_Motor::MotionType
{
public:
    //virtual
    boolean CanMove(const EP_Motor*) const      {return digitalRead(FWD_SWITCH_IPIN) == 0;}
    boolean IsForward() const                   {return true;}
    void MotorStarted(EP_Motor *m)              {}

    //virtual
    void MotorStopped(EP_Motor *m)
    {
        EP_Sound(400);
        m->Release();
        PrintMotorAngle(m);
        dMotorLastEndAngle = m->GetAngle();
        SetMotorAngle(m, dMotorEndAngle);
    }
};

///////////////////////////////////////////////////////////////////////////////////////
class RevMotion : public EP_Motor::MotionType
{
public:
    //virtual
    boolean CanMove(const EP_Motor*) const      {return digitalRead(REV_SWITCH_IPIN) == 0;}
    boolean IsForward() const                   {return false;}
    void MotorStarted(EP_Motor *m)              {}

    //virtual
    void MotorStopped(EP_Motor *m)
    {
        EP_Sound(400);
        m->Release();
        PrintMotorAngle(m);
        dMotorLastStartAngle = m->GetAngle();
        SetMotorAngle(m, dMotorStartAngle);
    }
};


///////////////////////////////////////////////////////////////////////////////////////
// geometry
#define h       503.15 //502.24
#define A       (-0.580333)         //-tan(37.3)^2
#define p2x     23.5
//#define p2x     0
#define p2y     503.15 //502.24
#define aN      (h-A*p2y)
#define dn      (p2x*(1+A))
#define angle2pos   (25.4/(360.*double(GEAR_RATIO)*double(THREAD_PER_INCH)))
#define angle2posA  (A*25.4/(360.*double(GEAR_RATIO)*double(THREAD_PER_INCH)))

// motor position to equatorial angle conversion (more geometry)
static double CalcEquAngle(double motorAngle)
{
    double den = motorAngle*angle2pos + dn;
    if(den == 0)
        return 0;

    double a = aN/den;
    double a2 = a*a;
    double b = (motorAngle*angle2posA + dn)/den;
    double res = sqrt(a2-b*b+1);
    res = (a > 0) ? (-a*b + res)/(a2+1) : (-a*b - res)/(a2+1);
    return asin(res);
}

///////////////////////////////////////////////////////////////////////////////////////
class TestMotion : public FwdMotion
{
    double afactor_;
public:
    TestMotion(double secPerRev) : afactor_(secPerRev/(2.*3.1415926536)) {}

    //virtual
    uint8_t GetStepStyle() const                {return EP_1_32_STEP;}
    boolean IsConstantSpeed() const             {return false;}

    //virtual
    double Angle2TimeSec(double angle) const
    {
        return CalcAngle2TimeSec(angle);
    }

    //virtual
    void MotorStarted(EP_Motor *m)
    {
        FwdMotion::MotorStarted(m);
        fMotorFastMotion = false;
    }

private:
    double CalcAngle2TimeSec(double angle) const
    {
        return CalcEquAngle(angle) * afactor_;
    }
};

///////////////////////////////////////////////////////////////////////////////////////
TestMotion sunMotion (24.*60.*60.);
TestMotion moonMotion(89428.32869);
TestMotion starMotion(86164.09053);

#ifdef MOTOR_OSCILLOSCOPE_DEBUG
#define FAST_SPEED 2.5
#define FAST_STEPPING EP_1_4_STEP
#else
#define FAST_SPEED 2.8
#define FAST_STEPPING EP_1_8_STEP
#endif

///////////////////////////////////////////////////////////////////////////////////////
class FastRevMotion : public RevMotion
{
public:
    //virtual
    uint8_t GetStepStyle() const                {return FAST_STEPPING;}
    boolean IsConstantSpeed() const             {return true;}
    double  Angle2TimeSec(double angle) const   {return -angle/360./FAST_SPEED;}

    //virtual
    void MotorStarted(EP_Motor *m)
    {
        RevMotion::MotorStarted(m);
        fMotorFastMotion = true;
    }
    //virtual
    void MotorStopped(EP_Motor *m)
    {
        RevMotion::MotorStopped(m);
        fMotorFastMotion = false;
    }
}
fastRevMotion;

///////////////////////////////////////////////////////////////////////////////////////
class FastFwdMotion : public FwdMotion
{
public:
    //virtual
    uint8_t GetStepStyle() const                {return FAST_STEPPING;}
    boolean IsConstantSpeed() const             {return true;}
    double  Angle2TimeSec(double angle) const   {return angle/360./FAST_SPEED;}

    //virtual
    void MotorStarted(EP_Motor *m)
    {
        FwdMotion::MotorStarted(m);
        fMotorFastMotion = true;
    }
    //virtual
    void MotorStopped(EP_Motor *m)
    {
        FwdMotion::MotorStopped(m);
        fMotorFastMotion = false;
    }
}
fastFwdMotion;

///////////////////////////////////////////////////////////////////////////////////////
EP_Motor motor(MOTOR_STEPS_PER_REV, STEP_OPIN, DIR_OPIN, MODE0_OPIN, MODE1_OPIN, MODE2_OPIN, ENABLE_OPIN, true);

///////////////////////////////////////////////////////////////////////////////////////
class CenterRevMotion : public FastRevMotion
{
public:
    //virtual
    boolean CanMove(const EP_Motor *m) const    {return FastRevMotion::CanMove(m) && m->GetAngle() > 0;}
    void MotorStopped(EP_Motor *m)
    {
        EP_Sound(50, 1000);
        m->Release();
        PrintMotorAngle(m);
        EP_WriteCurrentAnglePos(m->GetAngle());
        fMotorFastMotion = false;
    }
}
centerRevMotion;

///////////////////////////////////////////////////////////////////////////////////////
class CenterFwdMotion : public FastFwdMotion
{
public:
    //virtual
    boolean CanMove(const EP_Motor *m) const    {return FastFwdMotion::CanMove(m) && m->GetAngle() < 0;}
    void MotorStopped(EP_Motor *m)
    {
        EP_Sound(50, 1000);
        m->Release();
        PrintMotorAngle(m);
        EP_WriteCurrentAnglePos(m->GetAngle());
        fMotorFastMotion = false;
    }
}
centerFwdMotion;

uint8_t uSessionId;

///////////////////////////////////////////////////////////////////////////////////////
void setup()
{
    // read stored values
    fastRPS = EP_ReadFastRPS();
    fastStepping = EP_ReadFastStepping();
    slowStepping = EP_ReadSlowStepping();

    // interlock
    pinMode(FWD_SWITCH_IPIN, INPUT_PULLUP);
    pinMode(REV_SWITCH_IPIN, INPUT_PULLUP);
    
    fMotorAngleSync = false;
    double angle;
    EP_ReadCurrentAnglePos(&angle);
    motor.Setup();
    motor.SetAngle(angle);

    EP_EncodersSetup();

    EP_ReadSessionId(&uSessionId);
    EP_WriteSessionId(++uSessionId);

    //Serial.begin(9600);
    Serial.begin(115200);
    //delay(2000);

    //motor.Start(&fastFwdMotion);
}

///////////////////////////////////////////////////////////////////////////////////////
enum EP_KpdMode
{
    EP_NORMAL,
    EP_SRV_CMD,
    EP_SRV_CAPABILITIES
    //EP_SET_FASTRPS,
    //EP_SET_FASTSTEPPING,
    //EP_SET_SLOWSTEPPING
};    

static void CantStartFwd()
{
    EP_Sound();
    dMotorLastEndAngle = motor.GetAngle();
    if(!fMotorAngleSync)
        SetMotorAngle(&motor, dMotorEndAngle);
}

static void CantStartRev()
{
    EP_Sound();
    dMotorLastStartAngle = motor.GetAngle();
    if(!fMotorAngleSync)
        SetMotorAngle(&motor, dMotorStartAngle);
}

static bool FastMoveToCenter()
{
    if(motor.GetAngle() > 0)
        return motor.Start(&centerRevMotion);
    else
        return motor.Start(&centerFwdMotion);
}

/*
static void printEncoderValue(long val)
{
    if (val < 0)
        Serial.print("-");
    else
        Serial.print("+");

    unsigned long aval = abs(val);

    if (aval < 10)
        Serial.print("0000");
    else if (aval < 100)
        Serial.print("000");
    else if (aval < 1000)
        Serial.print("00");
    else if (aval < 10000)
        Serial.print("0");

    Serial.print(aval);
}
*/

///////////////////////////////////////////////////////////////////////////////////////
static void printHexEncoderValue(long val)
{
    byte buf[2];
    buf[0] = val - (buf[1] = val / 256) * 256;
    Serial.write(buf, 2);

    //byte high = val / 256;
    //byte low = val - high * 256;
    //Serial.write(low);
    //Serial.write(high);

    //static const char hex[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    //Serial.print(hex[(val >> 4)  & 0xf]);
    //Serial.print(hex[val         & 0xf]);
    //Serial.print(hex[(val >> 12) & 0xf]);
    //Serial.print(hex[(val >> 8)  & 0xf]);
}

///////////////////////////////////////////////////////////////////////////////////////
static void printHexEncoderValue32(long val)
{
    byte buf[4];
    buf[0] = val;
    buf[1] = val >> 8;
    buf[2] = val >> 16;
    buf[3] = val >> 24;
    Serial.write(buf, 4);
}

///////////////////////////////////////////////////////////////////////////////////////
static bool ProcessCommand(EP_KpdMode *pkpdMode, char inchar, bool fromKbd)
{
    switch(*pkpdMode)
    {
    case EP_NORMAL:
        switch(inchar)
        {
        case '1':   //star tracking
            if(!motor.Start(&starMotion))
                CantStartFwd();
            break;
        case '2':   // moon tracking
            if(!motor.Start(&moonMotion))
                CantStartFwd();
            break;
        case '3':   // sun tracking
            if(!motor.Start(&sunMotion))
                CantStartFwd();
            break;
        case '8':   // fast move to center
            if(!FastMoveToCenter() && fromKbd)
                EP_Sound(50, 1000);
            break;
        case '#':   // fast forward
            if(!motor.Start(&fastFwdMotion))
                CantStartFwd();
            break;
        case '0':   // stop
            motor.Stop();
            PrintMotorAngle(&motor);
            EP_WriteCurrentAnglePos(motor.GetAngle());
            fMotorFastMotion = false;
            break;
        case '*':   // fast backward
            if(!motor.Start(&fastRevMotion))
                CantStartRev();
            break;
        case 'A':   // service mode
            if(fromKbd)
                EP_Sound(50, 3000);
            *pkpdMode = EP_SRV_CMD;
            break;
        default:
            return false;
        }
        break;

    case EP_SRV_CMD:
        switch(inchar)
        {
        case '1':
            motor.TestEnable();
            if(fromKbd)
                EP_Sound(50, 2000);
            *pkpdMode = EP_NORMAL;
            break;
        case '2':
            if(fromKbd)
                EP_Sound(50, 2000);
            *pkpdMode = EP_SRV_CAPABILITIES;
            break;
        default:
        case 'D':
            if(fromKbd)
                EP_Sound(50, 2000);
            *pkpdMode = EP_NORMAL;
            break;
        }
        break;

    case EP_SRV_CAPABILITIES:
        switch(inchar)
        {
        case '0':   case '1':
        case '2':   case '3':
        case '4':   case '5':
        case '6':   case '7':
            EP_WriteCapabilities(inchar - '0');
            if(fromKbd)
                EP_Sound(50, 2000);
            *pkpdMode = EP_NORMAL;
            break;
        default:
        case 'D':
            if(fromKbd)
                EP_Sound(50, 2000);
            *pkpdMode = EP_NORMAL;
            break;
        }
        break;

    default:
        return false;
    }
    return true;
}

#define EQU_RES 0x8000
#define EQU_ANGLE_FACTOR (double(EQU_RES)/(2.*3.1415926536))

static void ReportCapabilities()
{
    uint8_t capabilities;
    if(!EP_ReadCapabilities(&capabilities))
        capabilities = 0;
    Serial.write(capabilities);
}

static bool SetSessionResolution(byte buf[], int, int)
{
    EP_SetAltEncoderResolution  (buf[0] + ((long)buf[1]) * 256);
    EP_SetAzEncoderResolution   (buf[2] + ((long)buf[3]) * 256);
    Serial.print("r");
}

static bool SetDefaultResolution(byte buf[], int, int)
{
    long lAlt = long(buf[0]) + long(buf[1]) * 256;
    long lAzm = long(buf[2]) + long(buf[3]) * 256;

    EP_WriteDefEncResolutionAlt (lAlt);
    EP_WriteDefEncResolutionAzm (lAzm);

    EP_SetAltEncoderResolution  (lAlt);
    EP_SetAzEncoderResolution   (lAzm);

    Serial.print("r");
}

///////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_BUF_SZ 8
static byte serialBuf[SERIAL_BUF_SZ];
static int serialBufCurr = 0;
static int serialBufWait = 0;
typedef bool (*SERIAL_FN)(byte buf[], int curr, int wait);
static SERIAL_FN serialFn;
static void SetSerialBuf(int bufWait, SERIAL_FN fn)
{
    serialBufWait = bufWait;
    serialBufCurr = 0;
    serialFn = fn;
}

///////////////////////////////////////////////////////////////////////////////////////
static void ProcessSerialCommand(EP_KpdMode *pkpdMode, char inchar)
{
    if(serialBufWait)
    {
        serialBuf[serialBufCurr++] = (byte)inchar;
        if(serialBufCurr >= serialBufWait)
        {
            byte buf[SERIAL_BUF_SZ];
            memcpy(buf, serialBuf, sizeof(serialBuf));
            int bufCurr = serialBufCurr;
            int bufWait = serialBufWait;
            serialBufCurr = serialBufWait = 0;
            serialFn(buf, bufCurr, bufWait);
        }
        return;
    }

    switch(inchar)
    {
    case 'z':
        // Dave Ek's format: set the encoder resolution for this session only
        SetSerialBuf(4, SetSessionResolution);
        break;

    case 'x':
        // set the persistent encoder resolution
        SetSerialBuf(4, SetDefaultResolution);
        break;

    case 'h':
        // Dave Ek's format: report the encoder resolutions
        printHexEncoderValue(EP_GetAltEncoderResolution());
        printHexEncoderValue(EP_GetAzEncoderResolution());
        break;

    case 'y':
        // Dave Ek's format: report the encoder positions
        printHexEncoderValue(EP_GetAltEncoderPosition());
        printHexEncoderValue(EP_GetAzEncoderPosition());
        break;

    case 'p':
        // Dave Ek's format: report the number of encoder errors and reset the counter to zero
        Serial.write(EP_GetEncoderErrorCount());
        break;

    case 'q':
        // report the equatorial angle resolution
        printHexEncoderValue(EQU_RES);
        break;

    case 'e':
        // report the equatorial angle
        {
            static long lastEquAngle = 0;
            if(!fMotorFastMotion)
                lastEquAngle = long(CalcEquAngle(motor.GetAngle())*EQU_ANGLE_FACTOR) + EQU_RES/2;
            printHexEncoderValue(lastEquAngle);
        }            
        break;

    case 'c':
        // report capabilities
        ReportCapabilities();
        break;

    case 's':
        // report capabilities and session id
        ReportCapabilities();
        Serial.write(uSessionId);
        break;

    case 't':
        printHexEncoderValue32(dMotorLastStartAngle*100);
        printHexEncoderValue32(dMotorLastEndAngle*100);
        break;

    case '1':   //star tracking
    case '2':   // moon tracking
    case '3':   // sun tracking
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':   // fast move to center
    case '#':   // fast forward
    case '0':   // stop
    case '*':   // fast backward
    case 'A':   // service mode
    case 'D':   // exit service mode
        if(ProcessCommand(pkpdMode, inchar, false) && *pkpdMode == EP_NORMAL)
            Serial.print("r");
        break;

    default:
        break;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    static EP_KpdMode kpdMode = EP_NORMAL;

    // motor
    bool safe = motor.Run();

    // keypad
    char inchar = EP_GetKey();
    if(inchar != 0)
        ProcessCommand(&kpdMode, inchar, true);

    // serial
    static int safeSkip = 0;
    if(Serial.available() && (safe || ++safeSkip > 10))
    {
        safeSkip = 0;
        inchar = Serial.read();
        ProcessSerialCommand(&kpdMode, inchar);
    }            
}
