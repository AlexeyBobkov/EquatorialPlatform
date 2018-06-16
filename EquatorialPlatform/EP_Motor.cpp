/*
 * EP_Motor.cpp
 *
 * Created: 10/28/2013 6:36:08 PM
 *  Author: Alexey
 */ 

#include "Arduino.h"

#include <limits.h>

#include "EP_Motor.h"

//#define ADD_STAT

///////////////////////////////////////////////////////////////////////////////////////
#define ENABLE_MOTOR            (enblHigh_ ? HIGH : LOW)
#define DISABLE_MOTOR           (enblHigh_ ? LOW : HIGH)

///////////////////////////////////////////////////////////////////////////////////////
EP_Motor::EP_Motor(uint16_t stepPerRev, uint8_t stepPin, uint8_t dirPin, uint8_t mode0Pin, uint8_t mode1Pin, uint8_t mode2Pin, uint8_t enblPin, boolean enblHigh)
        : stepAngle_(360./stepPerRev), stepPin_(stepPin), dirPin_(dirPin), mode0Pin_(mode0Pin), mode1Pin_(mode1Pin), mode2Pin_(mode2Pin), enblPin_(enblPin), enblHigh_(enblHigh), angle_(0), steps_(0), mt_(NULL)
{
#ifdef MOTOR_OSCILLOSCOPE_DEBUG
    ocsDbgDel_ = 0;
    ocsDbgSync_ = 0;
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_Motor::onestep()
{
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1);
    digitalWrite(stepPin_, LOW);

#ifdef MOTOR_OSCILLOSCOPE_DEBUG
    digitalWrite(MOTOR_OSCILLOSCOPE_DEBUG_PIN, (++ocsDbgSync_ & ocsDbgDel_) != 0 ? HIGH : LOW);
#endif
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_Motor::Setup()
{
    pinMode(enblPin_, OUTPUT);
    digitalWrite(enblPin_, DISABLE_MOTOR);
    pinMode(stepPin_, OUTPUT);
    digitalWrite(stepPin_, LOW);
    pinMode(dirPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
    pinMode(mode0Pin_, OUTPUT);
    digitalWrite(mode0Pin_, LOW);
    pinMode(mode1Pin_, OUTPUT);
    digitalWrite(mode1Pin_, LOW);
    pinMode(mode2Pin_, OUTPUT);
    digitalWrite(mode2Pin_, LOW);

#ifdef MOTOR_OSCILLOSCOPE_DEBUG
    pinMode(MOTOR_OSCILLOSCOPE_DEBUG_PIN, OUTPUT);
    digitalWrite(MOTOR_OSCILLOSCOPE_DEBUG_PIN, LOW);
#endif
}

#ifdef MOTOR_OSCILLOSCOPE_DEBUG
#define MAX_DELAY 0
#else
#define MAX_DELAY 7200
#endif

///////////////////////////////////////////////////////////////////////////////////////
boolean EP_Motor::Start(MotionType *mt)
{
    if(IsMoving())
        Stop();

    if(!mt || !mt->CanMove(this))
        return false;

    mt_ = mt;

    uint8_t style   = mt_->GetStepStyle();
    boolean fwd     = mt_->IsForward();

    uint8_t angleDel;
    uint8_t mode0, mode1, mode2;
    switch(style)
    {
    default:
    case EP_FULLSTEP:   angleDel = 1;  mode0 = LOW;  mode1 = LOW;  mode2 = LOW;  break;
    case EP_HALFSTEP:   angleDel = 2;  mode0 = HIGH; mode1 = LOW;  mode2 = LOW;  break;
    case EP_1_4_STEP:   angleDel = 4;  mode0 = LOW;  mode1 = HIGH; mode2 = LOW;  break;
    case EP_1_8_STEP:   angleDel = 8;  mode0 = HIGH; mode1 = HIGH; mode2 = LOW;  break;
    case EP_1_16_STEP:  angleDel = 16; mode0 = LOW;  mode1 = LOW;  mode2 = HIGH; break;
    case EP_1_32_STEP:  angleDel = 32; mode0 = HIGH; mode1 = LOW;  mode2 = HIGH; break;
    }

    angleDiff_  = fwd ? (stepAngle_/angleDel) : -(stepAngle_/angleDel);     // angle increment
    constant_   = mt_->IsConstantSpeed() ? AngleToMicros(angleDiff_) : 0;   // time increment for constant speed
    start_      = micros() - AngleToMicros(angle_);                         // start time
    delayCnt_   = MAX_DELAY;

    // increment angle and calculate time to next step
    steps_ = 1;
    endSleep_ = start_ + AngleToMicros(angle_ + angleDiff_);

    digitalWrite(dirPin_,   fwd ? HIGH : LOW);
    digitalWrite(mode0Pin_, mode0);
    digitalWrite(mode1Pin_, mode1);
    digitalWrite(mode2Pin_, mode2);
    digitalWrite(enblPin_,  ENABLE_MOTOR);

#ifdef MOTOR_OSCILLOSCOPE_DEBUG
    ocsDbgDel_ = (angleDel<<1);
    ocsDbgSync_ = 0;
    digitalWrite(MOTOR_OSCILLOSCOPE_DEBUG_PIN, LOW);
#endif

    mt_->MotorStarted(this);

    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
boolean EP_Motor::IsMoving() const
{
    return mt_ != NULL;
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_Motor::Stop(boolean release)
{
    if(mt_)
    {
        mt_ = NULL;
        angle_ += steps_*angleDiff_;
        steps_ = 0;
    }
    if(release)
        Release();
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_Motor::Release()
{
    if(!IsMoving())
        digitalWrite(enblPin_, DISABLE_MOTOR);
}

///////////////////////////////////////////////////////////////////////////////////////
void EP_Motor::TestEnable()
{
    Stop();
    digitalWrite(mode0Pin_, LOW);
    digitalWrite(mode1Pin_, LOW);
    digitalWrite(mode2Pin_, LOW);
    digitalWrite(enblPin_, ENABLE_MOTOR);
    onestep();
}

///////////////////////////////////////////////////////////////////////////////////////
boolean EP_Motor::SetAngle(double angle)
{
    //Serial.print("SetAngle()");

    // works only if motor is stopped
    if(IsMoving())
        return false;

    //Serial.print(" old = ");
    //Serial.print(angle_);
    angle_ = angle;
    //Serial.print(" new = ");
    //Serial.println(angle_);
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
#ifdef ADD_STAT
// statistics
static unsigned long t_max = 0, t_min = ULONG_MAX, t_sum = 0;
static unsigned long s_max = 0, s_min = ULONG_MAX, s_sum = 0;
static int cnt = 0, scnt = 0;
static unsigned long prev = 0;
#define MAX_CNT 3000

static void add_statistics(unsigned long us, unsigned long t)
{
    if(t_min > t)
        t_min = t;
    if(t_max < t)
        t_max = t;
    t_sum += t;

    if(prev != 0)
    {
        unsigned long st = us - prev;
        if(s_min > st)
            s_min = st;
        if(s_max < st)
            s_max = st;
        s_sum += st;
        ++scnt;
    }
    prev = us;

    if(++cnt == MAX_CNT)
    {
        Serial.print("cnt=");
        Serial.print(cnt);
        Serial.print(" t_min=");
        Serial.print(t_min);
        Serial.print(" t_max=");
        Serial.print(t_max);
        Serial.print(" t_ave=");
        Serial.print(double(t_sum)/cnt);
        Serial.print(" scnt=");
        Serial.print(scnt);
        Serial.print(" s_min=");
        Serial.print(s_min);
        Serial.print(" s_max=");
        Serial.print(s_max);
        Serial.print(" s_ave=");
        Serial.println(double(s_sum)/scnt);

        s_max = t_max = 0;
        s_min = t_min = ULONG_MAX;
        s_sum = t_sum = 0;
        scnt = cnt = 0;
        prev = 0;
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////
bool EP_Motor::Run()
{
    //Serial.print("micros()=");
    //Serial.println(micros());

    if(!mt_)
        return true;

    if(!mt_->CanMove(this))
    {
        MotionType *mt = mt_;
        Stop(false);
        mt->MotorStopped(this);
        return true;
    }

    if(micros() - endSleep_ >= ULONG_MAX/2)
        return false;

    int cnt = 20;
    do
    {
        // make step

#ifdef ADD_STAT
        unsigned long vSec = micros();
#endif

        onestep();

#ifdef ADD_STAT
        unsigned long vSec2 = micros();
#endif

        // increment angle and calculate time to next step
        ++steps_;
        if(constant_ == 0)
            endSleep_ = start_ + AngleToMicros(angle_ + steps_*angleDiff_);
        else
        {
            if(delayCnt_ > 0)
            {
                endSleep_ += (unsigned long)(double(constant_)*(1.+5.*double(delayCnt_)/double(MAX_DELAY)));
                --delayCnt_;
            }            
            else
                endSleep_ += constant_;
        }        

#ifdef ADD_STAT
        //add_statistics(vSec, AngleToMicros(angleDiff_));
        add_statistics(endSleep_, vSec2-vSec);
        //add_statistics(vSec, vSec2-vSec);
#endif
    }    
    while(micros() - endSleep_ < ULONG_MAX/2 && --cnt >= 0);
    return true;
}

///////////////////////////////////////////////////////////////////////////////////////
unsigned long EP_Motor::AngleToMicros(double angle) const
{
    double time = mt_->Angle2TimeSec(angle)*1000000;
    return (unsigned long)(time - floor(time/ULONG_MAX)*ULONG_MAX);
}

