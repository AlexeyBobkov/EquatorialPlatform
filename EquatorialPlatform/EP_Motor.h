/*
 * EP_Motor.h
 *
 * Created: 10/28/2013 6:35:45 PM
 *  Author: Alexey
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

///////////////////////////////////////////////////////////////////////////////////////
// step styles
#define EP_FULLSTEP     2
#define EP_HALFSTEP     3
#define EP_1_4_STEP     4
#define EP_1_8_STEP     8
#define EP_1_16_STEP    9
#define EP_1_32_STEP    10

#define EP_1_2_STEP     EP_HALFSTEP
#define EP_1_1_STEP     EP_FULLSTEP

//#define MOTOR_OSCILLOSCOPE_DEBUG
//#define MOTOR_OSCILLOSCOPE_DEBUG_PIN A1

///////////////////////////////////////////////////////////////////////////////////////
// motor class
class EP_Motor
{
public:
    EP_Motor(uint16_t stepPerRev, uint8_t stepPin, uint8_t dirPin, uint8_t mode0Pin, uint8_t mode1Pin, uint8_t mode2Pin, uint8_t enblPin, boolean enblHigh);

    // call once in setup()
    void    Setup();
    // call periodically in loop()
    // returns true if safe to do some long job
    bool    Run();

    // motion law
    class MotionType
    {
    public:
        virtual boolean CanMove(const EP_Motor *m) const    = 0;    // can the motor move?
        virtual boolean IsForward() const                   = 0;    // is it a forward (or backward) motion?
        virtual uint8_t GetStepStyle() const                = 0;    // step style
        virtual boolean IsConstantSpeed() const             = 0;    // is it constant speed motion?
        virtual double  Angle2TimeSec(double angle) const   = 0;    // convert motor angle (degree) to time mark (sec)

        virtual void    MotorStarted(EP_Motor *m)           = 0;    // action on motor started
        virtual void    MotorStopped(EP_Motor *m)           = 0;    // action on motor stopped
    };

    boolean Start(MotionType *mt);
    boolean IsMoving() const;
    void    Stop(boolean release = true);
    void    Release();

    boolean SetAngle(double angle);     // works only if motor is stopped
    double  GetAngle() const            {return angle_ + steps_*angleDiff_;}

    // test mode only!
    void    TestEnable();               // stop if running; enable

private:
    EP_Motor(const EP_Motor&);
    EP_Motor& operator=(const EP_Motor&);

    unsigned long AngleToMicros(double angle) const;
    void onestep();

    //configuration
    double          stepAngle_; // step angle
    uint8_t         stepPin_, dirPin_, mode0Pin_, mode1Pin_, mode2Pin_, enblPin_; // pins
    boolean         enblHigh_;  // enable level high?

    // state
    double          angle_;     // current angle (degree)
    long            steps_;     // steps done

    // state for current motion
    double          angleDiff_; // angle increment
    unsigned long   constant_;  // time increment for constant motion
    int             delayCnt_;
    MotionType      *mt_;       // motion object
    unsigned long   start_;     // start time, micro sec
    unsigned long   endSleep_;  // end of pause, micro sec

#ifdef MOTOR_OSCILLOSCOPE_DEBUG
    uint8_t         ocsDbgDel_, ocsDbgSync_;
#endif
};

#endif /* MOTOR_H_ */