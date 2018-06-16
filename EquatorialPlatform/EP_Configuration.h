/*
 * EP_Configuration.h
 *
 * Created: 10/28/2013 6:34:03 PM
 *  Author: Alexey
 */ 


#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_


///////////////////////////////////////////////////////////////////////////////////////
// motor configuration
#define STEP_OPIN               13      // step pin #
#define DIR_OPIN                12      // direction pin #
#define MODE0_OPIN              10      // mode0 pin #
#define MODE1_OPIN              11      // mode1 pin #
#define MODE2_OPIN              5       // mode2 pin #
#define ENABLE_OPIN             4       // enable pin #
#define FWD_SWITCH_IPIN         2       // forward motion switch pin #
#define REV_SWITCH_IPIN         3       // reverse motion switch pin #
#define MOTOR_STEPS_PER_REV     400     // motor steps per revolution

///////////////////////////////////////////////////////////////////////////////////////
// keypad configuration
#define KPD_COUNT_OPIN          3
#define KPD_RESET_OPIN          2
#define KPD_VALUE_IPIN          A1

///////////////////////////////////////////////////////////////////////////////////////
// sound
#define SOUND_PIN               A0

///////////////////////////////////////////////////////////////////////////////////////
// altitude/azimuth encoders
#define AZ_A_IPIN               6       // digital pin 6 = PD6
#define AZ_B_IPIN               7       // digital pin 7 = PD7

#define ALT_A_IPIN              8       // digital pin 8 = PB0
#define ALT_B_IPIN              9       // digital pin 9 = PB1

#endif /* CONFIGURATION_H_ */