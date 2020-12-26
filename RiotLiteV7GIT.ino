// RiotLiteV7GIT
// Senser (02/17/2020)
// GIT: 2020-12-26
// (c) copyright 2020 by Robert Senser
/*

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
//
const int VERSION = 7;
//
// 2020-10-19: F3 version (trails ~~ RiotAPMxx)
// 1) Intended for final "Lite" flights
// 2) Needs to be calibrated for plane
//
// 2020-09-17: F2 version (some fixes, after flight ltest 9/15
// 1)  Timer for autolock too long, changed to 5000
// 2)  Find error in code to hold lock, appears to randomly toggle on and off when should be on  ((uhg))
//     Cause: Looks like once lock is set, it can easily be unset if ???
//
//     These changes need porting to APM version!
//
// 2020-09-12: FI version
//
// 2020-06-28: Backport minor Logger fix from RiotApmV4
//
// 2020-06-22: More tuning for Riot-III test
//              * after flight: less climb in terms need, more turn intensity, 2020-06-22b
//              *               longer action timeout, go to 15 secs,
//              *               move log interval to 2 secs, from 10 secs
//              * less climb on turns (1/3 less)
//              * No - set *action* timeout to 8000 (in mission.h)
//
// 2020-06-20: Restart for summer testing
//             Setup for flight, adjust throws, etc.
//
// 2020-03-05: Created from V5
//             Remove code replaced by "wigwam2"
//             Enhance logging to show panic/debug messages
//
// 2020-02-25: Add "wigwam2" rudder controller
//             simpified rudder driver for rudderController Class
//
// 2020-02-17: Created from V4
//             Simplify: fly slower and just on existing course
//             Goal: split determine needed rudder action
//                   from performing rudder action
//                   see navigate() method
//                   see manageRudder() method
//
// 2019-11-03:  Loaded to RiotIII (old Riot 2),
//              delta multiplier set to 2
//
// 2019-10-22: Debug "DynoDog" code
//             Add be Sloppy and 2nd slop zone (to avoid noise)
//
// 2019-10-21: Change rudder code to use approach from "DynoDogSymmetic"
//             proof of concept code, which uses rate-of-change-heading
//             to modulate turns
//
// 2019-10-20: Simplify getting rate-of-change (heading) by just subtracting 
//             last heading.
//
// 2019-10-14: Start changes to change rudder management to use rate-of-change
//             need to know heading rate of change
//
// 2019-10-09: Bring back '$' to enable GPS data recording (MD_GPS and friends...)
//             Temp return to long logging interval (every 5 secs, not 2)...
//             Also, timer fix. Grrr....
//
// 2019-09-25: Add just so rudder does not move when auto on
//             and make first right/left rudder movement smaller
//
// 2019-09-24: Looks like *physics* causes an over shoot.  The *high* turn rate is too
//             aggressive so we will have wider ranges for OK, off course, and really off course
//
// Arduino Tools Settings:
// BOARD: Pro Trinket V5/16 MHz (USB) 
// ISP: USBtinyISP
//
//  ################ PLANE settings  ################
// 
#undef P_TESTBED
#undef P_RIOTIII
#undef P_RIOT3B
#undef P_RIOT4
#undef P_RIOTX
#undef PLANE_NAME
//
#include "plane.h"
//
#ifndef P_TESTBED
#ifndef P_RIOTIII
#ifndef P_RIOT3B
#ifndef P_RIOT4
#ifndef P_RIOTX
#error No_plane, barf, barf -- no plane defined
#endif
#endif
#endif
#endif
#endif 
//          
// ############# end PLANE settings  ################
//
// NO CHANGE to these constancts, they are used in the mission settings...
const int DYN_VAL = -9998;               // do not change this :)
const int MIS_END = -9997;               // do not change this :)
const int A_LOCK = 1;         // LOCK OUT SWTICH             // do not change this :)
const int A_TEMP = 2;         // MOTOR OFF UNTIL SWITCH OFF  // do not change this :)
const int NOT_IN_USE = -9999;            // do not change this :)
// modes of operation:
const int O_PASSTHRU = 0;           // for manual flight
const int O_PASSTHRU_CALIBRATE = 1; // NO FLIGHT!!
const int O_HOLD = 2;               // for flight, when engaged, follow plan
// END NO CHANGE ^^^^^
//
//  ################ MISSION settings  ################
//    
struct stepMission {
  int mHeading;      // degrees or DYN_VAL
  int mAltitude;     // feet or DYN_VAL
  unsigned long mDuration;    // Duration in minutes, 0 is no limit on step  
};        
struct planeMission {
  unsigned int mActionTimer;  // milliseconds
  int mAction;       // see SERVO4 code
  int mMaxAGLinFeet; // in feet
  int mMode;         // See O_xxx values in code
  char mName[16];    // Mission/Plane name 
  int mAltitudeJump; // feet to rise when engaged
  int mSteps;        // Mission Steps, often 1, max 4
  stepMission mStep[4];
}; 
//
// ############# start MISSION settings  ################
//   
#include "mission.h"  
//          
// ############# end MISSION settings  ################
//
// ############# TESTING & SERVO CALIBRATION settings  ################
#undef P_DELTA_GROUND_ONLY
#undef P_TEST_SKIP_FAIL_CLIMB
#undef TEST_ALT_MANUAL_MODE
#undef UNO
// For Testing:
// P_DELTA_GROUND_ONLY: override plane's servo delta settings for ground testing & shorten logging timer
// #define P_DELTA_GROUND_ONLY
// P_TEST_SKIP_FAIL_CLIMB: prevent failure to climb override
// #define P_TEST_SKIP_FAIL_CLIMB  
// TEST_ALT_MANUAL_MODE: simulate different altitudes from stationary, ground level
// #define TEST_ALT_MANUAL_MODE
// UNO: Configure for Arduino UNO
// #define UNO
//
// these two used to help calibrate servos
#undef C_SETUP_1
#undef C_SETUP_2
// C_SETUP_1 shows the raw servo input data, in a tight loop
// C_SETUP_2 disables the 'delta' inputs so autopilot is stalled 
// and is used to verify switch from manual to auto does not change rudder and elevator positions
// #define C_SETUP_1
// #define C_SETUP_2
//
// ############# CALIBRATION notes for servos  ################
// HORROR: servos must be/can be manually calibrate for each radio & servo (that is plane) combination  <---- HORROR!
// servo calibration
// see ZZCALIBRATE below for *notion* of table to populate with calibration values:
// 1) Use '#define C_SETUP_1' (below) to find the needed "timer" (angle) readings by each servo:
//    * min, middle, max.  Have an accurate middle is critical.
//    * use these to create new ServoCalibration entries for each servo
//    * be sure to understand the different between timer and angle entries :)
// 2) Put first entries in table (for timer values, guess at angles)
// 3) Use '#define C_SETUP_2' to find the actual "angles" for each servo
//    * This setting zeros the 'delta' values
//    * Makes it possible to read the actual throw angles
//    * Makes it possible to see the servo behavior when autopilot engaged 
// 4) Use P_DELTA_GROUND_ONLY to increase deltas for sanity check of throw directions
//
// DO NOT FLY PLANE with C_SETUP_1 or C_SETUP_2 engaged!
//
// ############# end CALIBRATION notes for servos  ################
// ############# end TESTING & SERVO CALIBRATION  ################
//
// ############# DELTA settings  ################
// delta management: These are the control throw values for various actions
//
// when flying, delta should be at most "one or two marks" on the R/C transmitter stick :)
//
#ifdef P_DELTA_GROUND_ONLY 
// these are wildly exagerated for testbed, ground testing only
const int ELEVATOR_FULL_DELTA = 20; 
const int ELEVATOR_CLIMB_DELTA = 10; 
const int RUDDER_FULL_DELTA = 10;  
const int THROTTLE_FULL_DELTA = 20; 
#else
// actual flying values:
const int ELEVATOR_FULL_DELTA = 6; // want 6 to be "one mark", was 20; 
// DEBUG DEBUG WIGWAM  -- for ground pre test
// const int ELEVATOR_CLIMB_DELTA = 4; 
const int ELEVATOR_CLIMB_DELTA = 2; // was 1, 2020-06-22
const int RUDDER_FULL_DELTA = 4; // 2019-09-25: back to 4, up to 5, was 4; was 3;
const int THROTTLE_FULL_DELTA = 4; // 20; 
#endif

const int GLIDE_DELTA_MULTIPLIER = 5; // up elevator factor when power out, was 2, 3, 4 (2019-06-05)
// DEBUG DEBUG WIGWAM  -- for ground pre test
const int HEAVY_CLIMB_DELTA_MULTIPLIER = 3; // heavy climb multiplier, use with
// const int HEAVY_CLIMB_DELTA_MULTIPLIER = 2; // heavy climb multiplier, use with
                                            // (sharp) turns
// const int RUDDER_HARD_MULTIPLIER = 2;
// const int RUDDER_HARD_ADDER = 3; // 2019-07-17, was 2, now 3!
const int FAILURE_TO_CLIMB_MSECS = 15000; // *watch dog* timmer in ms 

// SLOP factors -- these are swags ....
// 2019-09-24: try very loose values so that *overshooting* is less of an issue
const int HEADING_MAX_SLOP = 3; // was 7 2020-03-08; was 3 2019-09-23       // max slop in degrees -- either way
// DEBUGZZ
const int HEADING_PHASE_1_DIFFERENCE = 6; // was 10; 2020-03-08 // was 22;    // was 6; 2019-09-24 // over this far off course in degrees, try harder :)
const int HEADING_PHASE_2_DIFFERENCE = 25; // was 45;   // was 12; 2019-09-24 // over this far off course in degrees, try much harder :)
const int DESIRED_ALTITUDE_SLOP_OVER = 10;   // in feet
const int DESIRED_ALTITUDE_SLOP_UNDER = 10;  // in feet
//
// ############# end DELTA settings  ################
//
// ---------------------------------------------------------------------------------------------
//
// for brute force bit banger -- does not use interrupts as they are already in use
#include "BitBanger.h"

// for SFE_BMP180 (altitude)
#include <SFE_BMP180.h>
SFE_BMP180 pressure;

// for IC2
#include <Wire.h>
// *grins* NAV_MODE_1 went the way of the (Ford) Edsel!
#define NAV_MODE_2
#ifdef NAV_MODE_2
// for the MPU9255 Library
#include <MPU9255.h>// include MPU9255 library
MPU9255 mpu;
struct gyroData {
  long gStamp;
  float gHeading;
// int heading360;
  float gSwag;
  bool gCalibrated;
  float gAvelocity;
};
gyroData gyro;
#endif
///
#define serialPrintString(sss) {  serialPrint(sss); }
#define serialPrintStringln(sss) {  serialPrintln(sss); }

// these next 2 used to selectively turn off RUDDER and ELEVATOR auto function
// needed ony for exotic testing...
const int O_RUDDER_ACTIVE = 1;
const int O_ELEVATOR_ACTIVE = 1;
// End of TESTING TESTING TESTING
//
const int SC_LOWER = 0;
const int SC_LOBLIP = 1;
const int SC_MIDDLE90 = 2;
const int SC_HIBLIP = 3;
const int SC_UPPER = 4;
struct ServoCalibration {
  int preReceiverOffet;
  int receiverValues[5];
  int blipAngle;
  int minRangeAngle;
  int maxRangeAngle;
  int minClipAngle;
  int maxClipAngle;
}; 

// config servo settings
#ifdef P_TESTBED
// testbed with manual pots for radio!
// ZZCALIBRATE
//                                            6/21/2017 measurements!
//                                            ---------- timer-----------  ***  ^range^  ^clip^
//                                            pre low mid-  mid  mid+ hgh, blp, min, max min, max 
const ServoCalibration servoCalbElevator   = {0,  0,   80,   80,  80, 203, 0,    40, 150,  0, 179};  // 118
const ServoCalibration servoCalbRudder    =  {0,  0,   79,   79,  79, 216, 0,    30, 140,  0, 179};   // 125
const ServoCalibration servoCalbThrottle  =  {0,  0,   91,   91,  91, 213, 0,    30, 150,  0, 179};   // 116
#endif
#ifdef P_RIOTX  || P_RIOTIII
// For RiotIII plane (only):
//                                            6/21/2017 measurements!
//                                            ---------- timer ----------  ***  ^range^  ^clip^
//                                            pre low mid-  mid  mid+ hgh, blp, min, max min, max 
const ServoCalibration servoCalbElevator   = { 5,  0,   85,   85,  85, 177, 0,    50, 130,  0, 179};   // pre is 2 was 1,   minus lowers elevator
//   changed 2019-09-26 to center rudder exactly at switch to auto
// const ServoCalibration servoCalbRudder    =  {11,  0,   87,   87,  87, 177, 0,    50, 130,  0, 179};   // pre is 12?, minus moves rudder to left, looking back to front
const ServoCalibration servoCalbRudder    =  {11,  0,   88,   88,  88, 177, 0,    50, 130,  0, 179};   // pre is 12?, minus moves rudder to left, looking back to front
                                                                                                       // SEPT17 12 -> 13
const ServoCalibration servoCalbThrottle  =  { 5,  0,   86,   86,  86, 177, 0,    50, 130,  0, 179};   // pre is 5,    minus slows motor 
#endif
#ifdef P_RIOT3B
// not used -- no controller onboard :)
#error error error
// these values are just copies
//                                            6/21/2017 measurements!
//                                            ---------- time -----------  ***  ^range^  ^clip^
//                                            pre low mid-  mid  mid+ hgh, blp, min, max min, max 
const ServoCalibration servoCalbElevator   = {0,  0,   80,   80,  80, 203, 0,    40, 150,  0, 179};  // 118
const ServoCalibration servoCalbRudder    =  {0,  0,   79,   79,  79, 216, 0,    30, 140,  0, 179};   // 125
const ServoCalibration servoCalbThrottle  =  {0,  0,   91,   91,  91, 213, 0,    30, 150,  0, 179};   // 116
#endif
#ifdef P_RIOTXNO
// For RiotX plane (only):
// these values are just copies of above
//                                            6/21/2017 measurements!
//                                            ---------- timer ----------  ***  ^range^  ^clip^
//                                            pre low mid-  mid  mid+ hgh, blp, min, max min, max 
const ServoCalibration servoCalbElevator   = { 5,  0,   85,   85,  85, 177, 0,    50, 130,  0, 179};   // pre is 2 was 1,   minus lowers elevator
const ServoCalibration servoCalbRudder    =  {11,  0,   87,   87,  87, 177, 0,    50, 130,  0, 179};   // pre is 12?, minus moves rudder to left, looking back to front
                                                                                                       // SEPT17 12 -> 13
const ServoCalibration servoCalbThrottle  =  { 5,  0,   86,   86,  86, 177, 0,    50, 130,  0, 179};   // pre is 5,    minus slows motor 
#endif
//
// LOGGING 
const int LOGARRAY_SIZE = 4;
// ZZTIMERS
const int LEDTIMER_VALUE = 500; // .5 seconds
#ifdef P_DELTA_GROUND_ONLY
const int LOGGINGTIMER_VALUE = 8000 / LOGARRAYSIZE; // 2 seconds
const int LOGGINTTIMERGPS_VALUE = 200; //  (must be < .5 * LOGGINGTIMER_VALUE)
#else
// DEBUG DEBUG 2019-06-13: speed up logging to 2 seconds!  GPS turned off way below
// only every 4th log call results in output.  so .5 * 4 -> 2 seconds
// QQHERE
// const int LOGGINGTIMER_VALUE = 10000 / LOGARRAY_SIZE; // 5000 // 2019-10-09 500 // 2000; // 5000; // 5 seconds should be 10 later
const int LOGGINGTIMER_VALUE = 2000 / LOGARRAY_SIZE; // every 2 secs!!! 2020-06-22b
const int LOGGINTTIMERGPS_VALUE = 3000;  // 2000 //  2 seconds (must be < .5 * LOGGINGTIMER_VALUE)
#endif
const int ELEVATORTIMER_VALUE = 100; // .1 was .5 was 2 seconds
const int RUDDERTIMER_VALUE = 100; // back to .1 (100) 50; // .05 was .1 was .5 was 2 seconds
// this should be a multile of RUDDERTIMER_VALUE..
const int STAGEDRUDDERTIMER_VALUE = 350;    // .35 sec
const int STAGEDRUDDER_FACTOR = STAGEDRUDDERTIMER_VALUE / RUDDERTIMER_VALUE;
const int THROTTLETIMER_VALUE = 250; // 500; // .5 was 2 seconds
//
// const int MANUALACTIONTIMER_VALUE = 100; // .1 sec
//
const int FULLTHROTTLE_MARKER_VALUE = 120; // degrees ?
const int OFFTHROTTLE_MARKER_VALUE = 45;   // degrees ?
//
      int SER = 0;
const int LED = 1;         // must be on!!!
      int LOGGING = 0;     // requires SER
      int VERBOSE = 0;     // verbose logging, requires SER
      int DASHBOARD = 0;   // requires SER
      int BT = 0;
      int COMPASS = 0;
      int ALTIMETER = 0;
      int THROTTLE = 0;
      int SERVO4 = 0;      // manage UAV actions   
      int INTR = 0;
      int MAG_CALB = 0;
// ***********************************

// ===================================================
// pins:
// Note: pins 2 and 7 should NOT be used!  ((Due to Trinket limits)
// radio input: 3, 4, 5, 6
//              PIN_BASE, NUM_CHANNELS, and (saddly) hard-coded constants
// serial/BT: Trinket: 0,1 (RX, TX labels); UNO 0, 1, (RX, TX) 12 (RX2)
//                    SerialRX_BT, SerialTX_BT, and BitBanger.h (if used)
// SDA, SCL: Trinket: A4, A5; UNO 18,19 (ADA, SCL)
//                    controlled by wire library
// actual servos: 9, 10, 11
//         Servo1Pin .. Servo3Pin   
// LEDs: 13, 8
//      LEDPin
//      ReadyLEDPin
// ===================================================

// SoftwareSerial (for BT access) had interrupt conflict!!!
// uses interrupts for ALL channel inputs

// just manages servos when UAV control active, not related to channel input
#include <Servo.h>
//
struct Configuration {
  // these two tend to vary
  int cDesiredHeading;
  int cDesiredAltitude;
  // these are computed
  int cMeasuredAltitudeAtEngagement;
  int cActualDesiredAltitude;
  int cMeasuredHeadingAtEngagement;
  int cActualDesiredHeading;  
  // the following tend to be constants
  int cHeadingSlop;
  int cRudderDelta;
  int cElevatorDelta;
  int cElevatorClimbDelta;
  int cThrottleDelta;
}; 
Configuration flightConfig;
//
// ############# CODE starts here  ################
//
void serialPrint( const char * str) {
  if (SER) {
    #ifdef UNO
    Serial.print(str);
    #else
      UART_print(str);
    #endif
  }
}

void serialPrintln(const char * str) {
  if (SER) {
    #ifdef UNO
    Serial.println(str);
    #else
    UART_println(str);
  #endif  
  }
}

void serialPrintInt(const int d) {
  if (SER) {
    #ifdef UNO
    Serial.print(d);
    #else
      char buff[32];
      sprintf(buff,"%d", d);
      UART_print(buff);
    #endif  
  }
}

void serialPrintIntln(const int d) {
  if (SER) {
    #ifdef UNO
    Serial.println(d);
    #else
      char buff[32];
      sprintf(buff,"%d", d);
      UART_println(buff);
    #endif  
  }
}

float getAngle(int duration) {
  // float a = ((duration - 900.0) / 1200.0) * 180.0;  // generic
  float a = ((duration - 1080.0) / 870.0) * 180.0;  // DX6i + Orange Receiver :)
  if (a < 0.0) a = 0.0;
  return a;
}
// pwmOnOff <---------------------------------------------------- on/off
// 
int pwmOnOff(int value, int flag, const char* comment) {  
  int ret = flag;
  if (value > 1600 && flag == 0) {     
    ret = 1;
  } else if (value < 1400  && flag == 1) { // was 1000
    ret = 0;          
  } 
  return ret;  
}

#ifdef NAV_MODE_2
//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
float process_angular_velocity(int16_t input) // , scales sensor_scale )
{
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  //for +- 250 dps
  // if(sensor_scale == scale_250dps)
  {
    return input/131;
  }
#if 0
  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }
#endif  
  return 0;
}
float calbGyro (int led) {
  pinMode(led,OUTPUT);  
  float sum = 0.0;
  float val;
  long times = 20000;
#undef SHOW
// #define SHOW
#ifdef SHOW
  float vHigh = -9999.0;
  float vLow = -vHigh;
#endif    
  for (long k=0; k < times; k++) {
    mpu.read_gyro();
    val = process_angular_velocity(mpu.gz); // ,scale_250dps); 
    sum += val;
#ifdef SHOW
    if (val > vHigh) vHigh = val;
    if (val < vLow) vLow = val;
    int v = 1000 * val;
    serialPrintIntln(v);
    delay(50);
#endif    
    int bits = (k / 100) % 2;
    digitalWrite(led, bits);    
  }   
  digitalWrite(led, LOW);
#ifdef SHOW
  int vH = 1000 * vHigh;
  int vL = 1000 * vLow;
  serialPrint("DEBUG: "); serialPrintInt(vH); serialPrint(","); serialPrintIntln(vL);
#endif  
  return (sum / times);
}

// ZZHEREHERE
float getHeading() {
  // note gStamp, gSwag, gAvelocity and gHeading are GLOBALS
  float gFraction;
  float gDuration;
  float gChange; 
  mpu.read_gyro();  
  // X & Y axis ignored...
  // Z axis
  gyro.gAvelocity = process_angular_velocity(mpu.gz); // degs / sec   
  gyro.gAvelocity -= gyro.gSwag; 
  // Oh boy... convert velocity to approx degrees
  gDuration = (double) (millis() - gyro.gStamp);  // length of cycle, milliseconds
  gyro.gStamp = millis();    
  gFraction = gDuration/1000.0;              // length of cycle, in seconds
  // gAvelocity from right above
  gChange = (gyro.gAvelocity * gFraction);       // just degrees 
  gyro.gHeading -= gChange; 
  // bug fix, 2019-07-23, correct both returned value and harbored gyro.gHeading
  // debug gyro.gHeading = ((float) ( ( ((int) gyro.gHeading) + 360) % 360) );
  // bug fix, 2019-07-24, simplier version perhaps hetter!
  if (gyro.gHeading < 0.0) gyro.gHeading += 360.0;
  else if (gyro.gHeading > 359.0) gyro.gHeading -= 360.0;
  return gyro.gHeading;   // turn into compass heading
}
#endif

//
// altitude from SFE_BMP180 library example sketch
float getAltitudeInMeters() {
  char status;
  double T,P,p0,a;
  const float ALTITUDE = 1655.0;  // Altitude of SparkFun's HQ in Boulder, CO. in meters  
  a = -9;
  status = pressure.startTemperature();
  if (status != 0) {
     delay(status); 
     status = pressure.getTemperature(T);
     // serialPrint("T: "); serialPrintln(T);
     if (status != 0) { 
       status = pressure.startPressure(3);
       if (status != 0) {
         delay(status);  
         status = pressure.getPressure(P,T);
         if (status != 0) {
          //  p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          p0 = 1013.25;
           a = pressure.altitude(P,p0);
         } else a = -1;      
       } else a = -2;
     } else a = -3;       
  } else a = -5; 
  return a;
}

// declare class:

// poor man's flight clock and timers
// * no use of interrupts
// * preset table size
// * very little error handling
//
class Xclock {
  private:
    const static int timerCnt = 11; // was 10;
    int nextTimerAvailable;
    long clockBasis = 0;
    long timerBasis[timerCnt];
    long timerDelta[timerCnt];
    bool timerActive[timerCnt];
  public:
  Xclock () {
    clockBasis = millis();
    nextTimerAvailable = 0;
  }

  long getCurrentTime() {
    return micros() - clockBasis;
  }

  int allocateTimer() {
    if (nextTimerAvailable < timerCnt) {
      int index = nextTimerAvailable;
      timerDelta[index] = 0;
      timerActive[index] = false;
      // changed 2019-10-09 to be simple
      nextTimerAvailable++;
      return index;
    } else {
      while (1) {
        return NOT_IN_USE; // dirty!
      }  
    }
  }

  void freeTimer(int _timer) {
    return;
  }

  void startTimer(int _timer, unsigned int _delta) {
    // serialPrint(">>>>>>>>>>>>>>>>>> in ST: ");    
    // serialPrintInt(_timer);
    // serialPrint(" ");     
    // serialPrintInt(_delta);    
    // serialPrintln("..");             
    timerBasis[_timer] = millis();
    timerDelta[_timer] = _delta;
    timerActive[_timer] = true;    
  }

  boolean checkTimer( int _timer) {
    boolean r = false;
#if 0      
      serialPrint("@ CT: ");
      serialPrintInt(_timer);
      serialPrint(" ");
      serialPrintInt(timerActive[_timer]);
      serialPrint(" ");    
      serialPrintInt(timerDelta[_timer]);   
      serialPrint(" ");    
      serialPrintInt(timerBasis[_timer]);
      serialPrint(" ");    
      serialPrintInt(millis());
      serialPrintln("..");
#endif     
    // 2019-10-09, added timerActive[_timer] &&    
    if (timerActive[_timer] && timerDelta[_timer] > 0 && (millis() > (timerBasis[_timer] + timerDelta[_timer]))) {
                    
      timerBasis[_timer] = 0;
      timerDelta[_timer] = 0;
      timerActive[_timer] = false;       
      r = true;
    }
    return r;
  }

  boolean checkTimerWithRestart(int _timer) {
    boolean r = false;
    // 2019-10-09, added timerActive[_timer] &&
    if (timerActive[_timer] && timerDelta[_timer] > 0 && (millis() > (timerBasis[_timer] + timerDelta[_timer]))) {
      timerBasis[_timer] = millis();
      timerActive[_timer] = true;        
      r = true;
    }
    return r;    
  }

  boolean isActive( int _timer) {
    return timerActive[_timer];
  }
};

// ZZLOGGER
const static int LogDataItemSize = 11;
struct LogDataItem {
  char data[LogDataItemSize];  // this goes to the radio 
};

class Logger {
  private:
  const static int LogBufferSize = 60;      // mex in TrackerPlane1 is 60!! So really 59!!
  const static int arraySize = 17;
  const static int overrideMsgSize = 11;
  char commandCnts[arraySize]; // note small(char) datatype, this is input
  const static int itemArraySize = LOGARRAY_SIZE;  
  LogDataItem commandItems[itemArraySize]; 
  unsigned char mode;
  boolean active;
  bool flipflop;
  int heading;
  int alt;
  int throttle;
  int agentActive;
  boolean override;
  char overrideMsg[overrideMsgSize];
#include "encrypt.h"  
  void logClear() {
    for (int i=0; i < arraySize; i++) { commandCnts[i] = 0; }
  }
  public:
  // DEBUG DEBUG move count to public and make long
    long int count;
    char makeDEC(int v) {
        if (v > 9) v = 9;     
        return ('0' + v);       
    }
    char makeHEX(int v) {
      if (v <10) return makeDEC(v);
      v -= 10; // Hex 'A' is (v-10) + 'A'
               // Hex 'B' is 1 + 'A', etc....
      if (v > 5) v = 5;      
      return ('A' + v);
    }
    enum LOGCOMMAND { 
      // keep in sync with arraySize !!
    logMode = 0,
    headingIndicator = 1,  
    // 4 for rudder
    rudderLeft = 2,      
    rudderStraight = 3,
    rudderRight = 4,
    rudderOther = 5,
    // 3 for elevator
    elevatorUp = 6,
    elevatorLevel = 7,    
    elevatorDown = 8,
    // 3 for motor
    motorIncrease = 9,
    motorBase = 10,    
    motorDecrease = 11,
    // 
    // 5 for flags --> 1 in actual log    
    motorOff = 12,
    failedToClimb = 13,
    panic = 14,
    pStep = 15,
    locked = 16};

    // mode bit options
    const static unsigned char MD_ALL = 0xff;
    const static unsigned char MD_GPS = 0x01;
    const static unsigned char MD_COMMANDS = 0x02;     
    const static unsigned char MD_ENCRYPT = 0x04;
    const static unsigned char MD_HDR = 0x08;

  Logger(unsigned char _mode) {
    mode = _mode;
    active = false;
    flipflop = false;
    count = 0;
    agentActive = 0;
    logClear();
    override = false;
  }
  void LogMsg(char* msg) {
    override = true;  // override normal log output
    int len = strlen(msg);
    if (len > (overrideMsgSize - 1)) {
      len = (overrideMsgSize - 1);
    }
    memcpy(overrideMsg, msg, len);
    overrideMsg[len] = '\0';
    return;
  }  
  void logAgent(int val) {
    agentActive = val;
  }
  // ignore val and just bump counter
  bool logAction (LOGCOMMAND cmd, int val) {
    int i = (int) cmd;
    if (i < arraySize) {
      commandCnts[i]++;
      if (commandCnts[i] > 99) commandCnts[i] = 99;
      return true;
    } else { 
      return false;
    }
  }
  // put val into array
  bool logAction2 (LOGCOMMAND cmd, int val) {
    int i = (int) cmd;
    if (i < arraySize) {
      commandCnts[i] = val;
      if (commandCnts[i] > 99) commandCnts[i] = 99;
      return true;
    } else {
      return false;
    }
  }  
  bool logReadings (float _heading, float _alt, int _throttle) {
    int i = count % itemArraySize;
    // set these from the first call / set of calls
    if (i == 0) {
      heading = (int) _heading;
      alt = (int) _alt;
	    throttle = _throttle;
    }
    return true;
  }
  bool logSendGPS() {
      if (mode & MD_GPS) {
        serialPrintln("@$$");  // this special code signals the
                               // smart phone to output the GPS
                               // data, which the radio processes, etc.
                               // poor man's airborn "LAN"
      }      
  }
        // keep these comments... out of date tho...
        // L: rudder Left
        // R: rudder Righ
        // H: rudder Other
        // u: elevator Up
        // d: elevator Down
        // o: elevator Other
        // +: motor Increase
        // -: motor Decrease
        // b: motor base
        // X: motor off
        // F: Failed-to-Climb
        // P: PANIC!
        // S: Step in flight
        // L: Locked
        //    
  //
  // totally rewritten 2019-06-16 to contain 4 control reading items per log record
  //
  bool logSend(Xclock& _flightClock, int _logGpsTimer) {
    bool ret = true;
    if (active) {       
      count++;     // this value drives FSM....        
      // first collect auto controller data items
      int i = count % itemArraySize;
      // 202-06-28: backport fix from RiotApmV4
      //            avoid junk character/data in commandCnts[0]
      if (!isprint(commandCnts[0])) { commandCnts[0] = ' '; }
      commandItems[i].data[((int) logMode)] = commandCnts[0];    // noHEX, is mode :)        
      commandCnts[0] = ' '; 
      // replaced code:     
      // commandItems[i].data[logMode] = commandCnts[0];    // noHEX, is mode :)  
      // commandCnts[0] = 0; // clear
      // stop at motorIncrease (likely used 9 ...)  
      for (int j=1; j < ((int) motorIncrease); j++) {
        commandItems[i].data[j] = makeHEX(commandCnts[j]);  // might copy 1 item too many..   
        commandCnts[j] = 0; // clear           
      }  
      // pack remainint into in one field
#if 1    
      int v =  (commandCnts[motorIncrease] > 0)?8:0;
          v += (commandCnts[motorBase] > 0)?4:0;
          v += (commandCnts[motorDecrease] > 0)?2:0;                    
          v += (commandCnts[motorOff] > 0)?1:0 ;
      int w =  (commandCnts[failedToClimb] > 0)?4:0;
          w += (commandCnts[panic] > 0)?2:0;
          w += (commandCnts[locked] > 0)?1:0;          
      commandItems[i].data[LogDataItemSize-2] = makeHEX(v);
      commandItems[i].data[LogDataItemSize-1] = makeHEX(w);      
#else
      commandItems[i].data[LogDataItemSize-2] = 'V'; 
      commandItems[i].data[LogDataItemSize-1] = 'W';             
#endif         
      if (i == (itemArraySize - 1)) { // when log record is full, transmit it
        // second output log record 
        long countS = count / itemArraySize;
        char buf[LogBufferSize];  // hum, not too big! was 50, was 32
        bool type2; 
        char *ptr = buf;
        char *ptr2 = 0;
        // keep these comments...
        //     X: '1' or '2'
        //   cnt: line count
        //   NO-   TS: timestamp
        //   HHH: heading in degrees
        // if '1'
        // =alta: alt in feet
        // if '2'
        //   THR: throttle (units not clear)
        //     a: M:manual, A:auto
        //  step:           
        *ptr++ = '{';
        if (flipflop) {
          type2 = false;
          *ptr++ = '1';
        } else {
          type2 = true;
          *ptr++ = '2';           
        }
        flipflop = !flipflop;
#if 0        
        // 5 digit timestamp
        long ts = millis() % 100000;
        itoa( ((ts / 10000) % 10),ptr++,10);
        itoa( ((ts / 1000) % 10),ptr++,10); 
        itoa( ((ts / 100) % 10),ptr++,10); 
        itoa( ((ts / 10) % 10),ptr++,10);      
        itoa( (ts % 10),ptr++,10);
#endif                
        // 3 digit count
        itoa( ((countS / 100) % 10),ptr++,10);
        itoa( ((countS / 10) % 10),ptr++,10);      
        itoa( (countS % 10),ptr++,10);    
        // 3 digit heading
        itoa( ((heading / 100) % 10),ptr++,10);
        itoa( ((heading / 10) % 10),ptr++,10);      
        itoa( (heading % 10),ptr++,10);
        if (type2) {
          // Agent
          // changed
          if (agentActive) {
            // *ptr++ = 'A';
            // need Step from command cnts...    
            *ptr++ =  makeHEX(commandCnts[pStep]);            
          } else {
            // *ptr++ = 'M';
            *ptr++ = 'X'; // no step is in use
          }
          // 3 digit throttle
          itoa( ((throttle / 100) % 10),ptr++,10);
          itoa( ((throttle / 10) % 10),ptr++,10);      
          itoa( (throttle % 10),ptr++,10);
        } else { // type1
          // 5 digit alt
          // 4 digit alt really
          // itoa( ((alt / 10000) % 10),ptr++,10);  // we fly down low...
          itoa( ((alt / 1000) % 10),ptr++,10); 
          itoa( ((alt / 100) % 10),ptr++,10); 
          itoa( ((alt / 10) % 10),ptr++,10);      
          itoa( (alt % 10),ptr++,10);
        }  
        ptr2 = ptr;
        // header takes { + 11 chars, so 12       
        if ((mode & MD_COMMANDS) && (countS > 0)) {
          for (int i=0; i < itemArraySize; i++) {
            for (int j=0; j < LogDataItemSize; j++) {  // kludge
              *ptr++ = commandItems[i].data[j];
            }
          }       
        } 
        if (override) {       
          for (int i=0; i < strlen(overrideMsg); i++) {
             *ptr2++ = overrideMsg[i];
          }                   
        }
        if (ptr2 > ptr) {
          ptr = ptr2;  // in the event there we no log blocks...          
        }
        *ptr++ = '}';    
        // *ptr++ = '\n';  // DEBUG DEBUG        
        *ptr++ = 0;  // Arnold...
        int len = strlen(buf);
        // let's clean up -- just in case
        for (int i=0; i < len; i++) {
          if (!isprint(buf[i])) buf[i] = '.';
        }
        encrypt(buf,len);
        serialPrintln(buf);  // ca= false;uses transmit!
        logClear();
        override = false;
        // QQHERE 
        // patch to put GPS radio data in separate buffer (via a delay...)
        // allows send of GPS data to be delayed (this is kinda tricky)
        // DEBUG DEBUG 2019-10-09, turn back on for testing, 2019-6-13: turn off GPS so can have faster timers for logging  
#if 1  
          // if (_flightClock.checkTimer(_logGpsTimer)) {
          //  logSendGPS();
          // }
        _flightClock.startTimer(_logGpsTimer, LOGGINTTIMERGPS_VALUE);
#endif                
      }         
    } else {
      ret = false;
    }
 
    return ret;
  }
  void logActive() {
    // express multiple times due to serial errors
    for (int i=0; i<3; i++) {
      serialPrintln("@on");
    }
    active = true;
  }
  void logInactive() {
    serialPrintln("@off");
    active = false;
  }
  boolean isActive () {
    return active;
  }
};
#ifdef P_TESTBED
// with headers and no encrypt
Logger fLog(fLog.MD_ALL - fLog.MD_ENCRYPT); // - fLog.MD_COMMANDS ); // - fLog.MD_GPS); // - Flog.MD_HDR);
#else
// no headers in logging
Logger fLog(fLog.MD_ALL - fLog.MD_ENCRYPT - fLog.MD_HDR ); // - fLog.MD_COMMANDS ); // - fLog.MD_GPS); 
#endif
//

// Xclock was here, moved earlier

// XServo is a thin wrapper for the Servo object; it makes the shifting
// between radio-driven and program-driven servo control more obvious.
// 
// The attachXServo() and detachXServo() methods manage the mode of operation
//
class XServo {
  private:
  int myPin;
  Servo myServo;
  boolean isAttached;
  int value;

  public:
  XServo(int _pin) {
    setPin(_pin);
    pinMode(myPin, OUTPUT);
    isAttached = false;      
  }

  void setPin(int _pin) {
    myPin = _pin;
  }
  int getPin(void) {
    return myPin;
  }

  void attachXServo( int _value){
    myServo.attach(myPin);
    isAttached = true;  
    value = _value;
    write(value);     
  };
  void detachXServo() { 
    myServo.detach();  
    isAttached = false;    
  };

  void write(int _value) {    
    if (isAttached) {
      value = _value;  
      myServo.write(value);  
    } 
  };

  void intrDigitalWrite(int _value) {
    // old PLAN_B area, for PLAN_B disable this write  
    if (!isAttached) {
      digitalWrite(myPin, _value); 
    }  
  }
}; // end of XServo class

class Controller {
  private:
  boolean active;
  int baseServoSetting;
  int baseServoDelta;
  int currentServoSetting;
  char servoName;
  XServo* servoPtr;
  
  public:
  Controller(XServo* _s, char _n) {
    active = false;
    servoPtr = _s;
    baseServoSetting = NOT_IN_USE;  // this is important :)
    baseServoDelta = NOT_IN_USE;
    currentServoSetting = NOT_IN_USE;
    servoName = _n;
  }
  void setActive(boolean _b) {
    active = _b;
    if (1) { // baseServoDela > NOT_IN_USE) { // important because of this, needed when testing :)
      if (active == true) {
        if (VERBOSE) {
          char buf[2];
          buf[0] = servoName;
          buf[1] = 0;
          serialPrint(buf);  
          serialPrint(": Servo Basis angle: ");  
          serialPrintInt(baseServoSetting);
          serialPrint(", Delta angle ");
          serialPrintIntln(baseServoDelta);         
        }
        servoPtr -> attachXServo(baseServoSetting);
      } else {
        servoPtr -> detachXServo();    
      }
    }
  }
  boolean getActive() {
    return active;
  }

  // ZZTRANS
  // enable linear transformation of receiver timmer units to servo.h class actual angle 
  int writeServoAngleTrans5(int _a, int _d, const ServoCalibration *p ) {
    int distanceRange;
    int distanceRec;
    float ratio;
    int computedAngle = 0;    
    int effectiveAngleRange;

    _a += (p -> preReceiverOffet);
    // cleanup input reading
    if (_a < (p -> receiverValues[SC_LOWER]) )  { _a = (p -> receiverValues[SC_LOWER]); }
    if (_a > (p -> receiverValues[SC_UPPER]) ) { _a = (p -> receiverValues[SC_UPPER]); }  
    //
    // WARNING: this is dangerous code to change while sleepy!
    //
    // Translate, two cases... blipAngle is zero or not zero
    if ((p -> blipAngle) == 0) { // only 2 ranges!
      if (_a <= (p -> receiverValues[SC_MIDDLE90])) { // range 1 of 2
        // these are values from receiver
        distanceRange = (p -> receiverValues[SC_MIDDLE90]) - (p -> receiverValues[SC_LOWER]);
        distanceRec = (p -> receiverValues[SC_MIDDLE90]) - _a;
        // ratio is the fraction of range <= 90 that is used
        ratio = ((float) distanceRec) / ((float) distanceRange); // ratio, used from recvr
        // effectiveAngleRange is the degrees used from 90 on down, varies by servo and recvr!
        effectiveAngleRange = 90 - (p -> minRangeAngle);
        // computerAngle is the angle <=90 that maps ratio onto the angle range fpr this servo
        computedAngle = 90 - ((int) (ratio * ((float) effectiveAngleRange)) );
      } else { // range 2 of 2
        // see comments above
        distanceRange = (p -> receiverValues[SC_UPPER]) - (p -> receiverValues[SC_MIDDLE90]);
        distanceRec = _a - (p -> receiverValues[SC_MIDDLE90]);
        ratio = ((float) distanceRec) / ((float) distanceRange);
        effectiveAngleRange = (p -> maxRangeAngle) - 90;
        computedAngle = 90 + ((int) (ratio * ((float) effectiveAngleRange)) );      
      }   
    } else { // full 4 ranges: blips provide inner ranges about middle
      int range = 0;
      if (_a >= (p -> receiverValues[SC_HIBLIP])) { range = 3; }
      else if (_a >= (p -> receiverValues[SC_MIDDLE90])) { range = 2; }
      else if (_a >= (p -> receiverValues[SC_LOBLIP])) { range = 1; } 
      switch (range) {
        case (0): // 0 to SC_LOGBLIP
          distanceRange = (p -> receiverValues[SC_LOBLIP]);
          distanceRec = _a;
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = ((int) (ratio * (90.0 - ((float) p -> blipAngle) ) ) );   
          break;    
        case (1): // SC_LOGBLIP to 90
          distanceRange =(p -> receiverValues[SC_MIDDLE90]) - (p -> receiverValues[SC_LOBLIP]) ;
          distanceRec = p -> blipAngle;
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = ((int) (ratio * ((float) p -> blipAngle)  ) ) + (90 - p -> blipAngle) ;         
          break;
        case (2): // 90 to SC_HIBLOIP
          distanceRange =(p -> receiverValues[SC_HIBLIP]) - (p -> receiverValues[SC_MIDDLE90]) ;
          distanceRec = p -> blipAngle;
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = 90 + ((int) (ratio * ((float) p -> blipAngle)  ) );         
          break;
        case (3): // SC_HIBLIP to 179 or 180
          distanceRange =(p -> receiverValues[SC_UPPER]) - (p -> receiverValues[SC_HIBLIP]) ;
          distanceRec = 90 - (p -> blipAngle);
          ratio = ((float) distanceRec) / ((float) distanceRange);
          computedAngle = 90 + ((int) (ratio * (90.0  - ((float) p -> blipAngle) ) ) );         
          break;     
      }
      //
    } 
    // add what is likely the delta value (adjust supplied by autopilot)
    computedAngle += _d;
    // cleanup output anngle range (limit final angle to a clip range -- presents bad-date induced crashes)
    if (computedAngle < (p -> minClipAngle) )  { _a = (p -> minClipAngle); }
    if (computedAngle > (p -> maxClipAngle) )  { _a = (p -> maxClipAngle); }
    // beam us out of here Scotty!
    writeServoAngle(computedAngle);
    return computedAngle;
  }
  // enable linear transformation of transmitter units to servo.h class angles 
  void writeServoAngleTrans(int _a, float _slope, int _offset, int _min, int _max) {
    float angle = _a;
    angle *= _slope;
    int iAngle = (int) angle;
    iAngle += _offset;
    if (iAngle < _min) iAngle = _min;
    if (iAngle > _max) iAngle = _max;
    writeServoAngle(iAngle);
  }
  
  void writeServoAngle(int _a) {
    servoPtr -> write(_a);
    currentServoSetting = _a;
#ifdef C_SETUP_2
    if (VERBOSE) {
      char buf[2];
      buf[0] = servoName;
      buf[1] = 0;
      serialPrint(buf);       
      serialPrint(" Servo Basis angle: ");  
      serialPrintInt(baseServoSetting);
      serialPrint(", Servo angle ");
      serialPrintIntln(_a);       
      }
#endif     
  }
  void setServoBase(int _b) {
    baseServoSetting = _b;
  }
  int getServoBase() {
    return baseServoSetting;
  }
  void setServoDelta(int _d) {
    baseServoDelta = _d;
  }
  int getServoDelta() {
    return baseServoDelta;
  }
  int getServoCurrent() {
    return currentServoSetting;
  }  
}; // end of Controller class

//ZZTC ZZTH
class ThrottleController: public Controller {
  private:
    int altitudeMode;
    boolean forceLand;
    int prevAltitude;
    bool failedClimb;
    int failedClimbCnt;
  public:
    const int MC_CLIMB = 0;
    const int MC_LEVEL = 1;
    const int MC_DESCEND = 2;
    const int MC_FORCE = 3;
  public:
  ThrottleController(XServo* _s): Controller(_s, 'M') {
    altitudeMode = MC_LEVEL;
    prevAltitude = -1;
    failedClimb = false;
    failedClimbCnt = 0;
  }  // so super ....

    void configure() {
      setServoDelta(flightConfig.cThrottleDelta);  //  in degrees
      setForce(false);  // this permits forced gliding without loosing altitude mode...       
      clrFailedClimb();      
      return;
    }  

    boolean getForce() {
      return forceLand;
    }
    void setForce(boolean _b) {
      forceLand = _b;      
    }
      
    void setThrottleSetting (int _s) {      
      altitudeMode = _s;     
    }
    int getSetting() {
      return altitudeMode;
    }

    void clrFailedClimb() {
      failedClimb = false;
      failedClimbCnt = 0;       
    }

    void setFailedClimb() {
      failedClimb = true;
      failedClimbCnt = 0;
      // serialPrintIntln(failedClimb);      
    } 

     bool getFailedClimb() {
       return failedClimb;
    }       
    
    int getFailedClimbCnt() {
      return failedClimbCnt;
    }
    void setFailedClimbCnt(int value) {
      failedClimbCnt = value;
    }

    // throttle
    void act(int curAltitude, int curSpeed) { 
      // curSpeed is (not valid)/useless :(
      int angle;
      // static int anglePrev = -1;
      int servoAngle = getServoBase();
      // default is level flight MC_LEVEL
      int delta = 0;
#ifdef C_SETUP_2  
#else
      // this is active code ....
      // forceLand used when end to A_TEMP or end of multi-step FlightPlan
      // MC_FORCE used with altitude firewall 
      // MC_FORCE can be unset, forceLand is set once
      if (forceLand || getSetting() == MC_FORCE) {
        // delta = 0;
        // 2018-08-18: try 33% power LOSS
        // 2020-06-21: go with 10% power loss
        // servoAngle = (2 * servoAngle) / 3;  // 0;   // not reversed!
        servoAngle = servoAngle - ( servoAngle / 10);
        fLog.logAction2(fLog.motorOff, 1);
        // turn off failed to climb 
        clrFailedClimb();            
      } else {
        // turn FORCE log marker off..
        fLog.logAction2(fLog.motorOff, 0); // means motor is ON!        
        // this odd code senses failure to climb (battery/motor failure),
        // we have a prev altitude, being asked to climb, and have not      
        if (prevAltitude > 0 &&
            getSetting() == MC_CLIMB && 
            prevAltitude >= curAltitude) {
#ifndef P_TEST_SKIP_FAIL_CLIMB            
// can turn off to avoid messing up the testing :) 
          failedClimbCnt++;
          if (failedClimbCnt > (FAILURE_TO_CLIMB_MSECS / THROTTLETIMER_VALUE)) { // declare failure at nn ms
            setFailedClimb(); 
          }          
#endif               
        } else {
          // fix a problem with noisy readings from altimiter :)  this is suspect code
          if (curAltitude > (prevAltitude + 11)) { // 11 feet is a swag...
            failedClimbCnt = 0;
          }
        }   
        prevAltitude = curAltitude;        
        // Arduino switch stmt has issues with MC ... constants!
        if (getSetting() == MC_CLIMB) {
          //
          // could add code here to increase delta when FailedClimb is present....  ZZFAILEDCLIMB
          delta = getServoDelta();
        } else if (getSetting() == MC_DESCEND) {
          delta = -getServoDelta();        
        }
        // failedClimb turn off when situation has changed
        if (getSetting() != MC_CLIMB) {       
          clrFailedClimb(); 
        }
      }    
#endif        
      angle = writeServoAngleTrans5(servoAngle, delta, &servoCalbThrottle);
      return;
    }
}; // end of ThrottleController

//ZZEC ZZEL ZZELEVATOR
 class ElevatorController: public Controller {
  private:
    int maxAltitude;
    int desiredAltitude;
    int minAltitude;
    int elevatorOverrideMode;
    ThrottleController* throttleController;

  public:
      const int EL_NORMAL = 1;
      const int EL_HARD_CLIMB = 2;// currently used only in toggled hard turn :)

  ElevatorController(XServo* _s, ThrottleController* _m): Controller(_s, 'E') {
    throttleController = _m;
    elevatorOverrideMode = EL_NORMAL;
  }  // so super ....

  void configure() {
    setServoDelta(flightConfig.cElevatorDelta);  // in degrees
    return;
  }  
  void setParameters(int _v1, int _v2, int _v3) {
    maxAltitude = _v1;
    desiredAltitude = _v2;
    minAltitude = _v3;         
  }
  void getParameters( int& _v1, int& _v2, int& _v3) {
    _v1 = maxAltitude;
    _v2 = desiredAltitude;
    _v3 = minAltitude;
  }
  void setOverrideMode(int _m) {
    elevatorOverrideMode = _m;
  }
  int getOverrideMode() {
    return elevatorOverrideMode;
  }
  
  void setSpecialElevatorParameters( int encodedBaseAltitude ) {
    // set hase altitude and bounds, watch special cases
    // done for EACH interation, sloppy but bug free
    int baseAltitude = encodedBaseAltitude;     
    if (encodedBaseAltitude == DYN_VAL) {
      baseAltitude = (flightConfig.cMeasuredAltitudeAtEngagement + mission.mAltitudeJump); // up <feet> to show engaged
    }
    // else {
    //  baseAltitude = mission.mStep[1-1].mAltitude;  // in feet
    // }   
    flightConfig.cActualDesiredAltitude = baseAltitude; // helpful for logging     
    setParameters(baseAltitude + DESIRED_ALTITUDE_SLOP_OVER, baseAltitude, baseAltitude - DESIRED_ALTITUDE_SLOP_UNDER); 
    return;  
  }

  // elevator
  void act(int curAltitude, int curThrottle) { // , bool *failedClimbPtr) {
    // these must be set for EACH call to act....
    setSpecialElevatorParameters(flightConfig.cDesiredAltitude); // calls setParameters, which sets max and min Altitude
    // poor coding style
    int delta = 0;  
    int angle;
    static int anglePrev = -1;
    int servoAngle = getServoBase();  
#ifdef C_SETUP_2 
#else 
    // active code
    // SPECIAL CASES...
    if (curAltitude > mission.mMaxAGLinFeet) { // enforce altitude firewall
      // special case 1: over Max AGL      
      // serious descent via motor (lower power)
      delta = 0; 
      throttleController -> setThrottleSetting(throttleController -> MC_FORCE); 
      fLog.LogMsg(">MaxAGL!");             
    } else if (throttleController -> getFailedClimb() ){
      // special case 2: this is a catch all for motor issue(s), i.e.: coming down
      // better glide if nose is raised
      delta =  GLIDE_DELTA_MULTIPLIER * flightConfig.cElevatorClimbDelta ; // moderate nose raised 
      fLog.logAction2(fLog.elevatorUp, 5);
      fLog.LogMsg("FailClimb!");                        
    } else {
      // NORMAL CASES       
      if (curAltitude > maxAltitude) { // lower power setting
        // DESCEND via motor and elevator
        delta = -getServoDelta(); 
        throttleController -> setThrottleSetting(throttleController -> MC_DESCEND);
        fLog.logAction(fLog.motorDecrease, 1);  
        fLog.logAction(fLog.elevatorDown, 1);                      
      } else if (curAltitude < minAltitude) {
        // CLIMB via motor and elevator    
        if (elevatorOverrideMode == EL_NORMAL) {
          delta =  flightConfig.cElevatorClimbDelta ; // slightly raise nose 
          // fLog.LogMsg("Normal!");                    
        } else { // likely EL_HARD_CLIMB
          delta =  HEAVY_CLIMB_DELTA_MULTIPLIER *
                 flightConfig.cElevatorClimbDelta ; // raise nose drastically
          // fLog.LogMsg("Heavy!");
        }             
        throttleController -> setThrottleSetting(throttleController -> MC_CLIMB);
        fLog.logAction(fLog.motorIncrease, 1); 
        fLog.logAction(fLog.elevatorUp, 1);           
      } else {
        // LEVEL flight
        delta = 0;            
        throttleController -> setThrottleSetting(throttleController -> MC_LEVEL); 
        fLog.logAction(fLog.motorBase, 1);
        fLog.logAction(fLog.elevatorLevel, 1);                     
      }
    }  
#endif  
// end ifdef C_SETUP_2  
    angle = writeServoAngleTrans5(servoAngle, delta, &servoCalbElevator);    
    if (angle != anglePrev) {
      anglePrev = angle;
    }
  }
 }; // end of ElevatorController

// ZZRC ZZRUDDER
class RudderController: public Controller {
  private:
    ElevatorController* elevatorController;
    // these temp variables are the desired heading with slop factors
    int magHeadingPlus;
    int magHeadingMinus;
    // these are the above, normalized to 0 to 259
    int trueHeading;
    int trueHeadingPlus;
    int trueHeadingMinus; 
    int stagedCnt;
    bool stagedToggle; 
// 2019-10-21: for dyno rudder
    int lastDirection;             
  public:
    const int RC_LEFT_STEP3 = -3;  
    const int RC_LEFT_STEP2 = -2;
    const int RC_LEFT = -1;
    const int RC_STRAIGHT = 0;
    const int RC_RIGHT = 1;   
    const int RC_RIGHT_STEP2 = 2;
    const int RC_RIGHT_STEP3 = 3;    
  public:
  RudderController(XServo* _s, ElevatorController* _m /* , Xclock* _c, int _t */ ): Controller(_s, 'R') {
    elevatorController = _m;
    // rudderStagedTimer = _t;
    // clockPtr = _c;
    // 2019-10-21: for dyno rudder
    lastDirection = 0;
  }

  int makeRaw(int mag) {  
    return mag;  
  }

  int norm(int val) {
    return ((val + 720) % 360);
  }
  // ZZXX
  void configure() {
    setServoDelta(flightConfig.cRudderDelta);   // this sets Servo movement range
    stagedCnt = 0;
    stagedToggle = false;
    return;
  }  
  // this turkey tries to give a sense of being navigated by staying on a course
  int navigate(int actHeading) { // raw is the reality   
      int result = RC_STRAIGHT;
      flightConfig.cActualDesiredHeading = flightConfig.cDesiredHeading;    
      if (flightConfig.cDesiredHeading == DYN_VAL ) {
        flightConfig.cActualDesiredHeading = flightConfig.cMeasuredHeadingAtEngagement;
        configure();  // this needs to be cleaned up!   
      } 
      // should below be in configure??  
      magHeadingMinus = norm(flightConfig.cActualDesiredHeading - flightConfig.cHeadingSlop); // desired heading lower limit (minus)
      magHeadingPlus = norm(flightConfig.cActualDesiredHeading + flightConfig.cHeadingSlop); // desired heading upper limit (plus)
      trueHeadingMinus = norm( makeRaw(magHeadingMinus)); // true desired heading, lower limit
      trueHeading = norm(makeRaw(flightConfig.cActualDesiredHeading)); // true desired heading
      trueHeadingPlus = norm(makeRaw(magHeadingPlus));  // true desired heading, upper limit     
      // optional rotate to get math to work when compass is 0 .. 359 :) 
      int rotTrueHeading = trueHeading;
      int rotTrueHeadingPlus = trueHeadingPlus;
      int rotTrueHeadingMinus = trueHeadingMinus;
      int rotActHeading = actHeading;
      // to this point headings should be 0 .. 359
      // if in trueHeading < 90 or > 270, rotate
      if (trueHeading < 90 || trueHeading > 270) { 
        // rotate 180, normalize back to 0 .. 359
        // this gets us a number line with all the variable > 0
        rotTrueHeadingMinus = norm(trueHeadingMinus + 180);
        rotTrueHeading = norm(trueHeading + 180);
        rotTrueHeadingPlus = norm(trueHeadingPlus + 180);
        rotActHeading = norm(actHeading + 180);
      }
      // this code helps with making harder turns (third range) that don't destabilize
      stagedCnt++;
      if (stagedCnt > STAGEDRUDDER_FACTOR) {
        stagedToggle = !stagedToggle;
        stagedCnt = 0;
      }
      // 2019-09-24: we are trying too hard, need to honor slop factors
      // 1) if in slop range, just go rudder staight so plane can level!!!  **really**
      // 2) if out of slop range then try to adjust
      // DDHEADING  HEADING_MAX_SLOP
      // 5 possible actions, no action is just straight onwards   
      int lDif = rotActHeading - rotTrueHeadingPlus;
      int rDif = rotTrueHeadingMinus - rotActHeading;
      if ((rotActHeading > rotTrueHeadingPlus) && (lDif > HEADING_MAX_SLOP)) {
        // think LEFT!
        if (lDif > HEADING_PHASE_2_DIFFERENCE) {
          if (stagedToggle) result = RC_LEFT_STEP3; else result = RC_LEFT_STEP2;          
        } else if (lDif > HEADING_PHASE_1_DIFFERENCE) {
          result = RC_LEFT_STEP2;
        } else {
          result = RC_LEFT;    
        }  
      } else if ((rotActHeading < rotTrueHeadingMinus) && (rDif > HEADING_MAX_SLOP)) {
        // think RIGHT!  
        // serialPrintString("DEBUG: right: ");
        // serialPrintInt(rDif);              
        if (rDif > HEADING_PHASE_2_DIFFERENCE) {   
          if (stagedToggle) result =  RC_RIGHT_STEP3; else result = RC_RIGHT_STEP2;                  
        } else if (rDif > HEADING_PHASE_1_DIFFERENCE) {        
          result = RC_RIGHT_STEP2;
        } else {
          result = RC_RIGHT;
        }
      }
      if (result == RC_STRAIGHT) {
        // reset staged toggle
        stagedCnt = 0;
        stagedToggle = false;        
      }
      // DEBUG DEBUG 2019-06-20: Turn off _STEP3 via brute force
      //                         want very gentle turning....
      //  NOTE: 2020-02-25 wigwam needs the STEP2 and STEP3 off
#if 1      
      if (result == RC_LEFT_STEP2 || result == RC_LEFT_STEP3) {
              result = RC_LEFT;
      // if (result == RC_LEFT_STEP3) {        
      //   result = RC_LEFT_STEP2;
      } else if (result == RC_RIGHT_STEP2 || result == RC_RIGHT_STEP3) {
            result = RC_RIGHT;
      }
      // else if (result == RC_RIGHT_STEP3) {        
      //  result = RC_RIGHT_STEP2;
      // }
#endif         
      return result;
   }

   
   // introduce wigwam into rudder controller class
   // brings in rudder_driver() function
   #include "wigwam2.h"

  // 2019-10-21, add rateOfChange use via call to manageRudder
  // zzDYNO
  int setServoAngle( int rawCurHeading, int rateOfChangeHeading) {
    int delta = 0;
    int angle;
    int servoAngle = getServoBase();
#ifndef C_SETUP_2
    // navigate returns with direct code to set intent,
    // code below implements the intent    
    int  direct = navigate(rawCurHeading);
#else
    int direct = RC_STRAIGHT;
#endif 
    //
    // 2020-02-25: replace mess below with simple "wigwam" approach
    //
    // servoAngle is set above
    // won't use preferred getServoDelta() for now, use delta from wigwam
    if (direct == RC_STRAIGHT) {
      elevatorController -> setOverrideMode(elevatorController -> EL_NORMAL);
    } else {  
      // note this is only a suggestion to elevator
      elevatorController -> setOverrideMode(elevatorController -> EL_HARD_CLIMB);  // note this is a suggestion to elevator
    }
    long timer = millis();  // wigwam has its own timers!
    // heading not currently used
    delta = rudder_driver(timer, rawCurHeading, direct);
    // DEBUG WIGWAM -- for ground pre test
    // changed table in wigman....
    // delta *= 4;
    // delta *= 3;  // 2020-06-21 fix
    angle = writeServoAngleTrans5(servoAngle, delta, &servoCalbRudder);  
    if (direct == RC_STRAIGHT) {
       fLog.logAction(fLog.rudderStraight, 1);
    } else if (direct == RC_RIGHT ||
        direct == RC_RIGHT_STEP2 ||
        direct == RC_RIGHT_STEP3) { fLog.logAction(fLog.rudderRight, 1); }
    else if (direct == RC_LEFT ||
             direct == RC_LEFT_STEP2 ||
             direct == RC_LEFT_STEP3) { fLog.logAction(fLog.rudderLeft, 1);}
    if(direct != RC_STRAIGHT &&
       direct != RC_LEFT &&
       direct != RC_RIGHT) {         
      fLog.logAction(fLog.rudderOther, 1); 
    }     
  }
  // rudder
  // changed 2019-10-21 to pass thru rateOfChange
  void act(int rawCurHeading, int rateOfChangeHeading) {
    setServoAngle(rawCurHeading, rateOfChangeHeading);             
    return;
  }      
};



// AGENT007 :)
class FlightAgent {
  private:
      
    ElevatorController* elevatorController;
    RudderController* rudderController;
    ThrottleController* throttleController;
    Xclock* flightClock;
    int elevatorTimer;
    int rudderTimer;
    int throttleTimer;
    boolean active;

  public:

  FlightAgent (ElevatorController* _ec, RudderController* _rc, ThrottleController* _mc) {
    elevatorController = _ec;
    rudderController = _rc;
    throttleController = _mc;
    active = false;
  }  

  void configure(Xclock* _fc, int _et, int _rt, int _tt) {  // cannot be part of Constructor ...
    flightClock = _fc;
    elevatorTimer = _et;    
    rudderTimer = _rt;
    throttleTimer = _tt;
    throttleController -> configure();
    rudderController -> configure();
    elevatorController -> configure();
  }

  void setActive(bool _b) {
    active = _b;   
    if (active == true) {
      if (THROTTLE) setThrottleActive(_b);
      if (COMPASS) setRudderActive(_b);
      if (ALTIMETER) setElevatorActive(_b);
    } else {
      setThrottleActive(_b);
      setRudderActive(_b);
      setElevatorActive(_b);
    }    
  }
    
  boolean getActive() {
    return active;
  }

  void setServoBases(int elevatorServoSetting,
                     int rudderServoSetting,
                     int throttleServoSetting) {
        throttleController -> setServoBase(throttleServoSetting);
        rudderController -> setServoBase(rudderServoSetting); 
        elevatorController ->  setServoBase(elevatorServoSetting);
        // old PLAN_B was here, try to keep changes            
    if (0) { // active) {
      if (THROTTLE) {
        throttleController -> setServoBase(throttleServoSetting);
      }
      if (COMPASS) {
        rudderController -> setServoBase(rudderServoSetting);
      }
      if (ALTIMETER) {
        elevatorController ->  setServoBase(elevatorServoSetting);
      }
    }                                     
  }

  void actAgent( int curAltitude, int curHeading, int curSpeed,
            int curAngleRudder, int curAngleElevator, int curAngleThrottle) {
            // boolean *failClimbPtr) {
    if (active && MAG_CALB != 1) {
      if (flightClock -> checkTimerWithRestart(rudderTimer) && COMPASS) {
        // 2019-10-20: get rate-of-change easy way...
        static int lastHeading = -999;
        if (lastHeading < -360) lastHeading = curHeading; // avoid first-time junk
        //  move to a line (not a mod circle)
        int rateOfChangeHeading = curHeading  - lastHeading;
        if (abs(rateOfChangeHeading) > 180) {  // nonsense, fix it
          int c = (curHeading + 180) % 360;
          int l = (lastHeading + 180) % 360;
          rateOfChangeHeading = c - l;        
        }
        lastHeading = curHeading;
        rudderController -> act(curHeading, rateOfChangeHeading);
      }
      if (flightClock -> checkTimerWithRestart(elevatorTimer) && ALTIMETER) {        
        elevatorController -> act(curAltitude, curAngleThrottle); //, failClimbPtr);     
      }
      if (flightClock -> checkTimerWithRestart(throttleTimer) && THROTTLE) {         
        throttleController -> act(curAltitude, curSpeed); // , failClimbPtr);
      }      
    } 
  }

  // ELEVATOR
  // void setElevatorParameters(int _v1, int _v2, int _v3) {
    // maxAltitude = _v1;
    // desiredAltitude = _v2;
    // minAltitude = _v3;  
    // elevatorController -> setParameters( _v1, _v2, _v3);       
  // }
  void getElevatorParameters( int& _v1, int& _v2, int& _v3) {
    // _v1 = maxAltitude;
    // _v2 = desiredAltitude;
    // _v3 = minAltitude;
    elevatorController -> getParameters( _v1, _v2, _v3);      
  }
  void setElevatorActive(boolean _b) {
    elevatorController -> setActive(_b);   
  }
  boolean getElevatorActive() {
    return elevatorController -> getActive();
  }

  // THROTTLE
  void setThrottleSetting(int _s) { // MC_xxxxx
    throttleController -> setThrottleSetting(_s);   
  }
  boolean getThrottleSetting() {
    return throttleController -> getSetting();
  } 
  void setThrottleActive(boolean _b) {
    throttleController -> setActive(_b);   
  }
  boolean getThrottleActive() {
    return throttleController -> getActive();
  }  

  // RUDDER
  void setRudderActive(boolean _b) {
    rudderController -> setActive(_b);   
  }
  boolean getRudderActive() {
    return rudderController -> getActive();
  }  
  int parseInt(char* num) {
    int cnt = 0;
    int val = 0;
    while (*num != '$') {
      if (cnt > 5) return -1;
      if (*num >= '0' && *num <= '9') {
        val *= 10;
        val += (*num - '0');
        cnt++;
        num++;
      } else return -1;
    }
    return val;
  }
};

// constants
// actual servo pins
                           // note odd order .. 
const int Servo1Pin = 10;  // rudder  
const int Servo2Pin = 9;   // elevator
const int Servo3Pin = 11;  // throttle
const int LEDPin = 13;     // internal LED
const int ReadyLEDPin = 8; // Ready LED   
// InputPin1 thru 4 handled with interrupts
#ifdef UNO
// there for UNO
const int SerialRX_BT = 0; // 
const int SerialRX2_BT = 12; //
const int SerialTX_BT = 1; //  
#else
// there for Trinket
const int SerialRX_BT = 0; 
const int SerialRX2_BT = 12;  // overlaps 0 
const int SerialTX_BT = 1;
#endif

// indices for uSec array
const int indexElevator = 0;
const int indexRudder = 1;
const int indexThrottle = 2;
const int indexControl = 3;

// External Interrupts (note volatile attribute below)

#define NUM_CHANNELS 4
#define PIN_BASE 3
volatile uint8_t prev; // remembers state of input bits from previous interrupt
volatile uint32_t risingEdge[NUM_CHANNELS + 1]; // time of last rising edge for each channel
volatile uint32_t uSec[NUM_CHANNELS + 1]; // the latest measured pulse width for each channel

Xclock flightClock;
//
// eXtended Servos
XServo xservo2(Servo2Pin);  // elevator
XServo xservo1(Servo1Pin);  // rudder
XServo xservo3(Servo3Pin);  // throttle

// controllers that drive Servos

ThrottleController controlThrottle(&xservo3);
ElevatorController controlElevator(&xservo2, &controlThrottle);
RudderController controlRudder(&xservo1, &controlElevator); // , &flightClock, STAGEDRUBBERTIMER_VALUE);

// readUsec method to 'protect' uSec reads
uint32_t readUSec(int index) {
  noInterrupts(); 
  uint32_t val = uSec[index];
  interrupts();
  return val;
}

// Agent [[ Pilot ]]
FlightAgent flightAgent = FlightAgent(&controlElevator, &controlRudder, &controlThrottle);
// other global variables  ZZGLOBAL
// the biggies:
int missionStepCnt = 0;
float measuredHeading = -1;
float measuredAltitude = -1;
float measuredThrottle = -1;
float measuredRudder = -1;
float measuredElevator = -1;
// control active?
int controlAgentActive = 0;
int controlHeading = 0;
int controlAltitude = 0;
int controlThrust = 0;
int controlLED = 0;
// others:
int error = 0;
int Servo4PWMvalue = 0;
boolean agentFullLock = false;
// 2020-09-17 added count:
int unsetAutoCount = 0;
// boolean failedClimb = false;

// timers <--------------------------- TIMERS
int LEDtimer;
int loggingTimer;
int logGpsTimer;
int elevatorTimer;
int rudderTimer;
int throttleTimer;
int missionStepTimer;
int missionActionTimer;
int manualActionTimer;
int stagedRudderTimer;
//
// SETUP
// ZZSETUP
void setup() { 

// ****************************************************************
// ----------------------------------------------------------------
  // magnetic heading...
  flightConfig.cDesiredHeading = mission.mStep[0].mHeading;
  flightConfig.cHeadingSlop = HEADING_MAX_SLOP; 
  flightConfig.cRudderDelta = RUDDER_FULL_DELTA;   // rudder rigth/left delta; 
  // altitude...  
  flightConfig.cDesiredAltitude = mission.mStep[0].mAltitude;   // can also be a value in feet :)
  flightConfig.cElevatorDelta = ELEVATOR_FULL_DELTA; // / 3; // 2; 
  flightConfig.cElevatorClimbDelta = ELEVATOR_CLIMB_DELTA;
  // power/throttle
  flightConfig.cThrottleDelta = THROTTLE_FULL_DELTA; 
// ----------------------------------------------------------------  
// ****************************************************************
//  
  gyro.gCalibrated = false;
  boolean ConfigError = false;
  LOGGING = 1; // for now
  switch (mission.mMode) {
    case O_PASSTHRU:
    case O_PASSTHRU_CALIBRATE:
      SER = 0; 
      COMPASS = 0;
      ALTIMETER = 0;
      MAG_CALB = 0;       
      SERVO4 = 1;   
      INTR = 1;   
      if (mission.mMode == O_PASSTHRU_CALIBRATE) {
        SER = 1;
        COMPASS = 1;
        MAG_CALB = 1;
        VERBOSE = 1;
      }
      break;
    // case O_HOLD_ALTITUDE:      
    case O_HOLD:
      MAG_CALB = 0; // check this !!!
      SER = 1;
      VERBOSE = 0; // quiet for now, was 1; // SEPT17 DEBUG
      DASHBOARD = 0;
      ALTIMETER = O_ELEVATOR_ACTIVE; 
      COMPASS = O_RUDDER_ACTIVE; 
      THROTTLE = 1;    // Altitude control may change throttle setting 
      SERVO4 = 1;
      INTR = 1; 
      break;           
  }
  if (!SER) LOGGING = 0;
  if (!SER) VERBOSE = 0;
  if (!SER) DASHBOARD = 0;
  const char startMsg[] = "Setup: ";
#ifdef UNO  
  // use UNO Serial class
  if (SER) Serial.begin(9600);
  serialPrint(startMsg); 
  serialPrintln("(UNO)");
#else
   // for Trinket Pro
  // use brute-force bit-banger routines
  if (SER) {
    InitSoftUART(SerialRX_BT, SerialTX_BT); 
    UART_print(startMsg);
    UART_println("(Trinket)");     
  }
#endif  
  if(SER) {
    serialPrintString("Vers: ");
    serialPrintIntln(VERSION); 
#ifdef PLANE_NAME
    serialPrintString("PL: ");
    serialPrintStringln(PLANE_NAME);
#endif
#ifdef C_SETUP_2
    serialPrintString("!!C_2; ");
#endif
#ifdef P_DELTA_GROUND_ONLY
    serialPrintString("!!GRD; ");
#endif
#ifdef TEST_ALT_MANUAL_MODE
    serialPrintString("!!TM - ALT; ");
#endif  
    if (MAG_CALB) {
      serialPrintString("!!MAG-CALB; ");    
    }        
    serialPrintString("MS: ");
    serialPrintStringln(mission.mName);
    serialPrintString("ActionTimer (Secs): ");
    serialPrintIntln((mission.mActionTimer / 1000));  
    serialPrintString("Action: ");
    if (mission.mAction == A_LOCK) {
      serialPrintStringln("LOCK"); 
    } else {
      serialPrintStringln("TEMP");  
    }  
#ifdef P_TEST_SKIP_FAIL_CLIMB 
    serialPrintStringln("!!TM - Fail Climb Check Off");
#else    
    serialPrintString("Fail Climb Timer (Secs): ");  
    serialPrintIntln((FAILURE_TO_CLIMB_MSECS / 1000));    
#endif        
    serialPrintString("Firewall Alt (ft): ");
    serialPrintIntln(mission.mMaxAGLinFeet);
    serialPrintString("Steps: ");
    serialPrintIntln(mission.mSteps);
    serialPrintString("Alt Step1 (ft): ");
    serialPrintIntln(mission.mStep[0].mAltitude);
    serialPrintString("Head Step1 (deg): ");
    serialPrintIntln(mission.mStep[0].mHeading);      
  } // end of SER

  // with passthru and log, LED blinks
  // with HOLD, LED blinks with config error, is on when no config errs and active
  if (LED == 1) {
    pinMode(LEDPin, OUTPUT);
    digitalWrite( LEDPin, LOW);    
    pinMode(ReadyLEDPin, OUTPUT);
    digitalWrite( ReadyLEDPin, LOW);     
  }
  Wire.begin();
  if (COMPASS == 1) {
#ifdef NAV_MODE_2
    gyro.gHeading = 0.0; // initial heading
    gyro.gStamp = millis();
    if(mpu.init()){
      serialPrintln("Gyro failed!");
      ConfigError = true;  
      measuredHeading = NOT_IN_USE;
    } else {
      serialPrint("GY UP, ");
      gyro.gSwag = calbGyro(LEDPin);
      gyro.gCalibrated = true;
      serialPrint(" COR (*1000): ");
      serialPrintIntln( ((int) (gyro.gSwag * 1000)) );
      // serialPrintString("Cur Head: ");
      // first time, value wonky....
      measuredHeading = getHeading(); // in degress, already rotated, etc.  
      // serialPrintIntln(measuredHeading);        
    }
#endif 
  } 
  if (ALTIMETER == 1) { 
    // ** altimiter **
    if (pressure.begin()) {
      // serialPrintStringln("BMP180 init success");
    } else {
      // Oops, something went wrong, this is usually a connection problem,
      // see the comments at the top of this sketch for the proper connections.
      serialPrintStringln("BMP180 init fail!");
      ConfigError = true;      
    }  
  }  
  serialPrintString("Cur Alt: ");
  measuredAltitude = getAltitudeInMeters()*3.28084; // convert to feet;
  serialPrintIntln(measuredAltitude);   
  if (COMPASS == 1) {
      // wait for gyro to get some active time
      serialPrintString("Est Head: ");
      measuredHeading = getHeading(); // in degress, already rotated, etc.  
      serialPrintIntln(measuredHeading);
  }  

  if (!ConfigError) {
    LEDtimer = flightClock.allocateTimer();
    loggingTimer = flightClock.allocateTimer();
    logGpsTimer = flightClock.allocateTimer();
    elevatorTimer = flightClock.allocateTimer();
    rudderTimer = flightClock.allocateTimer();
    throttleTimer = flightClock.allocateTimer();
    missionStepTimer = flightClock.allocateTimer();
    missionActionTimer = flightClock.allocateTimer(); 
    manualActionTimer = flightClock.allocateTimer();
    stagedRudderTimer = flightClock.allocateTimer();
    if (LEDtimer + loggingTimer + logGpsTimer +
        elevatorTimer + rudderTimer + throttleTimer +
        missionStepTimer + missionActionTimer + manualActionTimer +
        stagedRudderTimer < 0) {
        ConfigError = 1; 
        serialPrintln("Timer fail!");           
    }
  }   
  if (ConfigError) {
    // crap!  hold the show
    while (1) {
      if (LED) {   
        digitalWrite( ReadyLEDPin, digitalRead(ReadyLEDPin) ? LOW : HIGH);  // blink LED
      }  
      delay(500);
    }
  }
  // interrupt driven radio inputs (servo PWM input)??
  if (INTR != 0) {
      serialPrintStringln("Int On!");
      for (int pin = PIN_BASE; pin < (NUM_CHANNELS + PIN_BASE); pin++) { // enable pins 3 to 6 as our 4 input bits
        pinMode(pin, INPUT);
      }
    // PCMSK2 |= 0xFC; // set the mask to allow those n pins to generate interrupts
    // this value is dendent on BIN_BASE and NUM_CHANNELs
    // 0x78: 0111 1000  7c fc
    PCMSK2 |= 0x78; // set the mask to allow those n pins to generate interrupts  
    PCICR |= 0x04;  // enable interupt for port D    
  }
  //  

  if (LED) {
    // off to begin
    digitalWrite( LEDPin, LOW); 
    digitalWrite( ReadyLEDPin, LOW);     
  }

  // TIMER setup!

  // set some timer values  
  flightClock.startTimer(LEDtimer, LEDTIMER_VALUE);
  flightClock.startTimer(loggingTimer, LOGGINGTIMER_VALUE);
  flightClock.startTimer(elevatorTimer, ELEVATORTIMER_VALUE);
  flightClock.startTimer(rudderTimer, RUDDERTIMER_VALUE); 
  flightClock.startTimer(throttleTimer, THROTTLETIMER_VALUE);   
  // flightClock.startTimer(manualActionTimer, MANUALACTIONTIMER_VALUE);  
  // configure flightAgent
  flightAgent.configure(&flightClock, elevatorTimer, rudderTimer, throttleTimer);
  if (LED && ConfigError == false) {
    digitalWrite( ReadyLEDPin, HIGH);  
  }
  // startup mission steps
  missionStepCnt = 0;
  if (LOGGING) {
    fLog.logActive();
  }  

  if (SER) {
    delay(1000); // arbitary delay helps the logger blocking, < 1000 fails ....    
    serialPrintString("M: ");
    switch (mission.mMode) {
      case (O_PASSTHRU):      serialPrintStringln("O_PS"); break;
      case (O_PASSTHRU_CALIBRATE): serialPrintStringln("O_PC"); break;   
      case (O_HOLD):         
        serialPrintString("O_H, h:");
        serialPrintInt(flightConfig.cDesiredHeading);
        serialPrintString(" a:");
        serialPrintInt(flightConfig.cDesiredAltitude);        
        break;         
    }                 
    serialPrintStringln(" *run*");
  } else {
    // no serial IO activeHI BKR-s
    delay(500);  // let the interrupts get active, etc.        
  } 
#ifdef NAV_MODE_2
  gyro.gHeading = 0.0; // initial heading
  gyro.gStamp = millis(); 
#endif
}

//
// Arduino LOOP
// ZZZLOOP
void loop() {   
  //
  // phase 0: get current R/C inputs, set mission parameters  
  //
  measuredRudder = getAngle(readUSec(indexRudder));
  measuredElevator = getAngle(readUSec(indexElevator));
  measuredThrottle = getAngle(readUSec(indexThrottle));  
  // missionStepCnt is critical ..
  flightConfig.cDesiredHeading = mission.mStep[missionStepCnt].mHeading;  
  flightConfig.cDesiredAltitude = mission.mStep[missionStepCnt].mAltitude;           
  //
  // phase 1: read instruments, as requested
  //
  if (COMPASS) {
    measuredHeading = getHeading(); // in degress, already rotated, etc.  
  }
  if (ALTIMETER) { 
#ifdef TEST_ALT_MANUAL_MODE    
    //simulatedAltitude = testAltitudeInFeet(simulatedAltitude); 
    // measuredAltitude = simulatedAltitude;
    char c[2];
    c[0] = UART_RecieveWait(10);  // 10 is a swag
    if (c[0] != -1) {
          c[1] = 0;
          serialPrintString(c);
          switch (c[0]){  // '+' is up 100, '-' is down 100, 'R' is reset
            case '+': measuredAltitude += 100; break;
            case '-': measuredAltitude -= 100; break;
            case 'R': measuredAltitude = getAltitudeInMeters()*3.28084; break;
          }       
    }
#else
    // actual flight mode...
    measuredAltitude = getAltitudeInMeters()*3.28084; // convert to feet
#endif
  }
  // note mix of digital instrument readings and throttle
  // ZZFIX getAngle(uSec[indexThrottle]) (controlAgentActive == 1)
  {
    int tempThr = measuredThrottle; // manual flight
    if (controlAgentActive == 1) {
      tempThr = controlThrottle.getServoCurrent();   // was getAngle(uSec[indexThrottle]); and was  WRONG
    }
    fLog.logReadings(measuredHeading, measuredAltitude, tempThr);
    if (controlAgentActive) {
      fLog.logAction2(fLog.logMode,'A');
    } else {
      fLog.logAction2(fLog.logMode,'M');        
    }
  }  

#ifdef C_SETUP_1
  while (1) { // broken....
    serialPrint(">>> C_SETUP_1, EL:");
    serialPrintInt(measuredElevator);    
    serialPrintString(", RD:");   
    serialPrintInt(measuredRubbed);
    serialPrint(", TH:");
    serialPrintIntln(measuredThrottle);  
    delay(2000);   
  }
#endif             
  // 
  // engage and disengage Flight Agent <----------------- SERVO4 
  //
  if (SERVO4 == 1) {
    Servo4PWMvalue = readUSec(indexControl);  
    bool changed = false;
    // in non-agent modes, let SERVO4 setting drive main LED
    // SERVO4:
    if (mission.mMode == O_PASSTHRU || mission.mMode == O_PASSTHRU_CALIBRATE) {
      controlLED =  pwmOnOff(Servo4PWMvalue, controlLED, "Control LED");
      controlAgentActive = 0; 
      // insurance
      controlHeading = 0;
      controlAltitude = 0;
      controlThrust = 0;      
    } else { 
      // AGENT MODE -- Trinket may take control   
      int oldAgentActive = controlAgentActive;
      controlAgentActive = pwmOnOff(Servo4PWMvalue, controlAgentActive, "Agent Control"); 
      // enforce FullLock: release only if at full throttle when SERVO4 goes off
      if (agentFullLock == true && 
        oldAgentActive == 1 &&
        controlAgentActive == 0) { // special case
        if (measuredThrottle < FULLTHROTTLE_MARKER_VALUE) {
          // 2020-09-17 fix
          // loop() occurs 20-30 times / sec with Trinket
          // want 10 consecutive tries before really unsetting
            controlAgentActive = 1;  // undo unset request!
            unsetAutoCount == 0;             
        } else {           
          unsetAutoCount++;
          if (unsetAutoCount < 10) { 
            // serialPrintln("skip");  // DEBUG 
            controlAgentActive = 1;  // undo unset request!
          } else {
            // serialPrintln("unset");  // DEBUG 
            unsetAutoCount == 0;
            // and leave controlAgentActive as zero, leading to unset!!!
            agentFullLock = false;
          }
          // end changes
        }      
      }
      if (oldAgentActive != controlAgentActive) {
        changed = true;
        fLog.logAgent(controlAgentActive);       
      }
      // 2020-09-17 fix, reset counter when controlAgent goes on
      if (controlAgentActive == 1 && 
          oldAgentActive != controlAgentActive) {
        unsetAutoCount = 0;      
      }   
      if (controlAgentActive == 1 && // agent is now on 
         oldAgentActive != controlAgentActive && // has just changed changed  
          changed && // have multistep mission // FIX01 from APM 2020-07-12
          mission.mStep[0].mDuration > 0) { // has a time value
        // start timer
        missionStepCnt = 0;        
        flightClock.startTimer(missionStepTimer, mission.mStep[missionStepCnt].mDuration);
      }      
    }
    //
    // phase 2: flip/unflip each control to Flight Agent
    //
    // these CONTROL the INTERRUPTS, not the main classes!
    //  * controlHeading
    //  * controlAltitude
    //  * controlThrust        
    //
    // SERVO4:
    if (COMPASS) { // rudder -- servo1; heading
      if (changed) {
        if (controlAgentActive == 1) { 
          flightConfig.cMeasuredHeadingAtEngagement = measuredHeading; // needed in special cases 
          controlHeading = 1;                         
        } else {
          controlHeading = 0;
        }
      }
    }
    // SERVO4:
    if (ALTIMETER) { // elevator -- servo2; altitude
      if (changed) {
        if (controlAgentActive == 1) {
          if (LED) {
            digitalWrite( LEDPin, HIGH); // on
          }
          flightConfig.cMeasuredAltitudeAtEngagement = measuredAltitude; // needed in special cases
          controlAltitude = 1;                      
        } else {
          controlAltitude = 0;          
          if (LED) {
            digitalWrite( LEDPin, LOW); // off
          }        
        }
      } 
    }
    // SERVO4:
    if (THROTTLE) { // throttle -- servo3; thrust, which impacts altitude and speed
      if (changed) {
        if (controlAgentActive == 1) {                     
          controlThrust = 1;    
        }  else {
          controlThrust = 0;
        }
      }    
    }
    // SERVO4: flightAgent actual activate/deactivate
    //
    // ZZBUG: maybe collect last N (odd sized) readings, make median
    //
    if (changed) {
      if (controlAgentActive == 1) {
        // FIX02 from APM 2020-07-12
        // if going active, update heading and elevation
        // doing this prevents odd actions when the time to next scheduled reads is long
        measuredHeading = getHeading();
        measuredAltitude = getAltitudeInMeters()*3.28084; // getAltitudeInFeet();     
        // end FIX02
        //
        // ZZBUG: possible bug -- getAngle(uSec[xx]) can be noisy!
        //        or maybe "pause" for 50 to 100 ms to get good readings... from uSec
        //
        flightAgent.setServoBases(getAngle(readUSec(indexElevator)), // + ELEVATOR_SERVO_SWAG),
                                  getAngle(readUSec(indexRudder)), // + RUDDER_SERVO_SWAG),
                                  getAngle(readUSec(indexThrottle))); // +  THROTTLE_SERVO_SWAG ));                                     
        if (flightAgent.getActive() == false) { 
          // in the event of Agent restart        
          controlThrottle.setForce(false);                   
          controlThrottle.clrFailedClimb();                     
          flightAgent.setActive(true);
          if (VERBOSE == 1) {
            serialPrintln("Agent ON!");
          }           
        }
        // start lock timer ..
        flightClock.startTimer(missionActionTimer, mission.mActionTimer);
        agentFullLock = false;   
        controlThrottle.setForce(false);  // well, a little bit of brute force!                      
      } else {
        flightAgent.setActive(false);
        // doing these makes logging more accurate
        controlThrottle.setForce(false); 
        controlThrottle.clrFailedClimb();             
        if (VERBOSE == 1) {
          serialPrintln("Agent OFF!");
        }  
      }
    } else { // !changed
      // manage auto mission actions
      if (controlAgentActive == 1 ) {
        if (flightClock.checkTimer(missionActionTimer)) {
          // 2020-09-17, move LogMsg and make clearer
           // fLog.LogMsg("*A_Timer*");
           if (mission.mAction == A_LOCK) {
              if (agentFullLock == false) {
                fLog.LogMsg("*A_LOCK*");
                agentFullLock = true;
              }
           } else {
              if (mission.mAction == A_TEMP) {
                 fLog.LogMsg("*A_TEMP*"); 
                 controlThrottle.setForce(true);  // well, a little bit of brute force!
              }              
           }
        } 
      }
    }
  } // Servo4 on

  //
  // phase 3: let the Flight Agent act as needed
  //
  int measuredSpeed = -1; // sigh!
  // it all happens here .... fly babe, fly! This calls three *lesser* act() methods   
  flightAgent.actAgent(measuredAltitude, measuredHeading, measuredSpeed,     // units: feet, degrees, 0
                       measuredRudder, measuredElevator, measuredThrottle);        // units: degrees 
  // phase 4: misc. logging, blinks, and such are coded here
  // LED blinking
  if (flightClock.checkTimerWithRestart(LEDtimer)) {  
    // timestampOutput = micros(); // check is based on this ...    
    if (mission.mMode == O_PASSTHRU || mission.mMode == O_PASSTHRU_CALIBRATE) { 
      // don't do this for other MODEs ...     
      if (LED) {  // LED on where SERVO4 is full
        if (controlLED) {
          digitalWrite( LEDPin,  HIGH);  // LED on          
        } else {
          digitalWrite( LEDPin, digitalRead(LEDPin) ? LOW : HIGH);  // blink LED
        }
      }
    }
  }  
  // timed logging ZZLOG
  // DEBUG DEBUG 2019-06-14: don't flood log with M entries...
  bool showLogData = true;
#if 0  
  if (controlAgentActive != 1) {
    if ((fLog.count % 10) != 0) {
      showLogData = false;
    }
  }
#endif  
  // always log heading indicator
  fLog.logAction2(fLog.headingIndicator,(measuredHeading / 36));  // single digit approx of heading!
  if (flightClock.checkTimerWithRestart(loggingTimer)) { 
    fLog.logAction2(fLog.pStep,missionStepCnt);
    fLog.logAction2(fLog.locked,agentFullLock);
    if (controlThrottle.getFailedClimb()) {
      fLog.logAction2(fLog.failedToClimb, 5);  // use 5 just to visually stand out
    } else {
      fLog.logAction2(fLog.failedToClimb, 0);        
    }
    // DEBUG DEBUG 2019-06-14, conditional output
    if (showLogData) {
      fLog.logSend(flightClock, logGpsTimer);
      // flightClock.startTimer(logGpsTimer, LOGGINTTIMERGPS_VALUE); 2019-10-09 moved ...          
    } else {
      fLog.count++; // very dirty 
    }
  } /* end of loggingTimer */ 
  // 2019-10-09: logSendGPS code moved to logSend...
  // serialPrint("check for Timer pop ");
  // serialPrintInt(logGpsTimer);
  // serialPrintln("..");  
  //
  // turn off GPS send 2020-06-22b
#if 0  
  if (flightClock.checkTimer(logGpsTimer)) {
    fLog.logSendGPS();
  }  
#endif    

  // phase 5: see if we need to move to next mission step
  if (controlAgentActive == 1 
      // && missionStepCnt < ( sizeof(mission.mStep) -1 )
      && missionStepCnt < (mission.mSteps - 1)) { // active and is there is another step                 
    if (mission.mStep[missionStepCnt].mDuration > 0) { // likely a timer is going
      if (flightClock.isActive(missionStepTimer)) { // yup, we have an active timer
        if (flightClock.checkTimer(missionStepTimer)) { // and has it popped
          // yup, yup         
          missionStepCnt++;         
          if (mission.mStep[missionStepCnt].mHeading == MIS_END) {   
           // sh*t, what do I do???    
           controlThrottle.setForce(true);  // well, a little bit of brute force!     
          } else if (mission.mStep[missionStepCnt].mDuration > 0) {
            flightClock.startTimer(missionStepTimer, mission.mStep[missionStepCnt].mDuration);             
          } else {
            // no timer needed, just stay here
          }
        }     
      }
    } 
  }
}
// end of Arduino LOOP <----------- END


//
// interrupt handler -- used mainly for channel --> passthru and PWM measurements
//
ISR(PCINT2_vect) { // one or more of pins 2~7 have changed state
  uint32_t now = micros();
  uint8_t curr = PIND; // current state of the n input bits
  uint8_t changed = curr ^ prev;
  // changed &= 0x78;  // clear other bits
  int channel = 0;
  // this mask is dependent on NUM_BASE and NUM_CHANELS  (was 0x04)
  // 0x08 points to pin 3
  int c = 0;
  //
  // CAUTIONS:
  // 1) controlAltitude, etc. keep this handler away from
  //    the FlightAgent class :)
  // 2) uSec[] array is used by the controllers
  // 
  for (uint8_t mask = 0x08; mask; mask <<= 1) {
    if (changed & mask) { // this pin has changed state
      switch (channel) {
        case 0: // Servo2Pin
          if (controlAltitude != 1) {
            xservo2.intrDigitalWrite(curr & mask);               
            c++;
          }          
          break;        
        case 1: // Servo1Pin
          if (controlHeading != 1) {
            xservo1.intrDigitalWrite(curr & mask); 
            c++;
          }  
          break;
        case 2: // Servo3Pin
          if (controlThrust != 1) {
            xservo3.intrDigitalWrite(curr & mask);             
            // c++;
          }          
          break;
      }
      if (curr & mask) { // +ve edge so remember time
        risingEdge[channel] = now;
      }
      else { // -ve edge so store pulse width
        uSec[channel] = now - risingEdge[channel];
      }
    }
    channel++;
  }
  prev = curr;
}
// end of program
