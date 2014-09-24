
#include "AConfig.h"
#if(HAS_STD_PILOT)
#include "Device.h"
#include "Pin.h"
#include "Pilot.h"
#include "Timer.h"
#inculde "PID_v1.h"

Timer pilotTimer;
Timer deadmanSwitchTimer;
Timer blinklightTimer;
bool _headingHoldEnabled = false;
int  _headingHoldTarget = 0;
int hdg = 0;
int hdg_Error;
int raw_Left, raw_Right;
int left, right;  // motor outputs in microseconds, +/-500
float heading_loop_Gain = 1.0;
float depth_hold_loop_gain = 0.6;
int integral_Divisor = 100;
long hdg_Error_Integral = 0;
int tgt_Hdg = 0;
bool _depthHoldEnabled = false;
int _depthHoldTarget = 0;
int depth = 0;
int depth_Error = 0;
int raw_lift =0;
int lift = 0;
int target_depth;
int raw_yaw, yaw;
bool _deadmanSwitchEnabled = false;
bool blinkstate = false;
int depth_deadband = 4; // +/- cm
int heading_deadband = 4;  // +/i degrees

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);



void Pilot::device_setup(){
  pilotTimer.reset();
  deadmanSwitchTimer.reset();
  blinklightTimer.reset();
  Serial.println(F("log:pilot setup complete;"));
  //turn the PID on
  myPID.SetOutputLimits(-1.0,1.0);
  myPID.SetSampleTime(50);
  myPID.SetMode(AUTOMATIC);
}



void Pilot::device_loop(Command command){
//intended to respond to fly by wire commands: MaintainHeading(); TurnTo(compassheading); DiveTo(depth);

    if( command.cmp("ping")){
      deadmanSwitchTimer.reset();
      if (_deadmanSwitchEnabled){
        int argsToSend[] = {0};
        command.pushCommand("start",argsToSend);
        _deadmanSwitchEnabled = false;
      }
      Serial.print(F("pong:")); Serial.print(command.args[0]); Serial.print(","); Serial.print(millis()); Serial.print(";");
    }

    if (deadmanSwitchTimer.elapsed (2000)) {
      _depthHoldEnabled = false;
      _headingHoldEnabled = false;
      int argsToSend[] = {0};
      command.pushCommand("stop",argsToSend);
      _deadmanSwitchEnabled = true;

    }

    if (_deadmanSwitchEnabled && blinklightTimer.elapsed(500)){
      int argsToSend[] = {1,50};
      if (blinkstate){
        argsToSend[1] = 0;
      }
      command.pushCommand("ligt",argsToSend);
      blinkstate = !blinkstate;
    }


    if( command.cmp("holdHeading_toggle")){
      if (_headingHoldEnabled) {
        _headingHoldEnabled = false;
        raw_Left = 0;
        raw_Right = 0;
        hdg_Error_Integral = 0;  // Reset error integrator
        tgt_Hdg = -500;  // -500 = system not in hdg hold

        int argsToSend[] = {1,00}; //include number of parms as last parm
        command.pushCommand("yaw",argsToSend);
        Serial.println(F("log:heading_hold_disabled;"));

      } else {
        _headingHoldEnabled = true;
        if(command.args[0]==0){
          _headingHoldTarget = navdata::HDGD;
        } else {
          _headingHoldTarget = command.args[1];
        }
        tgt_Hdg = _headingHoldTarget;
        Serial.print(F("log:heading_hold_enabled on="));
        Serial.print(tgt_Hdg);
        Serial.println(';');
      }
      Serial.print(F("targetHeading:"));
      if (_headingHoldEnabled) {
        Serial.print(tgt_Hdg);
      } else {
        Serial.print(DISABLED);
      }
      Serial.println(';');
    }


    if( command.cmp("holdDepth_toggle")){
      if (_depthHoldEnabled) {
        _depthHoldEnabled = false;
        raw_lift = 0;
        target_depth = 0;

        int argsToSend[] = {1,0}; //include number of parms as last parm
        command.pushCommand("lift",argsToSend);
        Serial.println(F("log:depth_hold_disabled;"));

      } else {
        _depthHoldEnabled = true;
        if(command.args[0]==0){
          _depthHoldTarget = navdata::DEAP*100;  //casting to cm
        } else {
          _depthHoldTarget = command.args[1];
        }
        target_depth = _depthHoldTarget;
        Serial.print(F("log:depth_hold_enabled on="));
        Serial.print(target_depth);
        Serial.println(';');
      }
      Serial.print(F("targetDepth:"));
      if (_depthHoldEnabled) {
        Serial.print(target_depth);
      } else {
        Serial.print(DISABLED);
      }
      Serial.println(';');
    }


    if (pilotTimer.elapsed (50)) {

      // Autopilot Test #3 6 Jan 2014
      // Hold vehicle at arbitrary heading
      // Integer math; proportional control plus basic integrator
      // No hysteresis around 180 degree error

      // Check whether hold mode is on

      if (_depthHoldEnabled)
      {
        depth = navdata::DEAP*100;
        depth_Error = target_depth-depth;  //positive error = positive lift = go deaper.

        raw_lift = (float)depth_Error * depth_hold_loop_gain;
        lift = constrain(raw_lift, -100, 100);

        Serial.println(F("log:dhold pushing command;"));
        Serial.print(F("dp_er:"));
        Serial.print(depth_Error);
        Serial.println(';');
        if (abs(depth_Error)>depth_deadband){
          int argsToSend[] = {1,lift}; //include number of parms as last parm
          command.pushCommand("lift",argsToSend);
        } else {
          int argsToSend[] = {1,0}; //include number of parms as last parm
          command.pushCommand("lift",argsToSend);
        }

      }

      if (_headingHoldEnabled)
      {

        // Code for hold mode here
        hdg = navdata::HDGD;

        if (hdg > 180)
        {
        hdg = hdg - 360;
        }

        if (hdg < -179)
        {
        hdg = hdg + 360;
        }

        Input = hdg;


        double gap = abs(Setpoint-Input); //distance away from setpoint
        if(gap<10)
        {  //we're close to setpoint, use conservative tuning parameters
          myPID.SetTunings(consKp, consKi, consKd);
        }
        else
        {
           //we're far from setpoint, use aggressive tuning parameters
           myPID.SetTunings(aggKp, aggKi, aggKd);
        }

        myPID.Compute();

        // Constrain and output to motors

        int argsToSend[] = {1,Output}; //include number of parms as last parm
        command.pushCommand("yaw",argsToSend);

      }


    }
}
#endif
