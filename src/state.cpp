#include "state.h"
#include "globals.h"

void StateSwitch::initialize(){
  //frisbeem._com.log("Pointer Assignment");

  _states.push_back( new MotionState() );
  //frisbeem._com.log(String("Last Item") + String(_states.back() -> type() ));
}

void StateSwitch::handleInput( Event &event)
{ //frisbeem._com.log("Switch Handiling Input: "+event.type());
  State *s = stateNow();
  //event.visit( s );
}

void State::handleInput(Event &event)
{
  //frisbeem._com.log("State Handiling Event Input: "+event.type());
}

void State::handleInput(MotionEvent &event)
{
  //frisbeem._com.log("State Handiling MotionEvent Input: "+event.type());
}

void MotionState::handleInput(Event &event)
{
  //frisbeem._com.log("MotionState Handiling Event Input: "+event.type() );
}

void MotionState::handleInput(MotionEvent &event)
  { //Initialize and do some physics math
      //frisbeem._com.log("MotionState Handiling MotionEvent Input: "+event.type());
      now = micros();
      newOmega = fabs(event.omega);//();
      dOmega = newOmega - lastOmega;
      dt = (now - lastTime);
      dOmegaDt = dOmega *1E6/ dt;



      //Apply Torque Through Low Pass Filter
      _torque = 0.008748 * dOmegaDt;
      torque = 0.95 * _torque + 0.05 * torque;

      //Serial.print("Torque \t");Serial.println(torque);
      //Serial.print("Omega \t");Serial.println(newOmega);
      if (newOmega > 200){ //Airborne
          stationaryCount = 0;
          moving = true;
          //Serial.println("Moving...");
          sleepModeActivated = false;
      }
      else{
        //Serial.println("Stationary...");
        stationaryCount++;
        moving = false;
        if (stationaryCount > sleepThreshold){
          //Serial.println("Greater Than threshold");
         if (stationaryCount > stationaryReset){
           stationaryCount = 0;
         }
         stationaryCount = stationaryCount + 10;
         sleepModeActivated = true;
        }
        else{
          //Serial.print("stationaryCount");Serial.println( stationaryCount);
          sleepModeActivated = false;

        }
      }
      if (sleepModeActivated){
       //Turn Off Lights. Switch Arduino to Low Power Consumption.
       //frisbeem._com.log("Sleep Mode");

      }
      else {
        if (moving) { //Rainbow Pattern
          //frisbeem._com.log("Airborne");
          currentState = AIRBORNE;
        }
        else{ //Stationary
          //frisbeem._com.log("Stationary");
          currentState = STATIONARY;
        }
      }
      //delay(50);//temp
      //Setup For Next Update
      lastTime = now;
      lastOmega = newOmega;
  }
