#ifndef STATE_H
#define STATE_H


#include <Arduino.h>
#undef min
#undef max
#include <vector>
using namespace std;

class Event;
class MotionEvent;
#define MAX_STATES 50 //'Murica
//float MOI = 0.008748; //Frisbeem Moment Of Inertia
class IState
{
public:
  virtual void handleInput( Event &event ) = 0;
  //virtual void handleInput( MotionEvent &event ) = 0;
  virtual String type() = 0;
};


class State: public IState //In which we derive our actions
{
public:
  State() {};
  virtual ~State() {};

  bool sleepModeActivated = false;
  bool moving = true;

  //Override handleInput for visitor pattern
  virtual void handleInput( Event &event );
  virtual void handleInput( MotionEvent &event );
  //virtual void handleInput( COMEvent &event ) = 0;

  //Other Important Functions
  virtual void update(){};
  virtual void enter(){};
  virtual void leave(){};

  virtual String type() {return "state";};
};


//Initial States
//Airborne Transition If GYZ is greater than 450
//Stationary If GYZ is less than 10
//

class MotionState: public State
{
public:
  //Physical Values
  unsigned long lastTime = micros();
  unsigned long now = micros();
  unsigned long dt;

  float lastOmega;
  float newOmega;
  float dOmegaDt,dOmega;
  float torque, _torque;

  unsigned long stationaryCount = 0;
  unsigned long stationaryReset = -10;
  unsigned long sleepThreshold = 25000;

  enum MotionStates {
    STATIONARY,
    AIRBORNE,
  };

  int currentState;

  //Need To Define Method All Event Types... C++ cannot double dispatch so
  //It helps to overload the state event handlers... it can't do both at once
  virtual void handleInput( Event &event );
  virtual void handleInput( MotionEvent &event);
  virtual String type() {return "MotionState";};

};


class StateSwitch: public State
{
  //Class that passes argument to current state
public:
  int currentState = 0;
  std::vector<State*>  _states;
  //Important Funcitons
  virtual void initialize();
  virtual void handleInput( Event &event);

  virtual String type() {return "StateSwitch";};

  State* stateNow()
  {
    return _states.back();
  };
  //Not Implemented Yet
  virtual void update() {
    _states[ currentState ] -> update();
  };
  virtual void enter() {
    _states[ currentState ] -> enter();
  };
  virtual void leave() {
    _states[ currentState ] -> leave();
  };

};


#endif /* STATE_H */
