#ifndef EVENT_H
#define EVENT_H

#include <Arduino.h>
#include "state.h"
#include "subject.h"
#include "games.h"
#include <queue>
//#import "observer.h"
//#include "globals.h"

//Event Guidelines:
//Never send an event from a place where events are handled.
//Memory Managment Is Up To You Guy (unique_ptr may help)

//The Visitor pattern defines which events go where (Games, Audio, Image ect)

class Game; //Gotta Predeclare
class Subject;

enum EventTypes {EVENT,MOTION_EVENT,COM_EVENT,COM_GAMESEL_EVENT};

class IEvent
{
  // virtual void visit(State *s) =0;
  virtual void visit(Subject &sj) = 0;
  virtual void visit(Observer &o) =0;
  virtual void visit(Game &g) = 0;//Doesn't Work... I think this is due to storage of observer pointers
  // virtual void visit(StateSwitch *s) =0; //State

  EventTypes type_  = EVENT;
  virtual EventTypes type()=0;
};

class Event: public IEvent{
public:
  Event(){created = micros();};
  virtual ~Event() {}; //Gotta Have A Virtual Distructor otherwise memory leak
  unsigned long created;
  EventTypes type_  = EVENT;
  //Should Return The Type Of Event
  // virtual void visit(State *s); //State
  virtual void visit(Subject &sj); //Subject
  virtual void visit(Observer &o); //Observer
  virtual void visit(Game &g);
  // virtual void visit(StateSwitch *s); //State

  virtual EventTypes type();
};



class MotionEvent: public Event
{
public:
  //MotionEvent(){};
  MotionEvent( float omega ){ omega = omega;
                             created = micros();};
  virtual ~MotionEvent(){};
  EventTypes type_ = MOTION_EVENT;

  //Should Return The Type Of Event
  // virtual void visit(State *s); //State
  virtual void visit(Subject &sj); //Subject
  virtual void visit(Observer &o); //Observer
  virtual void visit(Game &g);
  // virtual void visit(StateSwitch *s); //State

  virtual EventTypes type();

  float omega;
};


class COMEvent: public Event {
public:
  COMEvent(String pk, String sk, String arg)
  {
      primary_key = pk;
      secondary_key = sk;
      message = arg;
  };
  virtual ~COMEvent(){};
  EventTypes type_ = COM_EVENT;
  String primary_key = "";
  String secondary_key = "";
  String message = "";
  //Should Return The Type Of Event
  // virtual void visit(State *s); //State
  virtual void visit(Subject &sj); //Subject
  virtual void visit(Observer &o); //Observer
  virtual void visit(Game &g); //Observer
  // virtual void visit(StateSwitch *s); //State

  virtual EventTypes type();
};

class COM_GameSelect: public COMEvent {
public:
  COM_GameSelect(unsigned int gameSelection): COMEvent("GAM","SEL","0")
  {
    selectedGame = gameSelection;
    primary_key = "GAM";
    secondary_key = "SEL";
    message = String(selectedGame);
  };

  virtual ~COM_GameSelect(){};
  EventTypes type_ = COM_GAMESEL_EVENT;

  unsigned int selectedGame = 0;
  //Should Return The Type Of Event
  // virtual void visit(State *s); //State
  virtual void visit(Subject &sj); //Subject
  //virtual void visit(Observer &o); //Observer
  //virtual void visit(Game &g); //Observer
  // virtual void visit(StateSwitch *s); //State

  virtual EventTypes type();
};

class EventQueue{
  public:
    EventQueue(){};
    ~EventQueue() {};
  private:
    std::queue <Event*> _events;
  public:
    void processEntries(Subject &subject);
    void addEvent(Event * event);
    Event * nextEvent();

};

#endif //EVENT_H
