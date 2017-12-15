#include "event.h"
#include "globals.h"

// void Event::visit(State *s) { s -> handleInput(*this); }; //State
void Event::visit(Subject &sj) { frisbeem.com.log("Event -> Subject");
                                 sj.notify(*this); }; //Subject
void Event::visit(Observer &o) { frisbeem.com.log("Event -> Observer");
                                 o.onNotify(*this); }; //Observer
void Event::visit(Game     &g) { frisbeem.com.log("Event -> Game");
                                      g.onNotify(*this); }; //Game
// void Event::visit(StateSwitch *s){ s -> handleInput(*this); }; //State
EventTypes Event::type() {return this -> type_;};


// void MotionEvent::visit(State *s) { s -> handleInput(*this); }; //State
void MotionEvent::visit(Subject &sj) { frisbeem.com.log("Motion -> Subject");
                                       sj.notify(*this); }; //Subject
void MotionEvent::visit(Observer &o) { frisbeem.com.log("Motion -> Observer");
                                       o.onNotify(*this); }; //Observer
void MotionEvent::visit(Game     &g) { frisbeem.com.log("Motion -> Game");
                                       g.onNotify(*this); }; //Game
// void MotionEvent::visit(StateSwitch *s){ s -> handleInput(*this); }; //State
EventTypes MotionEvent::type() {return this -> type_;};


// void COMEvent::visit(State *s) { s -> handleInput(*this); }; //State
void COMEvent::visit(Subject &sj) { frisbeem.com.log("COM -> Subject",true);
                                    sj.notify(*this); }; //Subject
void COMEvent::visit(Observer &o) { frisbeem.com.log("COM -> Observer",true);
                                    o.onNotify(*this); }; //Observer
void COMEvent::visit(Game     &g) { frisbeem.com.log("COM -> Game",true);
                                    g.onNotify(*this); }; //Game
EventTypes COMEvent::type() {return this -> type_;};


void COM_GameSelect::visit(Subject &sj) { frisbeem.com.log("GAM_SEL -> Subject",true);
                                          sj.notify(*this); }; //Subject
EventTypes COM_GameSelect::type() {return this -> type_;};

// void COMEvent::visit(StateSwitch *s){ s -> handleInput(*this); }; //State

void EventQueue::processEntries(Subject &subject)
{ if (!_events.empty()){
    //frisbeem.com.log("We Have Events: "+String(_events.size()));
    while (!_events.empty()){
       Event * event= nextEvent();
       String msg = String(event -> type());
       frisbeem.com.log( "Recv Event Type: " + msg );
       event -> visit( subject );

       //Save The Memory, Man. Fuck Alzheimers
       _events.pop(); // Dont Stop
       delete event; //Needs A Virtual Destructor
    }
  }
  else{
    frisbeem.com.log("Mmmm No Events");
  }
}

Event * EventQueue::nextEvent()
{
  return _events.front();
}

void EventQueue::addEvent(Event * event)
{
  _events.push(event);
}
