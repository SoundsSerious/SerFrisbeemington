
#ifndef OBSERVER_H
#define OBSERVER_H

#include <Arduino.h>

class Event;

enum ObserverType { OBSERVER, GAME};

class IObserver //In which we notify
{
public:
  virtual void onNotify( Event& ) = 0;
  virtual ObserverType type() = 0;
  ObserverType _type;
};

class Observer: public IObserver
{
public:
  virtual ~Observer() {};
  virtual void onNotify( Event& ){};
  virtual ObserverType type();
  ObserverType _type = OBSERVER;
  uint8_t _index;
};

#endif /* OBSERVER_H */
