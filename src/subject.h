#ifndef SUBJECT_H
#define SUBJECT_H

#include <Arduino.h>
#include "observer.h"
//#include "event.h"
#include "games.h"
#include <MPU9250_spi.h>
#include <vector>

#define MAX_OBSERVERS 20

class Event;
class COMEvent;
class COM_GameSelect; //Place Holder For Linker Or Something

class ISubject
{
  virtual void notify( Event &event) = 0;
};

class Subject: public ISubject //In which we send notifications
{
public:
  Subject(): observers() {};
  virtual ~Subject(){};
  //Observer Storage
  std::vector<Observer*> observers;
  //Observer *observers_[MAX_OBSERVERS];
  //int numObservers_;
  void addObserver(Observer *observer);
  void removeObserver(Observer *observer);
  virtual void notify( Event &event);
  virtual void notify( COMEvent &event){};
  virtual void notify( COM_GameSelect &event){};
};

#endif /* SUBJECT_H */
