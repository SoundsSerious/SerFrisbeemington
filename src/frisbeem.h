

#include <Arduino.h>
#include "mpu9250_spi.h"
//#include "mpu9250_i2c.h"
#include "lights.h"
#include "communication.h"
#include "state.h"
#include "entity.h"
#include "subject.h"
#include "event.h"
#include "games.h"
#include "system_info.h"

#ifndef FRISBEEM_H
#define FRISBEEM_H

using namespace std;

//In which we combine Lights + Motion + Games
class Frisbeem: public Subject, public Entity
{
public:
  Frisbeem(): mpu(MPU_SPI_CLOCK,MPU_CS), lights(), com(){};//Constructor
  virtual ~Frisbeem() {}; //Destructor (of doooom)

  //Physical parameters

  //States
  // StateSwitch powerState;
  // StateSwitch motionState;
  // StateSwitch lightMode;
  // StateSwitch gameMode;

  //Hardware
  MPU9250 mpu;
  Lights lights;

  //Connection
  COM com;

  //Games
  GameManager games_manager;

  //Events
  EventQueue event_queue;

  //Important Entity Functions
  virtual void initlaize();
  virtual void update();

  //virtual void notify( Event &event);
  virtual void notify( COMEvent &event);
  virtual void notify( COM_GameSelect &event);

  //Parameters For Interloop
  int start;
  int targetFPS = 60;
  uint32_t budgetRefreshTime = 2500;
  uint32_t renderInterval = (1000000 / targetFPS) - budgetRefreshTime; //Microseconds aka 60fps
  //Sandbox Functions
  unsigned long lastTime,thisTime;
  int lightOffset = 0;
  float thetaOffset = 0;
  float degPerPixel = 360/ NUM_LEDS ;
  void updateThetaOffset();


};

#endif //FRISBEEM_H
